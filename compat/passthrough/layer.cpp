// SPDX-License-Identifier: GPL-3.0-or-later
// XROS runtime adapter: project the existing camera owner's snapshots underneath
// application layers; leave final composition/reprojection to the active runtime.
#define XR_USE_GRAPHICS_API_VULKAN
#define XR_USE_TIMESPEC
#define XR_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

struct Eye { float orientation[4], position[3], fov[4]; };
extern "C" {
    bool xros_camera_config_valid();
    void *xros_camera_create(PFN_vkGetInstanceProcAddr, uint64_t, uint64_t, uint64_t, uint32_t, uint32_t, int32_t);
    int32_t xros_camera_add_image(void *, uint64_t, int32_t, uint32_t, uint32_t);
    void xros_camera_observe(void *, const Eye *, uint64_t, uint64_t);
    void xros_camera_reset(void *);
    int32_t xros_camera_render(void *, uint32_t, uint32_t, const Eye *);
    void xros_camera_submitted(void *, bool);
    void xros_camera_idle(void *);
    void xros_camera_destroy(void *);
}
struct Instance {
    XrInstance handle{};
    PFN_xrGetInstanceProcAddr gpa{};
    bool enabled=false;
    PFN_vkGetInstanceProcAddr vk_gpa{};
#define DISPATCH(X) \
    X(GetSystemProperties) X(EnumerateEnvironmentBlendModes) X(EnumerateViewConfigurationViews) \
    X(CreateSession) X(DestroySession) X(BeginSession) X(EndSession) X(EndFrame) X(LocateViews) \
    X(CreateSwapchain) X(DestroySwapchain) X(EnumerateSwapchainFormats) X(EnumerateSwapchainImages) \
    X(AcquireSwapchainImage) X(WaitSwapchainImage) X(ReleaseSwapchainImage) \
    X(CreateReferenceSpace) X(DestroySpace) X(DestroyInstance) X(PollEvent)
#define FIELD(n) PFN_xr##n n{};
    DISPATCH(FIELD)
#undef FIELD
    PFN_xrCreateVulkanInstanceKHR CreateVulkanInstanceKHR{};
    PFN_xrConvertTimespecTimeToTimeKHR ConvertTimespecTimeToTimeKHR{};
};
struct Chain {
    XrSwapchain handle{};
    uint32_t width=0, height=0;
    std::vector<uint32_t> targets;
};
struct Session {
    std::shared_ptr<Instance> i;
    XrSession handle{};
    XrSpace space{};
    std::array<Chain,2> chains;
    void *camera{};
    bool stereo=false, failed=false;
    XrTime reset_at=0;
    uint32_t layer_limit=0;
    ~Session() { release(); }
    void release() {
        // Framebuffers/image views must disappear before their XR storage.
        xros_camera_destroy(camera); camera=nullptr;
        for (auto &c:chains) {if(c.handle) i->DestroySwapchain(c.handle); c.handle=XR_NULL_HANDLE;}
        if(space) i->DestroySpace(space);
        space=XR_NULL_HANDLE;
    }
};
static std::mutex maps_mutex;
static std::unordered_map<XrInstance,std::shared_ptr<Instance>> instances;
static std::unordered_map<XrSession,std::shared_ptr<Session>> sessions;
template<class H,class T> static std::shared_ptr<T> find(const std::unordered_map<H,std::shared_ptr<T>> &map,H h) {
    std::lock_guard<std::mutex> lock(maps_mutex); auto it=map.find(h); return it==map.end()?nullptr:it->second;
}
static bool enabled(const std::shared_ptr<Instance> &i,XrSystemId system) {
    if(!i->enabled) return false;
    XrSystemProperties p{XR_TYPE_SYSTEM_PROPERTIES};
    return XR_SUCCEEDED(i->GetSystemProperties(i->handle,system,&p)) && p.graphicsProperties.maxLayerCount>=2;
}
static XrResult XRAPI_CALL get_system_properties(XrInstance instance,XrSystemId system,XrSystemProperties *p) {
    auto i=find(instances,instance); if(!i) return XR_ERROR_HANDLE_INVALID;
    auto r=i->GetSystemProperties(instance,system,p);
    if(XR_SUCCEEDED(r) && i->enabled && p->graphicsProperties.maxLayerCount>=2) --p->graphicsProperties.maxLayerCount;
    return r;
}
static XrResult XRAPI_CALL blend_modes(XrInstance instance,XrSystemId system,XrViewConfigurationType type,
    uint32_t capacity,uint32_t *count,XrEnvironmentBlendMode *modes) {
    auto i=find(instances,instance); if(!i) return XR_ERROR_HANDLE_INVALID;
    if(!enabled(i,system) || type!=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
        return i->EnumerateEnvironmentBlendModes(instance,system,type,capacity,count,modes);
    if(!count || (capacity && !modes)) return XR_ERROR_VALIDATION_FAILURE;
    uint32_t n=0; auto r=i->EnumerateEnvironmentBlendModes(instance,system,type,0,&n,nullptr); if(XR_FAILED(r)) return r;
    std::vector<XrEnvironmentBlendMode> values(n);
    r=i->EnumerateEnvironmentBlendModes(instance,system,type,n,&n,values.data()); if(XR_FAILED(r)) return r;
    values.resize(n);
    if(std::find(values.begin(),values.end(),XR_ENVIRONMENT_BLEND_MODE_OPAQUE)==values.end())
        return i->EnumerateEnvironmentBlendModes(instance,system,type,capacity,count,modes);
    if(std::find(values.begin(),values.end(),XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND)==values.end())
        values.push_back(XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND);
    *count=static_cast<uint32_t>(values.size());
    if(capacity==0) return XR_SUCCESS;
    if(capacity<*count) return XR_ERROR_SIZE_INSUFFICIENT;
    std::copy(values.begin(),values.end(),modes); return XR_SUCCESS;
}
static XrResult XRAPI_CALL create_vk_instance(XrInstance instance,const XrVulkanInstanceCreateInfoKHR *info,
    VkInstance *vk,VkResult *result) {
    auto i=find(instances,instance); if(!i) return XR_ERROR_HANDLE_INVALID;
    if(!i->CreateVulkanInstanceKHR) return XR_ERROR_FUNCTION_UNSUPPORTED;
    auto r=i->CreateVulkanInstanceKHR(instance,info,vk,result);
    if(XR_SUCCEEDED(r) && *result==VK_SUCCESS) i->vk_gpa=info->pfnGetInstanceProcAddr;
    return r;
}
#define XR_TRY(expr) do {auto result=(expr); if(XR_FAILED(result)) return result;} while(0)
static XrResult initialize(Session &s,const XrGraphicsBindingVulkanKHR &binding,XrSystemId system) {
    auto &i=*s.i;
    XrSystemProperties p{XR_TYPE_SYSTEM_PROPERTIES};
    XR_TRY(i.GetSystemProperties(i.handle,system,&p)); s.layer_limit=p.graphicsProperties.maxLayerCount-1;
    uint32_t n=0;
    XR_TRY(i.EnumerateViewConfigurationViews(i.handle,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,0,&n,nullptr));
    if(n!=2) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    XrViewConfigurationView views[2]={{XR_TYPE_VIEW_CONFIGURATION_VIEW},{XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    XR_TRY(i.EnumerateViewConfigurationViews(i.handle,system,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,2,&n,views));
    XR_TRY(i.EnumerateSwapchainFormats(s.handle,0,&n,nullptr));
    std::vector<int64_t> formats(n); XR_TRY(i.EnumerateSwapchainFormats(s.handle,n,&n,formats.data()));
    int64_t format=0;
    for(auto candidate:{VK_FORMAT_R8G8B8A8_SRGB,VK_FORMAT_B8G8R8A8_SRGB,VK_FORMAT_R8G8B8A8_UNORM,VK_FORMAT_B8G8R8A8_UNORM})
        if(std::find(formats.begin(),formats.end(),candidate)!=formats.end()) {format=candidate;break;}
    if(!format) return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    s.camera=xros_camera_create(i.vk_gpa,reinterpret_cast<uint64_t>(binding.instance),
        reinterpret_cast<uint64_t>(binding.physicalDevice),reinterpret_cast<uint64_t>(binding.device),
        binding.queueFamilyIndex,binding.queueIndex,static_cast<int32_t>(format));
    if(!s.camera) return XR_ERROR_INITIALIZATION_FAILED;
    XrReferenceSpaceCreateInfo reference{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    reference.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_LOCAL;reference.poseInReferenceSpace.orientation.w=1;
    XR_TRY(i.CreateReferenceSpace(s.handle,&reference,&s.space));
    for(size_t eye=0;eye<2;++eye) {
        auto &c=s.chains[eye]; c.width=views[eye].recommendedImageRectWidth;c.height=views[eye].recommendedImageRectHeight;
        XrSwapchainCreateInfo ci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        ci.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;ci.format=format;ci.sampleCount=1;
        ci.width=c.width;ci.height=c.height;ci.faceCount=ci.arraySize=ci.mipCount=1;
        XR_TRY(i.CreateSwapchain(s.handle,&ci,&c.handle));
        XR_TRY(i.EnumerateSwapchainImages(c.handle,0,&n,nullptr));
        std::vector<XrSwapchainImageVulkanKHR> images(n,{XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
        XR_TRY(i.EnumerateSwapchainImages(c.handle,n,&n,reinterpret_cast<XrSwapchainImageBaseHeader *>(images.data())));
        for(auto &image:images) {
            auto target=xros_camera_add_image(s.camera,(uint64_t)image.image,static_cast<int32_t>(format),c.width,c.height);
            if(target<0) return XR_ERROR_INITIALIZATION_FAILED;
            c.targets.push_back(static_cast<uint32_t>(target));
        }
    }
    xros_camera_idle(s.camera);
    std::fprintf(stderr,"XROS_PASSTHROUGH_READY mode=alpha-blend backend=runtime-projection layers=%u\n",s.layer_limit);
    return XR_SUCCESS;
}
static XrResult XRAPI_CALL create_session(XrInstance instance,const XrSessionCreateInfo *info,XrSession *out) {
    auto i=find(instances,instance); if(!i) return XR_ERROR_HANDLE_INVALID;
    auto r=i->CreateSession(instance,info,out); if(XR_FAILED(r)) return r;
    auto s=std::make_shared<Session>();s->i=i;s->handle=*out;
    if(enabled(i,info->systemId)) {
        const XrGraphicsBindingVulkanKHR *binding=nullptr;
        for(auto p=static_cast<const XrBaseInStructure *>(info->next);p;p=p->next)
            if(p->type==XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) binding=reinterpret_cast<const XrGraphicsBindingVulkanKHR *>(p);
        r=binding && i->vk_gpa?initialize(*s,*binding,info->systemId):XR_ERROR_GRAPHICS_DEVICE_INVALID;
        if(XR_FAILED(r)) {s->release();i->DestroySession(*out);*out=XR_NULL_HANDLE;return r;}
    }
    std::lock_guard<std::mutex> lock(maps_mutex);sessions[*out]=s;return XR_SUCCESS;
}
static XrResult XRAPI_CALL begin_session(XrSession session,const XrSessionBeginInfo *info) {
    auto s=find(sessions,session);if(!s)return XR_ERROR_HANDLE_INVALID;
    auto r=s->i->BeginSession(session,info);
    if(XR_SUCCEEDED(r)){s->stereo=info->primaryViewConfigurationType==XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        if(s->camera)xros_camera_reset(s->camera);}
    return r;
}
static XrResult XRAPI_CALL end_session(XrSession session) {
    auto s=find(sessions,session);if(!s)return XR_ERROR_HANDLE_INVALID;
    auto r=s->i->EndSession(session);
    if(XR_SUCCEEDED(r)){s->stereo=false;if(s->camera){xros_camera_reset(s->camera);xros_camera_idle(s->camera);}}
    return r;
}
static std::array<Eye,2> pack(const XrView (&views)[2]) {
    std::array<Eye,2> out{};
    for(size_t n=0;n<2;++n) {const auto &v=views[n];out[n]={
        {v.pose.orientation.x,v.pose.orientation.y,v.pose.orientation.z,v.pose.orientation.w},
        {v.pose.position.x,v.pose.position.y,v.pose.position.z},
        {v.fov.angleLeft,v.fov.angleRight,v.fov.angleUp,v.fov.angleDown}};}
    return out;
}
static XrResult locate(Session &s,XrTime time,XrView (&views)[2],XrViewState &state) {
    XrViewLocateInfo info{XR_TYPE_VIEW_LOCATE_INFO};info.viewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    info.displayTime=time;info.space=s.space;uint32_t n=0;
    auto r=s.i->LocateViews(s.handle,&info,&state,2,&n,views);
    return XR_SUCCEEDED(r) && n!=2?XR_ERROR_RUNTIME_FAILURE:r;
}
// Acquired image cleanup is explicit; a wait failure poisons this adapter session
// and is surfaced to the app, rather than reusing an image with unknown ownership.
struct Acquired {
    Session &s; std::array<bool,2> waited{};
    XrResult release() {
        XrResult result=XR_SUCCESS;
        for(size_t eye=0;eye<2;++eye) if(waited[eye]) {
            XrSwapchainImageReleaseInfo info{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            auto r=s.i->ReleaseSwapchainImage(s.chains[eye].handle,&info);
            waited[eye]=false;if(XR_FAILED(r))result=r;
        }
        return result;
    }
    ~Acquired(){release();}
};
static XrResult XRAPI_CALL end_frame(XrSession session,const XrFrameEndInfo *info) {
    auto s=find(sessions,session);if(!s)return XR_ERROR_HANDLE_INVALID;
    auto &i=*s->i;
    if(!s->camera) return i.EndFrame(session,info);
    if(s->failed) return XR_ERROR_RUNTIME_FAILURE;
    if(s->reset_at && info->displayTime>=s->reset_at) {xros_camera_reset(s->camera);s->reset_at=0;}
    // The history is stamped with converted CLOCK_MONOTONIC, never predicted display time.
    if(i.ConvertTimespecTimeToTimeKHR && s->stereo) {
        timespec now{};clock_gettime(CLOCK_MONOTONIC,&now);XrTime time=0;
        XrView v[2]={{XR_TYPE_VIEW},{XR_TYPE_VIEW}};XrViewState state{XR_TYPE_VIEW_STATE};
        if(XR_SUCCEEDED(i.ConvertTimespecTimeToTimeKHR(i.handle,&now,&time)) && XR_SUCCEEDED(locate(*s,time,v,state))) {
            auto eyes=pack(v);xros_camera_observe(s->camera,eyes.data(),state.viewStateFlags,uint64_t(now.tv_sec)*1000000000+now.tv_nsec);
        } else xros_camera_reset(s->camera);
    }
    if(info->environmentBlendMode!=XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) {
        xros_camera_idle(s->camera);return i.EndFrame(session,info);
    }
    if(!s->stereo) return XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED;
    if(info->layerCount>s->layer_limit) return XR_ERROR_LAYER_LIMIT_EXCEEDED;
    XrFrameEndInfo frame=*info;frame.environmentBlendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    // A zero-layer frame means render nothing, including our background.
    if(info->layerCount==0) {xros_camera_idle(s->camera);return i.EndFrame(session,&frame);}
    if(!info->layers) return XR_ERROR_VALIDATION_FAILURE;
    XrView views[2]={{XR_TYPE_VIEW},{XR_TYPE_VIEW}};XrViewState state{XR_TYPE_VIEW_STATE};
    XR_TRY(locate(*s,info->displayTime,views,state));
    if(!(state.viewStateFlags&XR_VIEW_STATE_ORIENTATION_VALID_BIT)) {
        xros_camera_reset(s->camera);xros_camera_idle(s->camera);return i.EndFrame(session,&frame);
    }
    std::array<uint32_t,2> indices{};Acquired acquired{*s};
    for(size_t eye=0;eye<2;++eye) {
        auto &c=s->chains[eye];uint32_t index=0;
        XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        auto r=i.AcquireSwapchainImage(c.handle,&ai,&index);
        if(XR_FAILED(r)){s->failed=true;return r;}
        XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};wi.timeout=XR_INFINITE_DURATION;
        r=i.WaitSwapchainImage(c.handle,&wi);
        if(r!=XR_SUCCESS){s->failed=true;return XR_FAILED(r)?r:XR_ERROR_RUNTIME_FAILURE;}
        acquired.waited[eye]=true;
        if(index>=c.targets.size()){s->failed=true;return XR_ERROR_RUNTIME_FAILURE;}
        indices[eye]=c.targets[index];
    }
    auto eyes=pack(views);auto active=xros_camera_render(s->camera,indices[0],indices[1],eyes.data());
    if(active<0){s->failed=true;return XR_ERROR_RUNTIME_FAILURE;}
    auto release=acquired.release();if(XR_FAILED(release)){s->failed=true;return release;}
    XrCompositionLayerProjectionView pv[2]={{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    for(size_t eye=0;eye<2;++eye) {
        const auto &c=s->chains[eye];pv[eye].pose=views[eye].pose;pv[eye].fov=views[eye].fov;
        pv[eye].subImage.swapchain=c.handle;pv[eye].subImage.imageRect.extent={static_cast<int32_t>(c.width),static_cast<int32_t>(c.height)};
    }
    XrCompositionLayerProjection background{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    background.space=s->space;background.viewCount=2;background.views=pv;
    std::vector<const XrCompositionLayerBaseHeader *> layers;
    layers.reserve(info->layerCount+1);layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&background));
    layers.insert(layers.end(),info->layers,info->layers+info->layerCount);
    frame.layerCount=static_cast<uint32_t>(layers.size());frame.layers=layers.data();
    auto r=i.EndFrame(session,&frame);xros_camera_submitted(s->camera,XR_SUCCEEDED(r)&&active==1);return r;
}
static XrResult XRAPI_CALL poll_event(XrInstance instance,XrEventDataBuffer *event) {
    auto i=find(instances,instance);if(!i)return XR_ERROR_HANDLE_INVALID;
    auto r=i->PollEvent(instance,event);
    if(r==XR_SUCCESS && event->type==XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) {
        auto e=reinterpret_cast<const XrEventDataReferenceSpaceChangePending *>(event);
        auto s=find(sessions,e->session);
        if(s && e->referenceSpaceType==XR_REFERENCE_SPACE_TYPE_LOCAL) s->reset_at=e->changeTime;
    }
    return r;
}
static XrResult XRAPI_CALL destroy_session(XrSession session) {
    auto s=find(sessions,session);if(!s)return XR_ERROR_HANDLE_INVALID;
    s->release();auto r=s->i->DestroySession(session);
    if(XR_SUCCEEDED(r)){std::lock_guard<std::mutex> lock(maps_mutex);sessions.erase(session);}
    return r;
}
static XrResult XRAPI_CALL destroy_instance(XrInstance instance) {
    auto i=find(instances,instance);if(!i)return XR_ERROR_HANDLE_INVALID;
    std::vector<std::shared_ptr<Session>> children;
    {std::lock_guard<std::mutex> lock(maps_mutex);for(auto &entry:sessions)if(entry.second->i==i)children.push_back(entry.second);}
    for(auto &s:children)s->release();
    auto r=i->DestroyInstance(instance);
    if(XR_SUCCEEDED(r)){std::lock_guard<std::mutex> lock(maps_mutex);for(auto &s:children)sessions.erase(s->handle);instances.erase(instance);}
    return r;
}
static XrResult XRAPI_CALL get_proc(XrInstance instance,const char *name,PFN_xrVoidFunction *out) {
    if(!name || !out)return XR_ERROR_VALIDATION_FAILURE;
    auto i=find(instances,instance);if(!i){*out=nullptr;return XR_ERROR_HANDLE_INVALID;}
#define ENTRY(n,f) if(!std::strcmp(name,"xr" #n)){*out=reinterpret_cast<PFN_xrVoidFunction>(f);return XR_SUCCESS;}
    ENTRY(GetInstanceProcAddr,get_proc) ENTRY(GetSystemProperties,get_system_properties)
    ENTRY(EnumerateEnvironmentBlendModes,blend_modes) ENTRY(CreateSession,create_session)
    ENTRY(BeginSession,begin_session) ENTRY(EndSession,end_session) ENTRY(EndFrame,end_frame)
    ENTRY(PollEvent,poll_event) ENTRY(DestroySession,destroy_session) ENTRY(DestroyInstance,destroy_instance)
    if(i->CreateVulkanInstanceKHR){ENTRY(CreateVulkanInstanceKHR,create_vk_instance)}
#undef ENTRY
    return i->gpa(instance,name,out);
}
static XrResult XRAPI_CALL create_instance(const XrInstanceCreateInfo *info,const XrApiLayerCreateInfo *layer,XrInstance *out) {
    if(!info || !layer || !layer->nextInfo)return XR_ERROR_INITIALIZATION_FAILED;
    auto next=*layer;next.nextInfo=layer->nextInfo->next;
    auto r=layer->nextInfo->nextCreateApiLayerInstance(info,&next,out);if(XR_FAILED(r))return r;
    auto i=std::make_shared<Instance>();i->handle=*out;i->gpa=layer->nextInfo->nextGetInstanceProcAddr;
#define LOAD(n) if(XR_FAILED(i->gpa(*out,"xr" #n,reinterpret_cast<PFN_xrVoidFunction *>(&i->n)))) return XR_ERROR_INITIALIZATION_FAILED;
    // Load destroy first so partial initialization can clean up the downstream instance.
    i->gpa(*out,"xrDestroyInstance",reinterpret_cast<PFN_xrVoidFunction *>(&i->DestroyInstance));
    auto load=[&]()->XrResult {DISPATCH(LOAD) return XR_SUCCESS;};
    r=load();if(XR_FAILED(r)){if(i->DestroyInstance)i->DestroyInstance(*out);*out=XR_NULL_HANDLE;return r;}
#undef LOAD
    bool vulkan2=false,timespec=false;
    for(uint32_t n=0;n<info->enabledExtensionCount;++n){
        vulkan2 |= !std::strcmp(info->enabledExtensionNames[n],XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME);
        timespec |= !std::strcmp(info->enabledExtensionNames[n],XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME);
    }
    if(vulkan2)i->gpa(*out,"xrCreateVulkanInstanceKHR",reinterpret_cast<PFN_xrVoidFunction *>(&i->CreateVulkanInstanceKHR));
    if(timespec)i->gpa(*out,"xrConvertTimespecTimeToTimeKHR",reinterpret_cast<PFN_xrVoidFunction *>(&i->ConvertTimespecTimeToTimeKHR));
    i->enabled=vulkan2 && std::getenv("XROS_PASSTHROUGH_CONFIG") && xros_camera_config_valid();
    std::lock_guard<std::mutex> lock(maps_mutex);instances[*out]=i;return XR_SUCCESS;
}
extern "C" __attribute__((visibility("default"))) XrResult XRAPI_CALL
xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo *loader,const char *,XrNegotiateApiLayerRequest *request) {
    if(!loader || !request || loader->minInterfaceVersion>1 || loader->maxInterfaceVersion<1)return XR_ERROR_INITIALIZATION_FAILED;
    request->layerInterfaceVersion=1;request->layerApiVersion=XR_MAKE_VERSION(1,0,XR_VERSION_PATCH(XR_CURRENT_API_VERSION));
    request->getInstanceProcAddr=get_proc;request->createApiLayerInstance=create_instance;return XR_SUCCESS;
}
