// SPDX-License-Identifier: MIT
// Explicit native-app compatibility experiment; no frame-loop interception.
#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstddef>
#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

struct Instance {
    VkInstance instance;
    PFN_vkGetInstanceProcAddr gpa;
    PFN_GetPhysicalDeviceProcAddr pgpa;
};
struct Device {
    PFN_vkGetDeviceProcAddr gpa;
    PFN_vkCreateImage create_image;
    PFN_vkCreateImageView create_view;
    std::unordered_set<VkImage> observed;
};
static std::mutex mutex;
static std::unordered_map<void *, std::shared_ptr<Instance>> instances;
static std::unordered_map<void *, std::shared_ptr<Device>> devices;
template <class H> static void *key(H h) { return *reinterpret_cast<void **>(h); }
static std::shared_ptr<Instance> instance(void *k) {
    std::lock_guard<std::mutex> l(mutex); auto p = instances.find(k);
    return p == instances.end() ? nullptr : p->second;
}
static std::shared_ptr<Device> device(VkDevice d) {
    std::lock_guard<std::mutex> l(mutex); auto p = devices.find(key(d));
    return p == devices.end() ? nullptr : p->second;
}
static bool enabled(const char *n) { auto s = std::getenv(n); return s && !std::strcmp(s, "1"); }
static PFN_vkVoidFunction VKAPI_CALL get_instance(VkInstance, const char *);
static PFN_vkVoidFunction VKAPI_CALL get_device(VkDevice, const char *);

static VkResult VKAPI_CALL create_image(VkDevice d, const VkImageCreateInfo *ci,
        const VkAllocationCallbacks *a, VkImage *image) {
    auto c = device(d); if (!c) return VK_ERROR_INITIALIZATION_FAILED;
    const VkImageFormatListCreateInfo *list = nullptr;
    bool external = false;
    for (auto p = static_cast<const VkBaseInStructure *>(ci->pNext); p; p = p->pNext) {
        if (p->sType == VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO)
            list = reinterpret_cast<const VkImageFormatListCreateInfo *>(p);
        if (p->sType == VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO)
            external = reinterpret_cast<const VkExternalMemoryImageCreateInfo *>(p)->handleTypes != 0;
    }
    bool target = external && ci->arrayLayers == 2 && ci->mipLevels == 1 &&
        ci->samples == VK_SAMPLE_COUNT_1_BIT && (ci->flags & VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT) &&
        !(ci->flags & VK_IMAGE_CREATE_SUBSAMPLED_BIT_EXT) &&
        (ci->usage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) &&
        (ci->format == VK_FORMAT_R8G8B8A8_SRGB || ci->format == VK_FORMAT_B8G8R8A8_SRGB);
    VkFormat linear = ci->format == VK_FORMAT_R8G8B8A8_SRGB ? VK_FORMAT_R8G8B8A8_UNORM : VK_FORMAT_B8G8R8A8_UNORM;
    bool pair = list && list->viewFormatCount > 0;
    if (list) for (uint32_t i = 0; i < list->viewFormatCount; ++i)
        pair = pair && (list->pViewFormats[i] == ci->format || list->pViewFormats[i] == linear);
    VkImageCreateInfo copy = *ci;
    alignas(std::max_align_t) unsigned char prefix[8][128];
    bool applied = target && pair && enabled("QIYU_VK_SHARED_MUTABLE_LAYOUT");
    if (applied) {
        auto p = static_cast<const VkBaseInStructure *>(ci->pNext);
        VkBaseOutStructure *last = nullptr; size_t n = 0;
        // Clone known prefixes, preserving other extension data and the tail.
        for (; p && p->sType != VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO; p = p->pNext) {
            size_t size = p->sType == VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO ? sizeof(VkExternalMemoryImageCreateInfo) : 0;
            if (!size || n == 8 || size > sizeof(prefix[0])) { applied = false; break; }
            std::memcpy(prefix[n], p, size);
            auto node = reinterpret_cast<VkBaseOutStructure *>(prefix[n++]);
            if (last) last->pNext = node; else copy.pNext = node;
            last = node;
        }
        if (applied && p) {
            if (last) last->pNext = reinterpret_cast<VkBaseOutStructure *>(const_cast<VkBaseInStructure *>(p->pNext));
            else copy.pNext = p->pNext;
        } else { applied = false; copy = *ci; }
    }
    VkResult r = c->create_image(d, applied ? &copy : ci, a, image);
    if (target) {
        std::fprintf(stderr, "QIYU_INTEROP image format=%d flags=0x%x usage=0x%x %ux%u layers=%u list=%u applied=%d result=%d\n",
            ci->format, ci->flags, ci->usage, ci->extent.width, ci->extent.height,
            ci->arrayLayers, list ? list->viewFormatCount : 0, applied, r);
        if (r == VK_SUCCESS) { std::lock_guard<std::mutex> l(mutex); c->observed.insert(*image); }
    }
    return r;
}
static VkResult VKAPI_CALL create_view(VkDevice d, const VkImageViewCreateInfo *ci,
        const VkAllocationCallbacks *a, VkImageView *view) {
    auto c = device(d); if (!c) return VK_ERROR_INITIALIZATION_FAILED;
    bool tracked; { std::lock_guard<std::mutex> l(mutex); tracked = c->observed.count(ci->image); }
    if (tracked) std::fprintf(stderr, "QIYU_INTEROP view format=%d base_layer=%u layers=%u\n",
        ci->format, ci->subresourceRange.baseArrayLayer, ci->subresourceRange.layerCount);
    return c->create_view(d, ci, a, view);
}
static void VKAPI_CALL destroy_device(VkDevice d, const VkAllocationCallbacks *a) {
    auto c = device(d); if (!c) return;
    auto destroy = reinterpret_cast<PFN_vkDestroyDevice>(c->gpa(d, "vkDestroyDevice"));
    auto wait = reinterpret_cast<PFN_vkDeviceWaitIdle>(c->gpa(d, "vkDeviceWaitIdle"));
    bool idle = enabled("QIYU_VK_WAIT_IDLE_ON_DESTROY");
    auto begin = std::chrono::steady_clock::now(); VkResult r = VK_SUCCESS;
    if (idle) r = wait(d);
    std::fprintf(stderr, "QIYU_INTEROP destroy_device wait=%d result=%d elapsed_us=%lld\n", idle, r,
        (long long)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-begin).count());
    { std::lock_guard<std::mutex> l(mutex); devices.erase(key(d)); }
    destroy(d, a);
}
static VkResult VKAPI_CALL create_device(VkPhysicalDevice p, const VkDeviceCreateInfo *ci,
        const VkAllocationCallbacks *a, VkDevice *d) {
    auto c = instance(key(p)); if (!c) return VK_ERROR_INITIALIZATION_FAILED;
    auto link = reinterpret_cast<const VkLayerDeviceCreateInfo *>(ci->pNext);
    for (; link; link = reinterpret_cast<const VkLayerDeviceCreateInfo *>(link->pNext))
        if (link->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO && link->function == VK_LAYER_LINK_INFO) break;
    if (!link) return VK_ERROR_INITIALIZATION_FAILED;
    auto next_gpa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    auto next_gdpa = link->u.pLayerInfo->pfnNextGetDeviceProcAddr;
    auto next_create = reinterpret_cast<PFN_vkCreateDevice>(next_gpa(c->instance, "vkCreateDevice"));
    const_cast<VkLayerDeviceCreateInfo *>(link)->u.pLayerInfo = link->u.pLayerInfo->pNext;
    VkResult r = next_create(p, ci, a, d);
    if (r == VK_SUCCESS) {
        auto ctx = std::make_shared<Device>(); ctx->gpa = next_gdpa;
        ctx->create_image = reinterpret_cast<PFN_vkCreateImage>(next_gdpa(*d, "vkCreateImage"));
        ctx->create_view = reinterpret_cast<PFN_vkCreateImageView>(next_gdpa(*d, "vkCreateImageView"));
        std::lock_guard<std::mutex> l(mutex); devices[key(*d)] = ctx;
        std::fprintf(stderr, "QIYU_INTEROP device_created\n");
    }
    return r;
}
static VkResult VKAPI_CALL create_instance(const VkInstanceCreateInfo *ci,
        const VkAllocationCallbacks *a, VkInstance *i) {
    auto link = reinterpret_cast<const VkLayerInstanceCreateInfo *>(ci->pNext);
    for (; link; link = reinterpret_cast<const VkLayerInstanceCreateInfo *>(link->pNext))
        if (link->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO && link->function == VK_LAYER_LINK_INFO) break;
    if (!link) return VK_ERROR_INITIALIZATION_FAILED;
    auto next_gpa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    auto next_pgpa = link->u.pLayerInfo->pfnNextGetPhysicalDeviceProcAddr;
    auto next_create = reinterpret_cast<PFN_vkCreateInstance>(next_gpa(VK_NULL_HANDLE, "vkCreateInstance"));
    const_cast<VkLayerInstanceCreateInfo *>(link)->u.pLayerInfo = link->u.pLayerInfo->pNext;
    VkResult r = next_create(ci, a, i);
    if (r == VK_SUCCESS) {
        auto c = std::make_shared<Instance>(); c->instance = *i; c->gpa = next_gpa; c->pgpa = next_pgpa;
        std::lock_guard<std::mutex> l(mutex); instances[key(*i)] = c;
    }
    return r;
}
static void VKAPI_CALL destroy_instance(VkInstance i, const VkAllocationCallbacks *a) {
    auto c = instance(key(i)); if (!c) return;
    auto destroy = reinterpret_cast<PFN_vkDestroyInstance>(c->gpa(i, "vkDestroyInstance"));
    { std::lock_guard<std::mutex> l(mutex); instances.erase(key(i)); }
    destroy(i, a);
}
static PFN_vkVoidFunction VKAPI_CALL get_device(VkDevice d, const char *n) {
#define ENTRY(s, f) if (!std::strcmp(n, s)) return reinterpret_cast<PFN_vkVoidFunction>(f);
    ENTRY("vkGetDeviceProcAddr", get_device) ENTRY("vkDestroyDevice", destroy_device)
    ENTRY("vkCreateImage", create_image) ENTRY("vkCreateImageView", create_view)
#undef ENTRY
    auto c = device(d); return c ? c->gpa(d, n) : nullptr;
}
static PFN_vkVoidFunction VKAPI_CALL get_instance(VkInstance i, const char *n) {
#define ENTRY(s, f) if (!std::strcmp(n, s)) return reinterpret_cast<PFN_vkVoidFunction>(f);
    ENTRY("vkGetInstanceProcAddr", get_instance) ENTRY("vkGetDeviceProcAddr", get_device)
    ENTRY("vkCreateInstance", create_instance) ENTRY("vkDestroyInstance", destroy_instance)
    ENTRY("vkCreateDevice", create_device) ENTRY("vkDestroyDevice", destroy_device)
    ENTRY("vkCreateImage", create_image) ENTRY("vkCreateImageView", create_view)
#undef ENTRY
    auto c = i ? instance(key(i)) : nullptr; return c ? c->gpa(i, n) : nullptr;
}
static PFN_vkVoidFunction VKAPI_CALL get_physical(VkInstance i, const char *n) {
    auto c = instance(key(i)); return c ? c->pgpa ? c->pgpa(i, n) : c->gpa(i, n) : nullptr;
}
extern "C" __attribute__((visibility("default"))) VkResult VKAPI_CALL
vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *v) {
    if (!v || v->loaderLayerInterfaceVersion < 2) return VK_ERROR_INITIALIZATION_FAILED;
    v->loaderLayerInterfaceVersion = 2; v->pfnGetInstanceProcAddr = get_instance;
    v->pfnGetDeviceProcAddr = get_device; v->pfnGetPhysicalDeviceProcAddr = get_physical;
    return VK_SUCCESS;
}
