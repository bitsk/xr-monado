// SPDX-License-Identifier: MIT
// Explicit app layer: drain its Vulkan device before XR-owned storage teardown.
#define XR_USE_GRAPHICS_API_VULKAN
#define XR_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <atomic>
#include <unistd.h>
#include <memory>
#include <mutex>
#include <unordered_map>

struct Instance {
    PFN_xrGetInstanceProcAddr gpa;
    PFN_xrCreateSession create_session;
    PFN_xrDestroySession destroy_session;
    PFN_xrEndSession end_session;
    PFN_xrBeginSession begin_session;
    PFN_xrCreateSwapchain create_swapchain;
    PFN_xrDestroySwapchain destroy_swapchain;
    PFN_xrDestroyInstance destroy_instance;
    PFN_xrCreateVulkanInstanceKHR create_vulkan_instance;
    PFN_vkGetInstanceProcAddr vulkan_gpa = nullptr;
};
struct Session {
    std::shared_ptr<Instance> instance;
    VkDevice device = VK_NULL_HANDLE;
    PFN_vkDeviceWaitIdle wait_idle = nullptr;
    std::atomic<bool> drained_at_end{false};
};
static std::mutex mutex;
static std::unordered_map<XrInstance, std::shared_ptr<Instance>> instances;
static std::unordered_map<XrSession, std::shared_ptr<Session>> sessions;
static std::unordered_map<XrSwapchain, std::shared_ptr<Session>> swapchains;
static PFN_xrGetInstanceProcAddr fallback;
template <typename H, typename V> static std::shared_ptr<V> find(
        const std::unordered_map<H, std::shared_ptr<V>> &map, H key) {
    std::lock_guard<std::mutex> l(mutex); auto p = map.find(key);
    return p == map.end() ? nullptr : p->second;
}
static void wait_before(const std::shared_ptr<Session> &s, const char *operation) {
    const char *flag = std::getenv("QIYU_XR_WAIT_IDLE_BEFORE_DESTROY");
    bool enabled = flag && !std::strcmp(flag, "1");
    auto begin = std::chrono::steady_clock::now(); VkResult r = VK_SUCCESS;
    bool wait = enabled && !s->drained_at_end && s->wait_idle;
    if (wait) r = s->wait_idle(s->device);
    if (wait && r == VK_SUCCESS && !std::strcmp(operation, "end_session")) s->drained_at_end = true;
    std::fprintf(stderr, "QIYU_XR_LIFETIME %s wait=%d available=%d result=%d tid=%ld elapsed_us=%lld\n",
        operation, wait, s->wait_idle != nullptr, r, (long)gettid(),
        (long long)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-begin).count());
}
static XrResult XRAPI_CALL create_vk_instance(XrInstance i, const XrVulkanInstanceCreateInfoKHR *ci,
        VkInstance *vk, VkResult *result) {
    auto c = find(instances, i); if (!c) return XR_ERROR_HANDLE_INVALID;
    c->vulkan_gpa = ci->pfnGetInstanceProcAddr;
    return c->create_vulkan_instance(i, ci, vk, result);
}
static XrResult XRAPI_CALL create_session(XrInstance i, const XrSessionCreateInfo *ci, XrSession *out) {
    auto c = find(instances, i); if (!c) return XR_ERROR_HANDLE_INVALID;
    XrResult r = c->create_session(i, ci, out);
    if (XR_SUCCEEDED(r)) {
        auto s = std::make_shared<Session>(); s->instance = c;
        for (auto p = static_cast<const XrBaseInStructure *>(ci->next); p; p = p->next) {
            if (p->type == XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR && c->vulkan_gpa) {
                auto binding = reinterpret_cast<const XrGraphicsBindingVulkanKHR *>(p);
                auto gdpa = reinterpret_cast<PFN_vkGetDeviceProcAddr>(c->vulkan_gpa(binding->instance, "vkGetDeviceProcAddr"));
                s->device = binding->device;
                if (gdpa) s->wait_idle = reinterpret_cast<PFN_vkDeviceWaitIdle>(gdpa(s->device, "vkDeviceWaitIdle"));
            }
        }
        std::lock_guard<std::mutex> l(mutex); sessions[*out] = s;
        std::fprintf(stderr, "QIYU_XR_LIFETIME session_created vulkan_wait_available=%d\n", s->wait_idle != nullptr);
    }
    return r;
}
static XrResult XRAPI_CALL end_session(XrSession s) {
    auto c = find(sessions, s); if (!c) return XR_ERROR_HANDLE_INVALID;
    wait_before(c, "end_session");
    XrResult r = c->instance->end_session(s);
    if (XR_FAILED(r)) c->drained_at_end = false;
    std::fprintf(stderr, "QIYU_XR_LIFETIME end_session result=%d\n", r);
    return r;
}
static XrResult XRAPI_CALL begin_session(XrSession s, const XrSessionBeginInfo *ci) {
    auto c = find(sessions, s); if (!c) return XR_ERROR_HANDLE_INVALID;
    c->drained_at_end = false;
    return c->instance->begin_session(s, ci);
}
static XrResult XRAPI_CALL create_swapchain(XrSession s, const XrSwapchainCreateInfo *ci, XrSwapchain *out) {
    auto c = find(sessions, s); if (!c) return XR_ERROR_HANDLE_INVALID;
    XrResult r = c->instance->create_swapchain(s, ci, out);
    if (XR_SUCCEEDED(r)) { std::lock_guard<std::mutex> l(mutex); swapchains[*out] = c; }
    return r;
}
static XrResult XRAPI_CALL destroy_swapchain(XrSwapchain sc) {
    auto c = find(swapchains, sc); if (!c) return XR_ERROR_HANDLE_INVALID;
    wait_before(c, "destroy_swapchain");
    XrResult r = c->instance->destroy_swapchain(sc);
    std::fprintf(stderr, "QIYU_XR_LIFETIME destroy_swapchain result=%d\n", r);
    if (XR_SUCCEEDED(r)) { std::lock_guard<std::mutex> l(mutex); swapchains.erase(sc); }
    return r;
}
static XrResult XRAPI_CALL destroy_session(XrSession s) {
    auto c = find(sessions, s); if (!c) return XR_ERROR_HANDLE_INVALID;
    wait_before(c, "destroy_session");
    XrResult r = c->instance->destroy_session(s);
    std::fprintf(stderr, "QIYU_XR_LIFETIME destroy_session result=%d\n", r);
    if (XR_SUCCEEDED(r)) { std::lock_guard<std::mutex> l(mutex); sessions.erase(s); }
    return r;
}
static XrResult XRAPI_CALL destroy_instance(XrInstance i) {
    auto c = find(instances, i); if (!c) return XR_ERROR_HANDLE_INVALID;
    XrResult r = c->destroy_instance(i);
    if (XR_SUCCEEDED(r)) { std::lock_guard<std::mutex> l(mutex); instances.erase(i); }
    return r;
}
static XrResult XRAPI_CALL get_proc(XrInstance i, const char *name, PFN_xrVoidFunction *out) {
#define ENTRY(n, f) if (!std::strcmp(name, n)) { *out = reinterpret_cast<PFN_xrVoidFunction>(f); return XR_SUCCESS; }
    ENTRY("xrGetInstanceProcAddr", get_proc) ENTRY("xrCreateSession", create_session)
    ENTRY("xrEndSession", end_session) ENTRY("xrBeginSession", begin_session) ENTRY("xrDestroySession", destroy_session)
    ENTRY("xrCreateSwapchain", create_swapchain) ENTRY("xrDestroySwapchain", destroy_swapchain)
    ENTRY("xrDestroyInstance", destroy_instance) ENTRY("xrCreateVulkanInstanceKHR", create_vk_instance)
#undef ENTRY
    auto c = find(instances, i); return c ? c->gpa(i, name, out) : fallback ? fallback(i, name, out) : XR_ERROR_HANDLE_INVALID;
}
static XrResult XRAPI_CALL create_instance(const XrInstanceCreateInfo *ci,
        const XrApiLayerCreateInfo *li, XrInstance *out) {
    auto gpa = li->nextInfo->nextGetInstanceProcAddr; fallback = gpa;
    XrApiLayerCreateInfo next = *li; next.nextInfo = li->nextInfo->next;
    XrResult r = li->nextInfo->nextCreateApiLayerInstance(ci, &next, out);
    if (XR_SUCCEEDED(r)) {
        auto c = std::make_shared<Instance>(); c->gpa = gpa;
#define LOAD(n, field) gpa(*out, n, reinterpret_cast<PFN_xrVoidFunction *>(&c->field))
        LOAD("xrCreateSession", create_session); LOAD("xrDestroySession", destroy_session);
        LOAD("xrEndSession", end_session); LOAD("xrBeginSession", begin_session); LOAD("xrCreateSwapchain", create_swapchain);
        LOAD("xrDestroySwapchain", destroy_swapchain); LOAD("xrDestroyInstance", destroy_instance);
        LOAD("xrCreateVulkanInstanceKHR", create_vulkan_instance);
#undef LOAD
        std::lock_guard<std::mutex> l(mutex); instances[*out] = c;
    }
    return r;
}
extern "C" __attribute__((visibility("default"))) XrResult XRAPI_CALL
xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo *l, const char *, XrNegotiateApiLayerRequest *r) {
    if (!l || !r || l->minInterfaceVersion > 1 || l->maxInterfaceVersion < 1) return XR_ERROR_INITIALIZATION_FAILED;
    r->layerInterfaceVersion = 1; r->layerApiVersion = XR_MAKE_VERSION(1, 0, XR_VERSION_PATCH(XR_CURRENT_API_VERSION));
    r->getInstanceProcAddr = get_proc; r->createApiLayerInstance = create_instance; return XR_SUCCESS;
}
