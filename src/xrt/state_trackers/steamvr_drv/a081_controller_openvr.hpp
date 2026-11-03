// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <xr/controller/a081_controller_pose_wire.h>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

/* Private opt-in extension to the existing A081 Monado SteamVR provider.
 * RunFrame consumes a bounded number of actual SDK measurements; GetPose
 * never predicts, fabricates controller coordinates, or reuses stale poses.
 */
class A081ControllerPosePair {
    struct Sample {
        a081_controller_pose_packet packet{};
        uint64_t received_ns = 0;
    };
    class Device final : public vr::ITrackedDeviceServerDriver {
    public:
        Device(A081ControllerPosePair &owner, unsigned hand) : owner(owner), hand(hand) {}
        vr::EVRInitError Activate(vr::TrackedDeviceIndex_t object) override
        {
            index.store(object);
            auto p = vr::VRProperties()->TrackedDeviceToPropertyContainer(object);
            vr::VRProperties()->SetStringProperty(p, vr::Prop_TrackingSystemName_String, "monado");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ModelNumber_String,
                hand ? "XR MIX right controller (SDK pose candidate)" : "XR MIX left controller (SDK pose candidate)");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_SerialNumber_String, serial());
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ManufacturerName_String, "iQIYI");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ControllerType_String, "a081_controller_pose");
            vr::VRProperties()->SetInt32Property(p, vr::Prop_ControllerRoleHint_Int32,
                hand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
            // Diagnostic locator, not a fabricated vendor model or button layout.
            vr::VRProperties()->SetStringProperty(p, vr::Prop_RenderModelName_String, "locator_one_sided");
            return vr::VRInitError_None;
        }
        void Deactivate() override { index.store(vr::k_unTrackedDeviceIndexInvalid); }
        void EnterStandby() override {}
        void *GetComponent(const char *) override { return nullptr; }
        void DebugRequest(const char *, char *response, uint32_t size) override
        {
            if (!size) return;
            auto p = GetPose();
            std::snprintf(response, size, "A081 SDK pose hand=%u connected=%d valid=%d; no optical/coordinate acceptance implied",
                          hand, p.deviceIsConnected, p.poseIsValid);
        }
        vr::DriverPose_t GetPose() override { return owner.pose(hand); }
        const char *serial() const { return hand ? "A081-STOCK-CONTROLLER-R" : "A081-STOCK-CONTROLLER-L"; }
        void publish()
        {
            auto id = index.load();
            if (id != vr::k_unTrackedDeviceIndexInvalid)
                vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, GetPose(), sizeof(vr::DriverPose_t));
        }
    private:
        A081ControllerPosePair &owner;
        unsigned hand;
        std::atomic<vr::TrackedDeviceIndex_t> index{vr::k_unTrackedDeviceIndexInvalid};
    };
public:
    A081ControllerPosePair() : left(*this, 0), right(*this, 1) {}
    ~A081ControllerPosePair() { close_source(); }
    bool configure(bool a081_head)
    {
        const char *requested = std::getenv("A081_CONTROLLER_POSE_SOCKET");
        if (!requested || !*requested) return true; // baseline HMD-only path
        const char *profile = std::getenv("A081_CONTROLLER_POSE_PROFILE");
        if (!a081_head || !profile || std::strcmp(profile, "openvr-head-raw-v1") ||
            std::strncmp(requested, "/run/a081-controller-pose-", 26) ||
            std::strlen(requested) >= sizeof(path) || std::strstr(requested, "/../")) {
            ovrd_log("A081 controller pose refused: exact A081 HMD, private /run socket and explicit profile required\n");
            return false;
        }
        std::strcpy(path, requested);
        enabled = true;
        bool added_left = vr::VRServerDriverHost()->TrackedDeviceAdded(left.serial(), vr::TrackedDeviceClass_Controller, &left);
        bool added_right = vr::VRServerDriverHost()->TrackedDeviceAdded(right.serial(), vr::TrackedDeviceClass_Controller, &right);
        if (!added_left || !added_right) {
            ovrd_log("A081 controller registration failed left=%d right=%d\n", added_left, added_right);
            cleanup(); return false;
        }
        ovrd_log("A081 controller pose candidate registered: no buttons/haptics; SDK pose and coordinate gates required, optical flag reports recent vision\n");
        return true;
    }
    void frame()
    {
        if (!enabled) return;
        uint64_t now = os_monotonic_get_ns();
        if (fd < 0 && now >= retry_ns) {
            retry_ns = now + UINT64_C(1000000000);
            connect_source();
        }
        if (fd >= 0) {
            for (unsigned i = 0; i < 128; ++i) {
                a081_controller_pose_packet p{};
                ssize_t n = recv(fd, &p, sizeof(p), MSG_DONTWAIT | MSG_TRUNC);
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) break;
                if (n != sizeof(p) || !consume(p, os_monotonic_get_ns())) {
                    close_source(); break;
                }
            }
        }
        left.publish(); right.publish();
    }
    void cleanup()
    {
        enabled = false;
        close_source();
        left.Deactivate(); right.Deactivate();
        path[0] = '\0'; retry_ns = 0;
    }
private:
    void close_source()
    {
        if (fd >= 0) { close(fd); fd = -1; }
        std::lock_guard<std::mutex> lock(mutex);
        samples[0] = {}; samples[1] = {}; session = 0;
    }
    void connect_source()
    {
        // Require the root-owned relay used by the SDK producer. No sensor or
        // Android device nodes are opened from this unprivileged driver.
        struct stat info{};
        if (lstat(path, &info) || !S_ISSOCK(info.st_mode) || info.st_uid != 0 || (info.st_mode & 0002)) return;
        sockaddr_un addr{}; addr.sun_family = AF_UNIX;
        std::strcpy(addr.sun_path, path);
        int candidate = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (candidate < 0) return;
        if (connect(candidate, reinterpret_cast<sockaddr *>(&addr), sizeof(addr))) { close(candidate); return; }
        ucred peer{}; socklen_t length = sizeof(peer);
        if (getsockopt(candidate, SOL_SOCKET, SO_PEERCRED, &peer, &length) || length != sizeof(peer) || peer.uid != 0) {
            close(candidate); return;
        }
        fd = candidate;
        ovrd_log("A081 controller pose connected to root relay\n");
    }
    bool consume(const a081_controller_pose_packet &p, uint64_t now)
    {
        if (p.magic != A081_CONTROLLER_POSE_MAGIC || p.version != A081_CONTROLLER_POSE_VERSION ||
            p.bytes != sizeof(p) || p.hand > 1 || !p.session_id || !p.sequence ||
            (p.flags & ~A081_CONTROLLER_POSE_KNOWN_FLAGS) || p.reserved0 || p.reserved1 ||
            p.coordinate_profile > A081_CONTROLLER_PROFILE_OPENVR_HEAD_RAW ||
            !p.send_monotonic_ns || p.send_monotonic_ns > now ||
            now - p.send_monotonic_ns >= A081_CONTROLLER_POSE_MAX_AGE_NS || !std::isfinite(p.confidence)) return false;
        for (float v : p.position) if (!std::isfinite(v)) return false;
        for (float v : p.orientation_xyzw) if (!std::isfinite(v)) return false;
        if ((p.flags & A081_CONTROLLER_CLOCK_VALID) && (!p.sample_monotonic_ns ||
            p.sample_monotonic_ns > p.send_monotonic_ns)) return false;
        std::lock_guard<std::mutex> lock(mutex);
        Sample &s = samples[p.hand];
        if ((session && session != p.session_id) || p.sequence <= s.packet.sequence ||
            ((p.flags & A081_CONTROLLER_CLOCK_VALID) && (s.packet.flags & A081_CONTROLLER_CLOCK_VALID) &&
             p.sample_monotonic_ns < s.packet.sample_monotonic_ns)) return false;
        session = p.session_id;
        s.packet = p; s.received_ns = now;
        return true;
    }
    vr::DriverPose_t pose(unsigned hand)
    {
        vr::DriverPose_t out{};
        out.qWorldFromDriverRotation.w = 1;
        out.qDriverFromHeadRotation.w = 1;
        out.qRotation.w = 1;
        out.result = vr::TrackingResult_Uninitialized;
        Sample sample;
        { std::lock_guard<std::mutex> lock(mutex); sample = samples[hand]; }
        const auto &p = sample.packet;
        uint64_t now = os_monotonic_get_ns();
        if (!sample.received_ns || now < sample.received_ns ||
            now - sample.received_ns >= A081_CONTROLLER_POSE_MAX_AGE_NS || now < p.send_monotonic_ns ||
            now - p.send_monotonic_ns >= A081_CONTROLLER_POSE_MAX_AGE_NS) return out;
        out.deviceIsConnected = p.flags & A081_CONTROLLER_CONNECTED;
        out.result = vr::TrackingResult_Running_OutOfRange;
        float norm = 0;
        for (float v : p.orientation_xyzw) norm += v * v;
        bool valid = (p.flags & A081_CONTROLLER_POSE_REQUIRED_FLAGS) == A081_CONTROLLER_POSE_REQUIRED_FLAGS &&
            p.coordinate_profile == A081_CONTROLLER_PROFILE_OPENVR_HEAD_RAW && p.confidence >= .1f &&
            p.sample_monotonic_ns && p.sample_monotonic_ns <= now &&
            now - p.sample_monotonic_ns < A081_CONTROLLER_POSE_MAX_AGE_NS && std::fabs(norm - 1.f) < .01f;
        if (!valid) return out;
        out.poseIsValid = true;
        out.result = vr::TrackingResult_Running_OK;
        out.poseTimeOffset = -double(now - p.sample_monotonic_ns) / 1e9;
        for (unsigned i = 0; i < 3; ++i) out.vecPosition[i] = p.position[i];
        out.qRotation = {p.orientation_xyzw[3], p.orientation_xyzw[0], p.orientation_xyzw[1], p.orientation_xyzw[2]};
        return out;
    }
    Device left, right;
    std::mutex mutex;
    Sample samples[2]{};
    uint64_t session = 0, retry_ns = 0;
    int fd = -1;
    bool enabled = false;
    char path[sizeof(sockaddr_un::sun_path)]{};
};
