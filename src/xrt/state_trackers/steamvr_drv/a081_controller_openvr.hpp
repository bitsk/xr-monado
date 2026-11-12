// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <xr/controller/production-v2/a081_controller_pose_wire.h>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
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
public:
    struct Inputs {
        bool face0_click, face0_touch, face1_click, face1_touch, system_click, joystick_click, joystick_touch;
        float joystick_x, joystick_y, trigger_value;
        bool trigger_touch;
        float grip_value;
        bool grip_touch;
    };
private:
    // Vendor bits in a081_controller_pose_packet.controls, from right-hand
    // presses; face0 is A/X, face1 B/Y. Full travel is index 9, middle 10.
    static constexpr uint32_t button_face0 = 0x10, button_face1 = 0x20, button_system = 0x40, button_joystick = 0x80;
    static constexpr uint32_t touch_joystick = 1, touch_trigger = 2, touch_face1 = 4, touch_face0 = 8;
    static constexpr float trigger_full = 9, grip_full = 10;
    class Device final : public vr::ITrackedDeviceServerDriver {
    public:
        Device(A081ControllerPosePair &owner, unsigned hand) : owner(owner), hand(hand) {}
        vr::EVRInitError Activate(vr::TrackedDeviceIndex_t object) override
        {
            index.store(object);
            auto p = vr::VRProperties()->TrackedDeviceToPropertyContainer(object);
            vr::VRProperties()->SetStringProperty(p, vr::Prop_TrackingSystemName_String, "monado");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ModelNumber_String,
                hand ? "Qiyu MIX Right Controller" : "Qiyu MIX Left Controller");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_SerialNumber_String, serial());
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ManufacturerName_String, "iQIYI");
            vr::VRProperties()->SetInt32Property(p, vr::Prop_ControllerRoleHint_Int32,
                hand ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
            // Same control layout as Touch: reuse SteamVR's bundled profile,
            // compositor/legacy bindings and render model.
            vr::VRProperties()->SetStringProperty(p, vr::Prop_ControllerType_String, "oculus_touch");
            vr::VRProperties()->SetStringProperty(p, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
            // Button layout is independent of the device model's grip/aim
            // components. A board profile can select its own render model.
            const char *model = std::getenv(hand ? "A081_CONTROLLER_RENDER_MODEL_RIGHT" :
                                                  "A081_CONTROLLER_RENDER_MODEL_LEFT");
            if (!model || !*model)
                model = hand ? "oculus_quest2_controller_right" : "oculus_quest2_controller_left";
            vr::VRProperties()->SetStringProperty(p, vr::Prop_RenderModelName_String, model);
            auto in = vr::VRDriverInput();
            const char *face0 = hand ? "/input/a" : "/input/x", *face1 = hand ? "/input/b" : "/input/y";
            std::string f0 = face0, f1 = face1;
            in->CreateBooleanComponent(p, (f0 + "/click").c_str(), &h.face0_click);
            in->CreateBooleanComponent(p, (f0 + "/touch").c_str(), &h.face0_touch);
            in->CreateBooleanComponent(p, (f1 + "/click").c_str(), &h.face1_click);
            in->CreateBooleanComponent(p, (f1 + "/touch").c_str(), &h.face1_touch);
            in->CreateBooleanComponent(p, "/input/system/click", &h.system_click);
            in->CreateBooleanComponent(p, "/input/joystick/click", &h.joystick_click);
            in->CreateBooleanComponent(p, "/input/joystick/touch", &h.joystick_touch);
            in->CreateScalarComponent(p, "/input/joystick/x", &h.joystick_x, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
            in->CreateScalarComponent(p, "/input/joystick/y", &h.joystick_y, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
            in->CreateScalarComponent(p, "/input/trigger/value", &h.trigger_value, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
            in->CreateBooleanComponent(p, "/input/trigger/touch", &h.trigger_touch);
            in->CreateScalarComponent(p, "/input/grip/value", &h.grip_value, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
            in->CreateBooleanComponent(p, "/input/grip/touch", &h.grip_touch);
            in->CreateHapticComponent(p, "/output/haptic", &h.haptic);
            last = {};
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
            if (id == vr::k_unTrackedDeviceIndexInvalid) return;
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id, GetPose(), sizeof(vr::DriverPose_t));
            update_inputs(owner.inputs(hand));
        }
    private:
        struct Handles {
            vr::VRInputComponentHandle_t face0_click, face0_touch, face1_click, face1_touch, system_click,
                joystick_click, joystick_touch, joystick_x, joystick_y, trigger_value, trigger_touch,
                grip_value, grip_touch, haptic;
        };
        void boolean(vr::VRInputComponentHandle_t c, bool v, bool &previous)
        {
            if (v != previous) vr::VRDriverInput()->UpdateBooleanComponent(c, v, 0);
            previous = v;
        }
        void scalar(vr::VRInputComponentHandle_t c, float v, float &previous)
        {
            if (v != previous) vr::VRDriverInput()->UpdateScalarComponent(c, v, 0);
            previous = v;
        }
        void update_inputs(const Inputs &v)
        {
            boolean(h.face0_click, v.face0_click, last.face0_click);
            boolean(h.face0_touch, v.face0_touch, last.face0_touch);
            boolean(h.face1_click, v.face1_click, last.face1_click);
            boolean(h.face1_touch, v.face1_touch, last.face1_touch);
            boolean(h.system_click, v.system_click, last.system_click);
            boolean(h.joystick_click, v.joystick_click, last.joystick_click);
            boolean(h.joystick_touch, v.joystick_touch, last.joystick_touch);
            scalar(h.joystick_x, v.joystick_x, last.joystick_x);
            scalar(h.joystick_y, v.joystick_y, last.joystick_y);
            scalar(h.trigger_value, v.trigger_value, last.trigger_value);
            boolean(h.trigger_touch, v.trigger_touch, last.trigger_touch);
            scalar(h.grip_value, v.grip_value, last.grip_value);
            boolean(h.grip_touch, v.grip_touch, last.grip_touch);
        }
        A081ControllerPosePair &owner;
        unsigned hand;
        std::atomic<vr::TrackedDeviceIndex_t> index{vr::k_unTrackedDeviceIndexInvalid};
        Handles h{};
        Inputs last{};
    };
public:
    A081ControllerPosePair() : left(*this, 0), right(*this, 1) {}
    ~A081ControllerPosePair() { close_source(); }
    bool configure(bool a081_head, bool register_devices = true)
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
        if (!register_devices) return true;
        bool added_left = vr::VRServerDriverHost()->TrackedDeviceAdded(left.serial(), vr::TrackedDeviceClass_Controller, &left);
        bool added_right = vr::VRServerDriverHost()->TrackedDeviceAdded(right.serial(), vr::TrackedDeviceClass_Controller, &right);
        if (!added_left || !added_right) {
            ovrd_log("A081 controller registration failed left=%d right=%d\n", added_left, added_right);
            cleanup(); return false;
        }
        ovrd_log("A081 controllers registered as oculus_touch: Nordic controls, SDK pose; haptics not implemented\n");
        return true;
    }
    void frame(bool publish_devices = true)
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
        if (publish_devices) { left.publish(); right.publish(); }
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
public:
    Inputs inputs(unsigned hand)
    {
        Inputs v{};
        Sample sample;
        { std::lock_guard<std::mutex> lock(mutex); sample = samples[hand]; }
        const auto &p = sample.packet;
        uint64_t now = os_monotonic_get_ns();
        if (!sample.received_ns || now < sample.received_ns ||
            now - sample.received_ns >= A081_CONTROLLER_POSE_MAX_AGE_NS || !(p.flags & A081_CONTROLLER_CONTROLS_VALID))
            return v;
        const uint32_t mask = A081_CONTROLLER_CONTROLS_MASK(p.controls);
        if ((mask & A081_CONTROLLER_CONTROLS_MODE1) == A081_CONTROLLER_CONTROLS_MODE1) {
            const uint32_t buttons = A081_CONTROLLER_CONTROLS_BUTTONS(p.controls);
            const uint32_t touch = A081_CONTROLLER_CONTROLS_TOUCH(p.controls);
            v.face0_click = buttons & button_face0;
            v.face1_click = buttons & button_face1;
            v.joystick_click = buttons & button_joystick;
            v.system_click = buttons & button_system;
            v.face0_touch = v.face0_click || (touch & touch_face0);
            v.face1_touch = v.face1_click || (touch & touch_face1);
            v.joystick_touch = v.joystick_click || (touch & touch_joystick);
            v.trigger_value = std::fmin(1.f, A081_CONTROLLER_CONTROLS_INDEX(p.controls) / trigger_full);
            v.trigger_touch = v.trigger_value > 0 || (touch & touch_trigger);
            v.grip_value = std::fmin(1.f, A081_CONTROLLER_CONTROLS_MIDDLE(p.controls) / grip_full);
            v.grip_touch = v.grip_value > 0;
        }
        if (mask & A081_CONTROLLER_CONTROLS_JOYSTICK) {
            const auto axis = [](uint32_t raw) {
                const float value = int8_t(uint8_t(raw)) / 127.f;
                return std::fabs(value) < .04f ? 0.f : std::fmax(-1.f, std::fmin(1.f, value));
            };
            v.joystick_x = axis(p.joystick >> 8);
            v.joystick_y = axis(p.joystick);
            v.joystick_touch = v.joystick_touch || v.joystick_x != 0 || v.joystick_y != 0;
        }
        return v;
    }
private:
    bool consume(const a081_controller_pose_packet &p, uint64_t now)
    {
        const bool v2 = p.version == 2;
        if (p.magic != A081_CONTROLLER_POSE_MAGIC || (p.version != 1 && !v2) ||
            p.bytes != sizeof(p) || p.hand > 1 || !p.session_id || !p.sequence ||
            (p.flags & ~(v2 ? A081_CONTROLLER_POSE_KNOWN_FLAGS : 63u)) || (!v2 && (p.controls || p.joystick)) ||
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
public:
    vr::DriverPose_t pose(unsigned hand)
    {
        vr::DriverPose_t out{};
        out.qWorldFromDriverRotation.w = 1;
        // SDK controller forward is +Z; OpenVR controllers point along -Z.
        out.qDriverFromHeadRotation.y = 1;
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
private:
    Device left, right;
    std::mutex mutex;
    Sample samples[2]{};
    uint64_t session = 0, retry_ns = 0;
    int fd = -1;
    bool enabled = false;
    char path[sizeof(sockaddr_un::sun_path)]{};
};
