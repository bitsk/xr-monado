// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Used only by the existing, session-supervised pose consumer. No extra
// process, synthetic pose, altered tracking result or global registry change.
class A081TrackingDisplayGuard {
    vr::IVRCompositor *compositor;
    int previous = -1;
public:
    explicit A081TrackingDisplayGuard(vr::IVRCompositor *value) : compositor(value)
    {
        if (compositor) compositor->FadeToColor(0, 0, 0, 0, 1);
    }
    void update(const vr::TrackedDevicePose_t &pose, bool finite, int64_t now)
    {
        if (!compositor) return;
        bool usable = finite && pose.bDeviceIsConnected && pose.bPoseIsValid &&
                      (pose.eTrackingResult == vr::TrackingResult_Running_OK ||
                       pose.eTrackingResult == vr::TrackingResult_Fallback_RotationOnly);
        int state = !usable ? 0 : pose.eTrackingResult == vr::TrackingResult_Fallback_RotationOnly ? 1 : 2;
        if (state == previous) return;
        // Loss is immediate black; an available measured orientation can
        // reveal the scene without claiming positional tracking has recovered.
        compositor->FadeToColor(usable ? .05f : 0.f, 0, 0, 0, usable ? 0.f : 1.f);
        std::printf("A081_DISPLAY_GUARD monotonic_ns=%lld state=%s pose_valid=%d tracking=%d\n",
                    static_cast<long long>(now), state == 0 ? "protected_black" : state == 1 ? "rotation_only" : "tracked",
                    pose.bPoseIsValid, pose.eTrackingResult);
        previous = state;
    }
    void close()
    {
        if (compositor) compositor->FadeToColor(0, 0, 0, 0, 1);
    }
};
