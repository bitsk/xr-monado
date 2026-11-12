// SPDX-License-Identifier: MIT
#pragma once
#include <xr/hand/client.hpp>
#include <xr/hand/joints.hpp>
#include <atomic>
#include <cstdlib>

// Optional hand-input profile. The physical-controller profile is preserved;
// one profile registers one left/right input pair to avoid duplicate hand roles.
class XRHandPosePair {
    static void rotation(const float *p,float r[3][3]) {
        const float x=p[3],y=p[4],z=p[5],w=p[6];
        r[0][0]=1-2*(y*y+z*z);r[0][1]=2*(x*y-z*w);r[0][2]=2*(x*z+y*w);
        r[1][0]=2*(x*y+z*w);r[1][1]=1-2*(x*x+z*z);r[1][2]=2*(y*z-x*w);
        r[2][0]=2*(x*z-y*w);r[2][1]=2*(y*z+x*w);r[2][2]=1-2*(x*x+y*y);
    }
    static vr::HmdMatrix34_t relative(const float *grip,const float *aim) {
        float g[3][3],a[3][3];rotation(grip,g);rotation(aim,a);vr::HmdMatrix34_t out{};
        for (unsigned i=0;i<3;++i) {
            for (unsigned j=0;j<3;++j) for (unsigned k=0;k<3;++k) out.m[i][j]+=g[k][i]*a[k][j];
            for (unsigned k=0;k<3;++k) out.m[i][3]+=g[k][i]*(aim[k]-grip[k]);
        }
        return out;
    }
public:
    static bool skeleton(const xr_hand_state &hand,vr::VRBoneTransform_t (&bones)[31]) {
        xr::hand::Joints js{};if(!xr::hand::joints(hand,js))return false;
        using Q=std::array<float,4>;using V=xr::hand::Vec;
        struct Pose {V p{};Q q{0,0,0,1};};
        Pose global[31]{};Q grip{hand.grip[3],hand.grip[4],hand.grip[5],hand.grip[6]},inverse{-grip[0],-grip[1],-grip[2],grip[3]};
        const Q axis{0,hand.hand?-.7071067812f:.7071067812f,0,.7071067812f};
        // Bone 0 is fixed to /pose/raw (measured palm grip); bone 1 is wrist.
        // Finger bones use Valve's +X left / -X right longitudinal axes.
        for(unsigned j=1;j<26;++j){auto &v=js[j].pose;global[j].p=xr::hand::rotate(inverse,{v[0]-hand.grip[0],v[1]-hand.grip[1],v[2]-hand.grip[2]});Q q{v[3],v[4],v[5],v[6]};global[j].q=xr::hand::multiply(inverse,j==1?q:xr::hand::multiply(q,axis));}
        const unsigned aux[5]={4,9,14,19,24};for(unsigned a=0;a<5;++a)global[26+a]=global[aux[a]];
        for(unsigned j=0;j<31;++j){unsigned parent=j<=1||j>=26?0:j==2||j==6||j==11||j==16||j==21?1:j-1;
            auto &p=global[parent];Q inv{-p.q[0],-p.q[1],-p.q[2],p.q[3]};auto q=xr::hand::multiply(inv,global[j].q);auto t=xr::hand::rotate(inv,xr::hand::sub(global[j].p,p.p));
            bones[j].position={t[0],t[1],t[2],1};bones[j].orientation={q[3],q[0],q[1],q[2]};
        }return true;
    }
private:
    class Device final : public vr::ITrackedDeviceServerDriver {
    public:
        Device(XRHandPosePair &pair,unsigned hand):pair_(pair),hand_(hand){}
        const char *serial() const {return hand_?"XR-OPTICAL-HAND-R":"XR-OPTICAL-HAND-L";}
        vr::EVRInitError Activate(vr::TrackedDeviceIndex_t index) override {
            index_=index;auto p=vr::VRProperties()->TrackedDeviceToPropertyContainer(index);auto props=vr::VRProperties();auto input=vr::VRDriverInput();
            props->SetStringProperty(p,vr::Prop_TrackingSystemName_String,"monado");
            props->SetStringProperty(p,vr::Prop_ManufacturerName_String,"XR");
            props->SetStringProperty(p,vr::Prop_ModelNumber_String,hand_?"Optical right hand":"Optical left hand");
            props->SetStringProperty(p,vr::Prop_SerialNumber_String,serial());
            props->SetInt32Property(p,vr::Prop_ControllerRoleHint_Int32,hand_?vr::TrackedControllerRole_RightHand:vr::TrackedControllerRole_LeftHand);
            // Gesture actions use the existing Touch action layout. Grip and aim
            // are independent measured poses, supplied below as dynamic components.
            props->SetStringProperty(p,vr::Prop_ControllerType_String,"oculus_touch");
            props->SetStringProperty(p,vr::Prop_InputProfilePath_String,"{oculus}/input/touch_profile.json");
            input->CreateScalarComponent(p,"/input/trigger/value",&trigger_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedOneSided);
            input->CreateBooleanComponent(p,"/input/trigger/touch",&trigger_touch_);
            input->CreateScalarComponent(p,"/input/grip/value",&grab_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedOneSided);
            input->CreateBooleanComponent(p,"/input/grip/touch",&grab_touch_);
            input->CreatePoseComponent(p,"/pose/openxr_grip",&grip_);
            input->CreatePoseComponent(p,"/pose/handgrip",&legacy_grip_);
            input->CreatePoseComponent(p,"/pose/openxr_aim",&aim_);
            input->CreatePoseComponent(p,"/pose/tip",&legacy_aim_);
            auto error=input->CreateSkeletonComponent(p,hand_?"/input/skeleton/right":"/input/skeleton/left",hand_?"/skeleton/hand/right":"/skeleton/hand/left","/pose/raw",vr::VRSkeletalTracking_Partial,nullptr,31,&skeleton_);
            ovrd_log("XR_HAND_SKELETON hand=%u create=%d bones=31 source_landmarks=21 inferred_metacarpals=5\n",hand_,int(error));
            if(error!=vr::VRInputError_None)return vr::VRInitError_Driver_Failed;
            return vr::VRInitError_None;
        }
        void Deactivate() override {index_=vr::k_unTrackedDeviceIndexInvalid;}
        void EnterStandby() override {}
        void *GetComponent(const char *) override {return nullptr;}
        void DebugRequest(const char *,char *response,uint32_t bytes) override {
            if (bytes) std::snprintf(response,bytes,"XR optical hand=%u sequence=%llu valid=%d",hand_,(unsigned long long)pair_.client_.packet().sequence,pair_.client_.get(hand_).has_value());
        }
        vr::DriverPose_t GetPose() override {
            vr::DriverPose_t pose{};pose.qWorldFromDriverRotation.w=1;pose.qDriverFromHeadRotation.w=1;pose.qRotation.w=1;
            pose.deviceIsConnected=true;pose.result=vr::TrackingResult_Running_OutOfRange;
            xr_hand_packet snapshot{};auto h=pair_.client_.get(hand_,&snapshot);if (!h) return pose;
            for (unsigned i=0;i<3;++i) pose.vecPosition[i]=h->grip[i];
            pose.qRotation={h->grip[6],h->grip[3],h->grip[4],h->grip[5]};
            pose.poseTimeOffset=-double(xr::hand::Client::now()-snapshot.exposure_boottime_ns)/1e9;
            pose.poseIsValid=true;pose.result=vr::TrackingResult_Running_OK;return pose;
        }
        void publish() {
            auto index=index_.load();if (index==vr::k_unTrackedDeviceIndexInvalid) return;
            auto h=pair_.client_.get(hand_);auto input=vr::VRDriverInput();
            input->UpdateScalarComponent(trigger_,h?h->pinch:0,0);input->UpdateBooleanComponent(trigger_touch_,h && (h->flags&XR_HAND_PINCH),0);
            input->UpdateScalarComponent(grab_,h?h->grab:0,0);input->UpdateBooleanComponent(grab_touch_,h && (h->flags&XR_HAND_GRAB),0);
            if (h) {
                if(last_skeleton_!=pair_.client_.packet().sequence){
                    vr::VRBoneTransform_t bones[31]{};
                    if(skeleton(*h,bones)){
                        auto a=input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithoutController,bones,31);
                        auto b=input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithController,bones,31);
                        if(a||b)ovrd_log("XR_HAND_SKELETON hand=%u update=%d,%d\n",hand_,int(a),int(b));
                        last_skeleton_=pair_.client_.packet().sequence;
                    }
                }
                vr::HmdMatrix34_t identity{};for (unsigned i=0;i<3;++i) identity.m[i][i]=1;
                auto aim=relative(h->grip,h->aim);
                input->UpdatePoseComponent(grip_,&identity,0);input->UpdatePoseComponent(aim_,&aim,0);
                input->UpdatePoseComponent(legacy_grip_,&identity,0);input->UpdatePoseComponent(legacy_aim_,&aim,0);
            }
            auto pose=GetPose();vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index,pose,sizeof(pose));
        }
    private:
        XRHandPosePair &pair_;unsigned hand_;
        std::atomic<vr::TrackedDeviceIndex_t> index_{vr::k_unTrackedDeviceIndexInvalid};
        vr::VRInputComponentHandle_t skeleton_{};uint64_t last_skeleton_{};
        vr::VRInputComponentHandle_t trigger_{},grab_{},trigger_touch_{},grab_touch_{},grip_{},aim_{},legacy_grip_{},legacy_aim_{};
    };
public:
    XRHandPosePair():left_(*this,0),right_(*this,1){}
    bool requested() const {auto p=std::getenv("XR_HAND_STATE_SOCKET");return p && *p;}
    bool configure(bool native_head) {
        if (!requested()) return true;
        if (!native_head || !client_.configure(std::getenv("XR_HAND_STATE_SOCKET"))) return false;
        enabled_=vr::VRServerDriverHost()->TrackedDeviceAdded(left_.serial(),vr::TrackedDeviceClass_Controller,&left_) && vr::VRServerDriverHost()->TrackedDeviceAdded(right_.serial(),vr::TrackedDeviceClass_Controller,&right_);
        ovrd_log("XR_HAND_INPUT registered=%d source=live-shared-camera pose=grip+aim gestures=pinch+grab\n",enabled_);return enabled_;
    }
    void frame() {
        if (!enabled_) return;
        client_.update();left_.publish();right_.publish();
        auto p=client_.packet();if (p.sequence!=last_ && (p.sequence<3 || p.sequence>=last_logged_+100)) {
            ovrd_log("XR_HAND_STATE seq=%llu left=%d right=%d age_ms=%.2f\n",(unsigned long long)p.sequence,client_.get(0).has_value(),client_.get(1).has_value(),double(xr::hand::Client::now()-p.exposure_boottime_ns)/1e6);last_logged_=p.sequence;
        }
        last_=p.sequence;
    }
    void cleanup() {enabled_=false;client_.reset();left_.Deactivate();right_.Deactivate();}
private:
    xr::hand::Client client_;Device left_,right_;bool enabled_{};uint64_t last_{},last_logged_{};
};
