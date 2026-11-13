// SPDX-License-Identifier: MIT
#pragma once
#include <xr/hand/client.hpp>
#include <xr/hand/joints.hpp>
#include <atomic>
#include <cstdlib>
#include "xr_input_switch.hpp"
extern vr::VRBoneTransform_t rightOpenPose[31],leftOpenPose[31];

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
            const char *model=std::getenv(hand_?"A081_CONTROLLER_RENDER_MODEL_RIGHT":"A081_CONTROLLER_RENDER_MODEL_LEFT");
            if(model&&*model)props->SetStringProperty(p,vr::Prop_RenderModelName_String,model);
            auto face0=std::string(hand_?"/input/a":"/input/x"),face1=std::string(hand_?"/input/b":"/input/y");
            input->CreateBooleanComponent(p,(face0+"/click").c_str(),&face0_);input->CreateBooleanComponent(p,(face1+"/click").c_str(),&face1_);
            input->CreateBooleanComponent(p,(face0+"/touch").c_str(),&face0_touch_);input->CreateBooleanComponent(p,(face1+"/touch").c_str(),&face1_touch_);
            input->CreateBooleanComponent(p,"/input/system/click",&system_);input->CreateBooleanComponent(p,"/input/joystick/click",&stick_click_);input->CreateBooleanComponent(p,"/input/joystick/touch",&stick_touch_);
            input->CreateScalarComponent(p,"/input/joystick/x",&stick_x_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedTwoSided);input->CreateScalarComponent(p,"/input/joystick/y",&stick_y_,vr::VRScalarType_Absolute,vr::VRScalarUnits_NormalizedTwoSided);
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
            auto source=pair_.switch_.source();
            if(source==XRInputSwitch::Controllers){auto p=pair_.controllers_.pose(hand_);p.deviceIsConnected=true;return p;}
            vr::DriverPose_t pose{};pose.qWorldFromDriverRotation.w=1;pose.qDriverFromHeadRotation.w=1;pose.qRotation.w=1;
            pose.deviceIsConnected=true;pose.result=vr::TrackingResult_Running_OutOfRange;
            xr_hand_packet snapshot{};auto h=pair_.client_.get(hand_,&snapshot);if (source!=XRInputSwitch::Hands || !h) return pose;
            for (unsigned i=0;i<3;++i) pose.vecPosition[i]=h->grip[i];
            pose.qRotation={h->grip[6],h->grip[3],h->grip[4],h->grip[5]};
            pose.poseTimeOffset=-double(xr::hand::Client::now()-snapshot.exposure_boottime_ns)/1e9;
            pose.poseIsValid=true;pose.result=vr::TrackingResult_Running_OK;return pose;
        }
        void publish() {
            auto index=index_.load();if (index==vr::k_unTrackedDeviceIndexInvalid) return;
            auto source=pair_.switch_.source();auto h=source==XRInputSwitch::Hands?pair_.client_.get(hand_):std::nullopt;
            auto physical=source==XRInputSwitch::Controllers?pair_.controllers_.inputs(hand_):A081ControllerPosePair::Inputs{};
            auto input=vr::VRDriverInput();
            if(generation_!=pair_.switch_.generation()){generation_=pair_.switch_.generation();require_release_=true;last_skeleton_=0;}
            // Tracking may disappear while the selected source remains Hands.
            // Reacquiring a pinched hand must wait for release, just as a
            // source switch does; discard the old skeleton sequence too.
            if(source==XRInputSwitch::Hands&&!h){require_release_=true;last_skeleton_=0;}
            float trigger=h?h->pinch:physical.trigger_value,grab=h?h->grab:physical.grip_value;
            if(require_release_&&source!=XRInputSwitch::None&&trigger<=.35f&&grab<=.35f&&!physical.face0_click&&!physical.face1_click&&!physical.system_click&&!physical.joystick_click)require_release_=false;
            bool enabled=source!=XRInputSwitch::None&&!require_release_;
            input->UpdateScalarComponent(trigger_,enabled?trigger:0,0);input->UpdateBooleanComponent(trigger_touch_,enabled&&(h?bool(h->flags&XR_HAND_PINCH):physical.trigger_touch),0);
            input->UpdateScalarComponent(grab_,enabled?grab:0,0);input->UpdateBooleanComponent(grab_touch_,enabled&&(h?bool(h->flags&XR_HAND_GRAB):physical.grip_touch),0);
            input->UpdateBooleanComponent(face0_,enabled&&physical.face0_click,0);input->UpdateBooleanComponent(face1_,enabled&&physical.face1_click,0);
            input->UpdateBooleanComponent(face0_touch_,enabled&&physical.face0_touch,0);input->UpdateBooleanComponent(face1_touch_,enabled&&physical.face1_touch,0);
            input->UpdateBooleanComponent(system_,enabled&&physical.system_click,0);input->UpdateBooleanComponent(stick_click_,enabled&&physical.joystick_click,0);input->UpdateBooleanComponent(stick_touch_,enabled&&physical.joystick_touch,0);
            input->UpdateScalarComponent(stick_x_,enabled?physical.joystick_x:0,0);input->UpdateScalarComponent(stick_y_,enabled?physical.joystick_y:0,0);
            if(h){
                if(last_skeleton_!=pair_.client_.packet().sequence){vr::VRBoneTransform_t bones[31]{};if(skeleton(*h,bones)){
                    input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithoutController,bones,31);input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithController,bones,31);last_skeleton_=pair_.client_.packet().sequence;}}
                vr::HmdMatrix34_t identity{};for(unsigned i=0;i<3;++i)identity.m[i][i]=1;auto aim=relative(h->grip,h->aim);
                input->UpdatePoseComponent(grip_,&identity,0);input->UpdatePoseComponent(aim_,&aim,0);input->UpdatePoseComponent(legacy_grip_,&identity,0);input->UpdatePoseComponent(legacy_aim_,&aim,0);
            }else if(source==XRInputSwitch::Controllers){
                // Controller-derived reference skeleton, never stale optical bones.
                auto bones=hand_?rightOpenPose:leftOpenPose;input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithoutController,bones,31);input->UpdateSkeletonComponent(skeleton_,vr::VRSkeletalMotionRange_WithController,bones,31);
                input->UpdatePoseComponent(grip_,&pair_.physical_grip_,0);input->UpdatePoseComponent(aim_,&pair_.physical_aim_,0);input->UpdatePoseComponent(legacy_grip_,&pair_.physical_grip_,0);input->UpdatePoseComponent(legacy_aim_,&pair_.physical_aim_,0);
            }
            auto pose=GetPose();vr::VRServerDriverHost()->TrackedDevicePoseUpdated(index,pose,sizeof(pose));
        }
    private:
        XRHandPosePair &pair_;unsigned hand_;
        std::atomic<vr::TrackedDeviceIndex_t> index_{vr::k_unTrackedDeviceIndexInvalid};
        uint64_t generation_{};bool require_release_=true;
        vr::VRInputComponentHandle_t face0_{},face1_{},face0_touch_{},face1_touch_{},system_{},stick_click_{},stick_touch_{},stick_x_{},stick_y_{};
        vr::VRInputComponentHandle_t skeleton_{};uint64_t last_skeleton_{};
        vr::VRInputComponentHandle_t trigger_{},grab_{},trigger_touch_{},grab_touch_{},grip_{},aim_{},legacy_grip_{},legacy_aim_{};
    };
public:
    XRHandPosePair():left_(*this,0),right_(*this,1){}
    bool requested() const {auto p=std::getenv("XR_HAND_STATE_SOCKET");return p && *p;}
    bool configure(bool native_head) {
        if (!requested()) return true;
        if (!native_head || !client_.configure(std::getenv("XR_HAND_STATE_SOCKET"))) return false;
        if(!controllers_.configure(native_head,false)||!load_physical_anchors())return false;
        enabled_=vr::VRServerDriverHost()->TrackedDeviceAdded(left_.serial(),vr::TrackedDeviceClass_Controller,&left_) && vr::VRServerDriverHost()->TrackedDeviceAdded(right_.serial(),vr::TrackedDeviceClass_Controller,&right_);
        ovrd_log("XR_HAND_INPUT registered=%d source=live-shared-camera pose=grip+aim gestures=pinch+grab\n",enabled_);return enabled_;
    }
    void frame() {
        if (!enabled_) return;
        controllers_.frame(false);client_.update();
        bool cp[2]={controllers_.pose(0).poseIsValid,controllers_.pose(1).poseIsValid};bool hp[2]={client_.get(0).has_value(),client_.get(1).has_value()};
        bool activity=false,held=false;
        for(unsigned i=0;i<2;++i){auto v=controllers_.inputs(i);auto p=controllers_.pose(i);bool pressed=v.trigger_value>.35f||v.grip_value>.35f||v.face0_click||v.face1_click||v.system_click||v.joystick_click;
            bool moved=false;if(cp[i]&&previous_valid_[i]){double d=0,q=0;for(unsigned j=0;j<3;++j)d+=std::pow(p.vecPosition[j]-previous_pose_[i].vecPosition[j],2);q=p.qRotation.w*previous_pose_[i].qRotation.w+p.qRotation.x*previous_pose_[i].qRotation.x+p.qRotation.y*previous_pose_[i].qRotation.y+p.qRotation.z*previous_pose_[i].qRotation.z;moved=d>.000025||std::fabs(q)<.9998;}
            activity|=pressed||v.face0_touch||v.face1_touch||v.joystick_touch||v.trigger_touch||moved;
            if(switch_.source()==XRInputSwitch::Controllers)held|=pressed;
            if(switch_.source()==XRInputSwitch::Hands){auto h=client_.get(i);held|=h&&bool(h->flags&(XR_HAND_PINCH|XR_HAND_GRAB));}
            previous_pose_[i]=p;previous_valid_[i]=cp[i];}
        switch_.frame(cp[0]||cp[1],hp[0]||hp[1],activity,held);switch_.report(cp[0],cp[1],hp[0],hp[1]);left_.publish();right_.publish();
        auto p=client_.packet();if (p.sequence!=last_ && (p.sequence<3 || p.sequence>=last_logged_+100)) {
            ovrd_log("XR_HAND_STATE seq=%llu left=%d right=%d age_ms=%.2f\n",(unsigned long long)p.sequence,client_.get(0).has_value(),client_.get(1).has_value(),double(xr::hand::Client::now()-p.exposure_boottime_ns)/1e6);last_logged_=p.sequence;
        }
        last_=p.sequence;
    }
    void cleanup() {enabled_=false;controllers_.cleanup();client_.reset();left_.Deactivate();right_.Deactivate();}
private:
    bool load_physical_anchors(){
        const char *dir=std::getenv("XR_CONTROLLER_MODEL_DIR");std::string path=dir?dir:"/usr/share/xr/devices/a081/controllers";path+="/controller-anchors.json";
        std::ifstream f(path);std::string text((std::istreambuf_iterator<char>(f)),{});auto root=cJSON_Parse(text.c_str());if(!root)return false;
        auto anchors=cJSON_GetObjectItemCaseSensitive(root,"anchors");bool valid=true;
        const std::array<std::pair<const char *,vr::HmdMatrix34_t *>,2> transforms{{{"grip",&physical_grip_},{"aim",&physical_aim_}}};
        for(const auto &pair:transforms){
            auto a=cJSON_GetObjectItemCaseSensitive(anchors,pair.first);auto origin=cJSON_GetObjectItemCaseSensitive(a,"origin");auto rotate=cJSON_GetObjectItemCaseSensitive(a,"rotate_xyz");
            if(cJSON_GetArraySize(origin)!=3||cJSON_GetArraySize(rotate)!=3){valid=false;break;}float e[3]{};
            for(int i=0;i<3;++i){auto p=cJSON_GetArrayItem(origin,i),r=cJSON_GetArrayItem(rotate,i);if(!cJSON_IsNumber(p)||!cJSON_IsNumber(r)||!std::isfinite(p->valuedouble)||!std::isfinite(r->valuedouble)){valid=false;break;}pair.second->m[i][3]=float(p->valuedouble);e[i]=float(r->valuedouble)*.01745329252f;}
            float cx=std::cos(e[0]),sx=std::sin(e[0]),cy=std::cos(e[1]),sy=std::sin(e[1]),cz=std::cos(e[2]),sz=std::sin(e[2]);
            float m[3][3]={{cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx},{sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx},{-sy,cy*sx,cy*cx}};for(int i=0;i<3;++i)for(int j=0;j<3;++j)pair.second->m[i][j]=m[i][j];
        }cJSON_Delete(root);return valid;
    }
    A081ControllerPosePair controllers_;XRInputSwitch switch_;vr::HmdMatrix34_t physical_grip_{},physical_aim_{};
    vr::DriverPose_t previous_pose_[2]{};bool previous_valid_[2]{};
    xr::hand::Client client_;Device left_,right_;bool enabled_{};uint64_t last_{},last_logged_{};
};
