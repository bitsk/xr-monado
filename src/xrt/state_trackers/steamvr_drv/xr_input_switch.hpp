// SPDX-License-Identifier: MIT
#pragma once
#include <cjson/cJSON.h>
#include <sys/stat.h>
#include <fstream>
#include <iterator>
#include <string>
#include <atomic>
#include <cmath>

// Independent channels: holding grip at handoff must not block trigger/face
// buttons. Only a real fresh release can arm a channel after tracking loss.
class XRButtonReleaseGate {
public:
    void reset(){blocked_=63;}
    void sample(const float (&values)[6],bool valid){
        if(!valid){reset();return;}
        for(unsigned i=0;i<6;++i)if(std::isfinite(values[i])&&values[i]<=.35f)blocked_&=~(1u<<i);
    }
    void deliberate_press(unsigned mask){blocked_&=~mask;}
    bool allows(unsigned channel)const{return !(blocked_&(1u<<channel));}
    unsigned mask()const{return blocked_;}
private:
    unsigned blocked_=63;
};

// Selection and button neutralization are separate: the new valid pose is
// published during the neutral interval so action consumers can observe UP
// before the first intentional press. Invalid poses cannot rearm buttons.
class XRInputSwitch {
public:
    enum Source:unsigned {None,Controllers,Hands};
    Source source() const {return Source(selected_.load());}
    bool neutral() const {return uint64_t(os_monotonic_get_ns())<neutral_until_.load();}
    uint64_t neutral_until() const {return neutral_until_.load();}
    bool button_takeover() const {return button_takeover_;}
    bool hand_enabled()const{return hands_;}
    void frame(bool controllers,bool hands,bool activity,bool controller_press,bool hand_press,bool held){
        uint64_t now=os_monotonic_get_ns();read(now);
        const auto current=source();
        if(activity&&controllers)last_controller_=now;
        if(!hands||!hands_)hands_since_=0;
        else if(!hands_since_)hands_since_=now;
        bool ready=hands_since_&&now-hands_since_>=300000000ULL;
        Source desired=None;reason_="unavailable";
        if(active_){
            if(mode_=="controllers"){desired=Controllers;reason_="manual";}
            else if(mode_=="hands"){desired=hands_?Hands:None;reason_="manual";}
            else if(held&&((current==Controllers&&controllers)||(current==Hands&&hands&&hands_))){desired=current;reason_="interaction_held";}
            else if(controllers&&controller_press){desired=Controllers;reason_="controller_press";}
            else if(ready&&hand_press){desired=Hands;reason_="hand_pinch";}
            else if(current==Hands&&hands&&hands_&&!activity){desired=Hands;reason_="hands_active";}
            else if(controllers&&last_controller_&&now-last_controller_<800000000ULL){desired=Controllers;reason_="controller_activity";}
            else if(ready){desired=Hands;reason_="hands_ready";}
            else if(controllers){desired=Controllers;reason_="controller_available";}
        }
        if(desired!=candidate_){candidate_=desired;candidate_since_=now;}
        bool explicit_change=mode_!=previous_mode_||hands_!=previous_hands_||!active_;
        previous_mode_=mode_;previous_hands_=hands_;
        bool intentional=mode_=="auto"&&((desired==Controllers&&controller_press)||(desired==Hands&&hand_press));
        if(desired!=current&&(explicit_change||intentional||desired==None||now-candidate_since_>=120000000ULL)){
            button_takeover_=!explicit_change&&desired==Controllers&&controller_press;
            selected_=desired;report_after_=0;neutral_until_=now+150000000ULL;++generation_;
            ovrd_log("XR_INPUT_SWITCH source=%u generation=%llu mode=%s reason=%s neutral_ms=150\n",unsigned(desired),(unsigned long long)generation_,mode_.c_str(),reason_);
        }
    }
    uint64_t generation()const{return generation_;}
    void report(bool c0,bool c1,bool h0,bool h1,unsigned blocked0,unsigned blocked1,bool controller_aim_available=true){
        uint64_t now=os_monotonic_get_ns();if(now<report_after_)return;report_after_=now+100000000ULL;
        const char *path=std::getenv("XR_INPUT_STATUS_FILE");if(!path||!*path)return;
        std::string tmp=std::string(path)+".tmp";FILE *f=std::fopen(tmp.c_str(),"w");if(!f)return;
        auto s=source();std::fprintf(f,"{\"schema\":1,\"observed_monotonic_ns\":%llu,\"generation\":%llu,\"active_source\":\"%s\",\"mode\":\"%s\",\"reason\":\"%s\",\"neutral\":%s,\"blocked_buttons\":[%u,%u],\"controllers\":[%s,%s],\"hands\":[%s,%s],\"controller_aim_available\":%s}\n",(unsigned long long)now,(unsigned long long)generation_,s==Hands?"hands":s==Controllers?"controllers":"none",mode_.c_str(),reason_,neutral()?"true":"false",blocked0,blocked1,c0?"true":"false",c1?"true":"false",h0?"true":"false",h1?"true":"false",controller_aim_available?"true":"false");std::fclose(f);std::rename(tmp.c_str(),path);
    }
private:
    void read(uint64_t now){
        if(now<read_after_)return;
        read_after_=now+100000000ULL;
        const char *path=std::getenv("XR_SESSION_CONTROLS_FILE");if(!path||!*path)return;
        struct stat info{};if(lstat(path,&info)||!S_ISREG(info.st_mode)||info.st_uid||info.st_mode&0022||info.st_size>8192){active_=false;return;}
        std::ifstream f(path);std::string text((std::istreambuf_iterator<char>(f)),{});auto root=cJSON_Parse(text.c_str());if(!root){active_=false;return;}
        auto schema=cJSON_GetObjectItemCaseSensitive(root,"schema");auto active=cJSON_GetObjectItemCaseSensitive(root,"session_active");auto prefs=cJSON_GetObjectItemCaseSensitive(root,"preferences");auto mode=cJSON_GetObjectItemCaseSensitive(prefs,"input_mode");auto hand=cJSON_GetObjectItemCaseSensitive(prefs,"hand_tracking_enabled");
        active_=cJSON_IsNumber(schema)&&schema->valueint==1&&cJSON_IsTrue(active)&&cJSON_IsString(mode);
        if(active_){mode_=mode->valuestring;hands_=cJSON_IsTrue(hand);if(mode_!="auto"&&mode_!="controllers"&&mode_!="hands")active_=false;}
        cJSON_Delete(root);
    }
    std::atomic<unsigned> selected_{None};std::atomic<uint64_t> neutral_until_{};
    Source candidate_=None;uint64_t candidate_since_{},last_controller_{},hands_since_{},read_after_{},report_after_{},generation_{};
    const char *reason_="unavailable";
    std::string mode_="auto",previous_mode_;bool active_=true,hands_=true,previous_hands_=true,button_takeover_=false;
};
