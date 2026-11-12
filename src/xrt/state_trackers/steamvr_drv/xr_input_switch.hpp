// SPDX-License-Identifier: MIT
#pragma once
#include <cjson/cJSON.h>
#include <sys/stat.h>
#include <fstream>
#include <iterator>
#include <string>
#include <atomic>

class XRInputSwitch {
public:
    enum Source:unsigned {None,Controllers,Hands};
    Source source() const {return uint64_t(os_monotonic_get_ns())<neutral_until_.load()?None:Source(selected_.load());}
    bool hand_enabled()const{return hands_;}
    void frame(bool controllers,bool hands,bool activity,bool held){
        uint64_t now=os_monotonic_get_ns();read(now);
        if(activity)last_controller_=now;
        Source desired=None;
        if(active_){
            if(mode_=="controllers")desired=Controllers;
            else if(mode_=="hands"&&hands_)desired=Hands;
            else if(controllers&&(now-last_controller_<2000000000ULL||(Source(selected_.load())==Controllers&&held)))desired=Controllers;
            else if(hands&&hands_)desired=Hands;
            else if(controllers)desired=Controllers;
            if(mode_=="auto"&&held&&!activity&&source()==Hands&&hands&&hands_)desired=Hands;
        }
        if(desired!=candidate_){candidate_=desired;candidate_since_=now;}
        bool explicit_change=mode_!=previous_mode_||hands_!=previous_hands_||!active_;
        previous_mode_=mode_;previous_hands_=hands_;
        if(desired!=Source(selected_.load())&&(explicit_change||desired==None||now-candidate_since_>=150000000ULL)){
            selected_=desired;report_after_=0;neutral_until_=now+150000000ULL;++generation_;
            ovrd_log("XR_INPUT_SWITCH source=%u generation=%llu mode=%s neutral_ms=150\n",unsigned(desired),(unsigned long long)generation_,mode_.c_str());
        }
    }
    uint64_t generation()const{return generation_;}
    void report(bool c0,bool c1,bool h0,bool h1){
        uint64_t now=os_monotonic_get_ns();auto current=source();if(now<report_after_&&current==reported_)return;reported_=current;report_after_=now+200000000ULL;
        const char *path=std::getenv("XR_INPUT_STATUS_FILE");if(!path||!*path)return;
        std::string tmp=std::string(path)+".tmp";FILE *f=std::fopen(tmp.c_str(),"w");if(!f)return;
        auto s=source();std::fprintf(f,"{\"schema\":1,\"observed_monotonic_ns\":%llu,\"generation\":%llu,\"active_source\":\"%s\",\"mode\":\"%s\",\"controllers\":[%s,%s],\"hands\":[%s,%s]}\n",(unsigned long long)now,(unsigned long long)generation_,s==Hands?"hands":s==Controllers?"controllers":"none",mode_.c_str(),c0?"true":"false",c1?"true":"false",h0?"true":"false",h1?"true":"false");std::fclose(f);std::rename(tmp.c_str(),path);
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
    Source reported_=None;
    Source candidate_=None;uint64_t candidate_since_{},last_controller_{},read_after_{},report_after_{},generation_{};
    std::string mode_="hands",previous_mode_;bool active_=true,hands_=true,previous_hands_=true;
};
