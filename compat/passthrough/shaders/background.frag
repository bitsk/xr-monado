// SPDX-License-Identifier: GPL-3.0-or-later
#version 450
layout(location=0) in vec2 uv;
layout(location=0) out vec4 colour;
layout(set=0,binding=3) uniform sampler2D passthrough;
layout(set=0,binding=4,std430) readonly buffer CameraCalibration { vec4 camera[]; } calibration;
layout(push_constant) uniform View{vec4 fov;vec4 orientation;vec4 environment;vec4 origin;}view;
const float PI=3.141592653589793;
void main(){

    int eye=int(view.environment.w+.5),base=eye*5;
    vec3 ray=normalize(vec3(mix(view.fov.x,view.fov.y,uv.x),mix(view.fov.z,view.fov.w,uv.y),-1.));
    vec3 c=vec3(dot(calibration.camera[base+2].xyz,ray),dot(calibration.camera[base+3].xyz,ray),dot(calibration.camera[base+4].xyz,ray));
    if(c.z<=.0001){colour=vec4(0.,0.,0.,1.);return;}
    float r=length(c.xy),lookup=clamp(r/(r+c.z)*1024.,0.,1024.);
    int lower=min(int(lookup),1023),lut_base=12+eye*1025;
    vec4 intrinsics=calibration.camera[base];
    float distorted=mix(calibration.camera[lut_base+lower].x,calibration.camera[lut_base+lower+1].x,lookup-float(lower));
    vec2 pixel=intrinsics.zw+c.xy*distorted/max(r,.000001)*intrinsics.xy;
    vec2 size=calibration.camera[11].xy;
    if(any(lessThan(pixel,vec2(.5)))||any(greaterThanEqual(pixel,size-vec2(.5)))){colour=vec4(0.,0.,0.,1.);return;}
    vec2 sample_uv=vec2((pixel.x+float(eye)*size.x)/(2.*size.x),pixel.y/size.y);
    colour=vec4(texture(passthrough,sample_uv).rgb*calibration.camera[10].xyz,1.);return;
}
