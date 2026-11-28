// SPDX-License-Identifier: GPL-3.0-or-later
#version 450
layout(location=0) out vec2 view_xy;
layout(location=1) out vec3 camera_ray;
layout(set=0,binding=4,std430) readonly buffer CameraCalibration { vec4 camera[]; } calibration;
layout(push_constant) uniform View { vec4 fov; vec4 orientation; vec4 environment; vec4 origin; } view;

void main() {
    vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
    view_xy = vec2(mix(view.fov.x, view.fov.y, uv.x), mix(view.fov.z, view.fov.w, uv.y));
    vec3 ray = vec3(view_xy, -1.0);
    int base = int(view.environment.w + 0.5) * 5;
    camera_ray = vec3(dot(calibration.camera[base+2].xyz, ray),
                      dot(calibration.camera[base+3].xyz, ray),
                      dot(calibration.camera[base+4].xyz, ray));
}
