#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_ripples;
uniform vec2 u_ripples_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;
const float u_tile_scale = 10.; // Scale factor for tiling

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec2 eyeDirection;

void main() {
    vec4 worldPosition = u_model * in_position;
    gl_Position = u_view_projection * worldPosition;

    // Calculate eye direction in world space
    vec3 eyeDir = normalize(worldPosition.xyz - vec3(0.0, 0.0, 0.0));
    eyeDirection = eyeDir.xy;
}
