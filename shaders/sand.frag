#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_4;
uniform vec2 u_texture_4_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
    return texture(u_texture_4, uv).r;
}

// Pseudo-random number generator based on fragment position
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
}

// Function to determine if a sparkle should appear based on random value
float calculateSparkle(vec2 uv) {
    float sparkleThreshold = 0.95; // Adjust as needed for sparkle density
    float randomValue = random(uv);
    return smoothstep(sparkleThreshold - 0.05, sparkleThreshold, randomValue); // Adjust the range for sparkle appearance
}

void main() {
    // Bump Mapping
    float t_width = u_texture_4_size.x;
    float t_height = u_texture_4_size.y;
    vec3 t = normalize(vec3(v_tangent));
    vec3 b = cross(normalize(vec3(v_normal)), t);
    mat3 TBN = mat3(t, b, normalize(vec3(v_normal)));
    float u = v_uv.x;
    float v = v_uv.y;
    float normal_scaling = u_normal_scaling;
    float height_scaling = u_height_scaling;
    float dU = (h(vec2(u + normal_scaling / t_width, v)) - h(vec2(u, v))) / (height_scaling * normal_scaling);
    float dV = (h(vec2(u, v + normal_scaling / t_height)) - h(vec2(u, v))) / (height_scaling * normal_scaling);
    vec3 local_normal = vec3(-dU, -dV, 1.0);
    vec3 n_d = normalize(TBN * local_normal);

    // Phong Shading
    float k_d = 1.0;
    float k_a = 1.0;
    vec4 I_a = vec4(0.1, 0.1, 0.1, 0);
    float k_s = 0.95;
    float p = 25.0;
    vec3 l = u_light_pos - vec3(v_position);
    float r = length(l);
    l = normalize(l);
    float cos_theta_nl = dot(n_d, l);
    cos_theta_nl = max(0, cos_theta_nl);
    vec3 h = l + normalize(u_cam_pos - vec3(v_position));
    h = normalize(h);
    float cos_theta_nh = dot(n_d, h);
    cos_theta_nh = max(0, cos_theta_nh);
    vec4 I_r2 = vec4(u_light_intensity, 0) / (r * r);

    // Apply the phong model to the base color
    vec4 phong_color = (k_a * I_a) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * u_color * pow(cos_theta_nh, p));

    // Calculate sparkle intensity based on random value
    float sparkleIntensity = calculateSparkle(v_uv);

    // Add sparkle effect
    phong_color.rgb += sparkleIntensity * 0.5; // Adjust sparkle intensity and color as needed

    out_color = phong_color;
    out_color.a = 1.0;
}
