#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

const vec4 u_color = vec4(0.96, 0.84, 0.69, 0.0); // this is the color of the sand, on the lighter side since it looks a bit more like a real north american dune or a beach

uniform sampler2D u_texture_4;
uniform vec2 u_texture_4_size;

uniform sampler2D u_ripples;
uniform vec2 u_ripples_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
    return texture(u_ripples, uv).r;
}

// Pseudo-random number generator based on fragment position
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
}

float noise(vec2 st) {
    float total = 0.0;
    float persistence = 0.5;
    int octaves = 4;

    for (int i = 0; i < octaves; i++) {
        float frequency = pow(2.0, float(i));
        float amplitude = pow(persistence, float(octaves - i));

        total += random(st * frequency) * amplitude;
    }

    return total;
}

// Function to determine if a sparkle should appear based on random value
float calculateSparkle(vec2 uv) {
    float sparkleThreshold = 0.70; // Adjust as needed for sparkle density
    float randomValue = noise(uv);
    return smoothstep(sparkleThreshold - 0.05, sparkleThreshold, randomValue); // Adjust the range for sparkle appearance
}

void main() {
    // Bump Mapping
    // find the normal given the texture
    float t_width = u_ripples_size.x;
    float t_height = u_ripples_size.y;

    // find tangent vector
    vec3 t = normalize(vec3(v_tangent));

    // create B vector (cross product of normal and tangent)
    vec3 b = cross(normalize(vec3(v_normal)), t);

    // create TBN matrix
    mat3 TBN = mat3(t, b, normalize(vec3(v_normal)));

    // get current u and v values
    float u = v_uv.x;
    float v = v_uv.y;

    // scale these to allow tiling of the texture
    u *= 1.;
    v *= 1.;

    // grab scaling values
    float normal_scaling = u_normal_scaling;
    float height_scaling = u_height_scaling;

    // calculate dU and dV
    float dU = (h(vec2(u + normal_scaling / t_width, v)) - h(vec2(u, v))) / (height_scaling * normal_scaling);
    float dV = (h(vec2(u, v + normal_scaling / t_height)) - h(vec2(u, v))) / (height_scaling * normal_scaling);

    // local space normal 
    vec3 local_normal = vec3(-dU, -dV, 1.0);

    // calculate displaced normal
    vec3 n_d = normalize(TBN * local_normal);

    // Phong Shading
    // arbitrary diffuse coefficient
    float k_d = 2.5;

    // arbitrary ambient coefficient
    float k_a = 0.7;

    // arbitrary ambient intensity
    vec4 I_a = vec4(0.5, 0.5, 0.5, 0);

    // arbitrary specular coefficient
    float k_s = 2.0;

    // arbitrary p value
    float p = 20.0;

    // find normal and cos_theta_nl
    vec3 l = u_light_pos - vec3(v_position);
    float r = length(l);
    l = normalize(l);
    float cos_theta_nl = dot(n_d, l);
    cos_theta_nl = max(0, cos_theta_nl);

    // calculate cos_theta_nh, first find h
    vec3 h = l + normalize(u_cam_pos - vec3(v_position));
    h = normalize(h);
    float cos_theta_nh = dot(n_d, h);
    cos_theta_nh = max(0, cos_theta_nh);

    // calculate I/r^2
    vec4 I_r2 = vec4(u_light_intensity, 0) / (r * r);

    // Apply the phong model to the base color
    vec4 phong_color = (k_a * I_a) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * u_color * pow(cos_theta_nh, p));

    // Calculate sparkle intensity based on random value
    float sparkleIntensity = calculateSparkle(v_uv);

    // Add sparkle effect
    phong_color.rgb -= sparkleIntensity * 0.1; // Adjust sparkle intensity and color as needed

    out_color = phong_color;
    out_color.a = 1.0;
}
