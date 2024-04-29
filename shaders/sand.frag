#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;
uniform mat4 u_view_projection;

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

// Simplex 2D noise function
float snoise(vec2 v) {
    const vec4 C = vec4(0.211324865405187,  // (3.0 - sqrt(3.0)) / 6.0
                        0.366025403784439,  // 0.5 / sqrt(3.0)
                       -0.577350269189626,  // -1.0 + 2.0 * C.x
                        0.024390243902439); // 1.0 / 41.0
    vec2 i  = floor(v + dot(v, C.yy));
    vec2 x0 = v - i + dot(i, C.xx);
    vec2 i1;
    i1 = (x0.x > x0.y) ? vec2(1.0, 0.0) : vec2(0.0, 1.0);
    vec4 x12 = x0.xyxy + C.xxzz;
    x12.xy -= i1;
    i = mod(i, 289.0); // Avoid truncation effects in permutation
    vec3 p = mod(floor(vec3(i, i + 1.0) * 289.0), 289.0);
    vec3 m = max(0.5 - vec3(dot(x0, x0), dot(x12.xy, x12.xy), dot(x12.zw, x12.zw)), 0.0);
    m = m * m;
    m = m * m;
    vec3 x = 2.0 * fract(p * C.www) - 1.0;
    vec3 h = abs(x) - 0.5;
    vec3 ox = floor(x + 0.5);
    vec3 a0 = x - ox;
    m *= 1.79284291400159 - 0.85373472095314 * (a0 * a0 + h * h);
    vec3 g;
    g.x = a0.x * x0.x + h.x * x0.y;
    g.yz = a0.yz * x12.xz + h.yz * x12.yw;
    return 130.0 * dot(m, g);
}

// Define constants
const float frequency = 0.1; // Adjust according to your needs
const int numLayers = 5; // Number of layers
const float frequencyMult = 1.5; // Frequency multiplier
const float amplitudeMult = 0.5; // Amplitude multiplier

// Ripple noise function
float ripple_noise(vec2 p) {
    vec4 world_pos = u_view_projection * vec4(p, 0.0, 1.0);
    vec3 world_pos3 = world_pos.xyz / world_pos.w;

    // Translate UV to world coordinates
    vec2 world_uv = world_pos3.xy;
    p = world_uv;
    float amplitude = 1.0;
    float noiseValue = 0.0;

    // Compute fractal noise
    for (int l = 0; l < numLayers; ++l) {
        noiseValue += snoise(p * frequency) * amplitude;
        p *= frequencyMult;
        amplitude *= amplitudeMult;
    }

    // Apply ripple effect
    return (sin((noiseValue * 100.0) * 2.0 * 3.14159265358979323846 / 200.0) + 1.0) / 2.0;
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
vec3 calculateSparkle(vec2 uv) {
    // Convert UV to world space using u_view_projection
    vec4 world_pos = u_view_projection * vec4(uv, 0.0, 1.0);
    vec3 world_pos3 = world_pos.xyz / world_pos.w;

    // Translate UV to world coordinates
    vec2 world_uv = world_pos3.xy;

    float sparkleThreshold = 0.50; // Adjust as needed for sparkle density

    // Use different offsets for each component to ensure variation
    float randomValueX = noise(world_uv + vec2(0.1, 0.0));
    float randomValueY = noise(world_uv + vec2(0.0, 0.1));
    float randomValueZ = noise(world_uv + vec2(0.2, 0.3));
    
    // Apply smoothstep to each component individually
    float sparkleIntensityX = smoothstep(sparkleThreshold - 0.05, sparkleThreshold, randomValueX);
    float sparkleIntensityY = smoothstep(sparkleThreshold - 0.05, sparkleThreshold, randomValueY);
    float sparkleIntensityZ = smoothstep(sparkleThreshold - 0.05, sparkleThreshold, randomValueZ);
    
    // Return the perturbation vector
    return vec3(sparkleIntensityX, sparkleIntensityY, sparkleIntensityZ);
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

    // Adjust UV coordinates for tiling and ripple effect
    float rippleScale = 0.05; // Adjust ripple scale as needed
    u += rippleScale * ripple_noise(v_uv); // Perturb u coordinate with ripple noise
    v += rippleScale * ripple_noise(v_uv); // Perturb v coordinate with ripple noise
    u = mod(u, 1.0); // Apply tiling
    v = mod(v, 1.0); // Apply tiling

    // grab scaling values
    float normal_scaling = u_normal_scaling;
    float height_scaling = u_height_scaling;

    // calculate dU and dV
    float dU = (h(vec2(u + normal_scaling / t_width, v)) - h(vec2(u, v))) / (height_scaling * normal_scaling);
    float dV = (h(vec2(u, v + normal_scaling / t_height)) - h(vec2(u, v))) / (height_scaling * normal_scaling);

    // local space normal 
    vec3 local_normal = vec3(-dU, -dV, 1.0);

    // use the sparkle function to determine if a sparkle should appear and adjust the normal accordingly
    vec3 sparkle = calculateSparkle(vec2(u, v)) * 0.1; // Adjust sparkle intensity
    local_normal += sparkle; // Add sparkle perturbation to the normal

    // calculate displaced normal
    vec3 n_d = normalize(TBN * local_normal);

    // Phong Shading
    // arbitrary diffuse coefficient
    float k_d = 2.5;

    // arbitrary ambient coefficient
    float k_a = 0.5;

    // arbitrary ambient intensity
    vec4 I_a = vec4(0.5, 0.5, 0.5, 0);

    // arbitrary specular coefficient
    float k_s = 1.0;

    // arbitrary p value
    float p = 25.0;

    // find normal and cos_theta_nl
    vec3 l = u_light_pos - vec3(v_position);
    float r = length(l);
    l = normalize(l);
    float cos_theta_nl = dot(n_d, l);
    cos_theta_nl = max(0.0, cos_theta_nl);

    // calculate cos_theta_nh, first find h
    vec3 h = l + normalize(u_cam_pos - vec3(v_position));
    h = normalize(h);
    float cos_theta_nh = dot(n_d, h);
    cos_theta_nh = max(0.0, cos_theta_nh);

    // calculate I/r^2
    vec4 I_r2 = vec4(u_light_intensity, 0.0) / (r * r);

    // Apply the phong model to the base color
    vec4 phong_color = (k_a * I_a) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * u_color * pow(cos_theta_nh, p));

    out_color = phong_color;
    out_color.a = 1.0;
}

