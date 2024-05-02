#version 400

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;
uniform mat4 u_view_projection;
uniform mat4 u_inverse_view_projection;
uniform mat4 u_model;

const vec2 ripple_dirs[6]=vec2[6](
	vec2(1.0, 0.0),
	vec2(0.8, 0.2),
	vec2(0.8, -0.2),
	vec2(0.5, 0.5),
	vec2(0.6, -0.4),
	vec2(0.9, 0.1)
);

const float ripple_lengths[6]=float[6](
	2.01,
	1.83,
	0.96,
	0.54,
	0.21,
	0.4
);

const vec4 u_color = vec4(0.96, 0.84, 0.69, 0.0); // this is the color of the sand, on the lighter side since it looks a bit more like a real north american dune or a beach

uniform sampler2D u_texture_4;
uniform vec2 u_texture_4_size;

uniform sampler2D u_ripples;
uniform vec2 u_ripples_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform float u_time;

uniform mat3 collisionPoints;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;
in vec2 v_collision;

out vec4 out_color;

float h(vec2 uv) {
    return texture(u_texture_4, uv).r;
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
    /*vec4 world_pos = u_view_projection * vec4(p, 0.0, 1.0);
    vec3 world_pos3 = world_pos.xyz / world_pos.w;

    // Translate UV to world coordinates
    vec2 world_uv = world_pos3.xy;
    p = world_uv;*/
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

uint hash( uint x ) {
    x += ( x << 10u );
    x ^= ( x >>  6u );
    x += ( x <<  3u );
    x ^= ( x >> 11u );
    x += ( x << 15u );
    return x;
}

uint hash( uvec2 v ) { return hash( v.x ^ hash(v.y)                         ); }
uint hash( uvec3 v ) { return hash( v.x ^ hash(v.y) ^ hash(v.z)             ); }

// Construct a float with half-open range [0:1] using low 23 bits.
// All zeroes yields 0.0, all ones yields the next smallest representable value below 1.0.
float floatConstruct( uint m ) {
    const uint ieeeMantissa = 0x007FFFFFu; // binary32 mantissa bitmask
    const uint ieeeOne      = 0x3F800000u; // 1.0 in IEEE binary32

    m &= ieeeMantissa;                     // Keep only mantissa bits (fractional part)
    m |= ieeeOne;                          // Add fractional part to 1.0

    float  f = uintBitsToFloat( m );       // Range [1:2]
    return f - 1.0;                        // Range [0:1]
}

vec3 hash3(float p)
{
    uint v1 = hash(floatBitsToUint(p));
    uint v2 = hash(v1);
    uint v3 = hash(v2);

    return vec3(floatConstruct(v1), floatConstruct(v2), floatConstruct(v3));
}

vec3 hash3(ivec2 p)
{
    uint v1 = hash(uvec2(p));
    uint v2 = hash(v1);
    uint v3 = hash(v2);

    return vec3(floatConstruct(v1), floatConstruct(v2), floatConstruct(v3));
}

float voronoi( vec2 x )
{
    ivec2 p = ivec2(x);
    vec2 f = fract(x);
    
    float best = 0.0;
    float bestDist = 100.0;
    for( int j=-2; j<=2; j++ )
    for( int i=-2; i<=2; i++ )
    {
        ivec2 g = ivec2( i, j );
        vec3  o = hash3( g + p );
        vec2  r = g - f + (o.xy - 0.5);
        float d = dot(r,r);
        if (d < bestDist) {
          bestDist = d;
          best = o.z;
        }
    }

    return best;
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
    float rippleScale = 0.1; // Adjust ripple scale as needed
    u += rippleScale * ripple_noise(v_uv); // Perturb u coordinate with ripple noise
    v += rippleScale * ripple_noise(v_uv); // Perturb v coordinate with ripple noise
    u = mod(300 * u, 1.0); // Apply tiling
    v = mod(300 * v, 1.0); // Apply tiling

    // grab scaling values
    float normal_scaling = u_normal_scaling;
    float height_scaling = u_height_scaling;

    // calculate dU and dV
    float dU = 30 * (h(vec2(u + 1 / t_width, v)) - h(vec2(u, v))) * (height_scaling * normal_scaling);
    float dV = 30 * (h(vec2(u, v + 1 / t_height)) - h(vec2(u, v))) * (height_scaling * normal_scaling);

    // local space normal
    float voronoi_result = voronoi(v_uv * 500000);
    vec3 local_normal = vec3(-dU, -dV, 1.0) + 0.3 * hash3(voronoi_result);

    /*for (int i = 0; i < 6; i++) {
      float lambda = ripple_lengths[i];
      vec2 v = ripple_dirs[i];
      vec2 uv_p = vec2(v_uv.x + 0.0003 * cos(v_uv.y * 1000) + 0.0003 * cos(dot(v_uv, ripple_dirs[(i + 1) % 6]) * 1000), v_uv.y);
      local_normal += vec3(v.x, v.y, 0.0) * sin(dot(uv_p, v) * 20000 * lambda) * 0.15;
    }*/

    // use the sparkle function to determine if a sparkle should appear and adjust the normal accordingly
    // vec3 sparkle = calculateSparkle(vec2(u, v)) * 0.1; // Adjust sparkle intensity
    // local_normal += sparkle; // Add sparkle perturbation to the normal

    vec3 l = vec3(0.0, sin(u_time * 0.02), -cos(u_time * 0.02));

    // calculate displaced normal
    vec3 n_d = normalize(TBN * local_normal);

    // Phong Shading
    // arbitrary diffuse coefficient
    float k_d = 0.6;

    // arbitrary ambient coefficient
    float k_a = (0.2 * pow(max(l.y + 0.1, 0), 0.25) + 0.1);

    // arbitrary specular coefficient
    float k_s = 0.3;

    // arbitrary p value
    float p = 25.0;

    // find normal and cos_theta_nl
    float cos_theta_nl = dot(n_d, l);
    cos_theta_nl = max(0.0, cos_theta_nl);

    // calculate cos_theta_nh, first find h
    vec3 bisector = l + normalize(u_cam_pos - vec3(v_position));
    bisector = normalize(bisector);
    float cos_theta_nh = dot(n_d, bisector);
    float cos_theta_nl_l = max(0.0, dot(vec3(v_normal), l));
    if (cos_theta_nl_l < 0.1) {
      cos_theta_nh *= cos_theta_nl_l * 10;
    }
    cos_theta_nh = max(0.0, cos_theta_nh);

    // calculate I/r^2
    vec4 I_r2 = vec4(u_light_intensity, 0.0);

    // color for phong shading
    vec4 p_color = u_color * vec4(0.8 + 0.2 * vec3(1.0, 0.5, 0.0) * voronoi_result, 1.0);

    // if collision point, color it red
    if (v_collision.x > 0.0) {
		  p_color = vec4(1.0, 0.0, 0.0, 0.0);
	  }

    // Apply the phong model to the base color
    vec4 phong_color = k_a + (k_d * I_r2 * p_color * cos_theta_nl) + (k_s * I_r2 * pow(cos_theta_nh, p));

    out_color = phong_color;
    out_color.a = 1.0;
}

