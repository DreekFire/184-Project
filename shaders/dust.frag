#version 330 core

const vec3 u_skyBottomColor = vec3(0.1, 0.4, 0.90);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);    // Color of the sky at the top
const vec3 u_skySunColor = vec3(1, 0.3, 0.1);    // Color of the sky near the sun (for sunset/sunrise)
const float u_horizonHeight = 10.0;                 // Height of the horizon

uniform vec3 u_sun_color;            // Color of the sun
uniform vec3 u_sun_position;         // Position of the sun in global coordinates
uniform vec3 u_cam_pos;             // Position of the camera
uniform vec3 u_cam_target;             // Direction of the camera
uniform float u_time;               // Time in seconds
uniform vec2 u_resolution;          // Screen resolution
uniform mat4 u_inverse_view_projection;  // Inverse view + projection matrix

uniform sampler2D u_depth;

in vec2 eyeDirection;
out vec4 fragColor;

#define ITR 30
#define PI 3.14159265359
#define TWO_PI 6.28318530718

struct ray {
    vec3 o;
    vec3 d;
};

int hash( int x ) {
    x += ( x << 10 );
    x ^= ( x >>  6 );
    x += ( x <<  3 );
    x ^= ( x >> 11 );
    x += ( x << 15 );
    return x;
}

int hash( ivec2 v ) { return hash( v.x ^ hash(v.y)                         ); }
int hash( ivec3 v ) { return hash( v.x ^ hash(v.y) ^ hash(v.z)             ); }

// Construct a float with half-open range [0:1] using low 23 bits.
// All zeroes yields 0.0, all ones yields the next smallest representable value below 1.0.
float floatConstruct( int m ) {
    const int ieeeMantissa = 0x007FFFFF; // binary32 mantissa bitmask
    const int ieeeOne      = 0x3F800000; // 1.0 in IEEE binary32

    m &= ieeeMantissa;                     // Keep only mantissa bits (fractional part)
    m |= ieeeOne;                          // Add fractional part to 1.0

    float  f = intBitsToFloat( m );       // Range [1:2]
    return f - 1.0;                        // Range [0:1]
}

float rand( ivec3 p ) {
  return floatConstruct(hash(p));
}

float vnoise(vec3 p) {
    ivec3 u = ivec3(floor(p));
    vec3 v = fract(p);
    vec3 s = smoothstep(0.0, 1.0, v);
    highp float a = rand(u);
    highp float b = rand(u + ivec3(1, 0, 0));
    highp float c = rand(u + ivec3(0, 1, 0));
    highp float d = rand(u + ivec3(1, 1, 0));
    highp float e = rand(u + ivec3(0, 0, 1));
    highp float f = rand(u + ivec3(1, 0, 1));
    highp float g = rand(u + ivec3(0, 1, 1));
    highp float h = rand(u + ivec3(1, 1, 1));
    float r = mix(
      mix(mix(a, b, s.x), mix(c, d, s.x), s.y),
      mix(mix(e, f, s.x), mix(g, h, s.x), s.y),
    s.z);
    return smoothstep(0.0, 1.0, pow(r * 2 - 1, 0.5));
}

float fbm(vec3 p, vec3 targetPos, float t) {
    vec3 off = vec3(10.0, 50.0, 0.1);
    off.z *= t;
    vec3 q = p - off + targetPos;
    float f = 0.0;
    f += 0.5 * vnoise(q);
    q *= 2.0;
    f += 0.25 * vnoise(q);
    q *= 2.0;
    return smoothstep(0.0, 0.8, f);
}

float scene(in vec3 p, float iTime, vec3 targetPos) {
    vec3 q = p - vec3(0.0, 0.0, 1.0) * iTime * 0.4;
    float f = fbm(q, targetPos, iTime);
    return f;
}

float getDepth(vec2 uv) {
    return texture(u_depth, uv).r;
}

vec4 march(vec3 ro, vec3 rd, float iTime, float maxDepth) {
    float t = 0.0;
    float denseD = 10.0;
    float denseE = 1.0;
    vec4 col = vec4(0.0);
    vec3 targetPos = u_cam_target; // Passing the camera target position
    float tLast = 0;
    for (float i = 0.0; i < 1; i += 1.0 / ITR) {
      vec3 p = ro + t * rd;
      float d = scene(p, iTime, targetPos); // Passing the target position to the scene function
      vec4 color = vec4(d);
      color *= pow(d / (denseD + d), denseE);
      if (t > maxDepth) {
        col += color * (1.0 - col.a) * (maxDepth - tLast) / (t - tLast);
        break;
      }
      tLast = t;
      col += color * (1.0 - col.a);
      t += i * 0.5;
    }
    col.rgb = clamp(col.rgb, 0.0, 1.0);
    return col;
}

void main() {
    vec3 sunDirection = vec3(0, sin(u_time * 0.02), -cos(u_time * 0.02));
    vec2 uv_ofs = (gl_FragCoord.xy) / u_resolution;
    vec2 uv = (uv_ofs) * 2 - 1;
    float d = 2.0 * getDepth(uv_ofs) - 1;
    // float d = (2.0 * gl_FragCoord.z - gl_DepthRange.near - gl_DepthRange.far) / (gl_DepthRange.far - gl_DepthRange.near);
    vec4 viewDirWorldH = u_inverse_view_projection * vec4(uv, d, 1.0);
    vec3 viewDirWorld = vec3(viewDirWorldH) / viewDirWorldH.w;
    float vLength = length(viewDirWorld);
    viewDirWorld /= vLength;

    // Generate ray direction
    ray r;
    r.o = u_cam_pos;
    r.d = viewDirWorld;
        
    // Perform ray marching
    vec4 col = march(r.o, r.d, u_time * 0.5, vLength);
        
    // Apply fog effect
    float fog = 0.5 * col.a;
    vec3 colr = col.rgb * vec3(1.0, 0.75, 0.4);
    colr = (0.9 * pow(max(sunDirection.y + 0.1, 0), 0.25) + 0.1) * colr * 1.2 * exp(-0.01 * col.a * col.a);
        
    fragColor = vec4(colr, (0.2 + 0.5 * col.a));
}