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

in vec2 eyeDirection;
out vec4 fragColor;

#define ITR 30
#define PI 3.14159265359
#define TWO_PI 6.28318530718

const vec3 a1 = vec3(0. );
const vec3 b1 = vec3(1.0, 0.0, 0.0);
const vec3 c1 = vec3(0.0, 1.0, 0.0);
const vec3 d1 = vec3(1.0, 1.0, 0.0);
const vec3 e1 = vec3(0.0, 0.0, 1.0);
const vec3 f1 = vec3(1.0, 0.0, 1.0);
const vec3 g1 = vec3(0.0, 1.0, 1.0);
const vec3 h1 = vec3(1.0, 1.0, 1.0);

mat3 RX(float a) {
    return mat3(
        1.0, 0.0, 0.0,
        0.0, cos(a), -sin(a),
        0.0, sin(a), cos(a)
    );
}

mat3 RY(float a) {
    return mat3(
        cos(a), 0.0, sin(a),
        0.0, 1.0, 0.0,
        -sin(a), 0.0, cos(a)
    );
}

mat3 RZ(float a) {
    return mat3(
        cos(a), -sin(a), 0.0,
        sin(a), cos(a), 0.0,
        0.0, 0.0, 1.0
    );
}

struct ray {
    vec3 o;
    vec3 d;
};

ray raydir(vec2 uv, vec2 m, float iTime) {
    float ang = sin(TWO_PI * m.x) * 1.0;
    vec3 camR = vec3(sin(ang + m.x), 0.0, cos(ang + m.y));
    vec3 ro = vec3(15.0, 5.0, 15.0) + camR;
    ro *= RX(-m.x * TWO_PI + iTime * 0.001);
    ro *= RY(-m.y * TWO_PI + iTime * 0.001);
    ro *= RZ(-iTime * 0.01);
    vec3 lookat = vec3(0.0);
    float zoom = 1.0;
    vec3 forward = normalize(lookat - ro);
    vec3 temp = cross(vec3(0.0, 1.0, 0.0), forward);
    vec3 up = normalize(cross(forward, temp));
    vec3 right = cross(up, forward);
    vec3 screen_center = ro + forward * zoom;
    vec3 i = screen_center + uv.x * right + uv.y * up;
    vec3 rd = i - ro;
    ray r;
    r.o = ro;
    r.d = rd;
    return r;
}


float rand(vec3 p) {
    highp float a = 1.9898;
    highp float b = 7.233;
    highp float c = 6.2634;
    highp float d = 60;
    highp float dt= dot(floor(p - vec3(0.5)), vec3(a,b,c));
    highp float sn= mod(dt, PI);
    return fract(sin(sn) * d);
}

float perlin(vec3 p) {
    vec3 u = floor(p);
    vec3 v = p - u;
    vec3 s = smoothstep(0.0, 1.0, v);
    highp float a = rand(u);
    highp float b = rand(u + vec3(1, 0, 0));
    highp float c = rand(u + vec3(0, 1, 0));
    highp float d = rand(u + vec3(1, 1, 0));
    highp float e = rand(u + vec3(0, 0, 1));
    highp float f = rand(u + vec3(1, 0, 1));
    highp float g = rand(u + vec3(0, 1, 1));
    highp float h = rand(u + vec3(1, 1, 1));
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
    f += 0.5 * perlin(q);
    q *= 2.0;
    f += 0.25 * perlin(q);
    q *= 2.0;
    return smoothstep(0.0, 0.8, f);
}

float scene(in vec3 p, float iTime, vec3 targetPos) {
    vec3 q = p - vec3(0.0, 0.0, 1.0) * iTime * 0.4;
    float f = fbm(q, targetPos, iTime);
    return f;
}

vec4 march(vec3 ro, vec3 rd, float iTime) {
    float t = 0.0;
    float denseD = 10.0;
    float denseE = 1.0;
    vec4 col = vec4(0.0);
    vec3 targetPos = u_cam_target; // Passing the camera target position
    for (float i = 0.0; i < 1.0; i += 1.0 / ITR) {
        vec3 p = ro + t * rd;
        float d = scene(p, iTime, targetPos); // Passing the target position to the scene function
        vec4 color = vec4(d);
        color *= pow(d / (denseD + d), denseE);
        col += color * (1.0 - col.a);
        t += i * 0.5;
    }
    col.rgb = clamp(col.rgb, 0.0, 1.0);
    return col;
}


void main() {
    vec3 sunDirection = vec3(0, sin(u_time * 0.02), -cos(u_time * 0.02));
    vec2 uv = ((gl_FragCoord.xy) / u_resolution) * 2 - 1;
    //(2.0 * gl_FragCoord.z - gl_DepthRange.near - gl_DepthRange.far) /
    //(gl_DepthRange.far - gl_DepthRange.near);
    vec4 viewDirWorldH = u_inverse_view_projection * vec4(uv, gl_FragCoord.z, 1.0);
    vec3 viewDirWorld = normalize(vec3(viewDirWorldH) / viewDirWorldH.w);
    // vec3 viewDir = normalize(vec3(eyeDirection, 1.0));

    float sunSize = 0.00029;
    
    // Calculate the phase factor for sun color modulation
    float phaseFactor = 1 + 0.1 * sin(u_time * 0.5);
    
    // Calculate the dot product between view direction and sun direction
    float dotProduct = max(dot(viewDirWorld, sunDirection), 0.0);

    if (dotProduct > 1.0 - sunSize) { 
        // If the dot product exceeds 1 - sunSize, the sun is visible
        fragColor = vec4(1.0, 1.0, 1.0, 1.0);
    } else {
        // Interpolate sky color based on vertical position
        float t = gl_FragCoord.y / u_resolution.y;
        vec3 skyColor = mix(u_skyBottomColor, u_skyTopColor, t);
        skyColor = (0.9 * pow(max(sunDirection.y + 0.1, 0), 0.25) + 0.1) * mix(skyColor, u_skySunColor, pow(2.718, -4 * (sunDirection.y + 0.1)) * dotProduct);
        
        /*// Generate ray direction
        ray r;
        r.o = u_cam_pos;
        r.d = viewDirWorld;
        
        // Perform ray marching
        vec4 col = march(r.o, r.d, u_time * 0.25);
        
        // Apply fog effect
        float fog = 0.5 - 0.5 * col.a;
        vec3 colr = col.rgb;
        colr = (0.9 * pow(max(sunDirection.y + 0.1, 0), 0.25) + 0.1) * mix(colr * 1.2 * exp(-0.01 * col.a * col.a), skyColor, fog);*/
        
        float fac = pow(2.718, 256 * (pow(dotProduct + sunSize, 2) - 1)) * phaseFactor;
        fragColor = (1 - fac) * vec4(skyColor, 1.0) + fac * vec4(u_sun_color, 1.0);
    }
}