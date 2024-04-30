#version 330 core

const vec3 u_skyBottomColor = vec3(0.1, 0.4, 0.90);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);    // Color of the sky at the top
const float u_horizonHeight = 10.0;                 // Height of the horizon

uniform vec3 u_sun_color;            // Color of the sun
uniform vec3 u_sun_position;         // Position of the sun in global coordinates
uniform vec3 u_cam_pos;             // Position of the camera
uniform vec3 u_cam_target;             // Direction of the camera
uniform float u_time;               // Time in seconds
uniform vec2 u_resolution;          // Screen resolution

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
    vec3 q = vec3(12.345, 67.89, 412.12);
    return fract(sin(dot(p, q)) * 42123.45) * 2.0 - 1.0;
}

float perlin(vec3 p) {
    vec3 u = floor(p);
    vec3 v = fract(p);
    vec3 s = smoothstep(0.0, 1.0, v);
    float a = rand(u + a1);
    float b = rand(u + b1);
    float c = rand(u + c1);
    float d = rand(u + d1);
    float e = rand(u + e1);
    float f = rand(u + f1);
    float g = rand(u + g1);
    float h = rand(u + h1);
    float r = mix(
        mix(mix(a, b, s.x), mix(c, d, s.x), s.y),
        mix(mix(e, f, s.x), mix(g, h, s.x), s.y),
        s.z
    );
    return smoothstep(0.0, 1.0, pow(r, 0.5));
}

float fbm(vec3 p, vec3 targetPos, float iTime) {
    vec3 off = vec3(10.0, 50.0, 0.1);
    off.z *= iTime;
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
    vec3 sunDirection = normalize(u_cam_pos - u_sun_position);
    vec3 viewDir = normalize(vec3(eyeDirection, 1.0));
    
    // Calculate the relative size of the sun based on its distance from the camera
    float sunDistance = length(u_cam_pos - u_sun_position);
    float sunSize = 0.00029;
    
    // Calculate the phase factor for sun color modulation
    float phaseFactor = 0.5 + 0.5 * sin(u_time);
    
    // Calculate the dot product between view direction and sun direction
    float dotProduct = dot(viewDir, sunDirection);

    if (dotProduct > 1.0 - sunSize) { 
        // If the dot product exceeds 1 - sunSize, the sun is visible
        fragColor = vec4(u_sun_color * phaseFactor, 1.0);
    } else {
        // Interpolate sky color based on vertical position
        float t = gl_FragCoord.y / u_resolution.y;
        vec3 skyColor = mix(u_skyBottomColor, u_skyTopColor, t);
        
        // Calculate UV coordinates
        vec2 uv = (gl_FragCoord.xy - 0.5 * u_resolution.xy) / u_resolution.y;
        vec2 m = u_resolution.xy / u_resolution.xy;
        
        // Generate ray direction
        ray r = raydir(uv, m, u_time);
        
        // Perform ray marching
        vec4 col = march(r.o, r.d, u_time);
        
        // Apply fog effect
        float fog = 0.5 - 0.5 * col.a;
        vec3 colr = col.rgb;
        colr = mix(vec3(col.rgb) * 1.2 * exp(-0.01 * col.a * col.a), skyColor, fog);
        
        fragColor = vec4(colr, 1.0);
    }
}