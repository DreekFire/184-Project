#version 330

const vec3 u_skyBottomColor = vec3(0.1, 0.3, 0.8);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);   // Color of the sky at the top
const float u_horizonHeight = 10;                // Height of the horizon

uniform vec3 u_sunPosition;       // Position of the sun in world space
uniform vec3 u_sunColor;          // Color of the sun
uniform vec3 u_cam_pos;           // Position of the camera
uniform mat4 u_view_projection;   // View-projection matrix
uniform float u_time;             // Time in seconds
uniform vec2 u_resolution;        // Screen resolution

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;


out vec4 fragColor;

#define OCTAVES_LARGE 4
#define OCTAVES_SMALL 8

float fade(float t) {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(12.9898,78.233))) * 43758.5453);
}

float noise(in vec2 xy) {
    vec2 i = floor(xy);
    vec2 f = fract(xy);

    // Smooth the coordinate values
    vec2 u = f * f * (3.0 - 2.0 * f);

    // Hash the grid corners
    float a = hash(i + vec2(0.0, 0.0));
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));

    // Interpolate along x and y
    float x1 = mix(a, b, u.x);
    float x2 = mix(c, d, u.x);
    return mix(x1, x2, u.y);
}

float fbm(in vec2 uv) {
    float value_large = 0.;
    float amplitude_large = 0.5;
    float value_small = 0.;
    float amplitude_small = 0.5;

    // Large scale Perlin noise
    for (int i = 0; i < OCTAVES_LARGE; i++) {
        value_large += noise(uv) * amplitude_large;
        amplitude_large *= 0.5;
        uv *= 2.;
    }

    // Small scale Perlin noise
    uv *= 8.; // Adjust scale for small noise
    for (int i = 0; i < OCTAVES_SMALL; i++) {
        value_small += noise(uv) * amplitude_small;
        amplitude_small *= 0.5;
        uv *= 2.;
    }

    // Multiply large and small scale noise
    return value_large * value_small;
}

vec3 Sky(in vec3 ro, in vec3 rd, float time, vec2 resolution, vec3 sunPosition) {
    const float SC = 1e5;
    float dist = (SC - ro.y) / rd.y; 
    vec2 p = (ro + dist * rd).xz;
    p *= 1.2 / SC;
    
    vec3 lightDir = normalize(sunPosition - ro); // Calculate direction to the sun
    float sundot = clamp(dot(rd, lightDir), 0.0, 1.0);
    
    // Add sun as a white disk
    vec3 sun = vec3(1.0) * smoothstep(0.0, 0.1, sundot); // White disk at the position of the sun
    
    // Calculate cloud contribution
    float t = time * 0.1;
    float den = fbm(vec2(p.x - t, p.y - t));
    
    // Mix sun and cloud contribution
    vec3 cloudCol = vec3(1.);
    vec3 skyCol = vec3(0.3, 0.5, 0.85) - rd.y * rd.y * 0.5;
    skyCol = mix(skyCol, 0.85 * vec3(0.7, 0.75, 0.85), pow(1.0 - max(rd.y, 0.0), 4.0));
    skyCol = mix(skyCol, cloudCol, smoothstep(.4, .8, den));
    
    // Add sun to the sky color
    skyCol += sun;
    
    skyCol = mix(skyCol, 0.68 * vec3(.418, .394, .372), pow(1.0 - max(rd.y, 0.0), 16.0));
    
    return skyCol;
}




mat3 setCamera(in vec3 ro, in vec3 ta, float cr) {
    vec3 cw = normalize(ta - ro);
    vec3 cp = vec3(sin(cr), cos(cr), 0.0);
    vec3 cu = normalize(cross(cw, cp));
    vec3 cv = normalize(cross(cu, cw));
    return mat3(cu, cv, cw);
}

void main() {
    vec2 uv = gl_FragCoord.xy / u_resolution;
    uv -= 0.5;
    uv.x *= u_resolution.x / u_resolution.y;
  
    vec3 ro = vec3(0.0, 0.0, 0.0);
    vec3 ta = vec3(cos(u_time * 0.001), 0.5, sin(u_time * 0.001)); // Adjust the frequency of oscillation
    vec3 rd = normalize(ta - ro); // Calculate camera direction
    
    vec3 col = Sky(ro, rd, u_time, u_resolution, u_sunPosition);
    
    fragColor = vec4(vec3(col), 1.0);
}
