#version 330 core

const vec3 u_skyBottomColor = vec3(0.1, 0.3, 0.8);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);    // Color of the sky at the top
const float u_horizonHeight = 10.0;                 // Height of the horizon

uniform vec3 u_sunColor;            // Color of the sun
uniform vec3 u_sunPosition;         // Position of the sun in global coordinates
uniform vec3 u_cam_pos;             // Position of the camera
uniform float u_time;               // Time in seconds
uniform vec2 u_resolution;          // Screen resolution

in vec2 eyeDirection;
out vec4 fragColor;

void main() {
    vec3 sunDirection = normalize(u_sunPosition - u_cam_pos);

    float dotProduct = dot(normalize(vec3(eyeDirection, 1.0)), sunDirection);

    if (dotProduct > 0.95) { 
        fragColor = vec4(u_sunColor, 1.0);
    } else {
        // Interpolate sky color based on vertical position
        float t = gl_FragCoord.y / u_resolution.y;
        vec3 skyColor = mix(u_skyBottomColor, u_skyTopColor, t);

        fragColor = vec4(skyColor, 1.0); // Set the sky color
    }
}
