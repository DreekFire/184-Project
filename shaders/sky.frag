#version 330 core

const vec3 u_skyBottomColor = vec3(0.1, 0.3, 0.8);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);    // Color of the sky at the top
const float u_horizonHeight = 10.0;                 // Height of the horizon

uniform vec3 u_sunPosition;         // Position of the sun in world space
uniform vec3 u_sunColor;            // Color of the sun
uniform vec3 u_cam_pos;             // Position of the camera
uniform mat4 u_view_projection;     // View-projection matrix
uniform float u_time;               // Time in seconds
uniform vec2 u_resolution;          // Screen resolution

in vec2 v_uv;
out vec4 fragColor;

void main()
{
    // Calculate distance from fragment to sun position
    vec2 fragPos = gl_FragCoord.xy / u_resolution;
    vec2 sunPos = vec2(0.5, 0.8); // Set the sun position above the origin
    float distanceToSun = distance(fragPos, sunPos);

    // Check if fragment is within a certain radius of the sun position
    if (distanceToSun < 0.5) { // Adjust the radius as needed
        fragColor = vec4(u_sunColor, 1.0); // Draw the sun color
    } else {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0); // Black color
    }
}


