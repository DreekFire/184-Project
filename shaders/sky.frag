#version 330 core

const vec3 u_skyBottomColor = vec3(0.1, 0.3, 0.8);  // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.5, 0.7, 1.0);    // Color of the sky at the top
const float u_horizonHeight = 10.0;                 // Height of the horizon

uniform vec3 u_sun_color;            // Color of the sun
uniform vec3 u_sun_position;         // Position of the sun in global coordinates
uniform vec3 u_cam_pos;             // Position of the camera
uniform float u_time;               // Time in seconds
uniform vec2 u_resolution;          // Screen resolution

in vec2 eyeDirection;
out vec4 fragColor;

void main() {
    vec3 sunDirection = normalize(u_sun_position - u_cam_pos);
    vec3 viewDir = normalize(vec3(eyeDirection, 1.0));
    
    // Calculate the relatve size of the sun based on its distance from the camera
    float sunDistance = length(u_sun_position - u_cam_pos);
    float sunSize = 0.0099 / sunDistance;
    
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

        fragColor = vec4(skyColor, 1.0); // Set the sky color
    }
}
