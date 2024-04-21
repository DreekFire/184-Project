#version 330

const vec3 u_skyBottomColor = vec3(0.5, 0.7, 1.0); // Color of the sky near the horizon
const vec3 u_skyTopColor = vec3(0.1, 0.3, 0.8);    // Color of the sky at the top
const float u_horizonHeight = 0.25;                // Height of the horizon

uniform vec3 u_sunPosition;     // Position of the sun in world space
uniform vec3 u_sunColor;        // Color of the sun

out vec4 fragColor;

void main() {
    // Calculate the direction to the sun
    vec3 sunDir = normalize(u_sunPosition - gl_FragCoord.xyz);

    // Calculate the direction from the fragment to the camera
    vec3 viewDir = normalize(-gl_FragCoord.xyz);

    // Check if the sun is visible from the camera's perspective
    float visibility = max(dot(sunDir, viewDir), 0.0);

    // Interpolate between sky colors based on the angle
    vec3 skyColor = mix(u_skyBottomColor, u_skyTopColor, visibility);

    // Calculate the distance from the current fragment to the sun
    float distanceToSun = length(u_sunPosition - gl_FragCoord.xyz);

    // Calculate the size of the sun based on the distance
    float sunSize = max(1.0 - distanceToSun * 0.01, 0.0); // Adjust size as needed

    // Calculate the brightness of the sun based on the distance
    float sunBrightness = max(1.0 - distanceToSun * 0.005, 0.0); // Adjust brightness as needed

    // Calculate the color of the sun based on its brightness and color
    vec3 sunColorAdjusted = u_sunColor * sunBrightness;

    // Add the sun to the sky color only if it's visible
    skyColor += sunColorAdjusted * sunSize * visibility;

    // Set the final color
    fragColor = vec4(skyColor, 1.0);
}

