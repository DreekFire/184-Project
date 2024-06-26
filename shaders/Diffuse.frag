#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // arbitrary diffuse coefficient
  float k_d = 1.0;
  // find length and radius 
  vec3 l = u_light_pos - vec3(v_position);
  float r = length(l);
  l = normalize(l);
  // find normal and cos_theta
  vec3 n = normalize(vec3(v_normal));
  float cos_theta = dot(n, l);
  // evaluate the diffuse equation
  out_color = k_d * (vec4(u_light_intensity, 0) * u_color / (r * r)) * max(0.0, cos_theta);
  out_color.a = 1;
}
