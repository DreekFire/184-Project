#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // get w_out 
  vec3 w_out = u_cam_pos - v_position.xyz;

  // get normal 
  vec3 normal = normalize(v_normal.xyz);

  // calculate w_in
  vec3 w_in = w_out - 2.0 * dot(w_out, normal) * normal;
  // flip w_in to match the actual light direction
  w_in = -w_in;

  // get color
  out_color = texture(u_texture_cubemap, w_in);
}
