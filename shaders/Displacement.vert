#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_4;
uniform vec2 u_texture_4_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // returns the R channel value of the texture at the given uv
  return texture(u_texture_4, uv).r;
}

void main() {
  float height_scaling = u_height_scaling;

  // keep this as is
  v_position = u_model * in_position;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);

  // recalculate the position based on the height map
  vec4 v_pos_modifier = (in_position + in_normal * h(in_uv) * height_scaling);

  gl_Position = u_view_projection * u_model * v_pos_modifier;
}
