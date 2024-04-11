#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_4;
uniform vec2 u_texture_4_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // returns the R channel value of the texture at the given uv
  return texture(u_texture_4, uv).r;
}

void main() {
  // Bump Mapping
  // find the normal given the texture
  float t_width = u_texture_4_size.x;
  float t_height = u_texture_4_size.y;

  // find tangent vector
  vec3 t = normalize(vec3(v_tangent));

  // create B vector (cross product of normal and tangent)
  vec3 b = cross(normalize(vec3(v_normal)), t);

  // create TBN matrix
  mat3 TBN = mat3(t, b, normalize(vec3(v_normal)));

  // get current u and v values
  float u = v_uv.x;
  float v = v_uv.y;

  // grab scaling values
  float normal_scaling = u_normal_scaling;
  float height_scaling = u_height_scaling;

  // calculate dU and dV
  float dU = (h(vec2(u + normal_scaling / t_width, v)) - h(vec2(u, v))) / (height_scaling * normal_scaling);
  float dV = (h(vec2(u, v + normal_scaling / t_height)) - h(vec2(u, v))) / (height_scaling * normal_scaling);

  // local space normal 
  vec3 local_normal = vec3(-dU, -dV, 1.0);

  // calculate displaced normal
  vec3 n_d = normalize(TBN * local_normal);


  // Phong Shading
  // arbitrary diffuse coefficient
  float k_d = 1.0;

  // arbitrary ambient coefficient
  float k_a = 1.0;

  // arbitrary ambient intensity
  vec4 I_a = vec4(0.1, 0.1, 0.1, 0);

  // arbitrary specular coefficient
  float k_s = 0.5;

  // arbitrary p value
  float p = 25.0;

  // find length and radius 
  vec3 l = u_light_pos - vec3(v_position);
  float r = length(l);
  l = normalize(l);

  // cos_theta_nl
  float cos_theta_nl = dot(n_d, l);
  cos_theta_nl = max(0, cos_theta_nl);

  // calculate cos_theta_nh, first find h
  vec3 h = l + normalize(u_cam_pos - vec3(v_position));
  h = normalize(h);
  float cos_theta_nh = dot(n_d, h);
  cos_theta_nh = max(0, cos_theta_nh);
  
  // calculate I/r^2
  vec4 I_r2 = vec4(u_light_intensity, 0) / (r * r);

  // apply the phong model
    out_color = (k_a * I_a) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * u_color * pow(cos_theta_nh, p));

  out_color.a = 1;
}

