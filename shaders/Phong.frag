#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // arbitrary diffuse coefficient
  float k_d = 1.0;

  // arbitrary ambient coefficient
  float k_a = 0.3;

  // arbitrary specular coefficient
  float k_s = 0.5;

  // arbitrary p value
  float p = 25.0;

  // find length and radius 
  vec3 l = u_light_pos - vec3(v_position);
  float r = length(l);
  l = normalize(l);

  // find normal and cos_theta_nl
  vec3 n = normalize(vec3(v_normal));
  float cos_theta_nl = dot(n, l);
  cos_theta_nl = max(0, cos_theta_nl);

  // calculate cos_theta_nh, first find h
  vec3 h = l + normalize(u_cam_pos - vec3(v_position));
  h = normalize(h);
  float cos_theta_nh = dot(n, h);
  cos_theta_nh = max(0, cos_theta_nh);
  
  // calculate I/r^2
  vec4 I_r2 = vec4(u_light_intensity, 0) / (r * r);

  // apply the phong model
  out_color = (k_a * u_color) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * u_color * pow(cos_theta_nh, p));

  out_color.a = 1;
}

