#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;
uniform float u_time;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  vec3 l = vec3(0.0, sin(u_time * 0.02), -cos(u_time * 0.02));

  // arbitrary diffuse coefficient
  float k_d = 0.5;

  // arbitrary ambient coefficient
  float k_a = (0.2 * pow(max(l.y + 0.1, 0), 0.25) + 0.1);

  // arbitrary specular coefficient
  float k_s = 0.5;

  // arbitrary p value
  float p = 32.0;

  // find normal and cos_theta_nl
  vec3 n = normalize(vec3(v_normal));
  float cos_theta_nl = dot(n, l);
  cos_theta_nl = max(0, cos_theta_nl);
  cos_theta_nl += max(0, 0.2 * n.y);

  // calculate cos_theta_nh, first find h
  vec3 h = l + normalize(u_cam_pos - vec3(v_position));
  h = normalize(h);
  float cos_theta_nh = dot(n, h);
  if (cos_theta_nl == 0.0) {
    cos_theta_nh = 0.0;
  }
  cos_theta_nh = max(0, cos_theta_nh);
  
  // calculate I/r^2
  vec4 I_r2 = vec4(u_light_intensity, 0);

  // apply the phong model
  out_color = (0.9 * pow(max(l.y + 0.1, 0), 0.25) + 0.1) * ((k_a * u_color) + (k_d * I_r2 * u_color * cos_theta_nl) + (k_s * I_r2 * pow(cos_theta_nh, p)));

  out_color.a = 1;
}

