#version 400

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

void main() {
    out_color = vec4(0.5, 0.8, 0.3, 1.0);
}

