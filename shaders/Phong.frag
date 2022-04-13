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
  // YOUR CODE HERE
  float k_a = 0.3;
  float k_d = 1.0;
  float k_s = 0.8;
  vec3 i_a = vec3(1.0, 1.0, 1.0);
  float l = length(u_light_pos - v_position.xyz);
  float p = 20.0;
  vec3 hnum = (u_light_pos - v_position.xyz) + (u_cam_pos - v_position.xyz);
  vec3 h = hnum / length((u_light_pos - v_position.xyz) + (u_cam_pos - v_position.xyz));

  vec3 ambient = k_a * i_a;
  vec3 diffuse = k_d * (u_light_intensity / pow(length(l),2)) * max(0, dot(v_normal.xyz, normalize(u_light_pos - v_position.xyz))) ;
  vec3 specular = k_s * (u_light_intensity / pow(length(l),2)) * pow(max(0, dot(v_normal.xyz, h)), p);

  // (Placeholder code. You will want to replace it.)
  out_color = vec4(specular + ambient + diffuse, 0);
  out_color.a = 1;
}

