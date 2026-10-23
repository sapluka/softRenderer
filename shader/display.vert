#version 330 core

out vec2 uv;

const vec2 positions[6] = vec2[6](
    vec2(-1,-1), vec2(1,-1), vec2(1,1),
    vec2(-1,-1), vec2(1,1), vec2(-1,1));

void main() {
    vec2 p = positions[gl_VertexID];
    gl_Position = vec4(p, 0, 1);
    uv = vec2(p.x * 0.5 + 0.5, 0.5 - p.y * 0.5);
}
