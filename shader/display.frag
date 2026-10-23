#version 330 core

in vec2 uv;
out vec4 color;
uniform sampler2D screenTexture;

void main() {
    color = vec4(texture(screenTexture, uv).rgb, 1);
}
