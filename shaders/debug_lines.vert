#version 450

layout(set = 0, binding = 0, std140) uniform FrameUniforms {
    mat4 viewProjection;
} frame;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 color;

void main() {
    color = inColor;
    gl_Position = frame.viewProjection * vec4(inPosition, 1.0);
}
