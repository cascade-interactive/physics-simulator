#version 450

layout(set = 0, binding = 0, std140) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 cascadeViewProjection[4];
    vec4 cameraPosition;
    vec4 cameraForwardNear;
    vec4 sunDirectionIntensity;
    vec4 sunColorEnabled;
    vec4 environmentColorIntensity;
    vec4 materialSettings;
    vec4 cascadeSplits;
    vec4 shadowSettings;
    uvec4 flags;
    vec4 spotLightData[16];
} frame;

layout(push_constant) uniform DrawConstants {
    mat4 model;
    vec4 baseColorRoughness;
    uvec4 drawFlags;
} drawData;

layout(location = 0) in vec3 inPosition;

void main() {
    uint cascade = min(drawData.drawFlags.w, 3u);
    gl_Position =
        frame.cascadeViewProjection[cascade]
        * drawData.model
        * vec4(inPosition, 1.0);
}
