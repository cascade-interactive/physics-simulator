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

layout(location = 0) out vec3 worldDirection;

void main() {
    const vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2(3.0, -1.0),
        vec2(-1.0, 3.0)
    );
    vec2 clipPosition = positions[gl_VertexIndex];
    vec4 farWorld = inverse(frame.viewProjection) *
        vec4(clipPosition, 1.0, 1.0);
    float reciprocalW = abs(farWorld.w) > 1.0e-7
        ? 1.0 / farWorld.w
        : 0.0;
    worldDirection =
        farWorld.xyz * reciprocalW - frame.cameraPosition.xyz;
    gl_Position = vec4(clipPosition, 0.999999, 1.0);
}
