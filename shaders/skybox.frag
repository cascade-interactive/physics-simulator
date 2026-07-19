#version 450

const float PI = 3.14159265358979323846;

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

layout(set = 0, binding = 5) uniform sampler2D environmentMap;

layout(location = 0) in vec3 worldDirection;
layout(location = 0) out vec4 outColor;

vec3 acesFilm(vec3 color) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp(
        (color * (a * color + b)) /
            (color * (c * color + d) + e),
        0.0,
        1.0
    );
}

void main() {
    vec3 direction = normalize(worldDirection);

    vec2 environmentUv = vec2(
        atan(direction.z, direction.x) / (2.0 * PI) + 0.5,
        acos(clamp(direction.y, -1.0, 1.0)) / PI
    );
    vec3 radiance = max(texture(environmentMap, environmentUv).rgb, vec3(0.0));
    vec3 mappedColor = acesFilm(
        radiance * max(frame.materialSettings.y, 0.0)
    );
    outColor = vec4(mappedColor, 1.0);
}
