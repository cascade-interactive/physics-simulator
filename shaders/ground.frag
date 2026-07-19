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

layout(set = 0, binding = 1) uniform sampler2D baseColorMap;
layout(set = 0, binding = 2) uniform sampler2D normalMap;
layout(set = 0, binding = 3) uniform sampler2D aoRoughnessMap;
layout(set = 0, binding = 4) uniform sampler2DArrayShadow directionalShadowMap;

layout(push_constant) uniform DrawConstants {
    mat4 model;
    vec4 baseColorRoughness;
    uvec4 drawFlags;
} drawData;

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 2) in vec4 worldTangent;
layout(location = 3) in vec2 uv;

layout(location = 0) out vec4 outColor;

float distributionGgx(float nDotH, float roughness) {
    float alpha = roughness * roughness;
    float alphaSquared = alpha * alpha;
    float denominator = nDotH * nDotH * (alphaSquared - 1.0) + 1.0;
    return alphaSquared / max(PI * denominator * denominator, 1.0e-7);
}

float geometrySchlickGgx(float nDotDirection, float roughness) {
    float radius = roughness + 1.0;
    float k = (radius * radius) / 8.0;
    return nDotDirection / max(nDotDirection * (1.0 - k) + k, 1.0e-7);
}

float geometrySmith(float nDotV, float nDotL, float roughness) {
    return geometrySchlickGgx(nDotV, roughness)
        * geometrySchlickGgx(nDotL, roughness);
}

vec3 fresnelSchlick(float cosTheta, vec3 reflectanceAtNormalIncidence) {
    return reflectanceAtNormalIncidence
        + (1.0 - reflectanceAtNormalIncidence) * pow(1.0 - cosTheta, 5.0);
}

vec3 tangentSpaceNormal() {
    vec3 sampledNormal = texture(normalMap, uv).xyz * 2.0 - 1.0;
    if (frame.flags.x != 0u) {
        sampledNormal.y = -sampledNormal.y;
    }

    sampledNormal.xy *= max(frame.materialSettings.z, 0.0);
    sampledNormal = normalize(sampledNormal);

    vec3 normal = normalize(worldNormal);
    vec3 tangent = worldTangent.xyz - normal * dot(normal, worldTangent.xyz);
    tangent = normalize(tangent);
    vec3 bitangent = normalize(cross(normal, tangent)) * worldTangent.w;
    return normalize(mat3(tangent, bitangent, normal) * sampledNormal);
}

vec3 acesFilm(vec3 color) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((color * (a * color + b)) / (color * (c * color + d) + e), 0.0, 1.0);
}

vec3 evaluatePbr(
    vec3 normal,
    vec3 viewDirection,
    vec3 lightDirection,
    vec3 baseColor,
    float roughness
) {
    float nDotV = max(dot(normal, viewDirection), 0.0);
    float nDotL = max(dot(normal, lightDirection), 0.0);
    if (nDotV <= 0.0 || nDotL <= 0.0) {
        return vec3(0.0);
    }

    vec3 halfwayVector = viewDirection + lightDirection;
    float halfwayLengthSquared = dot(halfwayVector, halfwayVector);
    vec3 halfwayDirection = halfwayLengthSquared > 1.0e-8
        ? halfwayVector * inversesqrt(halfwayLengthSquared)
        : normal;
    float nDotH = max(dot(normal, halfwayDirection), 0.0);
    float hDotV = max(dot(halfwayDirection, viewDirection), 0.0);

    const vec3 f0 = vec3(0.04);
    vec3 fresnel = fresnelSchlick(hDotV, f0);
    float distribution = distributionGgx(nDotH, roughness);
    float geometry = geometrySmith(nDotV, nDotL, roughness);
    vec3 specular =
        (distribution * geometry * fresnel) / max(4.0 * nDotV * nDotL, 1.0e-5);
    vec3 diffuse = (vec3(1.0) - fresnel) * baseColor / PI;
    return (diffuse + specular) * nDotL;
}

float sampleShadowCascade(
    uint cascade,
    vec3 receiverPosition,
    vec3 normal,
    vec3 lightDirection
) {
    float normalBias = max(frame.shadowSettings.x, 0.0);
    float grazingScale = 0.25 + 0.75 * (1.0 - max(dot(normal, lightDirection), 0.0));
    vec3 biasedPosition = receiverPosition + normal * normalBias * grazingScale;
    vec4 lightClip =
        frame.cascadeViewProjection[cascade] * vec4(biasedPosition, 1.0);
    if (abs(lightClip.w) <= 1.0e-7) {
        return 1.0;
    }

    vec3 projected = lightClip.xyz / lightClip.w;
    vec2 shadowUv = projected.xy * 0.5 + 0.5;
    if (projected.z <= 0.0 || projected.z >= 1.0 ||
        any(lessThan(shadowUv, vec2(0.0))) ||
        any(greaterThan(shadowUv, vec2(1.0)))) {
        return 1.0;
    }

    float texelSize = max(frame.shadowSettings.y, 1.0 / 8192.0);
    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            visibility += textureGrad(
                directionalShadowMap,
                vec4(
                    shadowUv + vec2(float(x), float(y)) * texelSize,
                    float(cascade),
                    projected.z
                ),
                vec2(0.0),
                vec2(0.0)
            );
        }
    }
    return visibility / 9.0;
}

float directionalShadowVisibility(
    vec3 receiverPosition,
    vec3 normal,
    vec3 lightDirection
) {
    const uint shadowsEnabledFlag = 1u << 2u;
    if ((frame.flags.y & shadowsEnabledFlag) == 0u) {
        return 1.0;
    }

    float viewDepth = max(
        dot(
            receiverPosition - frame.cameraPosition.xyz,
            normalize(frame.cameraForwardNear.xyz)
        ),
        0.0
    );
    uint cascadeCount = clamp(frame.flags.w, 1u, 4u);
    if (viewDepth > frame.cascadeSplits[cascadeCount - 1u]) {
        return 1.0;
    }

    uint cascade = 0u;
    while (cascade + 1u < cascadeCount &&
           viewDepth > frame.cascadeSplits[cascade]) {
        ++cascade;
    }

    float visibility =
        sampleShadowCascade(cascade, receiverPosition, normal, lightDirection);
    float previousSplit = cascade == 0u
        ? frame.cameraForwardNear.w
        : frame.cascadeSplits[cascade - 1u];
    float split = frame.cascadeSplits[cascade];
    float blendWidth =
        max((split - previousSplit) * clamp(frame.shadowSettings.z, 0.0, 0.25), 1.0e-4);
    if (cascade + 1u < cascadeCount) {
        float blend = smoothstep(split - blendWidth, split, viewDepth);
        if (blend > 0.0) {
            float nextVisibility = sampleShadowCascade(
                cascade + 1u,
                receiverPosition,
                normal,
                lightDirection
            );
            visibility = mix(visibility, nextVisibility, blend);
        }
    } else {
        // Fade the last cascade into the unshadowed far field instead of
        // exposing the finite shadow distance as a hard line.
        float fade = smoothstep(split - blendWidth, split, viewDepth);
        visibility = mix(visibility, 1.0, fade);
    }
    return visibility;
}

void main() {
    bool isSolidObject = drawData.drawFlags.x != 0u;
    vec3 geometricNormal = normalize(worldNormal);
    // Keep implicit-derivative texture operations outside the material branch.
    // drawFlags is uniform per draw, but this also makes that guarantee explicit
    // to shader tooling and drivers.
    vec3 sampledBaseColor = texture(baseColorMap, uv).rgb;
    vec2 sampledMaterial = texture(aoRoughnessMap, uv).rg;
    vec3 sampledGroundNormal = tangentSpaceNormal();
    vec3 baseColor;
    float ambientOcclusion;
    float roughness;
    vec3 normal;
    if (isSolidObject) {
        baseColor = clamp(
            drawData.baseColorRoughness.rgb,
            vec3(0.0),
            vec3(100.0)
        );
        ambientOcclusion = 1.0;
        roughness = clamp(drawData.baseColorRoughness.a, 0.045, 1.0);
        normal = geometricNormal;
    } else {
        baseColor = sampledBaseColor;
        ambientOcclusion = clamp(sampledMaterial.r, 0.0, 1.0);
        roughness = clamp(
            sampledMaterial.g * max(frame.materialSettings.w, 0.0),
            0.045,
            1.0
        );
        normal = sampledGroundNormal;
    }

    vec3 viewDirection = normalize(frame.cameraPosition.xyz - worldPosition);
    vec3 directLighting = vec3(0.0);

    const uint sunEnabledFlag = 1u << 0u;
    if ((frame.flags.y & sunEnabledFlag) != 0u) {
        vec3 lightDirection = normalize(frame.sunDirectionIntensity.xyz);
        float shadowVisibility = directionalShadowVisibility(
            worldPosition,
            geometricNormal,
            lightDirection
        );
        vec3 sunlightColor =
            max(frame.sunColorEnabled.rgb, vec3(0.0))
            * max(frame.sunDirectionIntensity.w, 0.0);
        directLighting += evaluatePbr(
            normal,
            viewDirection,
            lightDirection,
            baseColor,
            roughness
        ) * sunlightColor * shadowVisibility;
    }

    uint spotLightCount = min(frame.flags.z, 4u);
    for (uint lightIndex = 0u; lightIndex < spotLightCount; ++lightIndex) {
        uint baseIndex = lightIndex * 4u;
        vec4 positionRange = frame.spotLightData[baseIndex + 0u];
        vec4 directionOuterCos = frame.spotLightData[baseIndex + 1u];
        vec4 colorIntensity = frame.spotLightData[baseIndex + 2u];
        vec4 parameters = frame.spotLightData[baseIndex + 3u];
        if (parameters.y < 0.5 || colorIntensity.w <= 0.0 ||
            positionRange.w <= 0.0) {
            continue;
        }

        vec3 toLight = positionRange.xyz - worldPosition;
        float distanceSquared = dot(toLight, toLight);
        float distanceToLight = sqrt(max(distanceSquared, 1.0e-8));
        if (distanceToLight >= positionRange.w) {
            continue;
        }
        vec3 lightDirection = toLight / distanceToLight;
        float coneCosine = dot(normalize(directionOuterCos.xyz), -lightDirection);
        float coneAttenuation = smoothstep(
            directionOuterCos.w,
            parameters.x,
            coneCosine
        );
        coneAttenuation *= coneAttenuation;
        float normalizedDistance = distanceToLight / positionRange.w;
        float rangeAttenuation =
            clamp(1.0 - pow(normalizedDistance, 4.0), 0.0, 1.0);
        rangeAttenuation *= rangeAttenuation;
        float inverseSquare = 1.0 / max(distanceSquared, 0.04);
        vec3 radiance =
            max(colorIntensity.rgb, vec3(0.0))
            * colorIntensity.w
            * coneAttenuation
            * rangeAttenuation
            * inverseSquare;
        directLighting += evaluatePbr(
            normal,
            viewDirection,
            lightDirection,
            baseColor,
            roughness
        ) * radiance;
    }

    const uint environmentEnabledFlag = 1u << 1u;
    vec3 ambientLighting = vec3(0.0);
    if ((frame.flags.y & environmentEnabledFlag) != 0u) {
        ambientLighting =
            baseColor
            * max(frame.environmentColorIntensity.rgb, vec3(0.0))
            * max(frame.environmentColorIntensity.w, 0.0)
            * ambientOcclusion;
    }
    float exposure = max(frame.materialSettings.y, 0.0);
    vec3 linearColor = ambientLighting + directLighting;

    if (isSolidObject) {
        const uint hoveredFlag = 1u << 1u;
        const uint selectedFlag = 1u << 2u;
        const uint grabbedFlag = 1u << 3u;
        uint interactionFlags = drawData.drawFlags.y;
        vec3 highlightColor = vec3(0.0);
        float highlightAmount = 0.0;
        if ((interactionFlags & grabbedFlag) != 0u) {
            highlightColor = vec3(1.0, 0.42, 0.08);
            highlightAmount = 1.0;
        } else if ((interactionFlags & selectedFlag) != 0u) {
            highlightColor = vec3(0.14, 0.57, 1.0);
            highlightAmount = 0.78;
        } else if ((interactionFlags & hoveredFlag) != 0u) {
            highlightColor = vec3(0.16, 0.82, 1.0);
            highlightAmount = 0.5;
        }
        float nDotV = max(dot(normal, viewDirection), 0.0);
        float rim = pow(1.0 - clamp(nDotV, 0.0, 1.0), 2.5);
        linearColor += highlightColor
            * highlightAmount
            * (0.08 + 0.8 * rim);
    }

    vec3 mappedColor = acesFilm(linearColor * exposure);

    // The swapchain uses an sRGB format, so this remains linear here and the
    // Vulkan attachment performs the final sRGB encoding.
    outColor = vec4(mappedColor, 1.0);
}
