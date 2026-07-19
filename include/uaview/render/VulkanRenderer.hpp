#pragma once

#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

struct GLFWwindow;

namespace uaview::render {

class Camera;

inline constexpr std::size_t kMaximumSpotLights = 4U;
inline constexpr std::uint32_t kDirectionalShadowCascadeCount = 4U;

namespace CubeRenderFlags {

inline constexpr std::uint32_t visible = 1U << 0U;
inline constexpr std::uint32_t hovered = 1U << 1U;
inline constexpr std::uint32_t selected = 1U << 2U;
inline constexpr std::uint32_t grabbed = 1U << 3U;
inline constexpr std::uint32_t showBounds = 1U << 4U;

} // namespace CubeRenderFlags

// Renderer-owned, trivially copyable scene data. bodyToWorld contains rotation
// and translation; halfExtents supplies the cube's physical dimensions.
struct CubeRenderData {
    glm::mat4 bodyToWorld{1.0F};
    glm::vec3 halfExtents{0.5F};
    float roughness{0.48F};
    glm::vec3 baseColor{0.19F, 0.42F, 0.72F};
    std::uint32_t flags{CubeRenderFlags::visible};
    std::uint32_t highlightedVertexMask{0U};
    std::uint32_t reserved0{0U};
    std::uint64_t objectId{0U};
};

struct DebugLineRenderData {
    glm::vec3 start{0.0F};
    float reserved0{0.0F};
    glm::vec3 end{0.0F};
    float reserved1{0.0F};
    glm::vec4 color{1.0F};
};

namespace SpotLightRenderFlags {

inline constexpr std::uint32_t enabled = 1U << 0U;

} // namespace SpotLightRenderFlags

// Renderer-owned, unshadowed editor light. direction points away from the
// emitter, along the center of the cone. Angles are half-angles in degrees.
struct SpotLightRenderData {
    glm::vec3 position{0.0F, 3.0F, 0.0F};
    float range{12.0F};
    glm::vec3 direction{0.0F, -1.0F, 0.0F};
    float outerConeDegrees{32.0F};
    glm::vec3 color{1.0F, 0.92F, 0.78F};
    float intensity{0.0F};
    float innerConeDegrees{22.0F};
    std::uint32_t flags{0U};
    std::uint32_t reserved0{0U};
    std::uint32_t reserved1{0U};
};

// Pointer storage is never retained. setSceneData copies this view immediately,
// which permits physics to publish a compact snapshot without sharing headers.
struct SceneRenderDataView {
    const CubeRenderData* cubes{nullptr};
    std::size_t cubeCount{0U};
    const DebugLineRenderData* debugLines{nullptr};
    std::size_t debugLineCount{0U};
    const SpotLightRenderData* spotLights{nullptr};
    std::size_t spotLightCount{0U};
};

// Framebuffer-pixel rectangle for the central 3D scene. A zero width or height
// selects the full swapchain, which remains the safe startup/default behavior.
struct SceneViewport {
    std::int32_t x{0};
    std::int32_t y{0};
    std::uint32_t width{0U};
    std::uint32_t height{0U};
};

struct GroundRenderSettings {
    float sunAzimuthDegrees{-38.0F};
    float sunElevationDegrees{48.0F};
    float sunIntensity{5.0F};
    glm::vec3 sunColor{1.0F, 0.955F, 0.88F};
    bool sunEnabled{true};
    float ambientStrength{0.055F};
    glm::vec3 environmentColor{0.72F, 0.80F, 1.0F};
    bool environmentEnabled{true};
    float exposure{1.05F};
    float normalStrength{1.0F};
    float roughnessScale{1.0F};
    bool flipNormalGreen{false};
    bool sunShadowsEnabled{true};
    float shadowDistance{180.0F};
    float shadowSplitLambda{0.72F};
    float shadowNormalBias{0.018F};
    float shadowDepthBiasConstant{1.25F};
    float shadowDepthBiasSlope{1.75F};
};

struct RendererDiagnostics {
    std::string gpuName{"Not initialized"};
    std::string apiVersion{"-"};
    std::string presentMode{"-"};
    std::uint32_t swapchainWidth{0};
    std::uint32_t swapchainHeight{0};
    std::uint32_t textureWidth{0};
    std::uint32_t textureHeight{0};
    std::uint32_t textureMipLevels{0};
    std::uint64_t renderedFrames{0};
    std::uint32_t validationWarnings{0};
    std::uint32_t validationErrors{0};
    std::uint32_t visibleCubeCount{0};
    std::uint32_t debugLineCount{0};
    float rendererCpuMilliseconds{0.0F};
    bool validationEnabled{false};
};

class VulkanRenderer {
public:
    VulkanRenderer();
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;
    VulkanRenderer(VulkanRenderer&&) noexcept;
    VulkanRenderer& operator=(VulkanRenderer&&) noexcept;

    void initialize(
        GLFWwindow* window,
        const std::filesystem::path& executableDirectory,
        bool requestValidation
    );
    void initializeImGui(GLFWwindow* window);
    void beginUiFrame();
    void setSceneData(const SceneRenderDataView& scene);
    void setSceneViewport(const SceneViewport& viewport) noexcept;
    void drawFrame(const Camera& camera, const GroundRenderSettings& settings);
    void requestSwapchainRebuild() noexcept;
    void waitIdle();
    void shutdown();

    [[nodiscard]] bool isInitialized() const noexcept;
    [[nodiscard]] const RendererDiagnostics& diagnostics() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uaview::render
