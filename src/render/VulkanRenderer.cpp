#include <uaview/render/VulkanRenderer.hpp>

#include <uaview/render/Camera.hpp>

#include <vulkan/vulkan.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <VkBootstrap.h>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4100 4189 4324)
#endif
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_vulkan.h>
#include <imgui.h>
#include <stb_image.h>

#include <glm/geometric.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/packing.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace uaview::render {
namespace {

constexpr std::uint32_t kFramesInFlight = 2U;
constexpr std::uint32_t kVulkanApiVersion = VK_API_VERSION_1_3;
constexpr const char* kValidationLayerName = "VK_LAYER_KHRONOS_validation";
constexpr std::size_t kMaxRenderedCubes = 4096U;
constexpr std::size_t kMaxDebugLines = 16384U;
constexpr std::size_t kMaximumGeneratedLinesPerCube = 36U;
constexpr std::uint32_t kShadowMapResolution = 2048U;
constexpr float kCameraNearPlane = 0.05F;
constexpr float kCameraFarPlane = 2'000.0F;
constexpr float kMinimumShadowDistance = 5.0F;
constexpr float kCascadeBlendFraction = 0.08F;
constexpr std::uint32_t kFrameFlagSunEnabled = 1U << 0U;
constexpr std::uint32_t kFrameFlagEnvironmentEnabled = 1U << 1U;
constexpr std::uint32_t kFrameFlagSunShadowsEnabled = 1U << 2U;

void requireVk(VkResult result, const char* operation) {
    if (result == VK_SUCCESS) {
        return;
    }
    std::ostringstream message;
    message << operation << " failed with VkResult " << static_cast<int>(result) << '.';
    throw std::runtime_error(message.str());
}

template <typename Function>
class ScopeExit final {
public:
    explicit ScopeExit(Function function)
        : function_(std::move(function)) {}

    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;

    ~ScopeExit() noexcept {
        if (active_) {
            function_();
        }
    }

    void release() noexcept {
        active_ = false;
    }

private:
    Function function_;
    bool active_{true};
};

template <typename Function>
auto makeScopeExit(Function&& function) {
    return ScopeExit<std::decay_t<Function>>(
        std::forward<Function>(function)
    );
}

std::string versionString(std::uint32_t version) {
    return std::to_string(VK_API_VERSION_MAJOR(version)) + "." +
           std::to_string(VK_API_VERSION_MINOR(version)) + "." +
           std::to_string(VK_API_VERSION_PATCH(version));
}

std::string presentModeName(VkPresentModeKHR mode) {
    switch (mode) {
    case VK_PRESENT_MODE_IMMEDIATE_KHR:
        return "Immediate";
    case VK_PRESENT_MODE_MAILBOX_KHR:
        return "Mailbox";
    case VK_PRESENT_MODE_FIFO_KHR:
        return "FIFO";
    case VK_PRESENT_MODE_FIFO_RELAXED_KHR:
        return "FIFO relaxed";
    default:
        return "Mode " + std::to_string(static_cast<int>(mode));
    }
}

bool validationLayerAvailable() {
    std::uint32_t layerCount = 0U;
    if (vkEnumerateInstanceLayerProperties(&layerCount, nullptr) != VK_SUCCESS) {
        return false;
    }
    std::vector<VkLayerProperties> layers(layerCount);
    if (vkEnumerateInstanceLayerProperties(&layerCount, layers.data()) != VK_SUCCESS) {
        return false;
    }
    return std::any_of(
        layers.begin(),
        layers.end(),
        [](const VkLayerProperties& layer) {
            return std::strcmp(layer.layerName, kValidationLayerName) == 0;
        }
    );
}

std::vector<std::uint32_t> readSpirv(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Unable to open SPIR-V shader: " + path.string());
    }

    const std::streamsize byteCount = file.tellg();
    if (byteCount <= 0 || (byteCount % 4) != 0) {
        throw std::runtime_error("Invalid SPIR-V byte count in: " + path.string());
    }

    std::vector<std::uint32_t> words(static_cast<std::size_t>(byteCount) / 4U);
    file.seekg(0, std::ios::beg);
    if (!file.read(
            reinterpret_cast<char*>(words.data()),
            static_cast<std::streamsize>(words.size() * sizeof(std::uint32_t))
        )) {
        throw std::runtime_error("Unable to read SPIR-V shader: " + path.string());
    }
    return words;
}

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec4 tangent;
    glm::vec2 textureCoordinates;
};

struct DebugVertex {
    glm::vec3 position;
    glm::vec4 color;
};

struct alignas(16) SceneUniform {
    glm::mat4 viewProjection{1.0F};
    std::array<glm::mat4, kDirectionalShadowCascadeCount>
        cascadeViewProjection{};
    glm::vec4 cameraPosition{0.0F};
    glm::vec4 cameraForwardNear{0.0F};
    glm::vec4 sunDirectionIntensity{0.0F};
    glm::vec4 sunColorEnabled{0.0F};
    glm::vec4 environmentColorIntensity{0.0F};
    glm::vec4 materialSettings{0.0F};
    glm::vec4 cascadeSplits{0.0F};
    glm::vec4 shadowSettings{0.0F};
    glm::uvec4 flags{0U};
    std::array<glm::vec4, kMaximumSpotLights * 4U> spotLightData{};
};

static_assert(
    sizeof(SceneUniform) == 720U &&
        offsetof(SceneUniform, cascadeViewProjection) == 64U &&
        offsetof(SceneUniform, cameraPosition) == 320U &&
        offsetof(SceneUniform, flags) == 448U &&
        offsetof(SceneUniform, spotLightData) == 464U,
    "SceneUniform must match the std140 shader block."
);

struct alignas(16) DrawPushConstant {
    glm::mat4 model{1.0F};
    glm::vec4 baseColorRoughness{1.0F};
    glm::uvec4 flags{0U};
};

static_assert(
    sizeof(DrawPushConstant) == 96U,
    "DrawPushConstant must match the GLSL push-constant block."
);
static_assert(
    std::is_trivially_copyable_v<CubeRenderData>,
    "CubeRenderData must remain a trivially copyable renderer input."
);
static_assert(
    std::is_trivially_copyable_v<DebugLineRenderData>,
    "DebugLineRenderData must remain a trivially copyable renderer input."
);
static_assert(
    std::is_trivially_copyable_v<SpotLightRenderData> &&
        sizeof(SpotLightRenderData) == 64U,
    "SpotLightRenderData must match the renderer's compact light input."
);

struct AllocatedBuffer {
    VkBuffer buffer{VK_NULL_HANDLE};
    VmaAllocation allocation{VK_NULL_HANDLE};
    void* mappedData{nullptr};
    VkDeviceSize size{0U};
};

struct AllocatedImage {
    VkImage image{VK_NULL_HANDLE};
    VmaAllocation allocation{VK_NULL_HANDLE};
};

struct TextureResource {
    AllocatedImage allocated;
    VkImageView view{VK_NULL_HANDLE};
    VkFormat format{VK_FORMAT_UNDEFINED};
    std::uint32_t width{0U};
    std::uint32_t height{0U};
    std::uint32_t mipLevels{0U};
};

struct ShadowMapFrame {
    AllocatedImage image;
    VkImageView arrayView{VK_NULL_HANDLE};
    std::array<VkImageView, kDirectionalShadowCascadeCount> layerViews{};
    std::array<VkFramebuffer, kDirectionalShadowCascadeCount> framebuffers{};
};

struct DirectionalCascadeData {
    std::array<glm::mat4, kDirectionalShadowCascadeCount> viewProjection{};
    glm::vec4 splitDepths{0.0F};
};

} // namespace

class VulkanRenderer::Impl {
public:
    ~Impl() noexcept {
        shutdown();
    }

    void initialize(
        GLFWwindow* window,
        const std::filesystem::path& executableDirectory,
        bool requestValidation
    ) {
        if (initialized_) {
            throw std::runtime_error("Vulkan renderer is already initialized.");
        }
        if (window == nullptr) {
            throw std::invalid_argument("Vulkan renderer requires a valid GLFW window.");
        }

        window_ = window;
        executableDirectory_ = executableDirectory;
        validationWarningCount_.store(0U, std::memory_order_relaxed);
        validationErrorCount_.store(0U, std::memory_order_relaxed);
        validationErrorLatched_.store(false, std::memory_order_relaxed);
        diagnostics_.validationWarnings = 0U;
        diagnostics_.validationErrors = 0U;

        try {
            prepareValidationEnvironment();
            createInstance(requestValidation);
            createSurface();
            selectPhysicalDeviceAndCreateLogicalDevice();
            createAllocator();
            createInitialSwapchain();
            createRenderPass();
            createCommandPoolAndBuffers();
            createShadowRenderPass();
            createShadowResources();
            createShadowSampler();
            createDepthResources();
            createFramebuffers();
            createDescriptorSetLayout();
            createUniformBuffers();
            createGroundGeometry();
            createCubeGeometry();
            createDebugBuffers();
            createGroundTextures();
            createEnvironmentTexture();
            createDescriptorPoolAndSets();
            createGraphicsPipeline();
            createSkyboxPipeline();
            createDebugLinePipeline();
            createShadowPipeline();
            createSyncObjects();
            throwIfValidationFailed();
            initialized_ = true;
        } catch (...) {
            shutdown();
            throw;
        }
    }

    void initializeImGui(GLFWwindow* window) {
        if (!initialized_) {
            throw std::runtime_error("Initialize Vulkan before initializing Dear ImGui.");
        }
        if (imguiVulkanInitialized_) {
            return;
        }
        if (ImGui::GetCurrentContext() == nullptr) {
            throw std::runtime_error(
                "Dear ImGui context must exist before backend initialization.");
        }

        if (!ImGui_ImplGlfw_InitForVulkan(window, true)) {
            throw std::runtime_error("Dear ImGui GLFW backend initialization failed.");
        }
        imguiGlfwInitialized_ = true;

        try {
            initializeImGuiVulkanBackend(renderPass_, swapchainImageViews_.size());
        } catch (...) {
            ImGui_ImplGlfw_Shutdown();
            imguiGlfwInitialized_ = false;
            throw;
        }
    }

    void initializeImGuiVulkanBackend(VkRenderPass renderPass, std::size_t imageCount) {
        ImGui_ImplVulkan_InitInfo info{};
        info.ApiVersion = kVulkanApiVersion;
        info.Instance = instance_.instance;
        info.PhysicalDevice = physicalDevice_.physical_device;
        info.Device = device_.device;
        info.QueueFamily = graphicsQueueFamily_;
        info.Queue = graphicsQueue_;
        info.DescriptorPoolSize = 64U;
        info.MinImageCount = 2U;
        info.ImageCount = static_cast<std::uint32_t>(imageCount);
        info.PipelineInfoMain.RenderPass = renderPass;
        info.PipelineInfoMain.Subpass = 0U;
        info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
        info.CheckVkResultFn = [](VkResult result) {
            if (result != VK_SUCCESS) {
                std::cerr << "[ImGui Vulkan] VkResult " << static_cast<int>(result) << '\n';
            }
        };

        if (!ImGui_ImplVulkan_Init(&info)) {
            throw std::runtime_error("Dear ImGui Vulkan backend initialization failed.");
        }
        imguiVulkanInitialized_ = true;
    }

    void beginUiFrame() {
      if (!imguiVulkanInitialized_) {
        throw std::runtime_error(
            "Dear ImGui Vulkan backend is not initialized.");
      }
      ImGui_ImplVulkan_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
    }

    void setSceneData(const SceneRenderDataView& scene) {
        if (scene.cubeCount > kMaxRenderedCubes) {
            throw std::length_error(
                "Scene cube count exceeds the renderer safety limit."
            );
        }
        if (scene.debugLineCount > kMaxDebugLines) {
            throw std::length_error(
                "Scene debug-line count exceeds the renderer safety limit."
            );
        }
        if (scene.spotLightCount > kMaximumSpotLights) {
            throw std::length_error(
                "Scene spot-light count exceeds the renderer safety limit."
            );
        }
        if (scene.cubeCount != 0U && scene.cubes == nullptr) {
            throw std::invalid_argument(
                "Scene cube data is null while cubeCount is nonzero."
            );
        }
        if (scene.debugLineCount != 0U && scene.debugLines == nullptr) {
            throw std::invalid_argument(
                "Scene debug-line data is null while debugLineCount is nonzero."
            );
        }
        if (scene.spotLightCount != 0U && scene.spotLights == nullptr) {
            throw std::invalid_argument(
                "Scene spot-light data is null while spotLightCount is nonzero."
            );
        }

        cubes_.clear();
        debugLines_.clear();
        if (scene.cubeCount != 0U) {
            cubes_.assign(scene.cubes, scene.cubes + scene.cubeCount);
        }
        if (scene.debugLineCount != 0U) {
            debugLines_.assign(
                scene.debugLines,
                scene.debugLines + scene.debugLineCount
            );
        }
        spotLights_.fill({});
        spotLightCount_ = scene.spotLightCount;
        if (scene.spotLightCount != 0U) {
            std::copy_n(
                scene.spotLights,
                scene.spotLightCount,
                spotLights_.begin()
            );
        }
    }

    void setSceneViewport(const SceneViewport& viewport) noexcept {
        sceneViewport_ = viewport;
    }

    void drawFrame(const Camera& camera, const GroundRenderSettings& settings) {
        if (!initialized_ || !imguiVulkanInitialized_) {
            throw std::runtime_error("Renderer is not ready to draw.");
        }
        throwIfValidationFailed();

        ImGui::Render();
        const auto cpuStart = std::chrono::steady_clock::now();

        if (swapchainDirty_) {
            recreateSwapchain();
            // A format change reinitializes the ImGui renderer backend. Skip
            // this already-built UI frame so the next one can publish fresh
            // texture bindings through the replacement backend.
            return;
        }

        int framebufferWidth = 0;
        int framebufferHeight = 0;
        glfwGetFramebufferSize(window_, &framebufferWidth, &framebufferHeight);
        if (framebufferWidth <= 0 || framebufferHeight <= 0) {
            return;
        }

        const VkFence frameFence = inFlightFences_[currentFrame_];
        requireVk(
            vkWaitForFences(device_.device, 1U, &frameFence, VK_TRUE, UINT64_MAX),
            "vkWaitForFences"
        );

        std::uint32_t imageIndex = 0U;
        const VkResult acquireResult = vkAcquireNextImageKHR(
            device_.device,
            swapchain_.swapchain,
            UINT64_MAX,
            imageAvailableSemaphores_[currentFrame_],
            VK_NULL_HANDLE,
            &imageIndex
        );
        if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
            swapchainDirty_ = true;
            recreateSwapchain();
            return;
        }
        if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
            requireVk(acquireResult, "vkAcquireNextImageKHR");
        }

        if (imageFences_[imageIndex] != VK_NULL_HANDLE) {
            requireVk(
                vkWaitForFences(
                    device_.device,
                    1U,
                    &imageFences_[imageIndex],
                    VK_TRUE,
                    UINT64_MAX
                ),
                "vkWaitForFences for swapchain image"
            );
        }
        imageFences_[imageIndex] = frameFence;

        activeSceneViewport_ = resolveSceneViewport();
        const bool hasSceneViewport =
            activeSceneViewport_.extent.width != 0U &&
            activeSceneViewport_.extent.height != 0U;
        renderSunShadowsThisFrame_ =
            hasSceneViewport &&
            settings.sunEnabled &&
            settings.sunShadowsEnabled;
        shadowDepthBiasConstant_ = std::clamp(
            std::isfinite(settings.shadowDepthBiasConstant)
                ? settings.shadowDepthBiasConstant
                : 1.25F,
            0.0F,
            64.0F
        );
        shadowDepthBiasSlope_ = std::clamp(
            std::isfinite(settings.shadowDepthBiasSlope)
                ? settings.shadowDepthBiasSlope
                : 1.75F,
            0.0F,
            64.0F
        );
        if (hasSceneViewport) {
            updateUniformBuffer(currentFrame_, camera, settings);
        }
        updateDebugBuffer(currentFrame_);
        recordCommandBuffer(commandBuffers_[currentFrame_], imageIndex);

        requireVk(vkResetFences(device_.device, 1U, &frameFence), "vkResetFences");

        const VkSemaphore waitSemaphore = imageAvailableSemaphores_[currentFrame_];
        const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        const VkSemaphore signalSemaphore = renderFinishedSemaphores_[imageIndex];
        const VkCommandBuffer commandBuffer = commandBuffers_[currentFrame_];

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.waitSemaphoreCount = 1U;
        submitInfo.pWaitSemaphores = &waitSemaphore;
        submitInfo.pWaitDstStageMask = &waitStage;
        submitInfo.commandBufferCount = 1U;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 1U;
        submitInfo.pSignalSemaphores = &signalSemaphore;
        requireVk(
            vkQueueSubmit(graphicsQueue_, 1U, &submitInfo, frameFence),
            "vkQueueSubmit"
        );

        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1U;
        presentInfo.pWaitSemaphores = &signalSemaphore;
        presentInfo.swapchainCount = 1U;
        presentInfo.pSwapchains = &swapchain_.swapchain;
        presentInfo.pImageIndices = &imageIndex;

        const VkResult presentResult = vkQueuePresentKHR(presentQueue_, &presentInfo);
        if (presentResult == VK_ERROR_OUT_OF_DATE_KHR ||
            presentResult == VK_SUBOPTIMAL_KHR ||
            acquireResult == VK_SUBOPTIMAL_KHR) {
            swapchainDirty_ = true;
        } else if (presentResult != VK_SUCCESS) {
            requireVk(presentResult, "vkQueuePresentKHR");
        }

        currentFrame_ = (currentFrame_ + 1U) % kFramesInFlight;
        ++diagnostics_.renderedFrames;
        diagnostics_.rendererCpuMilliseconds =
            std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - cpuStart
            ).count();
        throwIfValidationFailed();
    }

    void requestSwapchainRebuild() noexcept {
        swapchainDirty_ = true;
    }

    void waitIdle() {
        if (device_.device != VK_NULL_HANDLE) {
            requireVk(vkDeviceWaitIdle(device_.device), "vkDeviceWaitIdle");
            throwIfValidationFailed();
        }
    }

    void shutdown() noexcept {
        if (device_.device != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device_.device);
        }

        if (imguiVulkanInitialized_) {
            ImGui_ImplVulkan_Shutdown();
            imguiVulkanInitialized_ = false;
        }
        if (imguiGlfwInitialized_) {
            ImGui_ImplGlfw_Shutdown();
            imguiGlfwInitialized_ = false;
        }

        destroySyncObjects();

        if (device_.device != VK_NULL_HANDLE) {
            if (shadowPipeline_ != VK_NULL_HANDLE) {
                vkDestroyPipeline(device_.device, shadowPipeline_, nullptr);
                shadowPipeline_ = VK_NULL_HANDLE;
            }
            if (debugLinePipeline_ != VK_NULL_HANDLE) {
                vkDestroyPipeline(device_.device, debugLinePipeline_, nullptr);
                debugLinePipeline_ = VK_NULL_HANDLE;
            }
            if (skyboxPipeline_ != VK_NULL_HANDLE) {
                vkDestroyPipeline(device_.device, skyboxPipeline_, nullptr);
                skyboxPipeline_ = VK_NULL_HANDLE;
            }
            if (graphicsPipeline_ != VK_NULL_HANDLE) {
                vkDestroyPipeline(device_.device, graphicsPipeline_, nullptr);
                graphicsPipeline_ = VK_NULL_HANDLE;
            }
            if (pipelineLayout_ != VK_NULL_HANDLE) {
                vkDestroyPipelineLayout(device_.device, pipelineLayout_, nullptr);
                pipelineLayout_ = VK_NULL_HANDLE;
            }
            if (descriptorPool_ != VK_NULL_HANDLE) {
                vkDestroyDescriptorPool(device_.device, descriptorPool_, nullptr);
                descriptorPool_ = VK_NULL_HANDLE;
            }
            if (descriptorSetLayout_ != VK_NULL_HANDLE) {
                vkDestroyDescriptorSetLayout(device_.device, descriptorSetLayout_, nullptr);
                descriptorSetLayout_ = VK_NULL_HANDLE;
            }
        }
        descriptorSets_.clear();

        destroyBuffer(indexBuffer_);
        destroyBuffer(vertexBuffer_);
        destroyBuffer(cubeIndexBuffer_);
        destroyBuffer(cubeVertexBuffer_);
        for (AllocatedBuffer& debugBuffer : debugVertexBuffers_) {
            destroyBuffer(debugBuffer);
        }
        debugVertexBuffers_.clear();
        debugVertexCounts_.fill(0U);
        for (AllocatedBuffer& uniformBuffer : uniformBuffers_) {
            destroyBuffer(uniformBuffer);
        }
        uniformBuffers_.clear();

        if (device_.device != VK_NULL_HANDLE && materialSampler_ != VK_NULL_HANDLE) {
            vkDestroySampler(device_.device, materialSampler_, nullptr);
            materialSampler_ = VK_NULL_HANDLE;
        }
        if (device_.device != VK_NULL_HANDLE &&
            environmentSampler_ != VK_NULL_HANDLE) {
            vkDestroySampler(
                device_.device,
                environmentSampler_,
                nullptr
            );
            environmentSampler_ = VK_NULL_HANDLE;
        }
        destroyTexture(environmentTexture_);
        destroyTexture(materialTexture_);
        destroyTexture(normalTexture_);
        destroyTexture(baseColorTexture_);

        destroyShadowResources();
        if (device_.device != VK_NULL_HANDLE &&
            shadowSampler_ != VK_NULL_HANDLE) {
            vkDestroySampler(device_.device, shadowSampler_, nullptr);
            shadowSampler_ = VK_NULL_HANDLE;
        }
        if (device_.device != VK_NULL_HANDLE &&
            shadowRenderPass_ != VK_NULL_HANDLE) {
            vkDestroyRenderPass(device_.device, shadowRenderPass_, nullptr);
            shadowRenderPass_ = VK_NULL_HANDLE;
        }

        destroySwapchainAttachments();

        if (device_.device != VK_NULL_HANDLE && renderPass_ != VK_NULL_HANDLE) {
            vkDestroyRenderPass(device_.device, renderPass_, nullptr);
            renderPass_ = VK_NULL_HANDLE;
        }

        if (device_.device != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
            vkDestroyCommandPool(device_.device, commandPool_, nullptr);
            commandPool_ = VK_NULL_HANDLE;
        }
        commandBuffers_.clear();

        destroySwapchainHandleAndViews();

        if (allocator_ != VK_NULL_HANDLE) {
            vmaDestroyAllocator(allocator_);
            allocator_ = VK_NULL_HANDLE;
        }
        if (device_.device != VK_NULL_HANDLE) {
            vkb::destroy_device(device_);
            device_ = {};
        }
        if (surface_ != VK_NULL_HANDLE && instance_.instance != VK_NULL_HANDLE) {
            vkb::destroy_surface(instance_, surface_);
            surface_ = VK_NULL_HANDLE;
        }
        if (instance_.instance != VK_NULL_HANDLE) {
            vkb::destroy_instance(instance_);
            instance_ = {};
        }

        physicalDevice_ = {};
        graphicsQueue_ = VK_NULL_HANDLE;
        presentQueue_ = VK_NULL_HANDLE;
        graphicsQueueFamily_ = 0U;
        currentFrame_ = 0U;
        window_ = nullptr;
        initialized_ = false;
        swapchainDirty_ = false;
        cubes_.clear();
        debugLines_.clear();
        spotLights_.fill({});
        spotLightCount_ = 0U;
        renderSunShadowsThisFrame_ = false;
        shadowDepthFormat_ = VK_FORMAT_UNDEFINED;
        shadowLinearFiltering_ = false;
        diagnostics_.visibleCubeCount = 0U;
        diagnostics_.debugLineCount = 0U;
    }

    [[nodiscard]] bool isInitialized() const noexcept {
        return initialized_;
    }

    [[nodiscard]] const RendererDiagnostics& diagnostics() const noexcept {
        return diagnostics_;
    }

private:
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT,
        const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
        void* userData
    ) {
        auto* renderer = static_cast<Impl*>(userData);
        if (renderer != nullptr) {
            if ((messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0U) {
                renderer->validationErrorCount_.fetch_add(
                    1U,
                    std::memory_order_relaxed
                );
                renderer->validationErrorLatched_.store(
                    true,
                    std::memory_order_release
                );
            } else if ((messageSeverity &
                        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) != 0U) {
                renderer->validationWarningCount_.fetch_add(
                    1U,
                    std::memory_order_relaxed
                );
            }
        }

        const char* message =
            callbackData != nullptr && callbackData->pMessage != nullptr
                ? callbackData->pMessage
                : "Vulkan validation message without text.";
        std::cerr << "[Vulkan validation] " << message << '\n';
        return VK_FALSE;
    }

    void synchronizeValidationDiagnostics() noexcept {
        diagnostics_.validationWarnings = validationWarningCount_.load(
            std::memory_order_acquire
        );
        diagnostics_.validationErrors = validationErrorCount_.load(
            std::memory_order_acquire
        );
    }

    void throwIfValidationFailed() {
        synchronizeValidationDiagnostics();
        if (!validationErrorLatched_.load(std::memory_order_acquire)) {
            return;
        }
        throw std::runtime_error(
            "Vulkan validation reported " +
            std::to_string(diagnostics_.validationErrors) +
            " error(s); see the validation log above."
        );
    }

    void prepareValidationEnvironment() const {
#if defined(_WIN32) && defined(UAVIEW_VULKAN_SDK_DIR)
        char* existingLayerPath = nullptr;
        std::size_t existingLayerPathLength = 0U;
        _dupenv_s(
            &existingLayerPath,
            &existingLayerPathLength,
            "VK_LAYER_PATH"
        );
        const bool layerPathIsUnset =
            existingLayerPath == nullptr || existingLayerPathLength <= 1U;
        std::free(existingLayerPath);
        if (layerPathIsUnset) {
            const std::filesystem::path layerDirectory =
                std::filesystem::path(UAVIEW_VULKAN_SDK_DIR) / "Bin";
            if (std::filesystem::exists(
                    layerDirectory / "VkLayer_khronos_validation.json"
                )) {
                _putenv_s("VK_LAYER_PATH", layerDirectory.string().c_str());
            }
        }
#endif
    }

    void createInstance(bool requestValidation) {
        const bool validationCompiledIn =
#if defined(UAVIEW_ENABLE_VALIDATION)
            true;
#else
            false;
#endif
        const bool validationEnabled =
            validationCompiledIn && requestValidation && validationLayerAvailable();

        vkb::InstanceBuilder builder;
        builder.set_app_name("UAView Studio")
            .set_engine_name("UAView")
            .set_app_version(0, 1, 0)
            .require_api_version(1, 3, 0)
            .request_validation_layers(validationEnabled);

        if (validationEnabled) {
            builder.set_debug_callback(&Impl::debugCallback)
                .set_debug_callback_user_data_pointer(this)
                .set_debug_messenger_severity(
                    VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                    VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT
                );
        }

        auto result = builder.build();
        if (!result) {
            throw std::runtime_error(
                "Unable to create Vulkan instance: " + result.error().message()
            );
        }
        instance_ = result.value();
        diagnostics_.validationEnabled = validationEnabled;
    }

    void createSurface() {
        requireVk(
            glfwCreateWindowSurface(instance_.instance, window_, nullptr, &surface_),
            "glfwCreateWindowSurface"
        );
    }

    void selectPhysicalDeviceAndCreateLogicalDevice() {
        VkPhysicalDeviceFeatures requiredFeatures{};
        requiredFeatures.samplerAnisotropy = VK_TRUE;

        vkb::PhysicalDeviceSelector selector{instance_};
        auto physicalResult = selector.set_surface(surface_)
                                  .set_minimum_version(1, 3)
                                  .prefer_gpu_device_type(
                                      vkb::PreferredDeviceType::discrete
                                  )
                                  .set_required_features(requiredFeatures)
                                  .select();
        if (!physicalResult) {
            std::string details;
            for (const std::string& reason : physicalResult.detailed_failure_reasons()) {
                details += "\n  - " + reason;
            }
            throw std::runtime_error(
                "Unable to select a Vulkan physical device: " +
                physicalResult.error().message() + details
            );
        }
        physicalDevice_ = physicalResult.value();

        vkb::DeviceBuilder deviceBuilder{physicalDevice_};
        auto deviceResult = deviceBuilder.build();
        if (!deviceResult) {
            throw std::runtime_error(
                "Unable to create Vulkan logical device: " +
                deviceResult.error().message()
            );
        }
        device_ = deviceResult.value();

        auto graphicsQueueResult = device_.get_queue(vkb::QueueType::graphics);
        auto graphicsQueueIndexResult =
            device_.get_queue_index(vkb::QueueType::graphics);
        auto presentQueueResult = device_.get_queue(vkb::QueueType::present);
        if (!graphicsQueueResult || !graphicsQueueIndexResult || !presentQueueResult) {
            throw std::runtime_error(
                "Selected Vulkan device did not expose the required graphics/present queues."
            );
        }
        graphicsQueue_ = graphicsQueueResult.value();
        graphicsQueueFamily_ = graphicsQueueIndexResult.value();
        presentQueue_ = presentQueueResult.value();

        diagnostics_.gpuName = physicalDevice_.properties.deviceName;
        diagnostics_.apiVersion =
            versionString(physicalDevice_.properties.apiVersion);
    }

    void createAllocator() {
        VmaAllocatorCreateInfo allocatorInfo{};
        allocatorInfo.vulkanApiVersion = kVulkanApiVersion;
        allocatorInfo.physicalDevice = physicalDevice_.physical_device;
        allocatorInfo.device = device_.device;
        allocatorInfo.instance = instance_.instance;
        requireVk(vmaCreateAllocator(&allocatorInfo, &allocator_), "vmaCreateAllocator");
    }

    vkb::Swapchain buildSwapchain(const vkb::Swapchain* oldSwapchain) {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        if (width <= 0 || height <= 0) {
            throw std::runtime_error("Cannot create a swapchain for a zero-size framebuffer.");
        }

        vkb::SwapchainBuilder builder{device_};
        builder.set_desired_format(
                   VkSurfaceFormatKHR{
                       VK_FORMAT_B8G8R8A8_SRGB,
                       VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
                   }
               )
            .add_fallback_format(
                VkSurfaceFormatKHR{
                    VK_FORMAT_R8G8B8A8_SRGB,
                    VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
                }
            )
            .set_desired_present_mode(VK_PRESENT_MODE_MAILBOX_KHR)
            .add_fallback_present_mode(VK_PRESENT_MODE_FIFO_KHR)
            .set_desired_extent(
                static_cast<std::uint32_t>(width),
                static_cast<std::uint32_t>(height)
            )
            .set_image_usage_flags(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
        if (oldSwapchain != nullptr && oldSwapchain->swapchain != VK_NULL_HANDLE) {
            builder.set_old_swapchain(*oldSwapchain);
        }

        auto result = builder.build();
        if (!result) {
            throw std::runtime_error(
                "Unable to create Vulkan swapchain: " + result.error().message()
            );
        }
        return result.value();
    }

    void createInitialSwapchain() {
        swapchain_ = buildSwapchain(nullptr);

        auto imagesResult = swapchain_.get_images();
        auto viewsResult = swapchain_.get_image_views();
        if (!imagesResult || !viewsResult) {
            throw std::runtime_error("Unable to retrieve Vulkan swapchain images/views.");
        }
        swapchainImages_ = imagesResult.value();
        swapchainImageViews_ = viewsResult.value();
        updateSwapchainDiagnostics();
    }

    void updateSwapchainDiagnostics() {
        diagnostics_.swapchainWidth = swapchain_.extent.width;
        diagnostics_.swapchainHeight = swapchain_.extent.height;
        diagnostics_.presentMode = presentModeName(swapchain_.present_mode);
    }

    void createRenderPass() {
        depthFormat_ = findDepthFormat();
        renderPass_ = createRenderPassForFormat(swapchain_.image_format);
    }

    [[nodiscard]] VkRenderPass createRenderPassForFormat(VkFormat colorFormat) const {
        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = colorFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentDescription depthAttachment{};
        depthAttachment.format = depthFormat_;
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        const VkAttachmentReference colorReference{0U,
                                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        const VkAttachmentReference depthReference{
            1U, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1U;
        subpass.pColorAttachments = &colorReference;
        subpass.pDepthStencilAttachment = &depthReference;

        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0U;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                                  VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependency.dstStageMask = dependency.srcStageMask;
        dependency.srcAccessMask = 0U;
        dependency.dstAccessMask =
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        const std::array<VkAttachmentDescription, 2U> attachments{colorAttachment,
                                                                  depthAttachment};
        VkRenderPassCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        createInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
        createInfo.pAttachments = attachments.data();
        createInfo.subpassCount = 1U;
        createInfo.pSubpasses = &subpass;
        createInfo.dependencyCount = 1U;
        createInfo.pDependencies = &dependency;
        VkRenderPass renderPass = VK_NULL_HANDLE;
        requireVk(vkCreateRenderPass(device_.device, &createInfo, nullptr, &renderPass),
                  "vkCreateRenderPass");
        return renderPass;
    }

    VkFormat findDepthFormat() const {
        constexpr std::array<VkFormat, 3U> candidates{
            VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT};
        for (VkFormat format : candidates) {
            VkFormatProperties properties{};
            vkGetPhysicalDeviceFormatProperties(physicalDevice_.physical_device, format,
                                                &properties);
            if ((properties.optimalTilingFeatures &
                 VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0U) {
                return format;
            }
        }
        throw std::runtime_error("No supported Vulkan depth format was found.");
    }

    VkFormat findShadowDepthFormat() {
        constexpr std::array<VkFormat, 2U> candidates{
            VK_FORMAT_D16_UNORM,
            VK_FORMAT_D32_SFLOAT
        };
        constexpr VkFormatFeatureFlags2 requiredFeatures =
            VK_FORMAT_FEATURE_2_DEPTH_STENCIL_ATTACHMENT_BIT |
            VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_BIT |
            VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_DEPTH_COMPARISON_BIT;
        for (bool requireLinearFiltering : {true, false}) {
            const VkFormatFeatureFlags2 desiredFeatures =
                requiredFeatures |
                (requireLinearFiltering
                     ? VK_FORMAT_FEATURE_2_SAMPLED_IMAGE_FILTER_LINEAR_BIT
                     : 0U);
            for (VkFormat format : candidates) {
                VkFormatProperties3 properties3{};
                properties3.sType =
                    VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_3;
                VkFormatProperties2 properties2{};
                properties2.sType =
                    VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_2;
                properties2.pNext = &properties3;
                vkGetPhysicalDeviceFormatProperties2(
                    physicalDevice_.physical_device,
                    format,
                    &properties2
                );
                if ((properties3.optimalTilingFeatures & desiredFeatures) ==
                    desiredFeatures) {
                    shadowLinearFiltering_ = requireLinearFiltering;
                    return format;
                }
            }
        }
        throw std::runtime_error(
            "No sampleable comparison-capable Vulkan shadow-depth format was "
            "found."
        );
    }

    void createShadowRenderPass() {
        shadowDepthFormat_ = findShadowDepthFormat();

        VkAttachmentDescription depthAttachment{};
        depthAttachment.format = shadowDepthFormat_;
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout =
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        depthAttachment.finalLayout =
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

        const VkAttachmentReference depthReference{
            0U,
            VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
        };
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.pDepthStencilAttachment = &depthReference;

        const std::array<VkSubpassDependency, 2U> dependencies{
            VkSubpassDependency{
                VK_SUBPASS_EXTERNAL,
                0U,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                    VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                VK_ACCESS_SHADER_READ_BIT,
                VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                0U
            },
            VkSubpassDependency{
                0U,
                VK_SUBPASS_EXTERNAL,
                VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                    VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                VK_ACCESS_SHADER_READ_BIT,
                0U
            }
        };

        VkRenderPassCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        createInfo.attachmentCount = 1U;
        createInfo.pAttachments = &depthAttachment;
        createInfo.subpassCount = 1U;
        createInfo.pSubpasses = &subpass;
        createInfo.dependencyCount =
            static_cast<std::uint32_t>(dependencies.size());
        createInfo.pDependencies = dependencies.data();
        requireVk(
            vkCreateRenderPass(
                device_.device,
                &createInfo,
                nullptr,
                &shadowRenderPass_
            ),
            "vkCreateRenderPass for directional shadows"
        );
    }

    void createShadowResources() {
        for (ShadowMapFrame& frame : shadowMapFrames_) {
            frame.image = createImage(
                kShadowMapResolution,
                kShadowMapResolution,
                1U,
                shadowDepthFormat_,
                VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
                    VK_IMAGE_USAGE_SAMPLED_BIT,
                kDirectionalShadowCascadeCount
            );
            frame.arrayView = createImageView(
                frame.image.image,
                shadowDepthFormat_,
                VK_IMAGE_ASPECT_DEPTH_BIT,
                1U,
                VK_IMAGE_VIEW_TYPE_2D_ARRAY,
                0U,
                kDirectionalShadowCascadeCount
            );

            for (std::uint32_t cascade = 0U;
                 cascade < kDirectionalShadowCascadeCount;
                 ++cascade) {
                frame.layerViews[cascade] = createImageView(
                    frame.image.image,
                    shadowDepthFormat_,
                    VK_IMAGE_ASPECT_DEPTH_BIT,
                    1U,
                    VK_IMAGE_VIEW_TYPE_2D,
                    cascade,
                    1U
                );

                VkFramebufferCreateInfo framebufferInfo{};
                framebufferInfo.sType =
                    VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
                framebufferInfo.renderPass = shadowRenderPass_;
                framebufferInfo.attachmentCount = 1U;
                framebufferInfo.pAttachments =
                    &frame.layerViews[cascade];
                framebufferInfo.width = kShadowMapResolution;
                framebufferInfo.height = kShadowMapResolution;
                framebufferInfo.layers = 1U;
                requireVk(
                    vkCreateFramebuffer(
                        device_.device,
                        &framebufferInfo,
                        nullptr,
                        &frame.framebuffers[cascade]
                    ),
                    "vkCreateFramebuffer for directional shadow cascade"
                );
            }
        }

        const VkCommandBuffer commandBuffer = beginSingleUseCommands();
        std::array<VkImageMemoryBarrier, kFramesInFlight> barriers{};
        for (std::uint32_t frame = 0U; frame < kFramesInFlight; ++frame) {
            VkImageMemoryBarrier& barrier = barriers[frame];
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.srcAccessMask = 0U;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            barrier.newLayout =
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = shadowMapFrames_[frame].image.image;
            barrier.subresourceRange.aspectMask =
                VK_IMAGE_ASPECT_DEPTH_BIT;
            barrier.subresourceRange.baseMipLevel = 0U;
            barrier.subresourceRange.levelCount = 1U;
            barrier.subresourceRange.baseArrayLayer = 0U;
            barrier.subresourceRange.layerCount =
                kDirectionalShadowCascadeCount;
        }
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0U,
            0U,
            nullptr,
            0U,
            nullptr,
            static_cast<std::uint32_t>(barriers.size()),
            barriers.data()
        );
        endSingleUseCommands(commandBuffer);
    }

    void createShadowSampler() {
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter =
            shadowLinearFiltering_ ? VK_FILTER_LINEAR : VK_FILTER_NEAREST;
        samplerInfo.minFilter =
            shadowLinearFiltering_ ? VK_FILTER_LINEAR : VK_FILTER_NEAREST;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.mipLodBias = 0.0F;
        samplerInfo.anisotropyEnable = VK_FALSE;
        samplerInfo.compareEnable = VK_TRUE;
        samplerInfo.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        samplerInfo.minLod = 0.0F;
        samplerInfo.maxLod = 0.0F;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        requireVk(
            vkCreateSampler(
                device_.device,
                &samplerInfo,
                nullptr,
                &shadowSampler_
            ),
            "vkCreateSampler for directional shadows"
        );
    }

    void destroyShadowResources() noexcept {
        for (ShadowMapFrame& frame : shadowMapFrames_) {
            if (device_.device != VK_NULL_HANDLE) {
                for (VkFramebuffer& framebuffer : frame.framebuffers) {
                    if (framebuffer != VK_NULL_HANDLE) {
                        vkDestroyFramebuffer(
                            device_.device,
                            framebuffer,
                            nullptr
                        );
                        framebuffer = VK_NULL_HANDLE;
                    }
                }
                for (VkImageView& view : frame.layerViews) {
                    if (view != VK_NULL_HANDLE) {
                        vkDestroyImageView(device_.device, view, nullptr);
                        view = VK_NULL_HANDLE;
                    }
                }
                if (frame.arrayView != VK_NULL_HANDLE) {
                    vkDestroyImageView(
                        device_.device,
                        frame.arrayView,
                        nullptr
                    );
                    frame.arrayView = VK_NULL_HANDLE;
                }
            }
            if (allocator_ != VK_NULL_HANDLE &&
                frame.image.image != VK_NULL_HANDLE) {
                vmaDestroyImage(
                    allocator_,
                    frame.image.image,
                    frame.image.allocation
                );
            }
            frame.image = {};
        }
    }

    void createCommandPoolAndBuffers() {
        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = graphicsQueueFamily_;
        requireVk(
            vkCreateCommandPool(device_.device, &poolInfo, nullptr, &commandPool_),
            "vkCreateCommandPool"
        );

        commandBuffers_.resize(kFramesInFlight);
        VkCommandBufferAllocateInfo allocateInfo{};
        allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocateInfo.commandPool = commandPool_;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount =
            static_cast<std::uint32_t>(commandBuffers_.size());
        requireVk(
            vkAllocateCommandBuffers(
                device_.device,
                &allocateInfo,
                commandBuffers_.data()
            ),
            "vkAllocateCommandBuffers"
        );
    }

    AllocatedImage createImage(
        std::uint32_t width,
        std::uint32_t height,
        std::uint32_t mipLevels,
        VkFormat format,
        VkImageUsageFlags usage,
        std::uint32_t arrayLayers = 1U
    ) {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent = {width, height, 1U};
        imageInfo.mipLevels = mipLevels;
        imageInfo.arrayLayers = arrayLayers;
        imageInfo.format = format;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocationInfo{};
        allocationInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

        AllocatedImage image{};
        requireVk(
            vmaCreateImage(
                allocator_,
                &imageInfo,
                &allocationInfo,
                &image.image,
                &image.allocation,
                nullptr
            ),
            "vmaCreateImage"
        );
        return image;
    }

    VkImageView createImageView(
        VkImage image,
        VkFormat format,
        VkImageAspectFlags aspect,
        std::uint32_t mipLevels,
        VkImageViewType viewType = VK_IMAGE_VIEW_TYPE_2D,
        std::uint32_t baseArrayLayer = 0U,
        std::uint32_t layerCount = 1U
    ) const {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = viewType;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspect;
        viewInfo.subresourceRange.baseMipLevel = 0U;
        viewInfo.subresourceRange.levelCount = mipLevels;
        viewInfo.subresourceRange.baseArrayLayer = baseArrayLayer;
        viewInfo.subresourceRange.layerCount = layerCount;

        VkImageView view = VK_NULL_HANDLE;
        requireVk(vkCreateImageView(device_.device, &viewInfo, nullptr, &view),
                  "vkCreateImageView");
        return view;
    }

    void createDepthResources() {
        createDepthResourcesFor(swapchain_, swapchainImageViews_.size(), depthImages_,
                                depthImageViews_);
    }

    void createDepthResourcesFor(const vkb::Swapchain& swapchain, std::size_t imageCount,
                                 std::vector<AllocatedImage>& depthImages,
                                 std::vector<VkImageView>& depthImageViews) {
        depthImages.reserve(imageCount);
        depthImageViews.reserve(imageCount);
        auto cleanup = makeScopeExit([this, &depthImages, &depthImageViews]() noexcept {
            destroyDepthResources(depthImages, depthImageViews);
        });
        for (std::size_t index = 0; index < imageCount; ++index) {
            AllocatedImage depthImage =
                createImage(swapchain.extent.width, swapchain.extent.height, 1U, depthFormat_,
                            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
            VkImageView depthView = VK_NULL_HANDLE;
            try {
                depthView = createImageView(depthImage.image, depthFormat_,
                                            VK_IMAGE_ASPECT_DEPTH_BIT, 1U);
            } catch (...) {
                vmaDestroyImage(allocator_, depthImage.image, depthImage.allocation);
                throw;
            }
            depthImages.push_back(depthImage);
            depthImageViews.push_back(depthView);
        }
        cleanup.release();
    }

    void createFramebuffers() {
        createFramebuffersFor(renderPass_, swapchain_, swapchainImageViews_, depthImageViews_,
                              framebuffers_);
    }

    void createFramebuffersFor(VkRenderPass renderPass, const vkb::Swapchain& swapchain,
                               const std::vector<VkImageView>& swapchainImageViews,
                               const std::vector<VkImageView>& depthImageViews,
                               std::vector<VkFramebuffer>& framebuffers) {
        framebuffers.resize(swapchainImageViews.size());
        auto cleanup = makeScopeExit(
            [this, &framebuffers]() noexcept { destroyFramebuffers(framebuffers); });
        for (std::size_t index = 0; index < framebuffers.size(); ++index) {
            const std::array<VkImageView, 2U> attachments{swapchainImageViews[index],
                                                          depthImageViews[index]};
            VkFramebufferCreateInfo createInfo{};
            createInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            createInfo.renderPass = renderPass;
            createInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
            createInfo.pAttachments = attachments.data();
            createInfo.width = swapchain.extent.width;
            createInfo.height = swapchain.extent.height;
            createInfo.layers = 1U;
            requireVk(
                vkCreateFramebuffer(device_.device, &createInfo, nullptr, &framebuffers[index]),
                "vkCreateFramebuffer");
        }
        cleanup.release();
    }

    void createDescriptorSetLayout() {
      const std::array<VkDescriptorSetLayoutBinding, 6U> bindings{
          VkDescriptorSetLayoutBinding{
              0U, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1U,
              VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
              nullptr},
          VkDescriptorSetLayoutBinding{
              1U, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U,
              VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
          VkDescriptorSetLayoutBinding{
              2U, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U,
              VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
          VkDescriptorSetLayoutBinding{
              3U, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U,
              VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
          VkDescriptorSetLayoutBinding{
              4U, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U,
              VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
          VkDescriptorSetLayoutBinding{
              5U, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U,
              VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};

      VkDescriptorSetLayoutCreateInfo createInfo{};
      createInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
      createInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
      createInfo.pBindings = bindings.data();
      requireVk(vkCreateDescriptorSetLayout(device_.device, &createInfo,
                                            nullptr, &descriptorSetLayout_),
                "vkCreateDescriptorSetLayout");
    }

    AllocatedBuffer createBuffer(
        VkDeviceSize size,
        VkBufferUsageFlags usage,
        VmaMemoryUsage memoryUsage,
        VmaAllocationCreateFlags allocationFlags = 0U
    ) {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        VmaAllocationCreateInfo allocationInfo{};
        allocationInfo.usage = memoryUsage;
        allocationInfo.flags = allocationFlags;

        VmaAllocationInfo resultInfo{};
        AllocatedBuffer buffer{};
        buffer.size = size;
        requireVk(vmaCreateBuffer(allocator_, &bufferInfo, &allocationInfo,
                                  &buffer.buffer, &buffer.allocation,
                                  &resultInfo),
                  "vmaCreateBuffer");
        buffer.mappedData = resultInfo.pMappedData;
        return buffer;
    }

    void createUniformBuffers() {
      uniformBuffers_.reserve(kFramesInFlight);
      for (std::uint32_t index = 0U; index < kFramesInFlight; ++index) {
        uniformBuffers_.push_back(createBuffer(
            sizeof(SceneUniform), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                VMA_ALLOCATION_CREATE_MAPPED_BIT));
        if (uniformBuffers_.back().mappedData == nullptr) {
          throw std::runtime_error("VMA did not map a scene uniform buffer.");
        }
      }
    }

    VkCommandBuffer beginSingleUseCommands() {
        VkCommandBufferAllocateInfo allocateInfo{};
        allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocateInfo.commandPool = commandPool_;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = 1U;

        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        requireVk(vkAllocateCommandBuffers(device_.device, &allocateInfo, &commandBuffer),
                  "vkAllocateCommandBuffers for upload");

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        const VkResult beginResult = vkBeginCommandBuffer(commandBuffer, &beginInfo);
        if (beginResult != VK_SUCCESS) {
            vkFreeCommandBuffers(device_.device, commandPool_, 1U, &commandBuffer);
            requireVk(beginResult, "vkBeginCommandBuffer for upload");
        }
        return commandBuffer;
    }

    void endSingleUseCommands(VkCommandBuffer commandBuffer) {
        bool mayBePending = false;
        auto commandBufferCleanup =
            makeScopeExit([this, &commandBuffer, &mayBePending]() noexcept {
                if (commandBuffer == VK_NULL_HANDLE) {
                    return;
                }
                if (mayBePending && vkQueueWaitIdle(graphicsQueue_) != VK_SUCCESS) {
                    // A command buffer must never be freed while its submission
                    // may still be pending. Device/queue teardown owns it now.
                    return;
                }
                vkFreeCommandBuffers(device_.device, commandPool_, 1U, &commandBuffer);
                commandBuffer = VK_NULL_HANDLE;
            });

        requireVk(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer for upload");

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1U;
        submitInfo.pCommandBuffers = &commandBuffer;
        const VkResult submitResult =
            vkQueueSubmit(graphicsQueue_, 1U, &submitInfo, VK_NULL_HANDLE);
        mayBePending = submitResult == VK_SUCCESS;
        requireVk(submitResult, "vkQueueSubmit for upload");
        requireVk(vkQueueWaitIdle(graphicsQueue_), "vkQueueWaitIdle for upload");
        mayBePending = false;
    }

    void uploadBufferData(
        const void* source,
        VkDeviceSize byteCount,
        AllocatedBuffer& destination
    ) {
        AllocatedBuffer staging = createBuffer(
            byteCount,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                VMA_ALLOCATION_CREATE_MAPPED_BIT
        );
        auto stagingCleanup = makeScopeExit(
            [this, &staging]() noexcept { destroyBuffer(staging); }
        );
        if (staging.mappedData == nullptr) {
            throw std::runtime_error("VMA did not map a staging buffer.");
        }
        std::memcpy(staging.mappedData, source, static_cast<std::size_t>(byteCount));
        requireVk(
            vmaFlushAllocation(allocator_, staging.allocation, 0U, byteCount),
            "vmaFlushAllocation for staging buffer"
        );

        const VkBufferCopy copyRegion{0U, 0U, byteCount};
        const VkCommandBuffer commandBuffer = beginSingleUseCommands();
        vkCmdCopyBuffer(
            commandBuffer,
            staging.buffer,
            destination.buffer,
            1U,
            &copyRegion
        );
        endSingleUseCommands(commandBuffer);
    }

    void createGroundGeometry() {
        constexpr float halfExtent = 50.0F;
        constexpr float textureRepeats = 50.0F;
        const std::array<Vertex, 4U> vertices{
            Vertex{
                {-halfExtent, 0.0F, -halfExtent},
                {0.0F, 1.0F, 0.0F},
                {1.0F, 0.0F, 0.0F, -1.0F},
                {0.0F, 0.0F}
            },
            Vertex{
                {halfExtent, 0.0F, -halfExtent},
                {0.0F, 1.0F, 0.0F},
                {1.0F, 0.0F, 0.0F, -1.0F},
                {textureRepeats, 0.0F}
            },
            Vertex{
                {halfExtent, 0.0F, halfExtent},
                {0.0F, 1.0F, 0.0F},
                {1.0F, 0.0F, 0.0F, -1.0F},
                {textureRepeats, textureRepeats}
            },
            Vertex{
                {-halfExtent, 0.0F, halfExtent},
                {0.0F, 1.0F, 0.0F},
                {1.0F, 0.0F, 0.0F, -1.0F},
                {0.0F, textureRepeats}
            }
        };
        constexpr std::array<std::uint32_t, 6U> indices{0U, 2U, 1U, 0U, 3U, 2U};

        vertexBuffer_ = createBuffer(
            sizeof(vertices),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE
        );
        indexBuffer_ = createBuffer(
            sizeof(indices),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE
        );
        uploadBufferData(vertices.data(), sizeof(vertices), vertexBuffer_);
        uploadBufferData(indices.data(), sizeof(indices), indexBuffer_);
    }

    void createCubeGeometry() {
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(24U);
        indices.reserve(36U);

        const auto addFace = [&vertices, &indices](
                                 const glm::vec3& normal,
                                 const glm::vec3& tangent,
                                 const glm::vec3& corner0,
                                 const glm::vec3& corner1,
                                 const glm::vec3& corner2,
                                 const glm::vec3& corner3
                             ) {
            const std::uint32_t first =
                static_cast<std::uint32_t>(vertices.size());
            const glm::vec4 tangentAndSign{tangent, 1.0F};
            vertices.push_back({corner0, normal, tangentAndSign, {0.0F, 0.0F}});
            vertices.push_back({corner1, normal, tangentAndSign, {1.0F, 0.0F}});
            vertices.push_back({corner2, normal, tangentAndSign, {1.0F, 1.0F}});
            vertices.push_back({corner3, normal, tangentAndSign, {0.0F, 1.0F}});
            indices.insert(
                indices.end(),
                {first, first + 1U, first + 2U, first, first + 2U, first + 3U}
            );
        };

        addFace(
            {1.0F, 0.0F, 0.0F},
            {0.0F, 0.0F, 1.0F},
            {1.0F, -1.0F, -1.0F},
            {1.0F, -1.0F, 1.0F},
            {1.0F, 1.0F, 1.0F},
            {1.0F, 1.0F, -1.0F}
        );
        addFace(
            {-1.0F, 0.0F, 0.0F},
            {0.0F, 0.0F, -1.0F},
            {-1.0F, -1.0F, 1.0F},
            {-1.0F, -1.0F, -1.0F},
            {-1.0F, 1.0F, -1.0F},
            {-1.0F, 1.0F, 1.0F}
        );
        addFace(
            {0.0F, 1.0F, 0.0F},
            {1.0F, 0.0F, 0.0F},
            {-1.0F, 1.0F, -1.0F},
            {1.0F, 1.0F, -1.0F},
            {1.0F, 1.0F, 1.0F},
            {-1.0F, 1.0F, 1.0F}
        );
        addFace(
            {0.0F, -1.0F, 0.0F},
            {1.0F, 0.0F, 0.0F},
            {-1.0F, -1.0F, 1.0F},
            {1.0F, -1.0F, 1.0F},
            {1.0F, -1.0F, -1.0F},
            {-1.0F, -1.0F, -1.0F}
        );
        addFace(
            {0.0F, 0.0F, 1.0F},
            {1.0F, 0.0F, 0.0F},
            {1.0F, -1.0F, 1.0F},
            {-1.0F, -1.0F, 1.0F},
            {-1.0F, 1.0F, 1.0F},
            {1.0F, 1.0F, 1.0F}
        );
        addFace(
            {0.0F, 0.0F, -1.0F},
            {-1.0F, 0.0F, 0.0F},
            {-1.0F, -1.0F, -1.0F},
            {1.0F, -1.0F, -1.0F},
            {1.0F, 1.0F, -1.0F},
            {-1.0F, 1.0F, -1.0F}
        );

        cubeVertexBuffer_ = createBuffer(
            vertices.size() * sizeof(Vertex),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE
        );
        cubeIndexBuffer_ = createBuffer(
            indices.size() * sizeof(std::uint32_t),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE
        );
        uploadBufferData(
            vertices.data(),
            vertices.size() * sizeof(Vertex),
            cubeVertexBuffer_
        );
        uploadBufferData(
            indices.data(),
            indices.size() * sizeof(std::uint32_t),
            cubeIndexBuffer_
        );
    }

    void createDebugBuffers() {
        constexpr std::size_t maximumLineCount =
            kMaxDebugLines +
            kMaxRenderedCubes * kMaximumGeneratedLinesPerCube;
        constexpr VkDeviceSize bufferSize =
            maximumLineCount * 2U * sizeof(DebugVertex);

        debugVertexBuffers_.reserve(kFramesInFlight);
        for (std::uint32_t frame = 0U; frame < kFramesInFlight; ++frame) {
            debugVertexBuffers_.push_back(
                createBuffer(
                    bufferSize,
                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                    VMA_MEMORY_USAGE_AUTO,
                    VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                        VMA_ALLOCATION_CREATE_MAPPED_BIT
                )
            );
            if (debugVertexBuffers_.back().mappedData == nullptr) {
                throw std::runtime_error(
                    "VMA did not map a physics-debug vertex buffer."
                );
            }
        }
    }

    std::filesystem::path resolveAsset(const char* fileName) const {
        const std::filesystem::path relative =
            std::filesystem::path("textures") / "concrete" / fileName;
        const std::filesystem::path staged =
            executableDirectory_ / "assets" / relative;
        if (std::filesystem::exists(staged)) {
            return staged;
        }
        throw std::runtime_error(
            "Required staged concrete texture is missing: " + staged.string()
        );
    }

    std::filesystem::path resolveEnvironmentAsset(
        const char* fileName
    ) const {
        const std::filesystem::path staged =
            executableDirectory_ / "assets" / "textures" /
            "environment" / fileName;
        if (std::filesystem::exists(staged)) {
            return staged;
        }
        throw std::runtime_error(
            "Required staged environment texture is missing: " +
            staged.string()
        );
    }

    std::filesystem::path resolveShader(const char* fileName) const {
        const std::filesystem::path staged =
            executableDirectory_ / "shaders" / fileName;
        if (std::filesystem::exists(staged)) {
            return staged;
        }
        throw std::runtime_error(
            "Required staged shader is missing: " + staged.string()
        );
    }

    TextureResource uploadTexture(
        const std::uint8_t* pixels,
        std::uint32_t width,
        std::uint32_t height,
        std::uint32_t bytesPerPixel,
        VkFormat format
    ) {
        if (pixels == nullptr || width == 0U || height == 0U || bytesPerPixel == 0U) {
            throw std::invalid_argument("Cannot upload an empty Vulkan texture.");
        }

        VkFormatProperties formatProperties{};
        vkGetPhysicalDeviceFormatProperties(
            physicalDevice_.physical_device,
            format,
            &formatProperties
        );
        constexpr VkFormatFeatureFlags requiredBlitFeatures =
            VK_FORMAT_FEATURE_BLIT_SRC_BIT |
            VK_FORMAT_FEATURE_BLIT_DST_BIT |
            VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
        const bool supportsLinearBlit =
            (formatProperties.optimalTilingFeatures & requiredBlitFeatures) ==
            requiredBlitFeatures;
        const std::uint32_t mipLevels = supportsLinearBlit
            ? 1U + static_cast<std::uint32_t>(
                       std::floor(
                           std::log2(
                               static_cast<double>(std::max(width, height))
                           )
                       )
                   )
            : 1U;
        const VkDeviceSize byteCount =
            static_cast<VkDeviceSize>(width) *
            static_cast<VkDeviceSize>(height) *
            static_cast<VkDeviceSize>(bytesPerPixel);

        AllocatedBuffer staging = createBuffer(
            byteCount,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VMA_MEMORY_USAGE_AUTO,
            VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                VMA_ALLOCATION_CREATE_MAPPED_BIT
        );
        auto stagingCleanup = makeScopeExit(
            [this, &staging]() noexcept { destroyBuffer(staging); }
        );
        if (staging.mappedData == nullptr) {
            throw std::runtime_error("VMA did not map an image staging buffer.");
        }
        std::memcpy(staging.mappedData, pixels, static_cast<std::size_t>(byteCount));
        requireVk(
            vmaFlushAllocation(allocator_, staging.allocation, 0U, byteCount),
            "vmaFlushAllocation for texture"
        );

        TextureResource texture{};
        texture.width = width;
        texture.height = height;
        texture.mipLevels = mipLevels;
        texture.format = format;
        texture.allocated = createImage(
            width,
            height,
            mipLevels,
            format,
            VK_IMAGE_USAGE_TRANSFER_DST_BIT |
            VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                VK_IMAGE_USAGE_SAMPLED_BIT
        );
        auto textureCleanup = makeScopeExit(
            [this, &texture]() noexcept { destroyTexture(texture); }
        );

        const VkCommandBuffer commandBuffer = beginSingleUseCommands();

            VkImageMemoryBarrier toTransfer{};
            toTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            toTransfer.srcAccessMask = 0U;
            toTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            toTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            toTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            toTransfer.image = texture.allocated.image;
            toTransfer.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            toTransfer.subresourceRange.baseMipLevel = 0U;
            toTransfer.subresourceRange.levelCount = mipLevels;
            toTransfer.subresourceRange.baseArrayLayer = 0U;
            toTransfer.subresourceRange.layerCount = 1U;
            vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                0U,
                0U,
                nullptr,
                0U,
                nullptr,
                1U,
                &toTransfer
            );

            VkBufferImageCopy copyRegion{};
            copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copyRegion.imageSubresource.mipLevel = 0U;
            copyRegion.imageSubresource.baseArrayLayer = 0U;
            copyRegion.imageSubresource.layerCount = 1U;
            copyRegion.imageExtent = {width, height, 1U};
            vkCmdCopyBufferToImage(
                commandBuffer,
                staging.buffer,
                texture.allocated.image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1U,
                &copyRegion
            );

            std::int32_t mipWidth = static_cast<std::int32_t>(width);
            std::int32_t mipHeight = static_cast<std::int32_t>(height);
            for (std::uint32_t level = 1U; level < mipLevels; ++level) {
                VkImageMemoryBarrier previousToSource{};
                previousToSource.sType =
                    VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                previousToSource.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                previousToSource.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                previousToSource.oldLayout =
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                previousToSource.newLayout =
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                previousToSource.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                previousToSource.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                previousToSource.image = texture.allocated.image;
                previousToSource.subresourceRange.aspectMask =
                    VK_IMAGE_ASPECT_COLOR_BIT;
                previousToSource.subresourceRange.baseMipLevel = level - 1U;
                previousToSource.subresourceRange.levelCount = 1U;
                previousToSource.subresourceRange.baseArrayLayer = 0U;
                previousToSource.subresourceRange.layerCount = 1U;
                vkCmdPipelineBarrier(
                    commandBuffer,
                    VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT,
                    0U,
                    0U,
                    nullptr,
                    0U,
                    nullptr,
                    1U,
                    &previousToSource
                );

                const std::int32_t nextWidth = std::max(1, mipWidth / 2);
                const std::int32_t nextHeight = std::max(1, mipHeight / 2);
                VkImageBlit blit{};
                blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.srcSubresource.mipLevel = level - 1U;
                blit.srcSubresource.baseArrayLayer = 0U;
                blit.srcSubresource.layerCount = 1U;
                blit.srcOffsets[1] = {mipWidth, mipHeight, 1};
                blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.dstSubresource.mipLevel = level;
                blit.dstSubresource.baseArrayLayer = 0U;
                blit.dstSubresource.layerCount = 1U;
                blit.dstOffsets[1] = {nextWidth, nextHeight, 1};
                vkCmdBlitImage(
                    commandBuffer,
                    texture.allocated.image,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    texture.allocated.image,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    1U,
                    &blit,
                    VK_FILTER_LINEAR
                );

                VkImageMemoryBarrier previousToShader{};
                previousToShader.sType =
                    VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
                previousToShader.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
                previousToShader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                previousToShader.oldLayout =
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                previousToShader.newLayout =
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                previousToShader.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                previousToShader.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                previousToShader.image = texture.allocated.image;
                previousToShader.subresourceRange =
                    previousToSource.subresourceRange;
                vkCmdPipelineBarrier(
                    commandBuffer,
                    VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                    0U,
                    0U,
                    nullptr,
                    0U,
                    nullptr,
                    1U,
                    &previousToShader
                );

                mipWidth = nextWidth;
                mipHeight = nextHeight;
            }

            VkImageMemoryBarrier lastToShader{};
            lastToShader.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            lastToShader.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            lastToShader.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            lastToShader.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            lastToShader.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            lastToShader.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            lastToShader.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            lastToShader.image = texture.allocated.image;
            lastToShader.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            lastToShader.subresourceRange.baseMipLevel = mipLevels - 1U;
            lastToShader.subresourceRange.levelCount = 1U;
            lastToShader.subresourceRange.baseArrayLayer = 0U;
            lastToShader.subresourceRange.layerCount = 1U;
            vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                0U,
                0U,
                nullptr,
                0U,
                nullptr,
                1U,
                &lastToShader
            );

            endSingleUseCommands(commandBuffer);
        texture.view = createImageView(
            texture.allocated.image,
            format,
            VK_IMAGE_ASPECT_COLOR_BIT,
            mipLevels
        );
        textureCleanup.release();
        return texture;
    }

    void createGroundTextures() {
        int width = 0;
        int height = 0;
        int channels = 0;

        const std::filesystem::path baseColorPath = resolveAsset(
            "concrete_pavement_diff_2k.jpg"
        );
        stbi_uc* baseColorPixels =
            stbi_load(baseColorPath.string().c_str(), &width, &height, &channels, 4);
        if (baseColorPixels == nullptr) {
            throw std::runtime_error(
                "Unable to load concrete base color: " +
                std::string(stbi_failure_reason())
            );
        }
        try {
            baseColorTexture_ = uploadTexture(
                baseColorPixels,
                static_cast<std::uint32_t>(width),
                static_cast<std::uint32_t>(height),
                4U,
                VK_FORMAT_R8G8B8A8_SRGB
            );
        } catch (...) {
            stbi_image_free(baseColorPixels);
            throw;
        }
        stbi_image_free(baseColorPixels);

        const std::filesystem::path normalPath = resolveAsset(
            "concrete_pavement_nor_gl_2k.jpg"
        );
        stbi_uc* normalPixels =
            stbi_load(normalPath.string().c_str(), &width, &height, &channels, 4);
        if (normalPixels == nullptr) {
            throw std::runtime_error(
                "Unable to load concrete normal map: " +
                std::string(stbi_failure_reason())
            );
        }
        try {
            normalTexture_ = uploadTexture(
                normalPixels,
                static_cast<std::uint32_t>(width),
                static_cast<std::uint32_t>(height),
                4U,
                VK_FORMAT_R8G8B8A8_UNORM
            );
        } catch (...) {
            stbi_image_free(normalPixels);
            throw;
        }
        stbi_image_free(normalPixels);

        int aoWidth = 0;
        int aoHeight = 0;
        int roughnessWidth = 0;
        int roughnessHeight = 0;
        const std::filesystem::path aoPath = resolveAsset(
            "concrete_pavement_ao_2k.jpg"
        );
        const std::filesystem::path roughnessPath = resolveAsset(
            "concrete_pavement_rough_2k.jpg"
        );
        stbi_uc* aoPixels =
            stbi_load(aoPath.string().c_str(), &aoWidth, &aoHeight, &channels, 1);
        if (aoPixels == nullptr) {
            throw std::runtime_error(
                "Unable to load concrete AO map: " +
                std::string(stbi_failure_reason())
            );
        }
        stbi_uc* roughnessPixels = stbi_load(
            roughnessPath.string().c_str(),
            &roughnessWidth,
            &roughnessHeight,
            &channels,
            1
        );
        if (roughnessPixels == nullptr) {
            stbi_image_free(aoPixels);
            throw std::runtime_error(
                "Unable to load concrete roughness map: " +
                std::string(stbi_failure_reason())
            );
        }
        if (aoWidth != roughnessWidth || aoHeight != roughnessHeight) {
            stbi_image_free(roughnessPixels);
            stbi_image_free(aoPixels);
            throw std::runtime_error(
                "Concrete AO and roughness maps have different dimensions."
            );
        }

        const std::size_t pixelCount =
            static_cast<std::size_t>(aoWidth) * static_cast<std::size_t>(aoHeight);
        std::vector<std::uint8_t> packedMaterial(pixelCount * 2U);
        for (std::size_t pixel = 0U; pixel < pixelCount; ++pixel) {
            packedMaterial[pixel * 2U] = aoPixels[pixel];
            packedMaterial[pixel * 2U + 1U] = roughnessPixels[pixel];
        }
        stbi_image_free(roughnessPixels);
        stbi_image_free(aoPixels);

        materialTexture_ = uploadTexture(
            packedMaterial.data(),
            static_cast<std::uint32_t>(aoWidth),
            static_cast<std::uint32_t>(aoHeight),
            2U,
            VK_FORMAT_R8G8_UNORM
        );

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(
            physicalDevice_.physical_device,
            &properties
        );
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.mipLodBias = 0.0F;
        samplerInfo.anisotropyEnable = VK_TRUE;
        samplerInfo.maxAnisotropy =
            std::min(16.0F, properties.limits.maxSamplerAnisotropy);
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.minLod = 0.0F;
        samplerInfo.maxLod =
            static_cast<float>(baseColorTexture_.mipLevels - 1U);
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        requireVk(
            vkCreateSampler(
                device_.device,
                &samplerInfo,
                nullptr,
                &materialSampler_
            ),
            "vkCreateSampler"
        );

        diagnostics_.textureWidth = baseColorTexture_.width;
        diagnostics_.textureHeight = baseColorTexture_.height;
        diagnostics_.textureMipLevels = baseColorTexture_.mipLevels;
    }

    void createEnvironmentTexture() {
        int width = 0;
        int height = 0;
        int channels = 0;
        const std::filesystem::path environmentPath =
            resolveEnvironmentAsset(
                "kloofendal_48d_partly_cloudy_puresky_4k.hdr"
            );
        float* pixels = stbi_loadf(
            environmentPath.string().c_str(),
            &width,
            &height,
            &channels,
            4
        );
        if (pixels == nullptr || width <= 0 || height <= 0) {
            throw std::runtime_error(
                "Unable to load HDR environment: " +
                std::string(stbi_failure_reason())
            );
        }
        auto pixelsCleanup = makeScopeExit(
            [pixels]() noexcept { stbi_image_free(pixels); }
        );

        const std::size_t componentCount =
            static_cast<std::size_t>(width) *
            static_cast<std::size_t>(height) * 4U;
        std::vector<std::uint16_t> halfPixels(componentCount);
        for (std::size_t component = 0U;
             component < componentCount;
             ++component) {
            const float value = std::isfinite(pixels[component])
                ? std::clamp(pixels[component], 0.0F, 65504.0F)
                : 0.0F;
            halfPixels[component] = glm::packHalf1x16(value);
        }
        environmentTexture_ = uploadTexture(
            reinterpret_cast<const std::uint8_t*>(halfPixels.data()),
            static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height),
            8U,
            VK_FORMAT_R16G16B16A16_SFLOAT
        );

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(
            physicalDevice_.physical_device,
            &properties
        );
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.anisotropyEnable = VK_TRUE;
        samplerInfo.maxAnisotropy =
            std::min(16.0F, properties.limits.maxSamplerAnisotropy);
        samplerInfo.minLod = 0.0F;
        samplerInfo.maxLod = static_cast<float>(
            environmentTexture_.mipLevels - 1U
        );
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
        requireVk(
            vkCreateSampler(
                device_.device,
                &samplerInfo,
                nullptr,
                &environmentSampler_
            ),
            "vkCreateSampler for HDR environment"
        );
    }

    void createDescriptorPoolAndSets() {
        const std::array<VkDescriptorPoolSize, 2U> poolSizes{
            VkDescriptorPoolSize{
                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                kFramesInFlight
            },
            VkDescriptorPoolSize{
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                kFramesInFlight * 5U
            }
        };
        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.maxSets = kFramesInFlight;
        poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        requireVk(
            vkCreateDescriptorPool(
                device_.device,
                &poolInfo,
                nullptr,
                &descriptorPool_
            ),
            "vkCreateDescriptorPool"
        );

        const std::array<VkDescriptorSetLayout, kFramesInFlight> layouts{
            descriptorSetLayout_,
            descriptorSetLayout_
        };
        descriptorSets_.resize(kFramesInFlight);
        VkDescriptorSetAllocateInfo allocateInfo{};
        allocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocateInfo.descriptorPool = descriptorPool_;
        allocateInfo.descriptorSetCount = kFramesInFlight;
        allocateInfo.pSetLayouts = layouts.data();
        requireVk(
            vkAllocateDescriptorSets(
                device_.device,
                &allocateInfo,
                descriptorSets_.data()
            ),
            "vkAllocateDescriptorSets"
        );

        const VkDescriptorImageInfo baseColorInfo{
            materialSampler_,
            baseColorTexture_.view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };
        const VkDescriptorImageInfo normalInfo{
            materialSampler_,
            normalTexture_.view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };
        const VkDescriptorImageInfo materialInfo{
            materialSampler_,
            materialTexture_.view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };
        const VkDescriptorImageInfo environmentInfo{
            environmentSampler_,
            environmentTexture_.view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };

        for (std::uint32_t frame = 0U; frame < kFramesInFlight; ++frame) {
            const VkDescriptorBufferInfo uniformInfo{
                uniformBuffers_[frame].buffer,
                0U,
                sizeof(SceneUniform)
            };
            const VkDescriptorImageInfo shadowInfo{
                shadowSampler_,
                shadowMapFrames_[frame].arrayView,
                VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
            };
            const std::array<VkWriteDescriptorSet, 6U> writes{
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    0U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                    nullptr,
                    &uniformInfo,
                    nullptr
                },
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    1U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    &baseColorInfo,
                    nullptr,
                    nullptr
                },
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    2U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    &normalInfo,
                    nullptr,
                    nullptr
                },
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    3U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    &materialInfo,
                    nullptr,
                    nullptr
                },
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    4U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    &shadowInfo,
                    nullptr,
                    nullptr
                },
                VkWriteDescriptorSet{
                    VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    nullptr,
                    descriptorSets_[frame],
                    5U,
                    0U,
                    1U,
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    &environmentInfo,
                    nullptr,
                    nullptr
                }
            };
            vkUpdateDescriptorSets(
                device_.device,
                static_cast<std::uint32_t>(writes.size()),
                writes.data(),
                0U,
                nullptr
            );
        }
    }

    VkShaderModule createShaderModule(const std::vector<std::uint32_t>& code) const {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size() * sizeof(std::uint32_t);
        createInfo.pCode = code.data();

        VkShaderModule shaderModule = VK_NULL_HANDLE;
        requireVk(vkCreateShaderModule(device_.device, &createInfo, nullptr, &shaderModule),
                  "vkCreateShaderModule");
        return shaderModule;
    }

    void createGraphicsPipeline() {
        graphicsPipeline_ = buildGraphicsPipeline(renderPass_, true);
    }

    [[nodiscard]] VkPipeline buildGraphicsPipeline(VkRenderPass targetRenderPass,
                                                   bool createLayout) {
        const std::vector<std::uint32_t> vertexCode =
            readSpirv(resolveShader("ground.vert.spv"));
        const std::vector<std::uint32_t> fragmentCode =
            readSpirv(resolveShader("ground.frag.spv"));
        const VkShaderModule vertexModule = createShaderModule(vertexCode);
        VkShaderModule fragmentModule = VK_NULL_HANDLE;
        try {
            fragmentModule = createShaderModule(fragmentCode);

            const std::array<VkPipelineShaderStageCreateInfo, 2U> stages{
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0U,
                    VK_SHADER_STAGE_VERTEX_BIT, vertexModule, "main", nullptr},
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0U,
                    VK_SHADER_STAGE_FRAGMENT_BIT, fragmentModule, "main", nullptr}};

            const VkVertexInputBindingDescription bindingDescription{
                0U, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
            const std::array<VkVertexInputAttributeDescription, 4U> attributes{
                VkVertexInputAttributeDescription{
                    0U, 0U, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Vertex, position))},
                VkVertexInputAttributeDescription{
                    1U, 0U, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Vertex, normal))},
                VkVertexInputAttributeDescription{
                    2U, 0U, VK_FORMAT_R32G32B32A32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Vertex, tangent))},
                VkVertexInputAttributeDescription{
                    3U, 0U, VK_FORMAT_R32G32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Vertex, textureCoordinates))}};
            VkPipelineVertexInputStateCreateInfo vertexInput{};
            vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
            vertexInput.vertexBindingDescriptionCount = 1U;
            vertexInput.pVertexBindingDescriptions = &bindingDescription;
            vertexInput.vertexAttributeDescriptionCount =
                static_cast<std::uint32_t>(attributes.size());
            vertexInput.pVertexAttributeDescriptions = attributes.data();

            VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
            inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

            VkPipelineViewportStateCreateInfo viewportState{};
            viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            viewportState.viewportCount = 1U;
            viewportState.scissorCount = 1U;

            VkPipelineRasterizationStateCreateInfo rasterizer{};
            rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rasterizer.depthClampEnable = VK_FALSE;
            rasterizer.rasterizerDiscardEnable = VK_FALSE;
            rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
            rasterizer.cullMode = VK_CULL_MODE_NONE;
            rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rasterizer.depthBiasEnable = VK_FALSE;
            rasterizer.lineWidth = 1.0F;

            VkPipelineMultisampleStateCreateInfo multisampling{};
            multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

            VkPipelineDepthStencilStateCreateInfo depthStencil{};
            depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            depthStencil.depthTestEnable = VK_TRUE;
            depthStencil.depthWriteEnable = VK_TRUE;
            depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
            depthStencil.depthBoundsTestEnable = VK_FALSE;
            depthStencil.stencilTestEnable = VK_FALSE;

            VkPipelineColorBlendAttachmentState blendAttachment{};
            blendAttachment.blendEnable = VK_FALSE;
            blendAttachment.colorWriteMask =
                VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT;
            VkPipelineColorBlendStateCreateInfo colorBlend{};
            colorBlend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            colorBlend.logicOpEnable = VK_FALSE;
            colorBlend.attachmentCount = 1U;
            colorBlend.pAttachments = &blendAttachment;

            constexpr std::array<VkDynamicState, 2U> dynamicStates{VK_DYNAMIC_STATE_VIEWPORT,
                                                                   VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dynamicState{};
            dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
            dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
            dynamicState.pDynamicStates = dynamicStates.data();

            if (createLayout) {
                VkPipelineLayoutCreateInfo layoutInfo{};
                layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                layoutInfo.setLayoutCount = 1U;
                layoutInfo.pSetLayouts = &descriptorSetLayout_;
                const VkPushConstantRange pushConstantRange{
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0U,
                    static_cast<std::uint32_t>(sizeof(DrawPushConstant))};
                layoutInfo.pushConstantRangeCount = 1U;
                layoutInfo.pPushConstantRanges = &pushConstantRange;
                requireVk(vkCreatePipelineLayout(device_.device, &layoutInfo, nullptr,
                                                 &pipelineLayout_),
                          "vkCreatePipelineLayout");
            }

            VkGraphicsPipelineCreateInfo pipelineInfo{};
            pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
            pipelineInfo.pStages = stages.data();
            pipelineInfo.pVertexInputState = &vertexInput;
            pipelineInfo.pInputAssemblyState = &inputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlend;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pipelineLayout_;
            pipelineInfo.renderPass = targetRenderPass;
            pipelineInfo.subpass = 0U;
            VkPipeline pipeline = VK_NULL_HANDLE;
            requireVk(vkCreateGraphicsPipelines(device_.device, VK_NULL_HANDLE, 1U,
                                                &pipelineInfo, nullptr, &pipeline),
                      "vkCreateGraphicsPipelines");
            vkDestroyShaderModule(device_.device, fragmentModule, nullptr);
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            return pipeline;
        } catch (...) {
            if (fragmentModule != VK_NULL_HANDLE) {
                vkDestroyShaderModule(device_.device, fragmentModule, nullptr);
            }
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            throw;
        }
    }

    void createSkyboxPipeline() {
        skyboxPipeline_ = buildSkyboxPipeline(renderPass_);
    }

    [[nodiscard]] VkPipeline buildSkyboxPipeline(
        VkRenderPass targetRenderPass
    ) {
        const std::vector<std::uint32_t> vertexCode =
            readSpirv(resolveShader("skybox.vert.spv"));
        const std::vector<std::uint32_t> fragmentCode =
            readSpirv(resolveShader("skybox.frag.spv"));
        const VkShaderModule vertexModule = createShaderModule(vertexCode);
        VkShaderModule fragmentModule = VK_NULL_HANDLE;
        try {
            fragmentModule = createShaderModule(fragmentCode);
            const std::array<VkPipelineShaderStageCreateInfo, 2U> stages{
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                    nullptr,
                    0U,
                    VK_SHADER_STAGE_VERTEX_BIT,
                    vertexModule,
                    "main",
                    nullptr
                },
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                    nullptr,
                    0U,
                    VK_SHADER_STAGE_FRAGMENT_BIT,
                    fragmentModule,
                    "main",
                    nullptr
                }
            };

            VkPipelineVertexInputStateCreateInfo vertexInput{};
            vertexInput.sType =
                VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

            VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
            inputAssembly.sType =
                VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

            VkPipelineViewportStateCreateInfo viewportState{};
            viewportState.sType =
                VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            viewportState.viewportCount = 1U;
            viewportState.scissorCount = 1U;

            VkPipelineRasterizationStateCreateInfo rasterizer{};
            rasterizer.sType =
                VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
            rasterizer.cullMode = VK_CULL_MODE_NONE;
            rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rasterizer.lineWidth = 1.0F;

            VkPipelineMultisampleStateCreateInfo multisampling{};
            multisampling.sType =
                VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

            VkPipelineDepthStencilStateCreateInfo depthStencil{};
            depthStencil.sType =
                VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            depthStencil.depthTestEnable = VK_FALSE;
            depthStencil.depthWriteEnable = VK_FALSE;
            depthStencil.depthCompareOp = VK_COMPARE_OP_ALWAYS;

            VkPipelineColorBlendAttachmentState blendAttachment{};
            blendAttachment.colorWriteMask =
                VK_COLOR_COMPONENT_R_BIT |
                VK_COLOR_COMPONENT_G_BIT |
                VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT;
            VkPipelineColorBlendStateCreateInfo colorBlend{};
            colorBlend.sType =
                VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            colorBlend.attachmentCount = 1U;
            colorBlend.pAttachments = &blendAttachment;

            constexpr std::array<VkDynamicState, 2U> dynamicStates{
                VK_DYNAMIC_STATE_VIEWPORT,
                VK_DYNAMIC_STATE_SCISSOR
            };
            VkPipelineDynamicStateCreateInfo dynamicState{};
            dynamicState.sType =
                VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
            dynamicState.dynamicStateCount =
                static_cast<std::uint32_t>(dynamicStates.size());
            dynamicState.pDynamicStates = dynamicStates.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{};
            pipelineInfo.sType =
                VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            pipelineInfo.stageCount =
                static_cast<std::uint32_t>(stages.size());
            pipelineInfo.pStages = stages.data();
            pipelineInfo.pVertexInputState = &vertexInput;
            pipelineInfo.pInputAssemblyState = &inputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlend;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pipelineLayout_;
            pipelineInfo.renderPass = targetRenderPass;
            pipelineInfo.subpass = 0U;

            VkPipeline pipeline = VK_NULL_HANDLE;
            requireVk(
                vkCreateGraphicsPipelines(
                    device_.device,
                    VK_NULL_HANDLE,
                    1U,
                    &pipelineInfo,
                    nullptr,
                    &pipeline
                ),
                "vkCreateGraphicsPipelines for HDR skybox"
            );
            vkDestroyShaderModule(device_.device, fragmentModule, nullptr);
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            return pipeline;
        } catch (...) {
            if (fragmentModule != VK_NULL_HANDLE) {
                vkDestroyShaderModule(
                    device_.device,
                    fragmentModule,
                    nullptr
                );
            }
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            throw;
        }
    }

    void createDebugLinePipeline() {
        debugLinePipeline_ = buildDebugLinePipeline(renderPass_);
    }

    [[nodiscard]] VkPipeline buildDebugLinePipeline(VkRenderPass targetRenderPass) {
        const std::vector<std::uint32_t> vertexCode =
            readSpirv(resolveShader("debug_lines.vert.spv"));
        const std::vector<std::uint32_t> fragmentCode =
            readSpirv(resolveShader("debug_lines.frag.spv"));
        const VkShaderModule vertexModule = createShaderModule(vertexCode);
        VkShaderModule fragmentModule = VK_NULL_HANDLE;
        try {
            fragmentModule = createShaderModule(fragmentCode);
            const std::array<VkPipelineShaderStageCreateInfo, 2U> stages{
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0U,
                    VK_SHADER_STAGE_VERTEX_BIT, vertexModule, "main", nullptr},
                VkPipelineShaderStageCreateInfo{
                    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0U,
                    VK_SHADER_STAGE_FRAGMENT_BIT, fragmentModule, "main", nullptr}};

            const VkVertexInputBindingDescription bindingDescription{
                0U, sizeof(DebugVertex), VK_VERTEX_INPUT_RATE_VERTEX};
            const std::array<VkVertexInputAttributeDescription, 2U> attributes{
                VkVertexInputAttributeDescription{
                    0U, 0U, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(DebugVertex, position))},
                VkVertexInputAttributeDescription{
                    1U, 0U, VK_FORMAT_R32G32B32A32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(DebugVertex, color))}};
            VkPipelineVertexInputStateCreateInfo vertexInput{};
            vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
            vertexInput.vertexBindingDescriptionCount = 1U;
            vertexInput.pVertexBindingDescriptions = &bindingDescription;
            vertexInput.vertexAttributeDescriptionCount =
                static_cast<std::uint32_t>(attributes.size());
            vertexInput.pVertexAttributeDescriptions = attributes.data();

            VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
            inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

            VkPipelineViewportStateCreateInfo viewportState{};
            viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            viewportState.viewportCount = 1U;
            viewportState.scissorCount = 1U;

            VkPipelineRasterizationStateCreateInfo rasterizer{};
            rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
            rasterizer.cullMode = VK_CULL_MODE_NONE;
            rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rasterizer.lineWidth = 1.0F;

            VkPipelineMultisampleStateCreateInfo multisampling{};
            multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

            VkPipelineDepthStencilStateCreateInfo depthStencil{};
            depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            depthStencil.depthTestEnable = VK_TRUE;
            depthStencil.depthWriteEnable = VK_FALSE;
            depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
            depthStencil.depthBoundsTestEnable = VK_FALSE;
            depthStencil.stencilTestEnable = VK_FALSE;

            VkPipelineColorBlendAttachmentState blendAttachment{};
            blendAttachment.blendEnable = VK_TRUE;
            blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
            blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
            blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
            blendAttachment.colorWriteMask =
                VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT;
            VkPipelineColorBlendStateCreateInfo colorBlend{};
            colorBlend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            colorBlend.attachmentCount = 1U;
            colorBlend.pAttachments = &blendAttachment;

            constexpr std::array<VkDynamicState, 2U> dynamicStates{VK_DYNAMIC_STATE_VIEWPORT,
                                                                   VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dynamicState{};
            dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
            dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
            dynamicState.pDynamicStates = dynamicStates.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{};
            pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
            pipelineInfo.pStages = stages.data();
            pipelineInfo.pVertexInputState = &vertexInput;
            pipelineInfo.pInputAssemblyState = &inputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlend;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pipelineLayout_;
            pipelineInfo.renderPass = targetRenderPass;
            pipelineInfo.subpass = 0U;
            VkPipeline pipeline = VK_NULL_HANDLE;
            requireVk(vkCreateGraphicsPipelines(device_.device, VK_NULL_HANDLE, 1U,
                                                &pipelineInfo, nullptr, &pipeline),
                      "vkCreateGraphicsPipelines for debug lines");
            vkDestroyShaderModule(device_.device, fragmentModule, nullptr);
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            return pipeline;
        } catch (...) {
            if (fragmentModule != VK_NULL_HANDLE) {
                vkDestroyShaderModule(device_.device, fragmentModule, nullptr);
            }
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
            throw;
        }
    }

    void createShadowPipeline() {
        const std::vector<std::uint32_t> vertexCode =
            readSpirv(resolveShader("shadow.vert.spv"));
        const VkShaderModule vertexModule = createShaderModule(vertexCode);
        auto moduleCleanup = makeScopeExit([this, vertexModule]() noexcept {
            vkDestroyShaderModule(device_.device, vertexModule, nullptr);
        });

        const VkPipelineShaderStageCreateInfo shaderStage{
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            nullptr,
            0U,
            VK_SHADER_STAGE_VERTEX_BIT,
            vertexModule,
            "main",
            nullptr
        };

        const VkVertexInputBindingDescription bindingDescription{
            0U,
            sizeof(Vertex),
            VK_VERTEX_INPUT_RATE_VERTEX
        };
        const VkVertexInputAttributeDescription positionAttribute{
            0U,
            0U,
            VK_FORMAT_R32G32B32_SFLOAT,
            static_cast<std::uint32_t>(offsetof(Vertex, position))
        };
        VkPipelineVertexInputStateCreateInfo vertexInput{};
        vertexInput.sType =
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInput.vertexBindingDescriptionCount = 1U;
        vertexInput.pVertexBindingDescriptions = &bindingDescription;
        vertexInput.vertexAttributeDescriptionCount = 1U;
        vertexInput.pVertexAttributeDescriptions = &positionAttribute;

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType =
            VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1U;
        viewportState.scissorCount = 1U;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        // Scene render data is allowed to contain either handedness after a
        // transform. Keeping both faces makes those casts deterministic.
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_TRUE;
        rasterizer.lineWidth = 1.0F;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType =
            VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType =
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        depthStencil.depthBoundsTestEnable = VK_FALSE;
        depthStencil.stencilTestEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo colorBlend{};
        colorBlend.sType =
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlend.logicOpEnable = VK_FALSE;
        colorBlend.attachmentCount = 0U;
        colorBlend.pAttachments = nullptr;

        constexpr std::array<VkDynamicState, 3U> dynamicStates{
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR,
            VK_DYNAMIC_STATE_DEPTH_BIAS
        };
        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType =
            VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount =
            static_cast<std::uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 1U;
        pipelineInfo.pStages = &shaderStage;
        pipelineInfo.pVertexInputState = &vertexInput;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlend;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = pipelineLayout_;
        pipelineInfo.renderPass = shadowRenderPass_;
        pipelineInfo.subpass = 0U;

        requireVk(
            vkCreateGraphicsPipelines(
                device_.device,
                VK_NULL_HANDLE,
                1U,
                &pipelineInfo,
                nullptr,
                &shadowPipeline_
            ),
            "vkCreateGraphicsPipelines for directional shadows"
        );
    }

    void createSyncObjects() {
      imageAvailableSemaphores_.resize(kFramesInFlight);
      inFlightFences_.resize(kFramesInFlight);

      VkSemaphoreCreateInfo semaphoreInfo{};
      semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
      VkFenceCreateInfo fenceInfo{};
      fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
      fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

      for (std::uint32_t frame = 0U; frame < kFramesInFlight; ++frame) {
        requireVk(vkCreateSemaphore(device_.device, &semaphoreInfo, nullptr,
                                    &imageAvailableSemaphores_[frame]),
                  "vkCreateSemaphore for image acquisition");
        requireVk(vkCreateFence(device_.device, &fenceInfo, nullptr,
                                &inFlightFences_[frame]),
                  "vkCreateFence");
      }
      createPresentSemaphores();
    }

    [[nodiscard]] VkRect2D resolveSceneViewport() const noexcept {
        const VkExtent2D fullExtent = swapchain_.extent;
        if (sceneViewport_.width == 0U || sceneViewport_.height == 0U) {
            return {{0, 0}, fullExtent};
        }

        const std::int64_t fullRight = static_cast<std::int64_t>(fullExtent.width);
        const std::int64_t fullBottom = static_cast<std::int64_t>(fullExtent.height);
        const std::int64_t requestedLeft = sceneViewport_.x;
        const std::int64_t requestedTop = sceneViewport_.y;
        const std::int64_t requestedRight =
            requestedLeft + static_cast<std::int64_t>(sceneViewport_.width);
        const std::int64_t requestedBottom =
            requestedTop + static_cast<std::int64_t>(sceneViewport_.height);
        const std::int64_t clippedLeft = std::clamp(requestedLeft, std::int64_t{0}, fullRight);
        const std::int64_t clippedTop = std::clamp(requestedTop, std::int64_t{0}, fullBottom);
        const std::int64_t clippedRight =
            std::clamp(requestedRight, std::int64_t{0}, fullRight);
        const std::int64_t clippedBottom =
            std::clamp(requestedBottom, std::int64_t{0}, fullBottom);
        const std::uint32_t clippedWidth =
            clippedRight > clippedLeft ? static_cast<std::uint32_t>(clippedRight - clippedLeft)
                                       : 0U;
        const std::uint32_t clippedHeight =
            clippedBottom > clippedTop ? static_cast<std::uint32_t>(clippedBottom - clippedTop)
                                       : 0U;

        const VkRect2D resolved{
            {static_cast<std::int32_t>(clippedLeft), static_cast<std::int32_t>(clippedTop)},
            {clippedWidth, clippedHeight}};
        return resolved;
    }

    [[nodiscard]] static bool finiteVector(const glm::vec3 &value) noexcept {
      return std::isfinite(value.x) && std::isfinite(value.y) &&
             std::isfinite(value.z);
    }

    [[nodiscard]] static bool finiteVector(const glm::vec4 &value) noexcept {
      return std::isfinite(value.x) && std::isfinite(value.y) &&
             std::isfinite(value.z) && std::isfinite(value.w);
    }

    [[nodiscard]] static bool finiteMatrix(const glm::mat4 &value) noexcept {
      return finiteVector(value[0]) && finiteVector(value[1]) &&
             finiteVector(value[2]) && finiteVector(value[3]);
    }

    [[nodiscard]] static glm::vec3 cubeHalfExtents(
        const CubeRenderData& cube
    ) noexcept {
        return {
            std::max(std::abs(cube.halfExtents.x), 0.001F),
            std::max(std::abs(cube.halfExtents.y), 0.001F),
            std::max(std::abs(cube.halfExtents.z), 0.001F)
        };
    }

    [[nodiscard]] static glm::vec3 transformPoint(
        const glm::mat4& transform,
        const glm::vec3& point
    ) noexcept {
        return glm::vec3(transform * glm::vec4(point, 1.0F));
    }

    [[nodiscard]] static DirectionalCascadeData buildDirectionalCascades(
        const Camera& camera,
        float aspect,
        const glm::vec3& directionToSun,
        const GroundRenderSettings& settings
    ) {
        constexpr glm::vec3 worldUp{0.0F, 1.0F, 0.0F};
        constexpr glm::vec3 alternateUp{0.0F, 0.0F, 1.0F};
        constexpr float directionEpsilon = 1.0e-8F;
        constexpr float degreesToRadians =
            3.14159265358979323846F / 180.0F;

        const float safeAspect =
            std::isfinite(aspect) && aspect > 0.0F ? aspect : 1.0F;
        const float requestedDistance = std::isfinite(settings.shadowDistance)
            ? settings.shadowDistance
            : 180.0F;
        const float shadowFar = std::clamp(
            requestedDistance,
            kMinimumShadowDistance,
            kCameraFarPlane
        );
        const float splitLambda = std::clamp(
            std::isfinite(settings.shadowSplitLambda)
                ? settings.shadowSplitLambda
                : 0.72F,
            0.0F,
            1.0F
        );
        const float verticalFovRadians = std::clamp(
            std::isfinite(camera.fieldOfViewDegrees())
                ? camera.fieldOfViewDegrees()
                : 58.0F,
            1.0F,
            179.0F
        ) * degreesToRadians;
        const float tangentHalfFov = std::tan(verticalFovRadians * 0.5F);

        glm::vec3 cameraForward = camera.forward();
        const float forwardLengthSquared =
            glm::dot(cameraForward, cameraForward);
        cameraForward =
            finiteVector(cameraForward) &&
                    forwardLengthSquared > directionEpsilon
                ? cameraForward * glm::inversesqrt(forwardLengthSquared)
                : glm::vec3{0.0F, 0.0F, -1.0F};
        glm::vec3 cameraRight = glm::cross(cameraForward, worldUp);
        float rightLengthSquared = glm::dot(cameraRight, cameraRight);
        if (!finiteVector(cameraRight) ||
            rightLengthSquared <= directionEpsilon) {
            cameraRight = glm::cross(cameraForward, alternateUp);
            rightLengthSquared = glm::dot(cameraRight, cameraRight);
        }
        cameraRight = rightLengthSquared > directionEpsilon
            ? cameraRight * glm::inversesqrt(rightLengthSquared)
            : glm::vec3{1.0F, 0.0F, 0.0F};
        const glm::vec3 cameraUp =
            glm::normalize(glm::cross(cameraRight, cameraForward));
        const glm::vec3 cameraPosition = finiteVector(camera.position())
            ? camera.position()
            : glm::vec3{0.0F};

        glm::vec3 safeDirectionToSun = directionToSun;
        const float sunLengthSquared =
            glm::dot(safeDirectionToSun, safeDirectionToSun);
        safeDirectionToSun =
            finiteVector(safeDirectionToSun) &&
                    sunLengthSquared > directionEpsilon
                ? safeDirectionToSun * glm::inversesqrt(sunLengthSquared)
                : glm::normalize(glm::vec3{-0.5F, 1.0F, -0.35F});
        const glm::vec3 lightUp =
            std::abs(glm::dot(safeDirectionToSun, worldUp)) > 0.95F
                ? alternateUp
                : worldUp;

        DirectionalCascadeData cascades{};
        float previousSplit = kCameraNearPlane;
        for (std::uint32_t cascade = 0U;
             cascade < kDirectionalShadowCascadeCount;
             ++cascade) {
            const float splitFraction =
                static_cast<float>(cascade + 1U) /
                static_cast<float>(kDirectionalShadowCascadeCount);
            const float logarithmicSplit =
                kCameraNearPlane *
                std::pow(
                    shadowFar / kCameraNearPlane,
                    splitFraction
                );
            const float uniformSplit =
                kCameraNearPlane +
                (shadowFar - kCameraNearPlane) * splitFraction;
            const float cascadeFar = std::clamp(
                logarithmicSplit * splitLambda +
                    uniformSplit * (1.0F - splitLambda),
                previousSplit + 0.001F,
                shadowFar
            );
            cascades.splitDepths[cascade] = cascadeFar;

            std::array<glm::vec3, 8U> corners{};
            std::size_t cornerIndex = 0U;
            for (float depth : {previousSplit, cascadeFar}) {
                const glm::vec3 sliceCenter =
                    cameraPosition + cameraForward * depth;
                const float halfHeight = tangentHalfFov * depth;
                const float halfWidth = halfHeight * safeAspect;
                for (float verticalSign : {-1.0F, 1.0F}) {
                    for (float horizontalSign : {-1.0F, 1.0F}) {
                        corners[cornerIndex++] =
                            sliceCenter +
                            cameraRight * (halfWidth * horizontalSign) +
                            cameraUp * (halfHeight * verticalSign);
                    }
                }
            }

            glm::vec3 center{0.0F};
            for (const glm::vec3& corner : corners) {
                center += corner;
            }
            center /= static_cast<float>(corners.size());

            float radius = 0.0F;
            for (const glm::vec3& corner : corners) {
                radius = std::max(radius, glm::length(corner - center));
            }
            // A stable bounding sphere avoids cascade extents changing as the
            // camera rotates. The small guard band retains nearby off-frustum
            // casters and rounds the extent to suppress sub-texel shimmer.
            radius = std::max(radius * 1.05F + 0.5F, 1.0F);
            radius = std::ceil(radius * 16.0F) / 16.0F;

            const float depthPadding = std::max(10.0F, radius * 0.35F);
            const float eyeDistance = radius + depthPadding;
            const glm::mat4 lightView = glm::lookAtRH(
                center + safeDirectionToSun * eyeDistance,
                center,
                lightUp
            );

            float minimumZ = std::numeric_limits<float>::max();
            float maximumZ = std::numeric_limits<float>::lowest();
            for (const glm::vec3& corner : corners) {
                const float lightZ =
                    (lightView * glm::vec4(corner, 1.0F)).z;
                minimumZ = std::min(minimumZ, lightZ);
                maximumZ = std::max(maximumZ, lightZ);
            }
            const float nearDepth = std::max(
                0.1F,
                -maximumZ - depthPadding
            );
            const float farDepth = std::max(
                nearDepth + 1.0F,
                -minimumZ + depthPadding
            );

            glm::mat4 lightProjection = glm::orthoRH_ZO(
                -radius,
                radius,
                -radius,
                radius,
                nearDepth,
                farDepth
            );
            lightProjection[1][1] *= -1.0F;

            // Snap the orthographic projection to the texel grid. Because the
            // cascade uses a sphere, this leaves the coverage unchanged while
            // removing most camera-translation shimmer.
            const glm::mat4 unsnapped =
                lightProjection * lightView;
            const glm::vec4 worldOriginClip =
                unsnapped * glm::vec4(0.0F, 0.0F, 0.0F, 1.0F);
            const float texelScale =
                static_cast<float>(kShadowMapResolution) * 0.5F;
            const float offsetX =
                (std::round(worldOriginClip.x * texelScale) -
                 worldOriginClip.x * texelScale) /
                texelScale;
            const float offsetY =
                (std::round(worldOriginClip.y * texelScale) -
                 worldOriginClip.y * texelScale) /
                texelScale;
            lightProjection[3][0] += offsetX;
            lightProjection[3][1] += offsetY;

            cascades.viewProjection[cascade] =
                lightProjection * lightView;
            previousSplit = cascadeFar;
        }
        return cascades;
    }

    void updateDebugBuffer(std::uint32_t frame) {
        constexpr std::size_t maximumVertexCount =
            (kMaxDebugLines +
             kMaxRenderedCubes * kMaximumGeneratedLinesPerCube) *
            2U;
        debugVertexScratch_.clear();
        if (debugVertexScratch_.capacity() < maximumVertexCount) {
            debugVertexScratch_.reserve(maximumVertexCount);
        }

        const auto appendLine = [this](
                                    const glm::vec3& start,
                                    const glm::vec3& end,
                                    const glm::vec4& color
                                ) {
            if (debugVertexScratch_.size() + 2U >
                debugVertexScratch_.capacity()) {
                return;
            }
            if (!finiteVector(start) ||
                !finiteVector(end) ||
                !finiteVector(color)) {
                return;
            }
            debugVertexScratch_.push_back({start, color});
            debugVertexScratch_.push_back({end, color});
        };

        for (const DebugLineRenderData& line : debugLines_) {
            appendLine(line.start, line.end, line.color);
        }

        std::uint32_t visibleCubeCount = 0U;
        constexpr std::array<std::array<std::uint32_t, 2U>, 12U> edges{
            std::array<std::uint32_t, 2U>{0U, 1U},
            std::array<std::uint32_t, 2U>{0U, 2U},
            std::array<std::uint32_t, 2U>{0U, 4U},
            std::array<std::uint32_t, 2U>{1U, 3U},
            std::array<std::uint32_t, 2U>{1U, 5U},
            std::array<std::uint32_t, 2U>{2U, 3U},
            std::array<std::uint32_t, 2U>{2U, 6U},
            std::array<std::uint32_t, 2U>{3U, 7U},
            std::array<std::uint32_t, 2U>{4U, 5U},
            std::array<std::uint32_t, 2U>{4U, 6U},
            std::array<std::uint32_t, 2U>{5U, 7U},
            std::array<std::uint32_t, 2U>{6U, 7U}
        };

        for (const CubeRenderData& cube : cubes_) {
            if ((cube.flags & CubeRenderFlags::visible) == 0U ||
                !finiteMatrix(cube.bodyToWorld) ||
                !finiteVector(cube.halfExtents)) {
                continue;
            }
            ++visibleCubeCount;

            if ((cube.flags & CubeRenderFlags::showBounds) == 0U &&
                cube.highlightedVertexMask == 0U) {
                continue;
            }

            const glm::vec3 halfExtents = cubeHalfExtents(cube);
            std::array<glm::vec3, 8U> corners{};
            for (std::uint32_t corner = 0U; corner < corners.size(); ++corner) {
                const glm::vec3 local{
                    (corner & 1U) != 0U ? halfExtents.x : -halfExtents.x,
                    (corner & 2U) != 0U ? halfExtents.y : -halfExtents.y,
                    (corner & 4U) != 0U ? halfExtents.z : -halfExtents.z
                };
                corners[corner] = transformPoint(cube.bodyToWorld, local);
            }

            if ((cube.flags & CubeRenderFlags::showBounds) != 0U) {
                glm::vec4 boundsColor{0.18F, 0.7F, 1.0F, 0.82F};
                if ((cube.flags & CubeRenderFlags::grabbed) != 0U) {
                    boundsColor = {1.0F, 0.42F, 0.08F, 0.95F};
                } else if ((cube.flags & CubeRenderFlags::selected) != 0U) {
                    boundsColor = {0.15F, 0.58F, 1.0F, 0.95F};
                }
                for (const auto& edge : edges) {
                    appendLine(
                        corners[edge[0]],
                        corners[edge[1]],
                        boundsColor
                    );
                }
            }

            const float markerRadius = std::clamp(
                glm::length(halfExtents) * 0.045F,
                0.025F,
                0.12F
            );
            const glm::vec4 markerColor =
                (cube.flags & CubeRenderFlags::grabbed) != 0U
                    ? glm::vec4{1.0F, 0.42F, 0.08F, 1.0F}
                    : glm::vec4{0.2F, 0.9F, 1.0F, 1.0F};
            for (std::uint32_t corner = 0U; corner < corners.size(); ++corner) {
                if ((cube.highlightedVertexMask & (1U << corner)) == 0U) {
                    continue;
                }
                const glm::vec3 center = corners[corner];
                appendLine(
                    center - glm::vec3{markerRadius, 0.0F, 0.0F},
                    center + glm::vec3{markerRadius, 0.0F, 0.0F},
                    markerColor
                );
                appendLine(
                    center - glm::vec3{0.0F, markerRadius, 0.0F},
                    center + glm::vec3{0.0F, markerRadius, 0.0F},
                    markerColor
                );
                appendLine(
                    center - glm::vec3{0.0F, 0.0F, markerRadius},
                    center + glm::vec3{0.0F, 0.0F, markerRadius},
                    markerColor
                );
            }
        }

        diagnostics_.visibleCubeCount = visibleCubeCount;
        diagnostics_.debugLineCount = static_cast<std::uint32_t>(
            debugVertexScratch_.size() / 2U
        );
        debugVertexCounts_[frame] = static_cast<std::uint32_t>(
            debugVertexScratch_.size()
        );
        if (debugVertexScratch_.empty()) {
            return;
        }

        const VkDeviceSize byteCount =
            debugVertexScratch_.size() * sizeof(DebugVertex);
        std::memcpy(debugVertexBuffers_[frame].mappedData, debugVertexScratch_.data(),
                    static_cast<std::size_t>(byteCount));
        requireVk(vmaFlushAllocation(allocator_, debugVertexBuffers_[frame].allocation, 0U,
                                     byteCount),
                  "vmaFlushAllocation for physics-debug vertices");
    }

    void createPresentSemaphores() {
        createPresentSemaphoresFor(swapchainImageViews_.size(), renderFinishedSemaphores_,
                                   imageFences_);
    }

    void createPresentSemaphoresFor(std::size_t imageCount,
                                    std::vector<VkSemaphore>& semaphores,
                                    std::vector<VkFence>& imageFences) {
        semaphores.resize(imageCount);
        imageFences.assign(imageCount, VK_NULL_HANDLE);
        auto cleanup = makeScopeExit([this, &semaphores, &imageFences]() noexcept {
            destroyPresentSemaphores(semaphores, imageFences);
        });

        VkSemaphoreCreateInfo semaphoreInfo{};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
        for (VkSemaphore& semaphore : semaphores) {
            requireVk(vkCreateSemaphore(device_.device, &semaphoreInfo, nullptr, &semaphore),
                      "vkCreateSemaphore for presentation");
        }
        cleanup.release();
    }

    void updateUniformBuffer(
        std::uint32_t frame,
        const Camera& camera,
        const GroundRenderSettings& settings
    ) {
        const float aspect =
            static_cast<float>(activeSceneViewport_.extent.width) /
            static_cast<float>(activeSceneViewport_.extent.height);
        constexpr float degreesToRadians =
            3.14159265358979323846F / 180.0F;
        const float azimuthDegrees =
            std::isfinite(settings.sunAzimuthDegrees)
                ? settings.sunAzimuthDegrees
                : -38.0F;
        const float elevationDegrees = std::clamp(
            std::isfinite(settings.sunElevationDegrees)
                ? settings.sunElevationDegrees
                : 48.0F,
            -89.9F,
            89.9F
        );
        const float azimuth = azimuthDegrees * degreesToRadians;
        const float elevation = elevationDegrees * degreesToRadians;
        const glm::vec3 directionToSun = glm::normalize(glm::vec3{
            std::cos(elevation) * std::cos(azimuth),
            std::sin(elevation),
            std::cos(elevation) * std::sin(azimuth)
        });
        const DirectionalCascadeData cascades =
            buildDirectionalCascades(
                camera,
                aspect,
                directionToSun,
                settings
            );

        const auto finiteNonnegativeColor = [](
                                                const glm::vec3& color,
                                                const glm::vec3& fallback
                                            ) noexcept {
            if (!finiteVector(color)) {
                return fallback;
            }
            return glm::vec3{
                std::clamp(color.x, 0.0F, 100.0F),
                std::clamp(color.y, 0.0F, 100.0F),
                std::clamp(color.z, 0.0F, 100.0F)
            };
        };
        const auto finiteClamped = [](
                                       float value,
                                       float fallback,
                                       float minimum,
                                       float maximum
                                   ) noexcept {
            return std::clamp(
                std::isfinite(value) ? value : fallback,
                minimum,
                maximum
            );
        };

        const float sunIntensity =
            finiteClamped(settings.sunIntensity, 5.0F, 0.0F, 100'000.0F);
        const glm::vec3 sunColor = finiteNonnegativeColor(
            settings.sunColor,
            glm::vec3{1.0F, 0.955F, 0.88F}
        );
        const float environmentIntensity = finiteClamped(
            settings.ambientStrength,
            0.055F,
            0.0F,
            100.0F
        );
        const glm::vec3 environmentColor = finiteNonnegativeColor(
            settings.environmentColor,
            glm::vec3{0.72F, 0.80F, 1.0F}
        );
        const float shadowDistance = finiteClamped(
            settings.shadowDistance,
            180.0F,
            kMinimumShadowDistance,
            kCameraFarPlane
        );
        const float shadowNormalBias = finiteClamped(
            settings.shadowNormalBias,
            0.018F,
            0.0F,
            0.5F
        );

        bool cascadeMatricesFinite = true;
        for (const glm::mat4& cascade : cascades.viewProjection) {
            cascadeMatricesFinite =
                cascadeMatricesFinite && finiteMatrix(cascade);
        }
        const bool sunEnabled =
            settings.sunEnabled && sunIntensity > 0.0F;
        const bool environmentEnabled =
            settings.environmentEnabled && environmentIntensity > 0.0F;
        renderSunShadowsThisFrame_ =
            renderSunShadowsThisFrame_ &&
            sunEnabled &&
            cascadeMatricesFinite;

        SceneUniform uniform{};
        uniform.viewProjection = camera.viewProjectionMatrix(aspect);
        uniform.cascadeViewProjection = cascades.viewProjection;
        uniform.cameraPosition = glm::vec4(
            finiteVector(camera.position())
                ? camera.position()
                : glm::vec3{0.0F},
            1.0F
        );
        glm::vec3 cameraForward = camera.forward();
        const float cameraForwardLengthSquared =
            glm::dot(cameraForward, cameraForward);
        cameraForward =
            finiteVector(cameraForward) &&
                    cameraForwardLengthSquared > 1.0e-8F
                ? cameraForward *
                    glm::inversesqrt(cameraForwardLengthSquared)
                : glm::vec3{0.0F, 0.0F, -1.0F};
        uniform.cameraForwardNear =
            glm::vec4(cameraForward, kCameraNearPlane);
        uniform.sunDirectionIntensity =
            glm::vec4(directionToSun, sunIntensity);
        uniform.sunColorEnabled =
            glm::vec4(sunColor, sunEnabled ? 1.0F : 0.0F);
        uniform.environmentColorIntensity =
            glm::vec4(environmentColor, environmentIntensity);
        uniform.materialSettings = glm::vec4(
            environmentIntensity,
            finiteClamped(settings.exposure, 1.05F, 0.01F, 100.0F),
            finiteClamped(settings.normalStrength, 1.0F, 0.0F, 4.0F),
            finiteClamped(settings.roughnessScale, 1.0F, 0.05F, 4.0F)
        );
        uniform.cascadeSplits = cascades.splitDepths;
        uniform.shadowSettings = glm::vec4(
            shadowNormalBias,
            1.0F / static_cast<float>(kShadowMapResolution),
            kCascadeBlendFraction,
            shadowDistance
        );

        std::uint32_t frameFlags = 0U;
        if (sunEnabled) {
            frameFlags |= kFrameFlagSunEnabled;
        }
        if (environmentEnabled) {
            frameFlags |= kFrameFlagEnvironmentEnabled;
        }
        if (renderSunShadowsThisFrame_) {
            frameFlags |= kFrameFlagSunShadowsEnabled;
        }
        const std::uint32_t spotLightCount = static_cast<std::uint32_t>(
            std::min(spotLightCount_, kMaximumSpotLights)
        );
        uniform.flags = glm::uvec4(
            settings.flipNormalGreen ? 1U : 0U,
            frameFlags,
            spotLightCount,
            kDirectionalShadowCascadeCount
        );

        for (std::uint32_t lightIndex = 0U;
             lightIndex < spotLightCount;
             ++lightIndex) {
            const SpotLightRenderData& light = spotLights_[lightIndex];
            const std::uint32_t dataIndex = lightIndex * 4U;
            const bool positionFinite = finiteVector(light.position);
            const float range =
                finiteClamped(light.range, 12.0F, 0.0F, 100'000.0F);
            const float intensity =
                finiteClamped(light.intensity, 0.0F, 0.0F, 1'000'000.0F);
            glm::vec3 direction = light.direction;
            const float directionLengthSquared =
                glm::dot(direction, direction);
            const bool directionFinite =
                finiteVector(direction) &&
                directionLengthSquared > 1.0e-8F;
            direction = directionFinite
                ? direction * glm::inversesqrt(directionLengthSquared)
                : glm::vec3{0.0F, -1.0F, 0.0F};
            const float innerConeDegrees = finiteClamped(
                light.innerConeDegrees,
                22.0F,
                0.0F,
                88.9F
            );
            const float outerConeDegrees = finiteClamped(
                light.outerConeDegrees,
                32.0F,
                innerConeDegrees + 0.1F,
                89.9F
            );
            const glm::vec3 lightColor = finiteNonnegativeColor(
                light.color,
                glm::vec3{1.0F, 0.92F, 0.78F}
            );
            const bool enabled =
                (light.flags & SpotLightRenderFlags::enabled) != 0U &&
                positionFinite &&
                directionFinite &&
                range > 0.0F &&
                intensity > 0.0F;

            uniform.spotLightData[dataIndex + 0U] = glm::vec4(
                positionFinite ? light.position : glm::vec3{0.0F},
                range
            );
            uniform.spotLightData[dataIndex + 1U] = glm::vec4(
                direction,
                std::cos(outerConeDegrees * degreesToRadians)
            );
            uniform.spotLightData[dataIndex + 2U] =
                glm::vec4(lightColor, intensity);
            uniform.spotLightData[dataIndex + 3U] = glm::vec4(
                std::cos(innerConeDegrees * degreesToRadians),
                enabled ? 1.0F : 0.0F,
                0.0F,
                0.0F
            );
        }

        std::memcpy(
            uniformBuffers_[frame].mappedData,
            &uniform,
            sizeof(uniform)
        );
        requireVk(
            vmaFlushAllocation(
                allocator_,
                uniformBuffers_[frame].allocation,
                0U,
                sizeof(uniform)
            ),
            "vmaFlushAllocation for scene uniform"
        );
    }

    void recordDirectionalShadowPasses(VkCommandBuffer commandBuffer) {
        if (!renderSunShadowsThisFrame_) {
            return;
        }

        const VkViewport viewport{
            0.0F,
            0.0F,
            static_cast<float>(kShadowMapResolution),
            static_cast<float>(kShadowMapResolution),
            0.0F,
            1.0F
        };
        const VkRect2D scissor{
            {0, 0},
            {kShadowMapResolution, kShadowMapResolution}
        };
        VkClearValue clearValue{};
        clearValue.depthStencil = {1.0F, 0U};
        const VkDeviceSize vertexOffset = 0U;

        for (std::uint32_t cascade = 0U;
             cascade < kDirectionalShadowCascadeCount;
             ++cascade) {
            VkRenderPassBeginInfo renderPassInfo{};
            renderPassInfo.sType =
                VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            renderPassInfo.renderPass = shadowRenderPass_;
            renderPassInfo.framebuffer =
                shadowMapFrames_[currentFrame_].framebuffers[cascade];
            renderPassInfo.renderArea = scissor;
            renderPassInfo.clearValueCount = 1U;
            renderPassInfo.pClearValues = &clearValue;
            vkCmdBeginRenderPass(
                commandBuffer,
                &renderPassInfo,
                VK_SUBPASS_CONTENTS_INLINE
            );

            vkCmdSetViewport(commandBuffer, 0U, 1U, &viewport);
            vkCmdSetScissor(commandBuffer, 0U, 1U, &scissor);
            vkCmdSetDepthBias(
                commandBuffer,
                shadowDepthBiasConstant_,
                0.0F,
                shadowDepthBiasSlope_
            );
            vkCmdBindPipeline(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                shadowPipeline_
            );
            vkCmdBindDescriptorSets(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                pipelineLayout_,
                0U,
                1U,
                &descriptorSets_[currentFrame_],
                0U,
                nullptr
            );
            vkCmdBindVertexBuffers(
                commandBuffer,
                0U,
                1U,
                &cubeVertexBuffer_.buffer,
                &vertexOffset
            );
            vkCmdBindIndexBuffer(
                commandBuffer,
                cubeIndexBuffer_.buffer,
                0U,
                VK_INDEX_TYPE_UINT32
            );

            for (const CubeRenderData& cube : cubes_) {
                if ((cube.flags & CubeRenderFlags::visible) == 0U ||
                    !finiteMatrix(cube.bodyToWorld) ||
                    !finiteVector(cube.halfExtents)) {
                    continue;
                }

                const glm::vec3 halfExtents = cubeHalfExtents(cube);
                DrawPushConstant cubeDraw{};
                cubeDraw.model = cube.bodyToWorld;
                cubeDraw.model[0] *= halfExtents.x;
                cubeDraw.model[1] *= halfExtents.y;
                cubeDraw.model[2] *= halfExtents.z;
                cubeDraw.flags.w = cascade;
                vkCmdPushConstants(
                    commandBuffer,
                    pipelineLayout_,
                    VK_SHADER_STAGE_VERTEX_BIT |
                        VK_SHADER_STAGE_FRAGMENT_BIT,
                    0U,
                    sizeof(cubeDraw),
                    &cubeDraw
                );
                vkCmdDrawIndexed(commandBuffer, 36U, 1U, 0U, 0, 0U);
            }

            vkCmdEndRenderPass(commandBuffer);
        }
    }

    void recordCommandBuffer(VkCommandBuffer commandBuffer,
                             std::uint32_t imageIndex) {
      requireVk(vkResetCommandBuffer(commandBuffer, 0U),
                "vkResetCommandBuffer");

      VkCommandBufferBeginInfo beginInfo{};
      beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
      requireVk(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");

      recordDirectionalShadowPasses(commandBuffer);

      const std::array<VkClearValue, 2U> clearValues{
          VkClearValue{{{0.014F, 0.019F, 0.027F, 1.0F}}}, VkClearValue{{{1.0F, 0U}}}};
      VkRenderPassBeginInfo renderPassInfo{};
      renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
      renderPassInfo.renderPass = renderPass_;
      renderPassInfo.framebuffer = framebuffers_[imageIndex];
      renderPassInfo.renderArea.offset = {0, 0};
      renderPassInfo.renderArea.extent = swapchain_.extent;
      renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
      renderPassInfo.pClearValues = clearValues.data();
      vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

      const bool hasSceneViewport =
          activeSceneViewport_.extent.width != 0U && activeSceneViewport_.extent.height != 0U;
      if (hasSceneViewport) {
          const VkViewport viewport{static_cast<float>(activeSceneViewport_.offset.x),
                                    static_cast<float>(activeSceneViewport_.offset.y),
                                    static_cast<float>(activeSceneViewport_.extent.width),
                                    static_cast<float>(activeSceneViewport_.extent.height),
                                    0.0F,
                                    1.0F};
          const VkRect2D scissor = activeSceneViewport_;
          vkCmdSetViewport(commandBuffer, 0U, 1U, &viewport);
          vkCmdSetScissor(commandBuffer, 0U, 1U, &scissor);

          vkCmdBindPipeline(
              commandBuffer,
              VK_PIPELINE_BIND_POINT_GRAPHICS,
              skyboxPipeline_
          );
          vkCmdBindDescriptorSets(
              commandBuffer,
              VK_PIPELINE_BIND_POINT_GRAPHICS,
              pipelineLayout_,
              0U,
              1U,
              &descriptorSets_[currentFrame_],
              0U,
              nullptr
          );
          vkCmdDraw(commandBuffer, 3U, 1U, 0U, 0U);

          vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline_);
          const VkDeviceSize vertexOffset = 0U;
          vkCmdBindVertexBuffers(commandBuffer, 0U, 1U, &vertexBuffer_.buffer, &vertexOffset);
          vkCmdBindIndexBuffer(commandBuffer, indexBuffer_.buffer, 0U, VK_INDEX_TYPE_UINT32);
          vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                  pipelineLayout_, 0U, 1U, &descriptorSets_[currentFrame_], 0U,
                                  nullptr);

          const DrawPushConstant groundDraw{};
          vkCmdPushConstants(commandBuffer, pipelineLayout_,
                             VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0U,
                             sizeof(groundDraw), &groundDraw);
          vkCmdDrawIndexed(commandBuffer, 6U, 1U, 0U, 0, 0U);

          vkCmdBindVertexBuffers(commandBuffer, 0U, 1U, &cubeVertexBuffer_.buffer,
                                 &vertexOffset);
          vkCmdBindIndexBuffer(commandBuffer, cubeIndexBuffer_.buffer, 0U,
                               VK_INDEX_TYPE_UINT32);
          for (const CubeRenderData& cube : cubes_) {
              if ((cube.flags & CubeRenderFlags::visible) == 0U ||
                  !finiteMatrix(cube.bodyToWorld) || !finiteVector(cube.halfExtents) ||
                  !finiteVector(cube.baseColor) || !std::isfinite(cube.roughness)) {
                  continue;
              }

              const glm::vec3 halfExtents = cubeHalfExtents(cube);
              DrawPushConstant cubeDraw{};
              cubeDraw.model = cube.bodyToWorld;
              cubeDraw.model[0] *= halfExtents.x;
              cubeDraw.model[1] *= halfExtents.y;
              cubeDraw.model[2] *= halfExtents.z;
              cubeDraw.baseColorRoughness = {
                  std::clamp(cube.baseColor.x, 0.0F, 100.0F),
                  std::clamp(cube.baseColor.y, 0.0F, 100.0F),
                  std::clamp(cube.baseColor.z, 0.0F, 100.0F),
                  std::clamp(cube.roughness, 0.045F, 1.0F)};
              cubeDraw.flags = {1U, cube.flags, cube.highlightedVertexMask, 0U};
              vkCmdPushConstants(commandBuffer, pipelineLayout_,
                                 VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0U,
                                 sizeof(cubeDraw), &cubeDraw);
              vkCmdDrawIndexed(commandBuffer, 36U, 1U, 0U, 0, 0U);
          }

          if (debugVertexCounts_[currentFrame_] != 0U) {
              vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                debugLinePipeline_);
              vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                      pipelineLayout_, 0U, 1U, &descriptorSets_[currentFrame_],
                                      0U, nullptr);
              vkCmdBindVertexBuffers(commandBuffer, 0U, 1U,
                                     &debugVertexBuffers_[currentFrame_].buffer, &vertexOffset);
              vkCmdDraw(commandBuffer, debugVertexCounts_[currentFrame_], 1U, 0U, 0U);
          }
      }

      const VkViewport fullViewport{0.0F,
                                    0.0F,
                                    static_cast<float>(swapchain_.extent.width),
                                    static_cast<float>(swapchain_.extent.height),
                                    0.0F,
                                    1.0F};
      const VkRect2D fullScissor{{0, 0}, swapchain_.extent};
      vkCmdSetViewport(commandBuffer, 0U, 1U, &fullViewport);
      vkCmdSetScissor(commandBuffer, 0U, 1U, &fullScissor);
      ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
      vkCmdEndRenderPass(commandBuffer);
      requireVk(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer");
    }

    bool recreateSwapchain() {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window_, &width, &height);
        if (width <= 0 || height <= 0) {
            return false;
        }

        requireVk(vkDeviceWaitIdle(device_.device), "vkDeviceWaitIdle for resize");

        vkb::Swapchain oldSwapchain = swapchain_;
        const VkFormat oldFormat = oldSwapchain.image_format;

        vkb::Swapchain newSwapchain = buildSwapchain(&oldSwapchain);
        std::vector<VkImageView> newViews;
        auto newSwapchainCleanup = makeScopeExit([&newSwapchain, &newViews]() noexcept {
            newSwapchain.destroy_image_views(newViews);
            vkb::destroy_swapchain(newSwapchain);
        });
        auto newImagesResult = newSwapchain.get_images();
        auto newViewsResult = newSwapchain.get_image_views();
        if (!newImagesResult || !newViewsResult) {
            if (newViewsResult) {
                newViews = std::move(newViewsResult.value());
            }
            throw std::runtime_error("Unable to retrieve recreated swapchain images/views.");
        }

        std::vector<VkImage> newImages = std::move(newImagesResult.value());
        newViews = std::move(newViewsResult.value());
        const bool formatChanged = newSwapchain.image_format != oldFormat;

        VkRenderPass newRenderPass = formatChanged ? VK_NULL_HANDLE : renderPass_;
        VkPipeline newGraphicsPipeline = formatChanged ? VK_NULL_HANDLE : graphicsPipeline_;
        VkPipeline newSkyboxPipeline = formatChanged ? VK_NULL_HANDLE : skyboxPipeline_;
        VkPipeline newDebugLinePipeline = formatChanged ? VK_NULL_HANDLE : debugLinePipeline_;
        auto formatResourcesCleanup =
            makeScopeExit([this, &newRenderPass, &newGraphicsPipeline,
                           &newSkyboxPipeline, &newDebugLinePipeline,
                           formatChanged]() noexcept {
                if (!formatChanged) {
                    return;
                }
                if (newDebugLinePipeline != VK_NULL_HANDLE) {
                    vkDestroyPipeline(device_.device, newDebugLinePipeline, nullptr);
                }
                if (newSkyboxPipeline != VK_NULL_HANDLE) {
                    vkDestroyPipeline(device_.device, newSkyboxPipeline, nullptr);
                }
                if (newGraphicsPipeline != VK_NULL_HANDLE) {
                    vkDestroyPipeline(device_.device, newGraphicsPipeline, nullptr);
                }
                if (newRenderPass != VK_NULL_HANDLE) {
                    vkDestroyRenderPass(device_.device, newRenderPass, nullptr);
                }
            });
        if (formatChanged) {
            newRenderPass = createRenderPassForFormat(newSwapchain.image_format);
            newGraphicsPipeline = buildGraphicsPipeline(newRenderPass, false);
            newSkyboxPipeline = buildSkyboxPipeline(newRenderPass);
            newDebugLinePipeline = buildDebugLinePipeline(newRenderPass);
        }

        std::vector<AllocatedImage> newDepthImages;
        std::vector<VkImageView> newDepthImageViews;
        std::vector<VkFramebuffer> newFramebuffers;
        createDepthResourcesFor(newSwapchain, newViews.size(), newDepthImages,
                                newDepthImageViews);
        auto newAttachmentsCleanup = makeScopeExit(
            [this, &newDepthImages, &newDepthImageViews, &newFramebuffers]() noexcept {
                destroyFramebuffers(newFramebuffers);
                destroyDepthResources(newDepthImages, newDepthImageViews);
            });
        createFramebuffersFor(newRenderPass, newSwapchain, newViews, newDepthImageViews,
                              newFramebuffers);

        std::vector<VkSemaphore> newPresentSemaphores;
        std::vector<VkFence> newImageFences;
        createPresentSemaphoresFor(newViews.size(), newPresentSemaphores, newImageFences);
        auto newSemaphoresCleanup =
            makeScopeExit([this, &newPresentSemaphores, &newImageFences]() noexcept {
                destroyPresentSemaphores(newPresentSemaphores, newImageFences);
            });

        const bool reinitializeImGui = formatChanged && imguiVulkanInitialized_;
        if (reinitializeImGui) {
            ImGui_ImplVulkan_Shutdown();
            imguiVulkanInitialized_ = false;
            try {
                initializeImGuiVulkanBackend(newRenderPass, newViews.size());
            } catch (...) {
                const std::exception_ptr failure = std::current_exception();
                try {
                    initializeImGuiVulkanBackend(renderPass_, swapchainImageViews_.size());
                } catch (...) {
                    imguiVulkanInitialized_ = false;
                }
                std::rethrow_exception(failure);
            }
        }

        std::vector<VkImage> oldImages = std::move(swapchainImages_);
        std::vector<VkImageView> oldViews = std::move(swapchainImageViews_);
        std::vector<AllocatedImage> oldDepthImages = std::move(depthImages_);
        std::vector<VkImageView> oldDepthImageViews = std::move(depthImageViews_);
        std::vector<VkFramebuffer> oldFramebuffers = std::move(framebuffers_);
        std::vector<VkSemaphore> oldPresentSemaphores = std::move(renderFinishedSemaphores_);
        std::vector<VkFence> oldImageFences = std::move(imageFences_);
        const VkRenderPass oldRenderPass = renderPass_;
        const VkPipeline oldGraphicsPipeline = graphicsPipeline_;
        const VkPipeline oldSkyboxPipeline = skyboxPipeline_;
        const VkPipeline oldDebugLinePipeline = debugLinePipeline_;

        swapchain_ = newSwapchain;
        swapchainImages_ = std::move(newImages);
        swapchainImageViews_ = std::move(newViews);
        depthImages_ = std::move(newDepthImages);
        depthImageViews_ = std::move(newDepthImageViews);
        framebuffers_ = std::move(newFramebuffers);
        renderFinishedSemaphores_ = std::move(newPresentSemaphores);
        imageFences_ = std::move(newImageFences);
        if (formatChanged) {
            renderPass_ = newRenderPass;
            graphicsPipeline_ = newGraphicsPipeline;
            skyboxPipeline_ = newSkyboxPipeline;
            debugLinePipeline_ = newDebugLinePipeline;
            newRenderPass = VK_NULL_HANDLE;
            newGraphicsPipeline = VK_NULL_HANDLE;
            newSkyboxPipeline = VK_NULL_HANDLE;
            newDebugLinePipeline = VK_NULL_HANDLE;
        }

        newSwapchainCleanup.release();
        newAttachmentsCleanup.release();
        newSemaphoresCleanup.release();
        formatResourcesCleanup.release();

        destroyFramebuffers(oldFramebuffers);
        destroyDepthResources(oldDepthImages, oldDepthImageViews);
        destroyPresentSemaphores(oldPresentSemaphores, oldImageFences);
        oldSwapchain.destroy_image_views(oldViews);
        vkb::destroy_swapchain(oldSwapchain);
        oldImages.clear();

        if (formatChanged) {
            vkDestroyPipeline(device_.device, oldDebugLinePipeline, nullptr);
            vkDestroyPipeline(device_.device, oldSkyboxPipeline, nullptr);
            vkDestroyPipeline(device_.device, oldGraphicsPipeline, nullptr);
            vkDestroyRenderPass(device_.device, oldRenderPass, nullptr);
        } else if (imguiVulkanInitialized_) {
            ImGui_ImplVulkan_SetMinImageCount(2U);
        }
        updateSwapchainDiagnostics();
        swapchainDirty_ = false;
        return true;
    }

    void destroyBuffer(AllocatedBuffer& buffer) noexcept {
        if (allocator_ != VK_NULL_HANDLE && buffer.buffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(allocator_, buffer.buffer, buffer.allocation);
        }
        buffer = {};
    }

    void destroyTexture(TextureResource& texture) noexcept {
        if (device_.device != VK_NULL_HANDLE && texture.view != VK_NULL_HANDLE) {
            vkDestroyImageView(device_.device, texture.view, nullptr);
        }
        if (allocator_ != VK_NULL_HANDLE && texture.allocated.image != VK_NULL_HANDLE) {
            vmaDestroyImage(allocator_, texture.allocated.image, texture.allocated.allocation);
        }
        texture = {};
    }

    void destroyFramebuffers(std::vector<VkFramebuffer>& framebuffers) noexcept {
        if (device_.device != VK_NULL_HANDLE) {
            for (VkFramebuffer framebuffer : framebuffers) {
                if (framebuffer != VK_NULL_HANDLE) {
                    vkDestroyFramebuffer(device_.device, framebuffer, nullptr);
                }
            }
        }
        framebuffers.clear();
    }

    void destroyDepthResources(std::vector<AllocatedImage>& depthImages,
                               std::vector<VkImageView>& depthImageViews) noexcept {
        if (device_.device != VK_NULL_HANDLE) {
            for (VkImageView view : depthImageViews) {
                if (view != VK_NULL_HANDLE) {
                    vkDestroyImageView(device_.device, view, nullptr);
                }
            }
        }
        depthImageViews.clear();
        if (allocator_ != VK_NULL_HANDLE) {
            for (const AllocatedImage& image : depthImages) {
                if (image.image != VK_NULL_HANDLE) {
                    vmaDestroyImage(allocator_, image.image, image.allocation);
                }
            }
        }
        depthImages.clear();
    }

    void destroySwapchainAttachments() noexcept {
        destroyFramebuffers(framebuffers_);
        destroyDepthResources(depthImages_, depthImageViews_);
    }

    void destroyPresentSemaphores(std::vector<VkSemaphore>& semaphores,
                                  std::vector<VkFence>& imageFences) noexcept {
        if (device_.device != VK_NULL_HANDLE) {
            for (VkSemaphore semaphore : semaphores) {
                if (semaphore != VK_NULL_HANDLE) {
                    vkDestroySemaphore(device_.device, semaphore, nullptr);
                }
            }
        }
        semaphores.clear();
        imageFences.clear();
    }

    void destroyPresentSemaphores() noexcept {
        destroyPresentSemaphores(renderFinishedSemaphores_, imageFences_);
    }

    void destroySyncObjects() noexcept {
      destroyPresentSemaphores();
      if (device_.device != VK_NULL_HANDLE) {
        for (VkSemaphore semaphore : imageAvailableSemaphores_) {
          if (semaphore != VK_NULL_HANDLE) {
            vkDestroySemaphore(device_.device, semaphore, nullptr);
          }
        }
        for (VkFence fence : inFlightFences_) {
          if (fence != VK_NULL_HANDLE) {
            vkDestroyFence(device_.device, fence, nullptr);
          }
        }
      }
      imageAvailableSemaphores_.clear();
      inFlightFences_.clear();
    }

    void destroySwapchainHandleAndViews() noexcept {
        if (swapchain_.swapchain != VK_NULL_HANDLE) {
            swapchain_.destroy_image_views(swapchainImageViews_);
            swapchainImageViews_.clear();
            swapchainImages_.clear();
            vkb::destroy_swapchain(swapchain_);
            swapchain_ = {};
        }
    }

    GLFWwindow* window_{nullptr};
    std::filesystem::path executableDirectory_;
    bool initialized_{false};
    bool imguiGlfwInitialized_{false};
    bool imguiVulkanInitialized_{false};
    bool swapchainDirty_{false};
    std::uint32_t currentFrame_{0U};
    SceneViewport sceneViewport_{};
    VkRect2D activeSceneViewport_{{0, 0}, {1U, 1U}};
    std::vector<CubeRenderData> cubes_;
    std::vector<DebugLineRenderData> debugLines_;
    std::vector<DebugVertex> debugVertexScratch_;
    std::array<SpotLightRenderData, kMaximumSpotLights> spotLights_{};
    std::size_t spotLightCount_{0U};
    bool renderSunShadowsThisFrame_{false};
    float shadowDepthBiasConstant_{1.25F};
    float shadowDepthBiasSlope_{1.75F};
    std::atomic<std::uint32_t> validationWarningCount_{0U};
    std::atomic<std::uint32_t> validationErrorCount_{0U};
    std::atomic<bool> validationErrorLatched_{false};

    vkb::Instance instance_;
    VkSurfaceKHR surface_{VK_NULL_HANDLE};
    vkb::PhysicalDevice physicalDevice_;
    vkb::Device device_;
    VkQueue graphicsQueue_{VK_NULL_HANDLE};
    VkQueue presentQueue_{VK_NULL_HANDLE};
    std::uint32_t graphicsQueueFamily_{0U};
    VmaAllocator allocator_{VK_NULL_HANDLE};

    vkb::Swapchain swapchain_;
    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> swapchainImageViews_;
    VkFormat depthFormat_{VK_FORMAT_UNDEFINED};
    std::vector<AllocatedImage> depthImages_;
    std::vector<VkImageView> depthImageViews_;
    VkRenderPass renderPass_{VK_NULL_HANDLE};
    std::vector<VkFramebuffer> framebuffers_;
    VkFormat shadowDepthFormat_{VK_FORMAT_UNDEFINED};
    bool shadowLinearFiltering_{false};
    std::array<ShadowMapFrame, kFramesInFlight> shadowMapFrames_{};
    VkRenderPass shadowRenderPass_{VK_NULL_HANDLE};
    VkSampler shadowSampler_{VK_NULL_HANDLE};

    VkCommandPool commandPool_{VK_NULL_HANDLE};
    std::vector<VkCommandBuffer> commandBuffers_;
    std::vector<VkSemaphore> imageAvailableSemaphores_;
    std::vector<VkSemaphore> renderFinishedSemaphores_;
    std::vector<VkFence> inFlightFences_;
    std::vector<VkFence> imageFences_;

    VkDescriptorSetLayout descriptorSetLayout_{VK_NULL_HANDLE};
    VkDescriptorPool descriptorPool_{VK_NULL_HANDLE};
    std::vector<VkDescriptorSet> descriptorSets_;
    std::vector<AllocatedBuffer> uniformBuffers_;
    AllocatedBuffer vertexBuffer_;
    AllocatedBuffer indexBuffer_;
    AllocatedBuffer cubeVertexBuffer_;
    AllocatedBuffer cubeIndexBuffer_;
    std::vector<AllocatedBuffer> debugVertexBuffers_;
    std::array<std::uint32_t, kFramesInFlight> debugVertexCounts_{};
    TextureResource baseColorTexture_;
    TextureResource normalTexture_;
    TextureResource materialTexture_;
    TextureResource environmentTexture_;
    VkSampler materialSampler_{VK_NULL_HANDLE};
    VkSampler environmentSampler_{VK_NULL_HANDLE};
    VkPipelineLayout pipelineLayout_{VK_NULL_HANDLE};
    VkPipeline graphicsPipeline_{VK_NULL_HANDLE};
    VkPipeline skyboxPipeline_{VK_NULL_HANDLE};
    VkPipeline debugLinePipeline_{VK_NULL_HANDLE};
    VkPipeline shadowPipeline_{VK_NULL_HANDLE};

    RendererDiagnostics diagnostics_;
};

VulkanRenderer::VulkanRenderer()
    : impl_(std::make_unique<Impl>()) {}

VulkanRenderer::~VulkanRenderer() = default;

VulkanRenderer::VulkanRenderer(VulkanRenderer&&) noexcept = default;

VulkanRenderer& VulkanRenderer::operator=(VulkanRenderer&&) noexcept = default;

void VulkanRenderer::initialize(
    GLFWwindow* window,
    const std::filesystem::path& executableDirectory,
    bool requestValidation
) {
    impl_->initialize(window, executableDirectory, requestValidation);
}

void VulkanRenderer::initializeImGui(GLFWwindow* window) {
    impl_->initializeImGui(window);
}

void VulkanRenderer::beginUiFrame() {
    impl_->beginUiFrame();
}

void VulkanRenderer::setSceneData(const SceneRenderDataView& scene) {
    impl_->setSceneData(scene);
}

void VulkanRenderer::setSceneViewport(const SceneViewport& viewport) noexcept {
    impl_->setSceneViewport(viewport);
}

void VulkanRenderer::drawFrame(
    const Camera& camera,
    const GroundRenderSettings& settings
) {
    impl_->drawFrame(camera, settings);
}

void VulkanRenderer::requestSwapchainRebuild() noexcept {
    impl_->requestSwapchainRebuild();
}

void VulkanRenderer::waitIdle() {
    impl_->waitIdle();
}

void VulkanRenderer::shutdown() {
    impl_->shutdown();
}

bool VulkanRenderer::isInitialized() const noexcept {
    return impl_ != nullptr && impl_->isInitialized();
}

const RendererDiagnostics& VulkanRenderer::diagnostics() const noexcept {
    return impl_->diagnostics();
}

} // namespace uaview::render
