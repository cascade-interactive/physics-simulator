#include <uaview/app/StudioApplication.hpp>

#include <uaview/physics/World.hpp>
#include <uaview/render/Camera.hpp>
#include <uaview/render/VulkanRenderer.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>

namespace uaview::app {
namespace {

constexpr int kInitialWindowWidth = 1600;
constexpr int kInitialWindowHeight = 900;
constexpr int kMinimumWindowWidth = 960;
constexpr int kMinimumWindowHeight = 600;
constexpr int kSmokeTestFrameCount = 16;
constexpr int kSmokeTestTimeoutSeconds = 15;
constexpr float kMaximumFrameDeltaSeconds = 0.25F;
constexpr float kMaximumPickDistanceMeters = 2'000.0F;
constexpr float kDefaultGrabInfluenceRadiusMeters = 0.80F;
constexpr float kDefaultGrabFrequencyHz = 6.0F;
constexpr float kDefaultGrabMaximumAcceleration = 200.0F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kSpawnClearanceMeters = 0.006F;

enum class SpawnPlacementMode : int {
    ClickSurface = 0,
    OriginOrNearestFree = 1,
    RandomEmpty = 2,
    Predefined = 3,
};

struct CubeSpawnerSettings {
    SpawnPlacementMode placement{SpawnPlacementMode::OriginOrNearestFree};
    physics::Vec3 size{1.0F, 1.0F, 1.0F};
    float massKilograms{120.0F};
    physics::Vec3 predefinedPosition{0.0F, 3.0F, 0.0F};
    float randomRadiusMeters{12.0F};
    bool avoidOverlaps{true};

    bool initialImpulseEnabled{false};
    physics::Vec3 initialImpulseNewtonSeconds{};
    bool initialAngularMomentumEnabled{false};
    physics::Vec3 initialAngularMomentumNewtonMeterSeconds{};

    bool timedForceEnabled{false};
    physics::Vec3 timedForceNewtons{};
    physics::Vec3 timedTorqueNewtonMeters{};
    float timedForceDurationSeconds{1.0F};
    bool timedForceBodyLocal{false};
};

[[nodiscard]] ImVec4 uiColor(
    std::uint8_t red,
    std::uint8_t green,
    std::uint8_t blue,
    std::uint8_t alpha = 255U
) noexcept {
    constexpr float inverseByte = 1.0F / 255.0F;
    return {
        static_cast<float>(red) * inverseByte,
        static_cast<float>(green) * inverseByte,
        static_cast<float>(blue) * inverseByte,
        static_cast<float>(alpha) * inverseByte
    };
}

[[nodiscard]] glm::vec3 toGlm(const physics::Vec3& value) noexcept {
    return {value.x, value.y, value.z};
}

[[nodiscard]] physics::Vec3 toPhysics(const glm::vec3& value) noexcept {
    return {value.x, value.y, value.z};
}

[[nodiscard]] glm::mat4 bodyToWorldMatrix(
    const physics::RigidBody& body,
    const physics::Transform& transform
) noexcept {
    const glm::quat orientation{
        transform.orientation.w,
        transform.orientation.x,
        transform.orientation.y,
        transform.orientation.z
    };
    const physics::Vec3 geometryCenter =
        transform.position -
        transform.orientation.rotate(body.centerOfMassLocal());
    return glm::translate(glm::mat4{1.0F}, toGlm(geometryCenter)) *
           glm::mat4_cast(orientation);
}

[[nodiscard]] physics::Transform interpolatedTransform(
    const physics::RigidBody& body,
    float alpha
) noexcept {
    const physics::Transform& previous = body.previousTransform();
    const physics::Transform& current = body.transform();
    const float amount = std::clamp(alpha, 0.0F, 1.0F);

    physics::Transform result{};
    result.position =
        previous.position * (1.0F - amount) +
        current.position * amount;
    const glm::quat previousOrientation{
        previous.orientation.w,
        previous.orientation.x,
        previous.orientation.y,
        previous.orientation.z
    };
    const glm::quat currentOrientation{
        current.orientation.w,
        current.orientation.x,
        current.orientation.y,
        current.orientation.z
    };
    const glm::quat orientation =
        glm::normalize(glm::slerp(
            previousOrientation,
            currentOrientation,
            amount
        ));
    result.orientation = {
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z
    };
    return result;
}

void glfwErrorCallback(int errorCode, const char* description) {
    std::fprintf(
        stderr,
        "GLFW error %d: %s\n",
        errorCode,
        description != nullptr ? description : "Unknown error"
    );
}

std::filesystem::path executableDirectory(const std::filesystem::path& executablePath) {
    if (executablePath.empty()) {
        return std::filesystem::current_path();
    }

    std::error_code error;
    const std::filesystem::path absolutePath = std::filesystem::absolute(executablePath, error);
    const std::filesystem::path& resolvedPath = error ? executablePath : absolutePath;
    if (resolvedPath.has_filename()) {
        const std::filesystem::path parent = resolvedPath.parent_path();
        return parent.empty() ? std::filesystem::current_path() : parent;
    }
    return resolvedPath;
}

bool keyPressed(GLFWwindow* window, int key) {
    return glfwGetKey(window, key) == GLFW_PRESS;
}

void applyStudioTheme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    // UE5-style neutral charcoal layers. Interaction states stay deliberately
    // monochrome so selection, docking, and editing read as one system.
    style.WindowPadding = ImVec2(8.0F, 7.0F);
    style.FramePadding = ImVec2(6.0F, 3.0F);
    style.CellPadding = ImVec2(6.0F, 4.0F);
    style.ItemSpacing = ImVec2(6.0F, 5.0F);
    style.ItemInnerSpacing = ImVec2(5.0F, 4.0F);
    style.ScrollbarSize = 12.0F;
    style.GrabMinSize = 9.0F;
    style.WindowBorderSize = 1.0F;
    style.ChildBorderSize = 1.0F;
    style.PopupBorderSize = 1.0F;
    style.FrameBorderSize = 1.0F;
    style.TabBorderSize = 1.0F;
    style.WindowRounding = 0.0F;
    style.ChildRounding = 0.0F;
    style.FrameRounding = 0.0F;
    style.PopupRounding = 0.0F;
    style.ScrollbarRounding = 0.0F;
    style.GrabRounding = 0.0F;
    style.TabRounding = 0.0F;
    style.WindowMenuButtonPosition = ImGuiDir_Right;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = uiColor(218, 218, 218);
    colors[ImGuiCol_TextDisabled] = uiColor(126, 126, 126);
    colors[ImGuiCol_WindowBg] = uiColor(15, 15, 15);
    colors[ImGuiCol_ChildBg] = uiColor(19, 19, 19);
    colors[ImGuiCol_PopupBg] = uiColor(24, 24, 24);
    colors[ImGuiCol_Border] = uiColor(45, 45, 45);
    colors[ImGuiCol_BorderShadow] = uiColor(0, 0, 0, 0);
    colors[ImGuiCol_FrameBg] = uiColor(29, 29, 29);
    colors[ImGuiCol_FrameBgHovered] = uiColor(39, 39, 39);
    colors[ImGuiCol_FrameBgActive] = uiColor(51, 51, 51);
    colors[ImGuiCol_TitleBg] = uiColor(12, 12, 12);
    colors[ImGuiCol_TitleBgActive] = uiColor(22, 22, 22);
    colors[ImGuiCol_TitleBgCollapsed] = colors[ImGuiCol_TitleBg];
    colors[ImGuiCol_MenuBarBg] = uiColor(10, 10, 10);
    colors[ImGuiCol_ScrollbarBg] = uiColor(13, 13, 13, 180);
    colors[ImGuiCol_ScrollbarGrab] = uiColor(55, 55, 55);
    colors[ImGuiCol_ScrollbarGrabHovered] = uiColor(72, 72, 72);
    colors[ImGuiCol_ScrollbarGrabActive] = uiColor(92, 92, 92);
    colors[ImGuiCol_CheckMark] = uiColor(198, 198, 198);
    colors[ImGuiCol_SliderGrab] = uiColor(119, 119, 119);
    colors[ImGuiCol_SliderGrabActive] = uiColor(174, 174, 174);
    colors[ImGuiCol_Button] = uiColor(31, 31, 31);
    colors[ImGuiCol_ButtonHovered] = uiColor(44, 44, 44);
    colors[ImGuiCol_ButtonActive] = uiColor(58, 58, 58);
    colors[ImGuiCol_Header] = uiColor(36, 36, 36);
    colors[ImGuiCol_HeaderHovered] = uiColor(49, 49, 49);
    colors[ImGuiCol_HeaderActive] = uiColor(61, 61, 61);
    colors[ImGuiCol_Separator] = uiColor(45, 45, 45);
    colors[ImGuiCol_SeparatorHovered] = uiColor(86, 86, 86);
    colors[ImGuiCol_SeparatorActive] = uiColor(122, 122, 122);
    colors[ImGuiCol_ResizeGrip] = uiColor(108, 108, 108, 45);
    colors[ImGuiCol_ResizeGripHovered] = uiColor(130, 130, 130, 120);
    colors[ImGuiCol_ResizeGripActive] = uiColor(165, 165, 165, 190);
    colors[ImGuiCol_Tab] = uiColor(18, 18, 18);
    colors[ImGuiCol_TabHovered] = uiColor(31, 31, 31);
    colors[ImGuiCol_TabSelected] = uiColor(36, 36, 36);
    colors[ImGuiCol_TabSelectedOverline] = uiColor(142, 142, 142);
    colors[ImGuiCol_TabDimmed] = uiColor(14, 14, 14);
    colors[ImGuiCol_TabDimmedSelected] = uiColor(23, 23, 23);
    colors[ImGuiCol_DockingPreview] = uiColor(112, 112, 112, 130);
    colors[ImGuiCol_DockingEmptyBg] = uiColor(12, 12, 12);
    colors[ImGuiCol_TableHeaderBg] = uiColor(26, 26, 26);
    colors[ImGuiCol_TableBorderStrong] = uiColor(45, 45, 45);
    colors[ImGuiCol_TableBorderLight] = uiColor(35, 35, 35);
    colors[ImGuiCol_TableRowBg] = uiColor(0, 0, 0, 0);
    colors[ImGuiCol_TableRowBgAlt] = uiColor(255, 255, 255, 5);
    colors[ImGuiCol_NavCursor] = uiColor(190, 190, 190);
}

bool inputFloatClamped(
    const char* label,
    float* value,
    float minimum,
    float maximum,
    const char* format = "%.3f"
) {
    const bool changed = ImGui::InputFloat(
        label,
        value,
        0.0F,
        0.0F,
        format,
        ImGuiInputTextFlags_CharsScientific
    );
    if (changed && minimum < maximum) {
        *value = std::clamp(*value, minimum, maximum);
    }
    return changed;
}

bool inputFloat3Clamped(
    const char* label,
    float value[3],
    float minimum,
    float maximum,
    const char* format = "%.3f"
) {
    const bool changed = ImGui::InputFloat3(
        label,
        value,
        format,
        ImGuiInputTextFlags_CharsScientific
    );
    if (changed && minimum < maximum) {
        value[0] = std::clamp(value[0], minimum, maximum);
        value[1] = std::clamp(value[1], minimum, maximum);
        value[2] = std::clamp(value[2], minimum, maximum);
    }
    return changed;
}

bool editableSliderFloat(
    const char* label,
    float* value,
    float minimum,
    float maximum,
    const char* format = "%.3f",
    ImGuiSliderFlags flags = 0
) {
    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    const float available = ImGui::GetContentRegionAvail().x;
    const float inputWidth = std::clamp(available * 0.34F, 68.0F, 106.0F);
    ImGui::SetNextItemWidth(std::max(42.0F, available - inputWidth - 6.0F));
    bool changed = ImGui::SliderFloat(
        "##Slider",
        value,
        minimum,
        maximum,
        format,
        flags
    );
    ImGui::SameLine();
    ImGui::SetNextItemWidth(inputWidth);
    changed |= ImGui::InputFloat(
        "##Value",
        value,
        0.0F,
        0.0F,
        format,
        ImGuiInputTextFlags_CharsScientific
    );
    if (changed) {
        *value = std::clamp(*value, minimum, maximum);
    }
    ImGui::PopID();
    return changed;
}

bool editableSliderInt(
    const char* label,
    int* value,
    int minimum,
    int maximum,
    const char* format = "%d",
    ImGuiSliderFlags flags = 0
) {
    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine();
    const float available = ImGui::GetContentRegionAvail().x;
    const float inputWidth = std::clamp(available * 0.30F, 64.0F, 92.0F);
    ImGui::SetNextItemWidth(std::max(42.0F, available - inputWidth - 6.0F));
    bool changed = ImGui::SliderInt(
        "##Slider",
        value,
        minimum,
        maximum,
        format,
        flags
    );
    ImGui::SameLine();
    ImGui::SetNextItemWidth(inputWidth);
    changed |= ImGui::InputInt(
        "##Value",
        value,
        0,
        0,
        ImGuiInputTextFlags_CharsDecimal
    );
    if (changed) {
        *value = std::clamp(*value, minimum, maximum);
    }
    ImGui::PopID();
    return changed;
}

void diagnosticRow(const char* label, const char* value) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextDisabled("%s", label);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted(value);
}

} // namespace

class StudioApplication::Impl {
public:
    explicit Impl(StudioApplicationConfig config)
        : config_(std::move(config)),
          executableDirectory_(executableDirectory(config_.executablePath)) {}

    ~Impl() {
        shutdownNoThrow();
    }

    int run() {
        try {
            initialize();

            using Clock = std::chrono::steady_clock;
            auto previousFrameTime = Clock::now();
            const auto smokeDeadline =
                Clock::now() + std::chrono::seconds(kSmokeTestTimeoutSeconds);
            std::uint64_t smokeFramesRendered = 0U;
            bool smokeTimedOut = false;
            bool smokeFirstResizeRequested = false;
            bool smokeSecondResizeRequested = false;
            bool smokeSceneRendered = false;

            while (!glfwWindowShouldClose(window_)) {
                glfwPollEvents();
                if (config_.smokeTest && Clock::now() >= smokeDeadline) {
                    smokeTimedOut = true;
                    glfwSetWindowShouldClose(window_, GLFW_TRUE);
                    continue;
                }

                int framebufferWidth = 0;
                int framebufferHeight = 0;
                glfwGetFramebufferSize(window_, &framebufferWidth, &framebufferHeight);
                if (framebufferWidth == 0 || framebufferHeight == 0) {
                    releaseCameraLook();
                    glfwWaitEventsTimeout(0.050);
                    previousFrameTime = Clock::now();
                    continue;
                }

                const auto currentFrameTime = Clock::now();
                const float deltaSeconds = std::clamp(
                    std::chrono::duration<float>(currentFrameTime - previousFrameTime).count(),
                    0.0F,
                    kMaximumFrameDeltaSeconds
                );
                previousFrameTime = currentFrameTime;

                renderer_.beginUiFrame();
                const ImGuiID dockspaceId = drawDockspace();
                updateSceneViewport(dockspaceId);
                drawPanels();
                updateCamera(deltaSeconds);
                updateViewportInteraction();
                advanceSimulation(deltaSeconds);
                publishSceneData();
                drawViewportOverlay(dockspaceId);
                renderer_.drawFrame(camera_, groundSettings_);

                if (config_.smokeTest) {
                    smokeFramesRendered =
                        renderer_.diagnostics().renderedFrames;
                    smokeSceneRendered =
                        smokeSceneRendered ||
                        renderer_.diagnostics().visibleCubeCount > 0U;
                    if (!smokeFirstResizeRequested &&
                        smokeFramesRendered >= 3U) {
                        glfwSetWindowSize(window_, 1024, 640);
                        renderer_.requestSwapchainRebuild();
                        smokeFirstResizeRequested = true;
                    } else if (
                        !smokeSecondResizeRequested &&
                        smokeFramesRendered >= 9U
                    ) {
                        glfwSetWindowSize(window_, 1280, 720);
                        renderer_.requestSwapchainRebuild();
                        smokeSecondResizeRequested = true;
                    }
                    if (!renderer_.diagnostics().validationEnabled ||
                        renderer_.diagnostics().validationErrors > 0U ||
                        smokeFramesRendered >= kSmokeTestFrameCount) {
                        glfwSetWindowShouldClose(window_, GLFW_TRUE);
                    }
                }
            }

            releaseCameraLook();
            renderer_.waitIdle();
            std::uint32_t validationErrors = renderer_.diagnostics().validationErrors;
            renderer_.shutdown();
            validationErrors = std::max(
                validationErrors,
                renderer_.diagnostics().validationErrors
            );
            destroyImGuiContext();
            destroyWindow();

            if (config_.smokeTest &&
                !renderer_.diagnostics().validationEnabled) {
                std::fprintf(
                    stderr,
                    "UAView Studio smoke test requires the Khronos validation layer.\n"
                );
                return 4;
            }
            if (config_.smokeTest && smokeTimedOut) {
                std::fprintf(
                    stderr,
                    "UAView Studio smoke test timed out after %d seconds.\n",
                    kSmokeTestTimeoutSeconds
                );
                return 5;
            }
            if (validationErrors > 0U) {
                std::fprintf(
                    stderr,
                    "UAView Studio detected %u Vulkan validation error(s).\n",
                    validationErrors
                );
                return 2;
            }
            if (config_.smokeTest && smokeFramesRendered < kSmokeTestFrameCount) {
                std::fprintf(
                    stderr,
                    "UAView Studio smoke test exited after %llu of %d required frames.\n",
                    static_cast<unsigned long long>(smokeFramesRendered),
                    kSmokeTestFrameCount
                );
                return 3;
            }
            if (config_.smokeTest && !smokeSceneRendered) {
                std::fprintf(
                    stderr,
                    "UAView Studio smoke test did not render the physics cube.\n"
                );
                return 6;
            }
            return 0;
        } catch (...) {
            shutdownNoThrow();
            throw;
        }
    }

private:
    void initialize() {
        glfwSetErrorCallback(glfwErrorCallback);
        if (glfwInit() != GLFW_TRUE) {
            throw std::runtime_error("Unable to initialize GLFW.");
        }
        glfwInitialized_ = true;

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
        glfwWindowHint(GLFW_VISIBLE, config_.smokeTest ? GLFW_FALSE : GLFW_TRUE);
        glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

        window_ = glfwCreateWindow(
            config_.smokeTest ? 1280 : kInitialWindowWidth,
            config_.smokeTest ? 720 : kInitialWindowHeight,
            "UAView Studio",
            nullptr,
            nullptr
        );
        if (window_ == nullptr) {
            throw std::runtime_error("Unable to create the UAView Studio window.");
        }

        glfwSetWindowSizeLimits(
            window_,
            kMinimumWindowWidth,
            kMinimumWindowHeight,
            GLFW_DONT_CARE,
            GLFW_DONT_CARE
        );
        glfwSetWindowUserPointer(window_, this);
        glfwSetFramebufferSizeCallback(
            window_,
            [](GLFWwindow* window, int width, int height) {
                auto* application = static_cast<Impl*>(glfwGetWindowUserPointer(window));
                if (application != nullptr && width > 0 && height > 0 &&
                    application->renderer_.isInitialized()) {
                    application->renderer_.requestSwapchainRebuild();
                }
            }
        );

        if (!config_.smokeTest) {
            centerWindow();
        }

        renderer_.initialize(window_, executableDirectory_, true);
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        imguiContextCreated_ = true;
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigWindowsMoveFromTitleBarOnly = true;
        // A click-release on a drag control enters text mode. This keeps the
        // compact editor layout while making every displayed number directly
        // keyboard-editable without requiring Ctrl-click.
        io.ConfigDragClickToInputText = true;
        // Keep the editor self-contained and safe to launch from read-only or
        // non-ASCII install paths. Docking remains fully adjustable per session.
        io.IniFilename = nullptr;
        loadStudioFont(io);
        applyStudioTheme();
        renderer_.initializeImGui(window_);
        resetSimulation();
        publishSceneData();
    }

    void loadStudioFont(ImGuiIO& io) {
        interFontLoaded_ = false;
        const std::filesystem::path fontPath =
            executableDirectory_ / "assets" / "fonts" / "InterVariable.ttf";

        std::ifstream fontFile(fontPath, std::ios::binary | std::ios::ate);
        if (fontFile) {
            const std::streamsize byteCount = fontFile.tellg();
            constexpr std::streamsize maximumFontBytes = 16 * 1024 * 1024;
            if (byteCount > 0 && byteCount <= maximumFontBytes) {
                fontFileBytes_.resize(static_cast<std::size_t>(byteCount));
                fontFile.seekg(0, std::ios::beg);
                if (fontFile.read(
                        reinterpret_cast<char*>(fontFileBytes_.data()),
                        byteCount
                    )) {
                    ImFontConfig fontConfig{};
                    fontConfig.FontDataOwnedByAtlas = false;
                    fontConfig.OversampleH = 2;
                    fontConfig.OversampleV = 2;
                    fontConfig.PixelSnapH = false;
                    ImFont* font = io.Fonts->AddFontFromMemoryTTF(
                        fontFileBytes_.data(),
                        static_cast<int>(fontFileBytes_.size()),
                        13.0F,
                        &fontConfig,
                        io.Fonts->GetGlyphRangesDefault()
                    );
                    if (font != nullptr) {
                        io.FontDefault = font;
                        interFontLoaded_ = true;
                    }
                }
            }
        }

        if (!interFontLoaded_) {
            io.FontDefault = io.Fonts->AddFontDefault();
            std::fprintf(
                stderr,
                "UAView Studio could not load staged InterVariable.ttf; "
                "using Dear ImGui's embedded fallback font.\n"
            );
        }
    }

    void resetSimulation() {
        releaseGrab();
        world_.reset();
        world_.setSettings(physicsSettings_);

        physics::BodyDescription ground =
            physics::BodyDescription::makeStaticPlane({0.0F, 1.0F, 0.0F}, 0.0F);
        ground.debugName = "Concrete Ground";
        ground.material.staticFriction = 0.82F;
        ground.material.dynamicFriction = 0.68F;
        ground.material.restitution = 0.02F;
        ground.material.rollingFriction = 0.02F;
        groundId_ = world_.createBody(ground);

        physics::BodyDescription cube =
            physics::BodyDescription::makeDenseCube({0.0F, 3.0F, 0.0F});
        cube.debugName = "Physics Cube";
        cube.collider = physics::Collider::makeBox({0.5F, 0.5F, 0.5F});
        cube.mass = 120.0F;
        cube.material.staticFriction = 0.65F;
        cube.material.dynamicFriction = 0.50F;
        cube.material.restitution = 0.05F;
        cube.material.rollingFriction = 0.015F;
        cube.aerodynamics.enabled = true;
        cube.aerodynamics.dragCoefficient = 1.05F;
        cube.aerodynamics.angularDragCoefficient = 0.08F;
        cube.aerodynamics.projectedAreaScale = 1.0F;
        cubeId_ = world_.createBody(cube);

        physics::DroneDescription droneDescription{};
        droneDescription.transform.position = {2.0F, 2.5F, 0.0F};
        droneDescription.startArmed = true;
        droneDescription.motors.variationEnabled =
            droneMotorVariationEnabled_;
        droneDescription.motors.variationFraction =
            droneMotorVariationFraction_;
        droneId_ = world_.createDrone(droneDescription);
        if (physics::Drone* drone = world_.drone(droneId_)) {
            droneBodyId_ = drone->bodyId();
            selectedBodyId_ = droneBodyId_;
            controlledDroneId_ = droneId_;
            followDroneCamera_ = true;
            followCameraInitialized_ = false;
        } else {
            droneId_ = physics::kInvalidDroneId;
            droneBodyId_ = physics::kInvalidBodyId;
            controlledDroneId_ = physics::kInvalidDroneId;
            selectedBodyId_ = cubeId_;
            followDroneCamera_ = false;
        }
        hoveredBodyId_ = physics::kInvalidBodyId;
        hoveredVertexMask_ = 0U;
        spawnerPlacementArmed_ = false;
        renderCurrentPoseUntilNextTick_ = false;
        nextCubeOrdinal_ = 2U;
        spawnRandomEngine_.seed(0x55415653U);
        spawnerStatus_ = "Ready.";
        singleStepRequested_ = false;
        leftMouseWasPressed_ = false;
    }

    void centerWindow() const {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        if (monitor == nullptr) {
            return;
        }

        int workX = 0;
        int workY = 0;
        int workWidth = 0;
        int workHeight = 0;
        glfwGetMonitorWorkarea(monitor, &workX, &workY, &workWidth, &workHeight);

        int windowWidth = 0;
        int windowHeight = 0;
        glfwGetWindowSize(window_, &windowWidth, &windowHeight);
        glfwSetWindowPos(
            window_,
            workX + std::max(0, (workWidth - windowWidth) / 2),
            workY + std::max(0, (workHeight - windowHeight) / 2)
        );
    }

    ImGuiID drawDockspace() {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);
        ImGui::SetNextWindowViewport(viewport->ID);

        constexpr ImGuiWindowFlags hostFlags =
            ImGuiWindowFlags_MenuBar |
            ImGuiWindowFlags_NoDocking |
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        ImGui::Begin("UAView Studio Dockspace", nullptr, hostFlags);
        ImGui::PopStyleVar(3);

        drawMenuBar();

        const ImGuiID dockspaceId = ImGui::GetID("UAViewStudioDockspace");
        if (resetDockLayout_ || ImGui::DockBuilderGetNode(dockspaceId) == nullptr) {
            buildDefaultDockLayout(dockspaceId, viewport->Size);
            resetDockLayout_ = false;
        }

        constexpr ImGuiDockNodeFlags dockspaceFlags =
            ImGuiDockNodeFlags_PassthruCentralNode |
            ImGuiDockNodeFlags_NoDockingOverCentralNode;
        ImGui::DockSpace(dockspaceId, ImVec2(0.0F, 0.0F), dockspaceFlags);
        ImGui::End();
        return dockspaceId;
    }

    void buildDefaultDockLayout(ImGuiID dockspaceId, const ImVec2& size) const {
        ImGui::DockBuilderRemoveNode(dockspaceId);
        ImGui::DockBuilderAddNode(
            dockspaceId,
            ImGuiDockNodeFlags_DockSpace |
                ImGuiDockNodeFlags_PassthruCentralNode |
                ImGuiDockNodeFlags_NoDockingOverCentralNode
        );
        ImGui::DockBuilderSetNodeSize(dockspaceId, size);

        ImGuiID center = dockspaceId;
        const ImGuiID left = ImGui::DockBuilderSplitNode(
            center,
            ImGuiDir_Left,
            0.18F,
            nullptr,
            &center
        );
        const ImGuiID right = ImGui::DockBuilderSplitNode(
            center,
            ImGuiDir_Right,
            0.24F,
            nullptr,
            &center
        );
        ImGuiID rightUpper = right;
        const ImGuiID rightLower = ImGui::DockBuilderSplitNode(
            rightUpper,
            ImGuiDir_Down,
            0.43F,
            nullptr,
            &rightUpper
        );

        ImGui::DockBuilderDockWindow("Scene", left);
        ImGui::DockBuilderDockWindow("Inspector", rightUpper);
        ImGui::DockBuilderDockWindow("Renderer", rightLower);
        ImGui::DockBuilderDockWindow("Physics", rightLower);
        ImGui::DockBuilderDockWindow("Drone", rightLower);
        ImGui::DockBuilderDockWindow("Spawner", rightLower);
        ImGui::DockBuilderFinish(dockspaceId);
    }

    void drawMenuBar() {
        if (!ImGui::BeginMenuBar()) {
            return;
        }

        ImGui::PushStyleColor(ImGuiCol_Text, uiColor(210, 210, 210));
        ImGui::TextUnformatted("UAView Studio");
        ImGui::PopStyleColor();
        ImGui::SameLine(0.0F, 18.0F);

        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem(
                    "Delete Selected Primitive",
                    "Delete",
                    false,
                    selectedPrimitiveCanBeDeleted()
                )) {
                deleteSelectedPrimitive();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Scene", nullptr, &showScenePanel_);
            ImGui::MenuItem("Inspector", nullptr, &showInspectorPanel_);
            ImGui::MenuItem("Renderer", nullptr, &showRendererPanel_);
            ImGui::MenuItem("Physics", nullptr, &showPhysicsPanel_);
            ImGui::MenuItem("Drone", nullptr, &showDronePanel_);
            ImGui::MenuItem("Spawner", nullptr, &showSpawnerPanel_);
            ImGui::MenuItem("Viewport Controls", nullptr, &showViewportOverlay_);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset Panel Layout")) {
                resetDockLayout_ = true;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Camera")) {
            if (ImGui::MenuItem("Reset View", "Home")) {
                resetCamera();
            }
            ImGui::MenuItem(
                "Follow Controlled Drone",
                nullptr,
                &followDroneCamera_,
                world_.drone(controlledDroneId_) != nullptr
            );
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem(
                    simulationPaused_ ? "Resume" : "Pause",
                    "Space"
                )) {
                simulationPaused_ = !simulationPaused_;
            }
            if (ImGui::MenuItem("Step Fixed Tick", ".", false, simulationPaused_)) {
                singleStepRequested_ = true;
            }
            if (ImGui::MenuItem("Reset Scene", "Ctrl+R")) {
                resetSimulation();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About UAView Studio")) {
                openAboutPopup_ = true;
            }
            ImGui::EndMenu();
        }

        ImGui::SameLine(0.0F, 14.0F);
        if (ImGui::SmallButton(simulationPaused_ ? "Resume" : "Pause")) {
            simulationPaused_ = !simulationPaused_;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!simulationPaused_);
        if (ImGui::SmallButton("Step")) {
            singleStepRequested_ = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset")) {
            resetSimulation();
        }

        const char* milestoneLabel =
            "EDITOR UPDATE - LIGHTS, SHADOWS & SPAWNING";
        const float labelWidth = ImGui::CalcTextSize(milestoneLabel).x;
        const float rightEdge = ImGui::GetWindowContentRegionMax().x;
        if (ImGui::GetCursorPosX() + labelWidth + 12.0F < rightEdge) {
            ImGui::SetCursorPosX(rightEdge - labelWidth);
            ImGui::TextDisabled("%s", milestoneLabel);
        }

        ImGui::EndMenuBar();

        if (openAboutPopup_) {
            ImGui::OpenPopup("About UAView Studio");
            openAboutPopup_ = false;
        }
        if (ImGui::BeginPopupModal(
                "About UAView Studio",
                nullptr,
                ImGuiWindowFlags_AlwaysAutoResize
            )) {
            ImGui::TextUnformatted("UAView Studio");
            ImGui::TextDisabled(
                "Physics, Environment, Lighting & Spawning"
            );
            ImGui::Spacing();
            ImGui::TextWrapped(
                "A deterministic 120 Hz rigid-body laboratory with SI-unit "
                "materials, atmosphere, wind, configurable spawning, "
                "cascaded sun shadows, editor lights, collision diagnostics, "
                "and interactive point-spring manipulation."
            );
            ImGui::TextDisabled(
                "UI font: %s",
                interFontLoaded_ ? "Inter Variable" : "ImGui fallback"
            );
            ImGui::Spacing();
            if (ImGui::Button("Close", ImVec2(96.0F, 0.0F))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    void drawPanels() {
        if (showScenePanel_) {
            drawScenePanel();
        }
        if (showInspectorPanel_) {
            drawInspectorPanel();
        }
        if (showRendererPanel_) {
            drawRendererPanel();
        }
        if (showPhysicsPanel_) {
            drawPhysicsPanel();
        }
        if (showDronePanel_) {
            drawDronePanel();
        }
        if (showSpawnerPanel_) {
            drawSpawnerPanel();
        }
    }

    void drawScenePanel() {
        if (!ImGui::Begin("Scene", &showScenePanel_)) {
            ImGui::End();
            return;
        }

        ImGui::TextDisabled("SCENE CONTENTS");
        ImGui::Spacing();
        if (ImGui::Button("+ Cube")) {
            spawnUsingConfiguredPlacement();
        }
        ImGui::SameLine();
        if (ImGui::Button("+ Drone")) {
            spawnDrone();
        }
        ImGui::SameLine();
        if (ImGui::Button("Spawner settings")) {
            showSpawnerPanel_ = true;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!selectedPrimitiveCanBeDeleted());
        if (ImGui::Button("Delete")) {
            deleteSelectedPrimitive();
        }
        ImGui::EndDisabled();
        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0F, 5.0F));
        std::size_t liveObjectCount = 0U;
        physics::BodyId contextDeleteId = physics::kInvalidBodyId;
        for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
            const auto id = static_cast<physics::BodyId>(slot);
            const physics::RigidBody* body = world_.body(id);
            if (body == nullptr) {
                continue;
            }
            ++liveObjectCount;
            ImGui::PushID(static_cast<int>(id));
            if (ImGui::Selectable(
                    body->debugName(),
                    selectedBodyId_ == id,
                    ImGuiSelectableFlags_SpanAllColumns
                )) {
                selectedBodyId_ = id;
                if (physics::Drone* selectedDrone =
                        world_.droneForBody(id)) {
                    controlledDroneId_ = selectedDrone->id();
                    droneBodyId_ = id;
                    showDronePanel_ = true;
                }
            }
            if (ImGui::BeginPopupContextItem("##SceneObjectContext")) {
                selectedBodyId_ = id;
                ImGui::TextDisabled("%s", body->debugName());
                ImGui::Separator();
                if (ImGui::MenuItem(
                        "Delete Primitive",
                        "Delete",
                        false,
                        primitiveCanBeDeleted(id)
                    )) {
                    contextDeleteId = id;
                }
                ImGui::EndPopup();
            }
            ImGui::PopID();
        }
        ImGui::Separator();
        ImGui::TextDisabled("LIGHTS");
        if (ImGui::Selectable(
                groundSettings_.sunEnabled ? "Sun Light" : "Sun Light (disabled)",
                false
            )) {
            showRendererPanel_ = true;
            ImGui::SetWindowFocus("Renderer");
        }
        if (ImGui::Selectable(
                groundSettings_.environmentEnabled
                    ? "Environment Light"
                    : "Environment Light (disabled)",
                false
            )) {
            showRendererPanel_ = true;
            ImGui::SetWindowFocus("Renderer");
        }
        for (std::size_t index = 0U; index < spotLights_.size(); ++index) {
            ImGui::PushID(static_cast<int>(index));
            const std::string label =
                "Spot Light " + std::to_string(index + 1U);
            if (ImGui::Selectable(label.c_str(), false)) {
                showRendererPanel_ = true;
                ImGui::SetWindowFocus("Renderer");
            }
            ImGui::PopID();
        }
        ImGui::PopStyleVar();

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled(
            "%zu objects",
            liveObjectCount + spotLights_.size() + 2U
        );
        ImGui::End();
        if (contextDeleteId != physics::kInvalidBodyId) {
            selectedBodyId_ = contextDeleteId;
            deleteSelectedPrimitive();
        }
    }

    [[nodiscard]] bool primitiveCanBeDeleted(
        physics::BodyId id
    ) const noexcept {
        const physics::RigidBody* body = world_.body(id);
        return body != nullptr &&
               id != groundId_ &&
               body->collider().type != physics::ColliderType::Plane;
    }

    [[nodiscard]] bool selectedPrimitiveCanBeDeleted() const noexcept {
        return primitiveCanBeDeleted(selectedBodyId_);
    }

    void deleteSelectedPrimitive() {
        const physics::BodyId id = selectedBodyId_;
        const physics::RigidBody* body = world_.body(id);
        if (!primitiveCanBeDeleted(id) || body == nullptr) {
            return;
        }

        const std::string deletedName = body->debugName();
        if (grabbedBodyId_ == id) {
            releaseGrab();
        }
        if (hoveredBodyId_ == id) {
            clearHover();
        }

        if (!world_.destroyBody(id)) {
            return;
        }
        if (cubeId_ == id) {
            cubeId_ = physics::kInvalidBodyId;
        }
        if (droneBodyId_ == id) {
            droneId_ = physics::kInvalidDroneId;
            controlledDroneId_ = physics::kInvalidDroneId;
            droneBodyId_ = physics::kInvalidBodyId;
            followDroneCamera_ = false;
            followCameraInitialized_ = false;
        }
        selectedBodyId_ = physics::kInvalidBodyId;
        spawnerStatus_ = deletedName + " deleted.";
    }

    [[nodiscard]] physics::Vec3 spawnHalfExtents() const noexcept {
        return {
            std::clamp(spawnerSettings_.size.x, 0.02F, 100.0F) * 0.5F,
            std::clamp(spawnerSettings_.size.y, 0.02F, 100.0F) * 0.5F,
            std::clamp(spawnerSettings_.size.z, 0.02F, 100.0F) * 0.5F
        };
    }

    [[nodiscard]] physics::Vec3 projectSpawnOutsidePlanes(
        physics::Vec3 center,
        const physics::Vec3& halfExtents
    ) const noexcept {
        for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
            const physics::RigidBody* body =
                world_.body(static_cast<physics::BodyId>(slot));
            if (body == nullptr ||
                body->collider().type != physics::ColliderType::Plane) {
                continue;
            }
            const physics::PlaneCollider& plane = body->collider().plane;
            const physics::Vec3 normal = physics::normalized(plane.normal);
            const float support =
                std::abs(normal.x) * halfExtents.x +
                std::abs(normal.y) * halfExtents.y +
                std::abs(normal.z) * halfExtents.z;
            const float separation =
                physics::dot(normal, center) - plane.offset - support;
            if (separation < kSpawnClearanceMeters) {
                center +=
                    normal * (kSpawnClearanceMeters - separation);
            }
        }
        return center;
    }

    [[nodiscard]] bool spawnLocationIsFree(
        const physics::Vec3& center,
        const physics::Vec3& halfExtents
    ) const noexcept {
        if (!physics::isFinite(center) || !physics::isFinite(halfExtents)) {
            return false;
        }
        const physics::Aabb candidate{
            center - halfExtents,
            center + halfExtents
        };
        for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
            const physics::RigidBody* body =
                world_.body(static_cast<physics::BodyId>(slot));
            if (body == nullptr) {
                continue;
            }
            if (body->collider().type == physics::ColliderType::Plane) {
                const physics::PlaneCollider& plane =
                    body->collider().plane;
                const physics::Vec3 normal =
                    physics::normalized(plane.normal);
                const float support =
                    std::abs(normal.x) * halfExtents.x +
                    std::abs(normal.y) * halfExtents.y +
                    std::abs(normal.z) * halfExtents.z;
                if (physics::dot(normal, center) - plane.offset - support <
                    1.0e-5F) {
                    return false;
                }
                continue;
            }
            // Placement helpers already add the requested visible clearance.
            // Keep only a tiny numerical guard here so a cube placed exactly
            // above a clicked box is not mistaken for an overlap and moved
            // sideways by the fallback search.
            if (candidate.overlaps(body->worldAabb(), 1.0e-5F)) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool findNearestSpawnLocation(
        const physics::Vec3& preferred,
        physics::Vec3& result
    ) const noexcept {
        const physics::Vec3 halfExtents = spawnHalfExtents();
        const physics::Vec3 planeSafePreferred =
            spawnerSettings_.avoidOverlaps
                ? projectSpawnOutsidePlanes(preferred, halfExtents)
                : preferred;
        if (!spawnerSettings_.avoidOverlaps ||
            spawnLocationIsFree(planeSafePreferred, halfExtents)) {
            result = planeSafePreferred;
            return true;
        }

        float spacing =
            std::max(spawnerSettings_.size.x, spawnerSettings_.size.z) +
            2.0F * kSpawnClearanceMeters;
        for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
            const physics::RigidBody* body =
                world_.body(static_cast<physics::BodyId>(slot));
            if (body == nullptr ||
                body->collider().type != physics::ColliderType::Box) {
                continue;
            }
            const physics::Aabb bounds = body->worldAabb();
            spacing = std::max(
                spacing,
                std::max(
                    bounds.maximum.x - bounds.minimum.x,
                    bounds.maximum.z - bounds.minimum.z
                ) + std::max(
                    spawnerSettings_.size.x,
                    spawnerSettings_.size.z
                ) + 2.0F * kSpawnClearanceMeters
            );
        }
        constexpr int samplesPerRing = 16;
        constexpr int maximumRings = 32;
        for (int ring = 1; ring <= maximumRings; ++ring) {
            const float radius = spacing * static_cast<float>(ring);
            for (int sample = 0; sample < samplesPerRing; ++sample) {
                const float angle =
                    2.0F * kPi * static_cast<float>(sample) /
                    static_cast<float>(samplesPerRing);
                const physics::Vec3 candidate{
                    planeSafePreferred.x + std::cos(angle) * radius,
                    planeSafePreferred.y,
                    planeSafePreferred.z + std::sin(angle) * radius
                };
                if (spawnLocationIsFree(candidate, halfExtents)) {
                    result = candidate;
                    return true;
                }
            }
        }
        return false;
    }

    [[nodiscard]] bool chooseRandomSpawnLocation(
        physics::Vec3& result
    ) noexcept {
        const physics::Vec3 halfExtents = spawnHalfExtents();
        const float radius =
            std::clamp(spawnerSettings_.randomRadiusMeters, 0.5F, 500.0F);
        std::uniform_real_distribution<float> unitDistribution(0.0F, 1.0F);
        for (int attempt = 0; attempt < 160; ++attempt) {
            const float angle =
                2.0F * kPi * unitDistribution(spawnRandomEngine_);
            const float distance =
                std::sqrt(unitDistribution(spawnRandomEngine_)) * radius;
            const physics::Vec3 candidate{
                std::cos(angle) * distance,
                halfExtents.y + kSpawnClearanceMeters,
                std::sin(angle) * distance
            };
            if (!spawnerSettings_.avoidOverlaps ||
                spawnLocationIsFree(candidate, halfExtents)) {
                result = candidate;
                return true;
            }
        }

        const physics::Vec3 origin{
            0.0F,
            halfExtents.y + kSpawnClearanceMeters,
            0.0F
        };
        return findNearestSpawnLocation(origin, result);
    }

    physics::DroneId spawnDrone() {
        physics::DroneDescription description{};
        description.startArmed = true;
        description.motors.variationEnabled =
            droneMotorVariationEnabled_;
        description.motors.variationFraction =
            droneMotorVariationFraction_;

        physics::Vec3 position = projectSpawnOutsidePlanes(
            {2.0F, 2.5F, 0.0F},
            description.bodyHalfExtents
        );
        if (!spawnLocationIsFree(position, description.bodyHalfExtents)) {
            bool found = false;
            for (int ring = 1; ring <= 24 && !found; ++ring) {
                const float radius = static_cast<float>(ring) * 0.8F;
                for (int sample = 0; sample < 12; ++sample) {
                    const float angle =
                        2.0F * kPi * static_cast<float>(sample) / 12.0F;
                    physics::Vec3 candidate{
                        position.x + std::cos(angle) * radius,
                        position.y,
                        position.z + std::sin(angle) * radius
                    };
                    candidate = projectSpawnOutsidePlanes(
                        candidate,
                        description.bodyHalfExtents
                    );
                    if (spawnLocationIsFree(
                            candidate,
                            description.bodyHalfExtents
                        )) {
                        position = candidate;
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                spawnerStatus_ =
                    "No collision-free drone spawn location was found.";
                return physics::kInvalidDroneId;
            }
        }

        description.transform.position = position;
        const physics::DroneId id = world_.createDrone(description);
        physics::Drone* drone = world_.drone(id);
        if (drone == nullptr) {
            spawnerStatus_ = "Drone creation failed.";
            return physics::kInvalidDroneId;
        }
        droneId_ = id;
        controlledDroneId_ = id;
        droneBodyId_ = drone->bodyId();
        selectedBodyId_ = droneBodyId_;
        followDroneCamera_ = true;
        followCameraInitialized_ = false;
        showDronePanel_ = true;
        spawnerStatus_ = "Quadcopter spawned and armed.";
        return id;
    }

    physics::BodyId spawnCubeAt(const physics::Vec3& center) {
        spawnerStatus_.clear();
        if (!physics::isFinite(center)) {
            spawnerStatus_ = "Spawn rejected: position is not finite.";
            return physics::kInvalidBodyId;
        }

        physics::BodyDescription description =
            physics::BodyDescription::makeDenseCube(center);
        const std::string name =
            "Cube " + std::to_string(nextCubeOrdinal_++);
        description.debugName = name.c_str();
        description.collider = physics::Collider::makeBox(spawnHalfExtents());
        description.mass =
            std::clamp(spawnerSettings_.massKilograms, 0.01F, 100'000.0F);
        description.material.staticFriction = 0.65F;
        description.material.dynamicFriction = 0.50F;
        description.material.restitution = 0.05F;
        description.material.rollingFriction = 0.015F;
        description.aerodynamics.enabled = true;
        description.aerodynamics.dragCoefficient = 1.05F;
        description.aerodynamics.angularDragCoefficient = 0.08F;
        description.aerodynamics.projectedAreaScale = 1.0F;

        const physics::BodyId id = world_.createBody(description);
        physics::RigidBody* body = world_.body(id);
        if (body == nullptr) {
            spawnerStatus_ = "Spawn failed: the physics body was not created.";
            return physics::kInvalidBodyId;
        }

        if (spawnerSettings_.initialImpulseEnabled) {
            body->applyImpulseAtWorldPoint(
                spawnerSettings_.initialImpulseNewtonSeconds,
                body->transform().position
            );
        }
        if (spawnerSettings_.initialAngularMomentumEnabled) {
            body->applyAngularImpulse(
                spawnerSettings_.initialAngularMomentumNewtonMeterSeconds
            );
        }
        if (spawnerSettings_.timedForceEnabled &&
            spawnerSettings_.timedForceDurationSeconds > 0.0F) {
            physics::TimedBodyForce timedForce{};
            timedForce.body = id;
            timedForce.force = spawnerSettings_.timedForceNewtons;
            timedForce.torque = spawnerSettings_.timedTorqueNewtonMeters;
            timedForce.remainingSeconds =
                spawnerSettings_.timedForceDurationSeconds;
            timedForce.forceInBodyFrame =
                spawnerSettings_.timedForceBodyLocal;
            timedForce.torqueInBodyFrame =
                spawnerSettings_.timedForceBodyLocal;
            timedForce.enabled = true;
            if (world_.createTimedForce(timedForce) ==
                physics::kInvalidTimedForceId) {
                spawnerStatus_ =
                    "Cube created, but its timed force was rejected.";
            }
        }

        selectedBodyId_ = id;
        if (spawnerStatus_.empty() ||
            spawnerStatus_.find("rejected") == std::string::npos) {
            spawnerStatus_ =
                name + " spawned at (" +
                std::to_string(center.x) + ", " +
                std::to_string(center.y) + ", " +
                std::to_string(center.z) + ").";
        }
        return id;
    }

    void spawnUsingConfiguredPlacement() {
        physics::Vec3 position{};
        switch (spawnerSettings_.placement) {
        case SpawnPlacementMode::ClickSurface:
            spawnerPlacementArmed_ = true;
            spawnerStatus_ = "Click a surface in the scene viewport.";
            return;
        case SpawnPlacementMode::OriginOrNearestFree: {
            const physics::Vec3 halfExtents = spawnHalfExtents();
            const physics::Vec3 origin{
                0.0F,
                halfExtents.y + kSpawnClearanceMeters,
                0.0F
            };
            if (!findNearestSpawnLocation(origin, position)) {
                spawnerStatus_ = "No collision-free spawn position was found.";
                return;
            }
            break;
        }
        case SpawnPlacementMode::RandomEmpty:
            if (!chooseRandomSpawnLocation(position)) {
                spawnerStatus_ = "No random empty spawn position was found.";
                return;
            }
            break;
        case SpawnPlacementMode::Predefined:
            if (!findNearestSpawnLocation(
                    spawnerSettings_.predefinedPosition,
                    position
                )) {
                spawnerStatus_ =
                    "The predefined area is occupied and no nearby gap was found.";
                return;
            }
            break;
        }
        spawnerPlacementArmed_ = false;
        spawnCubeAt(position);
    }

    bool spawnCubeFromViewportRay(
        const physics::Vec3& rayOrigin,
        const physics::Vec3& rayDirection
    ) {
        physics::RaycastHit hit{};
        if (!world_.raycast(
                rayOrigin,
                rayDirection,
                kMaximumPickDistanceMeters,
                hit
            )) {
            spawnerStatus_ = "No scene surface was under the cursor.";
            return false;
        }

        const physics::Vec3 halfExtents = spawnHalfExtents();
        const physics::Vec3 absoluteNormal{
            std::abs(hit.normal.x),
            std::abs(hit.normal.y),
            std::abs(hit.normal.z)
        };
        const float support =
            absoluteNormal.x * halfExtents.x +
            absoluteNormal.y * halfExtents.y +
            absoluteNormal.z * halfExtents.z;
        const physics::Vec3 preferred =
            hit.point + hit.normal * (support + kSpawnClearanceMeters);
        physics::Vec3 position{};
        if (!findNearestSpawnLocation(preferred, position)) {
            spawnerStatus_ = "No collision-free position was found near the click.";
            return false;
        }
        return spawnCubeAt(position) != physics::kInvalidBodyId;
    }

    void drawSpawnerPanel() {
        if (!ImGui::Begin("Spawner", &showSpawnerPanel_)) {
            ImGui::End();
            return;
        }

        ImGui::TextUnformatted("CUBE SPAWNER");
        ImGui::TextDisabled(
            "Creates real rigid bodies with collision-safe placement and SI-unit launch forces."
        );
        ImGui::Spacing();

        if (ImGui::CollapsingHeader(
                "Placement",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            constexpr std::array<const char*, 4U> placementNames{
                "Click in viewport",
                "Origin / nearest empty",
                "Random empty spot",
                "Predefined position"
            };
            int placement = static_cast<int>(spawnerSettings_.placement);
            if (ImGui::Combo(
                    "Spawn at",
                    &placement,
                    placementNames.data(),
                    static_cast<int>(placementNames.size())
                )) {
                spawnerSettings_.placement =
                    static_cast<SpawnPlacementMode>(placement);
                spawnerPlacementArmed_ = false;
            }
            ImGui::Checkbox(
                "Avoid overlapping bodies",
                &spawnerSettings_.avoidOverlaps
            );

            if (spawnerSettings_.placement == SpawnPlacementMode::RandomEmpty) {
                inputFloatClamped(
                    "Search radius (m)",
                    &spawnerSettings_.randomRadiusMeters,
                    0.5F,
                    500.0F,
                    "%.2f"
                );
            } else if (
                spawnerSettings_.placement == SpawnPlacementMode::Predefined
            ) {
                float position[3]{
                    spawnerSettings_.predefinedPosition.x,
                    spawnerSettings_.predefinedPosition.y,
                    spawnerSettings_.predefinedPosition.z
                };
                if (inputFloat3Clamped(
                        "Location (m)",
                        position,
                        -10'000.0F,
                        10'000.0F
                    )) {
                    spawnerSettings_.predefinedPosition = {
                        position[0], position[1], position[2]
                    };
                }
            }
        }

        if (ImGui::CollapsingHeader(
                "Cube",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            float size[3]{
                spawnerSettings_.size.x,
                spawnerSettings_.size.y,
                spawnerSettings_.size.z
            };
            if (inputFloat3Clamped("Size (m)", size, 0.02F, 100.0F)) {
                spawnerSettings_.size = {size[0], size[1], size[2]};
            }
            inputFloatClamped(
                "Mass (kg)",
                &spawnerSettings_.massKilograms,
                0.01F,
                100'000.0F,
                "%.2f"
            );
        }

        if (ImGui::CollapsingHeader(
                "Launch",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox(
                "Initial linear impulse",
                &spawnerSettings_.initialImpulseEnabled
            );
            if (spawnerSettings_.initialImpulseEnabled) {
                float impulse[3]{
                    spawnerSettings_.initialImpulseNewtonSeconds.x,
                    spawnerSettings_.initialImpulseNewtonSeconds.y,
                    spawnerSettings_.initialImpulseNewtonSeconds.z
                };
                if (inputFloat3Clamped(
                        "Impulse (N s)",
                        impulse,
                        -1.0e7F,
                        1.0e7F
                    )) {
                    spawnerSettings_.initialImpulseNewtonSeconds = {
                        impulse[0], impulse[1], impulse[2]
                    };
                }
            }

            ImGui::Checkbox(
                "Initial angular momentum",
                &spawnerSettings_.initialAngularMomentumEnabled
            );
            if (spawnerSettings_.initialAngularMomentumEnabled) {
                float momentum[3]{
                    spawnerSettings_.initialAngularMomentumNewtonMeterSeconds.x,
                    spawnerSettings_.initialAngularMomentumNewtonMeterSeconds.y,
                    spawnerSettings_.initialAngularMomentumNewtonMeterSeconds.z
                };
                if (inputFloat3Clamped(
                        "Angular momentum (N m s)",
                        momentum,
                        -1.0e7F,
                        1.0e7F
                    )) {
                    spawnerSettings_.initialAngularMomentumNewtonMeterSeconds = {
                        momentum[0], momentum[1], momentum[2]
                    };
                }
                ImGui::TextDisabled(
                    "This is physical angular impulse, not a scripted spin."
                );
            }

            ImGui::Checkbox(
                "Constant force for a duration",
                &spawnerSettings_.timedForceEnabled
            );
            if (spawnerSettings_.timedForceEnabled) {
                float force[3]{
                    spawnerSettings_.timedForceNewtons.x,
                    spawnerSettings_.timedForceNewtons.y,
                    spawnerSettings_.timedForceNewtons.z
                };
                if (inputFloat3Clamped(
                        "Force (N)",
                        force,
                        -1.0e8F,
                        1.0e8F
                    )) {
                    spawnerSettings_.timedForceNewtons = {
                        force[0], force[1], force[2]
                    };
                }
                float torque[3]{
                    spawnerSettings_.timedTorqueNewtonMeters.x,
                    spawnerSettings_.timedTorqueNewtonMeters.y,
                    spawnerSettings_.timedTorqueNewtonMeters.z
                };
                if (inputFloat3Clamped(
                        "Torque (N m)",
                        torque,
                        -1.0e8F,
                        1.0e8F
                    )) {
                    spawnerSettings_.timedTorqueNewtonMeters = {
                        torque[0], torque[1], torque[2]
                    };
                }
                inputFloatClamped(
                    "Duration (s)",
                    &spawnerSettings_.timedForceDurationSeconds,
                    0.001F,
                    600.0F,
                    "%.3f"
                );
                ImGui::Checkbox(
                    "Direction follows body rotation",
                    &spawnerSettings_.timedForceBodyLocal
                );
            }
        }

        ImGui::Spacing();
        const char* action =
            spawnerSettings_.placement == SpawnPlacementMode::ClickSurface
                ? (spawnerPlacementArmed_
                    ? "Waiting for viewport click..."
                    : "Arm viewport placement")
                : "Spawn cube";
        ImGui::BeginDisabled(spawnerPlacementArmed_);
        if (ImGui::Button(action, ImVec2(-1.0F, 30.0F))) {
            spawnUsingConfiguredPlacement();
        }
        ImGui::EndDisabled();
        if (spawnerPlacementArmed_) {
            if (ImGui::Button("Cancel placement", ImVec2(-1.0F, 0.0F))) {
                spawnerPlacementArmed_ = false;
                spawnerStatus_ = "Viewport placement cancelled.";
            }
        }
        ImGui::TextWrapped("%s", spawnerStatus_.c_str());
        ImGui::End();
    }

    void drawInspectorPanel() {
        if (!ImGui::Begin("Inspector", &showInspectorPanel_)) {
            ImGui::End();
            return;
        }

        physics::RigidBody* body = world_.body(selectedBodyId_);
        if (body == nullptr) {
            ImGui::TextDisabled("No physics body selected.");
            ImGui::End();
            return;
        }

        const physics::BodyId inspectedBodyId = body->id();
        ImGui::PushID(static_cast<int>(body->id()));
        ImGui::TextUnformatted(body->debugName());
        ImGui::TextDisabled(
            body->motionType() == physics::MotionType::Dynamic
                ? "Dynamic rigid body"
                : "Static collision environment"
        );
        ImGui::Spacing();

        const float inspectorControlWidth =
            std::max(112.0F, ImGui::GetContentRegionAvail().x * 0.54F);
        ImGui::PushItemWidth(inspectorControlWidth);

        if (ImGui::CollapsingHeader(
                "Transform & Motion",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            if (body->motionType() == physics::MotionType::Dynamic) {
                physics::Transform transform = body->transform();
                float position[3]{
                    transform.position.x,
                    transform.position.y,
                    transform.position.z
                };
                if (inputFloat3Clamped(
                        "Position (m)",
                        position,
                        -500.0F,
                        500.0F
                    )) {
                    transform.position = {position[0], position[1], position[2]};
                    body->setTransform(transform);
                }

                physics::Vec3 linearVelocity = body->linearVelocity();
                float velocity[3]{
                    linearVelocity.x,
                    linearVelocity.y,
                    linearVelocity.z
                };
                if (inputFloat3Clamped(
                        "Velocity (m/s)",
                        velocity,
                        -500.0F,
                        500.0F
                    )) {
                    body->setLinearVelocity({velocity[0], velocity[1], velocity[2]});
                }

                physics::Vec3 angularVelocity = body->angularVelocity();
                float angular[3]{
                    angularVelocity.x,
                    angularVelocity.y,
                    angularVelocity.z
                };
                if (inputFloat3Clamped(
                        "Angular (rad/s)",
                        angular,
                        -100.0F,
                        100.0F
                    )) {
                    body->setAngularVelocity({angular[0], angular[1], angular[2]});
                }
                ImGui::TextDisabled(
                    "State: %s",
                    body->isSleeping() ? "Sleeping" : "Awake"
                );
            } else {
                ImGui::TextDisabled("The ground plane is fixed at Y = 0 m.");
            }
        }

        if (ImGui::CollapsingHeader(
                "Mass Properties",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            if (body->motionType() == physics::MotionType::Dynamic) {
                float mass = body->mass();
                if (inputFloatClamped(
                        "Mass (kg)",
                        &mass,
                        0.01F,
                        100'000.0F,
                        "%.2f"
                    )) {
                    body->setMass(mass);
                }

                if (body->collider().type == physics::ColliderType::Box) {
                    const physics::Vec3 half = body->collider().box.halfExtents;
                    float dimensions[3]{
                        half.x * 2.0F,
                        half.y * 2.0F,
                        half.z * 2.0F
                    };
                    if (inputFloat3Clamped(
                            "Size (m)",
                            dimensions,
                            0.01F,
                            100.0F
                        )) {
                        body->setBoxHalfExtents({
                            dimensions[0] * 0.5F,
                            dimensions[1] * 0.5F,
                            dimensions[2] * 0.5F
                        });
                    }
                }

                physics::Vec3 centerOfMass = body->centerOfMassLocal();
                float center[3]{
                    centerOfMass.x,
                    centerOfMass.y,
                    centerOfMass.z
                };
                if (inputFloat3Clamped(
                        "CoM offset (m)",
                        center,
                        -10.0F,
                        10.0F
                    )) {
                    body->setCenterOfMassLocal({center[0], center[1], center[2]});
                }

                bool allowSleep = body->allowsSleep();
                if (ImGui::Checkbox("Allow sleeping", &allowSleep)) {
                    body->setAllowSleep(allowSleep);
                }
                ImGui::TextDisabled(
                    "Volume %.3f m3   Density %.1f kg/m3",
                    body->volume(),
                    body->bulkDensity()
                );
                bool customInertia =
                    body->usesCustomLocalInertiaTensor();
                if (ImGui::Checkbox(
                        "Explicit inertia tensor",
                        &customInertia
                    )) {
                    if (customInertia) {
                        (void)body->setCustomLocalInertiaTensor(
                            body->lockedAssemblyInertiaTensor()
                        );
                    } else {
                        body->useAutomaticLocalInertiaTensor();
                    }
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip(
                        "Use a measured inertia tensor about the body CoM. "
                        "Invalid or non-physical tensors are rejected."
                    );
                }

                if (body->usesCustomLocalInertiaTensor()) {
                    const physics::Mat3 locked =
                        body->lockedAssemblyInertiaTensor();
                    float diagonal[3]{
                        locked.m00,
                        locked.m11,
                        locked.m22
                    };
                    float products[3]{
                        locked.m01,
                        locked.m02,
                        locked.m12
                    };
                    bool inertiaChanged = ImGui::DragFloat3(
                        "Principal diagonal",
                        diagonal,
                        0.01F,
                        0.0001F,
                        1.0e8F,
                        "%.4f kg m2"
                    );
                    inertiaChanged |= ImGui::DragFloat3(
                        "Products xy/xz/yz",
                        products,
                        0.005F,
                        -1.0e8F,
                        1.0e8F,
                        "%.4f kg m2"
                    );
                    if (inertiaChanged) {
                        (void)body->setCustomLocalInertiaTensor({
                            diagonal[0], products[0], products[1],
                            products[0], diagonal[1], products[2],
                            products[1], products[2], diagonal[2],
                        });
                    }
                }

                const physics::Vec3 inertia = body->localInertia();
                ImGui::TextDisabled(
                    "Effective inertia %.2f  %.2f  %.2f kg m2",
                    inertia.x,
                    inertia.y,
                    inertia.z
                );
            } else {
                ImGui::TextDisabled("Infinite-mass static body.");
            }
        }

        if (body->motionType() == physics::MotionType::Dynamic &&
            ImGui::CollapsingHeader("Gyroscopic Rotors")) {
            ImGui::TextWrapped(
                "Fixed-axis internal wheels exchange angular momentum with "
                "the chassis. Stabilization and motor reaction come from "
                "conservation, not a corrective controller."
            );
            if (ImGui::Button("Add Z-axis reaction wheel")) {
                physics::GyroscopicRotorDescription rotor{};
                rotor.axisLocal = {0.0F, 0.0F, 1.0F};
                rotor.axialInertia = std::max(
                    1.0e-4F,
                    body->lockedAssemblyInertiaTensor().m22 * 0.05F
                );
                (void)body->createGyroscopicRotor(rotor);
            }

            for (std::size_t slot = 1;
                 slot <= body->gyroscopicRotorSlotCount();
                 ++slot) {
                const auto rotorId =
                    static_cast<physics::RotorId>(slot);
                const physics::GyroscopicRotorState* rotor =
                    body->gyroscopicRotor(rotorId);
                if (rotor == nullptr) {
                    continue;
                }
                ImGui::PushID(static_cast<int>(rotorId));
                ImGui::SeparatorText("Reaction wheel");
                ImGui::TextDisabled(
                    "Axis %.2f %.2f %.2f   J %.5f kg m2",
                    rotor->axisLocal.x,
                    rotor->axisLocal.y,
                    rotor->axisLocal.z,
                    rotor->axialInertia
                );
                float relativeSpeed =
                    rotor->relativeAngularVelocity;
                if (ImGui::DragFloat(
                        "Relative speed",
                        &relativeSpeed,
                        10.0F,
                        -200'000.0F,
                        200'000.0F,
                        "%.1f rad/s",
                        ImGuiSliderFlags_AlwaysClamp
                    )) {
                    (void)body
                        ->setGyroscopicRotorRelativeAngularVelocity(
                            rotorId,
                            relativeSpeed
                        );
                }
                float motorTorque = rotor->motorTorqueCommand;
                if (ImGui::DragFloat(
                        "Motor torque",
                        &motorTorque,
                        0.01F,
                        -100'000.0F,
                        100'000.0F,
                        "%.3f N m",
                        ImGuiSliderFlags_AlwaysClamp
                    )) {
                    (void)body->setGyroscopicRotorMotorTorque(
                        rotorId,
                        motorTorque
                    );
                }
                ImGui::TextDisabled(
                    "Absolute axial momentum %.4f N m s",
                    rotor->absoluteAxialAngularMomentum
                );
                if (ImGui::Button("Remove wheel")) {
                    (void)body->destroyGyroscopicRotor(rotorId);
                    ImGui::PopID();
                    break;
                }
                ImGui::PopID();
            }
        }

        if (ImGui::CollapsingHeader(
                "Physics Material",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            physics::PhysicsMaterial material = body->material();
            bool changed = false;
            changed |= editableSliderFloat(
                "Static friction",
                &material.staticFriction,
                0.0F,
                2.0F,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp
            );
            changed |= editableSliderFloat(
                "Kinetic friction",
                &material.dynamicFriction,
                0.0F,
                2.0F,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp
            );
            changed |= editableSliderFloat(
                "Restitution",
                &material.restitution,
                0.0F,
                1.0F,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp
            );
            changed |= editableSliderFloat(
                "Rolling friction",
                &material.rollingFriction,
                0.0F,
                0.25F,
                "%.4f",
                ImGuiSliderFlags_AlwaysClamp
            );
            if (changed) {
                material.staticFriction =
                    std::max(material.staticFriction, material.dynamicFriction);
                body->setMaterial(material);
            }
        }

        if (body->motionType() == physics::MotionType::Dynamic &&
            ImGui::CollapsingHeader(
                "Aerodynamics",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            physics::AerodynamicProperties aerodynamics = body->aerodynamics();
            bool changed = ImGui::Checkbox(
                "Air drag enabled",
                &aerodynamics.enabled
            );
            changed |= ImGui::DragFloat(
                "Drag coefficient",
                &aerodynamics.dragCoefficient,
                0.01F,
                0.0F,
                5.0F,
                "%.3f"
            );
            changed |= ImGui::DragFloat(
                "Angular drag",
                &aerodynamics.angularDragCoefficient,
                0.002F,
                0.0F,
                5.0F,
                "%.3f"
            );
            changed |= ImGui::DragFloat(
                "Area scale",
                &aerodynamics.projectedAreaScale,
                0.01F,
                0.0F,
                10.0F,
                "%.3f"
            );
            float centerOfPressure[3]{
                aerodynamics.centerOfPressureLocal.x,
                aerodynamics.centerOfPressureLocal.y,
                aerodynamics.centerOfPressureLocal.z
            };
            if (ImGui::DragFloat3(
                    "CoP offset (m)",
                    centerOfPressure,
                    0.005F,
                    -10.0F,
                    10.0F
                )) {
                aerodynamics.centerOfPressureLocal = {
                    centerOfPressure[0],
                    centerOfPressure[1],
                    centerOfPressure[2]
                };
                changed = true;
            }
            if (changed) {
                body->setAerodynamics(aerodynamics);
            }
        }

        ImGui::PopItemWidth();
        bool deleteRequested = false;
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::BeginDisabled(!primitiveCanBeDeleted(inspectedBodyId));
        if (ImGui::Button("Delete Primitive", ImVec2(-1.0F, 30.0F))) {
            deleteRequested = true;
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) &&
            !primitiveCanBeDeleted(inspectedBodyId)) {
            ImGui::SetTooltip("The required ground environment cannot be deleted.");
        }
        ImGui::PopID();
        ImGui::End();
        if (deleteRequested) {
            selectedBodyId_ = inspectedBodyId;
            deleteSelectedPrimitive();
        }
    }

    void drawDronePanel() {
        if (!ImGui::Begin("Drone", &showDronePanel_)) {
            ImGui::End();
            return;
        }

        physics::Drone* drone = world_.drone(controlledDroneId_);
        if (drone == nullptr) {
            ImGui::TextDisabled("No controlled drone is available.");
            if (ImGui::Button("Spawn Quadcopter")) {
                spawnDrone();
            }
            ImGui::End();
            return;
        }
        physics::RigidBody* body = world_.body(drone->bodyId());
        if (body == nullptr) {
            ImGui::TextDisabled("The controlled drone body was removed.");
            ImGui::End();
            return;
        }

        ImGui::TextUnformatted(body->debugName());
        ImGui::SameLine();
        ImGui::TextDisabled(
            "%.2f kg   %.1f m",
            body->mass(),
            body->transform().position.y
        );
        ImGui::Spacing();

        bool armed = drone->isArmed();
        if (ImGui::Checkbox("Armed", &armed)) {
            drone->setArmed(armed);
        }
        ImGui::SameLine();
        ImGui::Checkbox("Keyboard control", &droneControlEnabled_);
        bool follow = followDroneCamera_;
        if (ImGui::Checkbox("Follow camera", &follow)) {
            followDroneCamera_ = follow;
            followCameraInitialized_ = false;
        }

        constexpr std::array<const char*, 3U> modeNames{
            "Stabilized PID",
            "PID bypass / direct mixer",
            "External actuator API"
        };
        int mode = static_cast<int>(drone->controlMode());
        if (ImGui::Combo(
                "Control mode",
                &mode,
                modeNames.data(),
                static_cast<int>(modeNames.size())
            )) {
            drone->setControlMode(
                static_cast<physics::DroneControlMode>(mode)
            );
        }
        if (drone->controlMode() ==
            physics::DroneControlMode::ExternalActuators) {
            ImGui::TextDisabled(
                "Keyboard input is bypassed. Submit DroneActuatorFrame values "
                "through the public physics API."
            );
        } else {
            ImGui::TextDisabled(
                "W/S forward   A/D strafe   Q/E yaw   Up/Down altitude"
            );
        }

        if (ImGui::CollapsingHeader(
                "Motor model",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            bool variation = drone->description().motors.variationEnabled;
            float variationFraction =
                drone->description().motors.variationFraction;
            bool variationChanged =
                ImGui::Checkbox("Motor variation", &variation);
            variationChanged =
                inputFloatClamped(
                    "Variation fraction",
                    &variationFraction,
                    0.0F,
                    0.20F,
                    "%.4f"
                ) || variationChanged;
            if (variationChanged) {
                droneMotorVariationEnabled_ = variation;
                droneMotorVariationFraction_ = variationFraction;
                drone->setMotorVariation(
                    variation,
                    variationFraction,
                    0x55415644U
                );
            }

            if (ImGui::BeginTable(
                    "##MotorTelemetry",
                    4,
                    ImGuiTableFlags_BordersInnerV |
                        ImGuiTableFlags_RowBg
                )) {
                ImGui::TableSetupColumn("Motor");
                ImGui::TableSetupColumn("Command");
                ImGui::TableSetupColumn("RPM");
                ImGui::TableSetupColumn("Thrust");
                ImGui::TableHeadersRow();
                const auto& motors = drone->motors();
                for (std::size_t index = 0U;
                     index < motors.size();
                     ++index) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%zu", index + 1U);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", motors[index].command);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text(
                        "%.0f",
                        motors[index].angularVelocity *
                            (60.0F / (2.0F * kPi))
                    );
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.2f N", motors[index].thrustNewtons);
                }
                ImGui::EndTable();
            }
        }

        if (ImGui::CollapsingHeader("PID controller")) {
            physics::DroneControllerSettings settings =
                drone->controllerSettings();
            bool changed = false;
            const auto editPid =
                [&](const char* label, physics::PidGains& gains) {
                    if (!ImGui::TreeNode(label)) {
                        return;
                    }
                    ImGui::PushID(label);
                    changed =
                        inputFloatClamped(
                            "P",
                            &gains.proportional,
                            0.0F,
                            100.0F,
                            "%.4f"
                        ) || changed;
                    changed =
                        inputFloatClamped(
                            "I",
                            &gains.integral,
                            0.0F,
                            100.0F,
                            "%.4f"
                        ) || changed;
                    changed =
                        inputFloatClamped(
                            "D",
                            &gains.derivative,
                            0.0F,
                            100.0F,
                            "%.5f"
                        ) || changed;
                    changed =
                        inputFloatClamped(
                            "Integral limit",
                            &gains.integralLimit,
                            0.0F,
                            100.0F,
                            "%.3f"
                        ) || changed;
                    changed =
                        inputFloatClamped(
                            "Output limit",
                            &gains.outputLimit,
                            0.0F,
                            100.0F,
                            "%.3f"
                        ) || changed;
                    ImGui::PopID();
                    ImGui::TreePop();
                };
            editPid("Horizontal velocity", settings.horizontalVelocity);
            editPid("Altitude", settings.altitude);
            editPid("Attitude", settings.attitude);
            editPid("Body rate", settings.bodyRate);
            changed =
                inputFloatClamped(
                    "Maximum horizontal speed",
                    &settings.maximumHorizontalSpeed,
                    0.0F,
                    50.0F,
                    "%.2f m/s"
                ) || changed;
            changed =
                inputFloatClamped(
                    "Maximum climb speed",
                    &settings.maximumClimbSpeed,
                    0.0F,
                    20.0F,
                    "%.2f m/s"
                ) || changed;
            float tiltDegrees =
                settings.maximumTiltRadians * (180.0F / kPi);
            if (inputFloatClamped(
                    "Maximum tilt",
                    &tiltDegrees,
                    0.0F,
                    80.0F,
                    "%.1f deg"
                )) {
                settings.maximumTiltRadians =
                    tiltDegrees * (kPi / 180.0F);
                changed = true;
            }
            if (changed) {
                drone->setControllerSettings(settings);
            }
            if (ImGui::Button("Reset PID defaults")) {
                drone->setControllerSettings({});
            }
        }

        if (ImGui::CollapsingHeader("Virtual sensors")) {
            const physics::DroneSensorFrame& sensors =
                drone->sensorFrame();
            ImGui::TextDisabled(
                "Frame %llu   %.3f s",
                static_cast<unsigned long long>(sensors.sequence),
                sensors.simulationTimeSeconds
            );
            ImGui::Text(
                "Gyro  %.3f  %.3f  %.3f rad/s",
                sensors.gyroscopeRadiansPerSecondBody.x,
                sensors.gyroscopeRadiansPerSecondBody.y,
                sensors.gyroscopeRadiansPerSecondBody.z
            );
            ImGui::Text(
                "Accel  %.3f  %.3f  %.3f m/s^2",
                sensors.accelerometerMetersPerSecondSquaredBody.x,
                sensors.accelerometerMetersPerSecondSquaredBody.y,
                sensors.accelerometerMetersPerSecondSquaredBody.z
            );
            ImGui::Text(
                "Barometer  %.1f Pa   %.2f m",
                sensors.pressurePascals,
                sensors.barometricAltitudeMeters
            );
            ImGui::TextDisabled(
                "LiDAR channel is reserved but intentionally not implemented."
            );
        }

        ImGui::End();
    }

    void drawPhysicsPanel() {
        if (!ImGui::Begin("Physics", &showPhysicsPanel_)) {
            ImGui::End();
            return;
        }

        const physics::PhysicsDebugStats& stats = world_.debugStats();
        ImGui::TextColored(
            simulationPaused_ ? uiColor(164, 164, 164) : uiColor(117, 174, 123),
            "%s",
            simulationPaused_ ? "PAUSED" : "SIMULATING"
        );
        ImGui::SameLine();
        ImGui::TextDisabled(
            "%u Hz / %u substeps",
            world_.settings().fixedUpdateHz,
            world_.settings().solverSubsteps
        );

        if (ImGui::CollapsingHeader(
                "Simulation",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            int updateRate = static_cast<int>(physicsSettings_.fixedUpdateHz);
            if (editableSliderInt(
                    "Physics rate",
                    &updateRate,
                    30,
                    480,
                    "%d Hz",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.fixedUpdateHz =
                    static_cast<std::uint32_t>(updateRate);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            int substeps = static_cast<int>(physicsSettings_.solverSubsteps);
            if (editableSliderInt(
                    "Solver substeps",
                    &substeps,
                    1,
                    8,
                    "%d",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.solverSubsteps =
                    static_cast<std::uint32_t>(substeps);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            int adaptiveSubsteps =
                static_cast<int>(physicsSettings_.maximumAdaptiveSubsteps);
            if (editableSliderInt(
                    "Adaptive substep cap",
                    &adaptiveSubsteps,
                    substeps,
                    64,
                    "%d",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.maximumAdaptiveSubsteps =
                    static_cast<std::uint32_t>(adaptiveSubsteps);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            if (editableSliderFloat(
                    "Max travel / half extent",
                    &physicsSettings_.maximumTravelFraction,
                    0.10F,
                    1.0F,
                    "%.2f",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            float maximumRotationStepDegrees =
                physicsSettings_.maximumRotationStepRadians *
                (180.0F / physics::kPi);
            if (editableSliderFloat(
                    "Max rotation step",
                    &maximumRotationStepDegrees,
                    1.0F,
                    45.0F,
                    "%.1f deg",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.maximumRotationStepRadians =
                    physics::radians(maximumRotationStepDegrees);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            int rotationMicrosteps =
                static_cast<int>(
                    physicsSettings_.maximumRotationMicrosteps
                );
            if (editableSliderInt(
                    "Rotor microstep cap",
                    &rotationMicrosteps,
                    1,
                    128,
                    "%d",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.maximumRotationMicrosteps =
                    static_cast<std::uint32_t>(rotationMicrosteps);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            int iterations = static_cast<int>(physicsSettings_.velocityIterations);
            if (editableSliderInt(
                    "Velocity iterations",
                    &iterations,
                    1,
                    32,
                    "%d",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.velocityIterations =
                    static_cast<std::uint32_t>(iterations);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            int catchUp = static_cast<int>(physicsSettings_.maximumCatchUpSteps);
            if (editableSliderInt(
                    "Maximum catch-up",
                    &catchUp,
                    1,
                    16,
                    "%d ticks",
                    ImGuiSliderFlags_AlwaysClamp
                )) {
                physicsSettings_.maximumCatchUpSteps =
                    static_cast<std::uint32_t>(catchUp);
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }

            ImGui::TextDisabled(
                "Tick %llu   Time %.3f s   Dropped %.4f s",
                static_cast<unsigned long long>(stats.fixedTick),
                stats.simulationTime,
                stats.droppedRealTime
            );
            ImGui::TextDisabled(
                "Bodies %zu   Awake %zu   Contacts %zu   Active substeps %u",
                stats.bodyCount,
                stats.awakeBodyCount,
                stats.contactCount,
                stats.internalSubsteps
            );
            ImGui::TextDisabled(
                "Broadphase %zu   Narrowphase %zu   Rotational CCD %zu",
                stats.broadphasePairs,
                stats.narrowphaseTests,
                stats.rotationalCcdHits
            );
            ImGui::TextDisabled(
                "CCD advances %zu   Converged %zu   Advance caps %zu",
                stats.rotationalCcdAdvances,
                stats.rotationalCcdConvergenceHits,
                stats.rotationalCcdAdvanceCapHits
            );
            ImGui::TextDisabled(
                "Rotor microsteps %zu   Peak %u   Cap hits %zu",
                stats.rotationMicrosteps,
                stats.maximumRotationMicrostepsUsed,
                stats.rotationMicrostepCapHits
            );
            if (stats.rotationMidpointNonConvergenceCount > 0) {
                ImGui::TextColored(
                    uiColor(207, 101, 92),
                    "Rotor solver warnings %zu   Max residual %.2e",
                    stats.rotationMidpointNonConvergenceCount,
                    static_cast<double>(
                        stats.maximumRotationMidpointResidual
                    )
                );
            }
        }

        if (ImGui::CollapsingHeader("Contact Solver")) {
            bool changed = false;
            changed |= ImGui::DragFloat(
                "Contact slop (m)",
                &physicsSettings_.contactSlop,
                0.00005F,
                0.00001F,
                0.05F,
                "%.5f"
            );
            changed |= ImGui::DragFloat(
                "Bounce threshold (m/s)",
                &physicsSettings_.restitutionVelocityThreshold,
                0.01F,
                0.0F,
                20.0F,
                "%.3f"
            );
            changed |= editableSliderFloat(
                "Penetration velocity",
                &physicsSettings_.penetrationVelocityFactor,
                0.0F,
                1.0F,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp
            );
            changed |= ImGui::DragFloat(
                "Max separation (m/s)",
                &physicsSettings_.maximumDepenetrationVelocity,
                0.05F,
                0.0F,
                50.0F,
                "%.2f"
            );
            changed |= editableSliderFloat(
                "Position correction",
                &physicsSettings_.positionCorrectionFraction,
                0.0F,
                1.0F,
                "%.3f",
                ImGuiSliderFlags_AlwaysClamp
            );
            if (changed) {
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }
        }

        if (ImGui::CollapsingHeader("Sleep Thresholds")) {
            bool changed = false;
            changed |= ImGui::DragFloat(
                "Linear speed (m/s)",
                &physicsSettings_.sleepLinearSpeed,
                0.001F,
                0.0F,
                10.0F,
                "%.3f"
            );
            changed |= ImGui::DragFloat(
                "Angular speed (rad/s)",
                &physicsSettings_.sleepAngularSpeed,
                0.001F,
                0.0F,
                10.0F,
                "%.3f"
            );
            changed |= ImGui::DragFloat(
                "Sleep delay (s)",
                &physicsSettings_.sleepDelaySeconds,
                0.01F,
                0.0F,
                30.0F,
                "%.2f"
            );
            changed |= ImGui::DragFloat(
                "Support polygon margin (m)",
                &physicsSettings_.sleepSupportMargin,
                0.0001F,
                0.0F,
                0.1F,
                "%.4f"
            );
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "A body may sleep only when its gravity-projected center "
                    "of mass lies at least this far inside its support "
                    "polygon. Increase the margin to keep marginal edge and "
                    "corner balances awake until they topple."
                );
            }
            if (changed) {
                world_.setSettings(physicsSettings_);
                physicsSettings_ = world_.settings();
            }
        }

        if (ImGui::CollapsingHeader(
                "Gravity & Atmosphere",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            physics::Environment& environment = world_.environment();
            physics::Vec3 gravity = environment.gravity();
            float gravityValues[3]{gravity.x, gravity.y, gravity.z};
            if (ImGui::DragFloat3(
                    "Gravity (m/s2)",
                    gravityValues,
                    0.01F,
                    -50.0F,
                    50.0F
                )) {
                    environment.setGravity({
                        gravityValues[0],
                        gravityValues[1],
                        gravityValues[2]
                    });
                    world_.wakeAllDynamic();
                }

            physics::AtmosphereSample atmosphere = environment.atmosphere();
            bool changed = false;
            changed |= ImGui::DragFloat(
                "Air density (kg/m3)",
                &atmosphere.density,
                0.001F,
                0.0F,
                10.0F,
                "%.4f"
            );
            changed |= ImGui::DragFloat(
                "Viscosity (Pa s)",
                &atmosphere.dynamicViscosity,
                0.000001F,
                0.0F,
                0.01F,
                "%.7f"
            );
            changed |= ImGui::DragFloat(
                "Temperature (K)",
                &atmosphere.temperatureKelvin,
                0.1F,
                1.0F,
                1'000.0F,
                "%.2f"
            );
            changed |= ImGui::DragFloat(
                "Pressure (Pa)",
                &atmosphere.pressurePascals,
                10.0F,
                0.0F,
                500'000.0F,
                "%.0f"
            );
            if (changed) {
                environment.setAtmosphere(atmosphere);
                world_.wakeAllDynamic();
            }
            ImGui::TextDisabled(
                "Milestone 2 uses a constant atmosphere; the sampler is "
                "position-aware for future altitude models."
            );
        }

        if (ImGui::CollapsingHeader(
                "Wind Field",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            physics::Environment& environment = world_.environment();
            physics::WindFieldSettings wind = environment.windSettings();
            float mean[3]{
                wind.meanVelocity.x,
                wind.meanVelocity.y,
                wind.meanVelocity.z
            };
            bool changed = ImGui::DragFloat3(
                "Mean wind (m/s)",
                mean,
                0.01F,
                -20.0F,
                20.0F
            );
            if (changed) {
                wind.meanVelocity = {mean[0], mean[1], mean[2]};
            }
            changed |= ImGui::DragFloat(
                "Turbulence (m/s)",
                &wind.turbulenceAmplitude,
                0.005F,
                0.0F,
                10.0F,
                "%.3f"
            );
            changed |= ImGui::DragFloat(
                "Spatial frequency",
                &wind.spatialFrequency,
                0.001F,
                0.0F,
                2.0F,
                "%.3f rad/m"
            );
            changed |= ImGui::DragFloat(
                "Temporal frequency",
                &wind.temporalFrequency,
                0.001F,
                0.0F,
                10.0F,
                "%.3f rad/s"
            );
            if (changed) {
                environment.setWindSettings(wind);
                world_.wakeAllDynamic();
            }
        }

        if (ImGui::CollapsingHeader(
                "Viewport Debug",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox("Collision contacts", &showContactDebug_);
            ImGui::Checkbox("Collider wireframes", &showColliderDebug_);
            ImGui::Checkbox("Velocity vectors", &showVelocityDebug_);
            ImGui::Checkbox("World AABBs", &showAabbDebug_);
            ImGui::Checkbox("Center of mass", &showCenterOfMassDebug_);
            ImGui::Checkbox("Wind vector field", &showWindDebug_);
            editableSliderFloat(
                "Wind arrow scale",
                &windDebugScale_,
                0.1F,
                10.0F,
                "%.2f",
                ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp
            );
        }

        if (ImGui::CollapsingHeader(
                "Ctrl Drag",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            editableSliderFloat(
                "Vertex influence (m)",
                &grabInfluenceRadius_,
                0.05F,
                2.0F,
                "%.2f",
                ImGuiSliderFlags_AlwaysClamp
            );
            editableSliderFloat(
                "Natural frequency",
                &grabFrequencyHz_,
                1.0F,
                15.0F,
                "%.1f Hz",
                ImGuiSliderFlags_AlwaysClamp
            );
            editableSliderFloat(
                "Maximum acceleration",
                &grabMaximumAcceleration_,
                10.0F,
                500.0F,
                "%.0f m/s2",
                ImGuiSliderFlags_AlwaysClamp
            );
            ImGui::TextDisabled(
                "Spring stiffness and damping scale with the selected body's "
                "mass; off-center pulls also produce physical torque."
            );
        }

        ImGui::End();
    }

    void drawRendererPanel() {
        if (!ImGui::Begin("Renderer", &showRendererPanel_)) {
            ImGui::End();
            return;
        }

        const float rendererControlWidth =
            std::max(124.0F, ImGui::GetContentRegionAvail().x * 0.52F);
        ImGui::PushItemWidth(rendererControlWidth);

        if (ImGui::CollapsingHeader(
                "Sun",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox("Enabled##Sun", &groundSettings_.sunEnabled);
            ImGui::ColorEdit3(
                "Color##Sun",
                &groundSettings_.sunColor.x,
                ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR
            );
            inputFloatClamped(
                "Azimuth (deg)",
                &groundSettings_.sunAzimuthDegrees,
                -180.0F,
                180.0F,
                "%.1f"
            );
            inputFloatClamped(
                "Elevation (deg)",
                &groundSettings_.sunElevationDegrees,
                1.0F,
                89.0F,
                "%.1f"
            );
            inputFloatClamped(
                "Intensity##Sun",
                &groundSettings_.sunIntensity,
                0.0F,
                100.0F,
                "%.3f"
            );
        }

        if (ImGui::CollapsingHeader(
                "Environment Light",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox(
                "Enabled##Environment",
                &groundSettings_.environmentEnabled
            );
            ImGui::ColorEdit3(
                "Color##Environment",
                &groundSettings_.environmentColor.x,
                ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR
            );
            inputFloatClamped(
                "Intensity##Environment",
                &groundSettings_.ambientStrength,
                0.0F,
                10.0F,
                "%.4f"
            );
        }

        if (ImGui::CollapsingHeader(
                "Spot Lights",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::BeginDisabled(
                spotLights_.size() >= render::kMaximumSpotLights
            );
            if (ImGui::Button("+ Add Spot Light", ImVec2(-1.0F, 0.0F))) {
                render::SpotLightRenderData light{};
                light.position =
                    camera_.position() + camera_.forward() * 2.0F;
                light.direction = camera_.forward();
                light.intensity = 80.0F;
                light.flags = render::SpotLightRenderFlags::enabled;
                spotLights_.push_back(light);
            }
            ImGui::EndDisabled();
            ImGui::TextDisabled(
                "%zu / %zu lights (sun shadows only)",
                spotLights_.size(),
                render::kMaximumSpotLights
            );

            int removeLight = -1;
            for (std::size_t index = 0U; index < spotLights_.size(); ++index) {
                render::SpotLightRenderData& light = spotLights_[index];
                ImGui::PushID(static_cast<int>(index));
                const std::string title =
                    "Spot Light " + std::to_string(index + 1U);
                if (ImGui::TreeNodeEx(
                        title.c_str(),
                        ImGuiTreeNodeFlags_DefaultOpen
                    )) {
                    bool enabled =
                        (light.flags &
                         render::SpotLightRenderFlags::enabled) != 0U;
                    if (ImGui::Checkbox("Enabled", &enabled)) {
                        if (enabled) {
                            light.flags |=
                                render::SpotLightRenderFlags::enabled;
                        } else {
                            light.flags &=
                                ~render::SpotLightRenderFlags::enabled;
                        }
                    }

                    float position[3]{
                        light.position.x,
                        light.position.y,
                        light.position.z
                    };
                    if (inputFloat3Clamped(
                            "Position (m)",
                            position,
                            -10'000.0F,
                            10'000.0F
                        )) {
                        light.position = {
                            position[0], position[1], position[2]
                        };
                    }
                    float direction[3]{
                        light.direction.x,
                        light.direction.y,
                        light.direction.z
                    };
                    if (inputFloat3Clamped(
                            "Direction",
                            direction,
                            -1.0F,
                            1.0F
                        )) {
                        glm::vec3 normalizedDirection{
                            direction[0], direction[1], direction[2]
                        };
                        const float lengthSquared =
                            glm::dot(
                                normalizedDirection,
                                normalizedDirection
                            );
                        light.direction =
                            lengthSquared > 1.0e-8F
                                ? glm::normalize(normalizedDirection)
                                : glm::vec3{0.0F, -1.0F, 0.0F};
                    }
                    ImGui::ColorEdit3(
                        "Color",
                        &light.color.x,
                        ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR
                    );
                    inputFloatClamped(
                        "Intensity",
                        &light.intensity,
                        0.0F,
                        1.0e6F,
                        "%.2f"
                    );
                    inputFloatClamped(
                        "Range (m)",
                        &light.range,
                        0.05F,
                        10'000.0F,
                        "%.2f"
                    );
                    inputFloatClamped(
                        "Inner cone (deg)",
                        &light.innerConeDegrees,
                        0.1F,
                        88.0F,
                        "%.1f"
                    );
                    inputFloatClamped(
                        "Outer cone (deg)",
                        &light.outerConeDegrees,
                        0.2F,
                        89.0F,
                        "%.1f"
                    );
                    light.innerConeDegrees = std::min(
                        light.innerConeDegrees,
                        light.outerConeDegrees - 0.1F
                    );
                    if (ImGui::Button("Remove")) {
                        removeLight = static_cast<int>(index);
                    }
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
            if (removeLight >= 0) {
                spotLights_.erase(
                    spotLights_.begin() +
                    static_cast<std::ptrdiff_t>(removeLight)
                );
            }
        }

        if (ImGui::CollapsingHeader(
                "Cascaded Sun Shadows",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox(
                "Enabled##SunShadows",
                &groundSettings_.sunShadowsEnabled
            );
            inputFloatClamped(
                "Shadow distance (m)",
                &groundSettings_.shadowDistance,
                5.0F,
                2'000.0F,
                "%.1f"
            );
            inputFloatClamped(
                "Cascade distribution",
                &groundSettings_.shadowSplitLambda,
                0.0F,
                1.0F,
                "%.3f"
            );
            inputFloatClamped(
                "Normal bias",
                &groundSettings_.shadowNormalBias,
                0.0F,
                0.25F,
                "%.5f"
            );
            inputFloatClamped(
                "Depth bias",
                &groundSettings_.shadowDepthBiasConstant,
                0.0F,
                20.0F,
                "%.3f"
            );
            inputFloatClamped(
                "Slope bias",
                &groundSettings_.shadowDepthBiasSlope,
                0.0F,
                20.0F,
                "%.3f"
            );
            ImGui::TextDisabled(
                "%u stabilized cascades with PCF filtering",
                render::kDirectionalShadowCascadeCount
            );
        }

        if (ImGui::CollapsingHeader(
                "Ground Material",
                ImGuiTreeNodeFlags_DefaultOpen
            )) {
            ImGui::Checkbox(
                "Invert normal map green channel",
                &groundSettings_.flipNormalGreen
            );
            inputFloatClamped(
                "Normal strength",
                &groundSettings_.normalStrength,
                0.0F,
                4.0F,
                "%.3f"
            );
            inputFloatClamped(
                "Roughness scale",
                &groundSettings_.roughnessScale,
                0.05F,
                4.0F,
                "%.3f"
            );
            inputFloatClamped(
                "Exposure",
                &groundSettings_.exposure,
                0.01F,
                20.0F,
                "%.3f"
            );
        }

        ImGui::PopItemWidth();
        ImGui::Spacing();

        const render::RendererDiagnostics& diagnostics = renderer_.diagnostics();
        if (ImGui::CollapsingHeader("Diagnostics")) {
        if (diagnostics.validationErrors > 0U) {
            ImGui::TextColored(
                ImVec4(0.96F, 0.35F, 0.32F, 1.0F),
                "VALIDATION ERRORS"
            );
        } else {
            ImGui::TextColored(
                ImVec4(0.35F, 0.82F, 0.57F, 1.0F),
                "RENDERER HEALTHY"
            );
        }
        ImGui::SameLine();
        ImGui::TextDisabled(
            diagnostics.validationEnabled ? "Validation on" : "Validation off"
        );

        ImGui::Spacing();
        if (ImGui::BeginTable(
                "##RendererDiagnostics",
                2,
                ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp
            )) {
            ImGui::TableSetupColumn(
                "Property",
                ImGuiTableColumnFlags_WidthFixed,
                112.0F
            );
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);

            diagnosticRow("GPU", diagnostics.gpuName.c_str());
            diagnosticRow("Vulkan API", diagnostics.apiVersion.c_str());
            diagnosticRow("Present mode", diagnostics.presentMode.c_str());
            diagnosticRow(
                "UI font",
                interFontLoaded_ ? "Inter Variable" : "ImGui fallback"
            );

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Swapchain");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(
                "%u x %u",
                diagnostics.swapchainWidth,
                diagnostics.swapchainHeight
            );

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Ground texture");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(
                "%u x %u, %u mips",
                diagnostics.textureWidth,
                diagnostics.textureHeight,
                diagnostics.textureMipLevels
            );

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Frames");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(
                "%llu",
                static_cast<unsigned long long>(diagnostics.renderedFrames)
            );

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Scene draw");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text(
                "%u cube, %u debug lines",
                diagnostics.visibleCubeCount,
                diagnostics.debugLineCount
            );

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Renderer CPU");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f ms", diagnostics.rendererCpuMilliseconds);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("UI rate");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Warnings");
            ImGui::TableSetColumnIndex(1);
            if (diagnostics.validationWarnings > 0U) {
                ImGui::TextColored(
                    ImVec4(0.95F, 0.70F, 0.26F, 1.0F),
                    "%u",
                    diagnostics.validationWarnings
                );
            } else {
                ImGui::TextUnformatted("0");
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("Errors");
            ImGui::TableSetColumnIndex(1);
            if (diagnostics.validationErrors > 0U) {
                ImGui::TextColored(
                    ImVec4(0.96F, 0.35F, 0.32F, 1.0F),
                    "%u",
                    diagnostics.validationErrors
                );
            } else {
                ImGui::TextUnformatted("0");
            }

            ImGui::EndTable();
        }
        }
        ImGui::End();
    }

    void updateSceneViewport(ImGuiID dockspaceId) {
        const ImGuiDockNode* centralNode =
            ImGui::DockBuilderGetCentralNode(dockspaceId);
        if (centralNode == nullptr || centralNode->Size.x <= 1.0F ||
            centralNode->Size.y <= 1.0F) {
            sceneViewportValid_ = false;
            sceneViewportFramebufferSize_ = {};
            renderer_.setSceneViewport({});
            return;
        }

        sceneViewportPosition_ = centralNode->Pos;
        sceneViewportSize_ = centralNode->Size;
        sceneViewportValid_ = true;

        const ImGuiIO& io = ImGui::GetIO();
        const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
        const float scaleX =
            std::isfinite(io.DisplayFramebufferScale.x) &&
                    io.DisplayFramebufferScale.x > 0.0F
                ? io.DisplayFramebufferScale.x
                : 1.0F;
        const float scaleY =
            std::isfinite(io.DisplayFramebufferScale.y) &&
                    io.DisplayFramebufferScale.y > 0.0F
                ? io.DisplayFramebufferScale.y
                : 1.0F;
        const ImVec2 displayOrigin =
            mainViewport != nullptr ? mainViewport->Pos : ImVec2{};

        const int x = std::max(
            0,
            static_cast<int>(std::lround(
                (centralNode->Pos.x - displayOrigin.x) * scaleX
            ))
        );
        const int y = std::max(
            0,
            static_cast<int>(std::lround(
                (centralNode->Pos.y - displayOrigin.y) * scaleY
            ))
        );
        const auto width = static_cast<std::uint32_t>(std::max(
            1L,
            std::lround(centralNode->Size.x * scaleX)
        ));
        const auto height = static_cast<std::uint32_t>(std::max(
            1L,
            std::lround(centralNode->Size.y * scaleY)
        ));
        sceneViewportFramebufferSize_ = {
            static_cast<float>(width),
            static_cast<float>(height)
        };
        renderer_.setSceneViewport({x, y, width, height});
    }

    [[nodiscard]] std::array<physics::Vec3, 8U> boxVerticesInRenderOrder(
        const physics::RigidBody& body,
        const physics::Transform& transform
    ) const noexcept {
        std::array<physics::Vec3, 8U> vertices{};
        if (body.collider().type != physics::ColliderType::Box) {
            vertices.fill(transform.position);
            return vertices;
        }

        const physics::Vec3 half = body.collider().box.halfExtents;
        for (std::uint32_t corner = 0U; corner < vertices.size(); ++corner) {
            const physics::Vec3 localPoint{
                (corner & 1U) != 0U ? half.x : -half.x,
                (corner & 2U) != 0U ? half.y : -half.y,
                (corner & 4U) != 0U ? half.z : -half.z
            };
            vertices[corner] =
                transform.position +
                transform.orientation.rotate(
                    localPoint - body.centerOfMassLocal()
                );
        }
        return vertices;
    }

    [[nodiscard]] std::array<physics::Vec3, 8U> boxVerticesInRenderOrder(
        const physics::RigidBody& body
    ) const noexcept {
        return boxVerticesInRenderOrder(body, body.transform());
    }

    void clearHover() noexcept {
        hoveredBodyId_ = physics::kInvalidBodyId;
        hoveredVertexMask_ = 0U;
        hoverAnchorWorld_ = {};
        hoverRayDistance_ = 0.0F;
    }

    void updateHoverFromRay(
        const physics::Vec3& rayOrigin,
        const physics::Vec3& rayDirection
    ) {
        clearHover();

        physics::RigidBody* candidateBody = nullptr;
        physics::Vec3 influenceCenter{};
        physics::Real candidateRayDistance = 0.0F;

        physics::RaycastHit hit{};
        if (world_.raycast(
                rayOrigin,
                rayDirection,
                kMaximumPickDistanceMeters,
                hit
            )) {
            physics::RigidBody* hitBody = world_.body(hit.body);
            if (hitBody != nullptr &&
                hitBody->motionType() == physics::MotionType::Dynamic &&
                hitBody->collider().type == physics::ColliderType::Box) {
                candidateBody = hitBody;
                influenceCenter = hit.point;
                candidateRayDistance = hit.distance;
            }
        }

        // A vertex can still be acquired when the cursor is just outside the
        // projected silhouette, which is the useful BeamNG-style hover case.
        if (candidateBody == nullptr) {
            physics::Real nearestSquared =
                std::numeric_limits<physics::Real>::max();
            physics::Vec3 nearestVertex{};
            physics::Real nearestDistance = 0.0F;
            const physics::Real acquisitionRadius =
                std::max(0.18F, grabInfluenceRadius_ * 0.45F);

            for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
                physics::RigidBody* body =
                    world_.body(static_cast<physics::BodyId>(slot));
                if (body == nullptr ||
                    body->motionType() != physics::MotionType::Dynamic ||
                    body->collider().type != physics::ColliderType::Box) {
                    continue;
                }

                const auto vertices = boxVerticesInRenderOrder(*body);
                for (const physics::Vec3& vertex : vertices) {
                    const physics::Real alongRay =
                        physics::dot(vertex - rayOrigin, rayDirection);
                    if (alongRay <= 0.0F ||
                        alongRay > kMaximumPickDistanceMeters) {
                        continue;
                    }
                    const physics::Vec3 nearestOnRay =
                        rayOrigin + rayDirection * alongRay;
                    const physics::Real distanceSquared =
                        physics::lengthSquared(vertex - nearestOnRay);
                    if (distanceSquared < nearestSquared) {
                        nearestSquared = distanceSquared;
                        nearestVertex = vertex;
                        nearestDistance = alongRay;
                        candidateBody = body;
                    }
                }
            }

            if (candidateBody == nullptr ||
                nearestSquared > acquisitionRadius * acquisitionRadius) {
                clearHover();
                return;
            }
            influenceCenter = nearestVertex;
            candidateRayDistance = nearestDistance;
        }

        const auto vertices = boxVerticesInRenderOrder(*candidateBody);
        const physics::Real radiusSquared =
            grabInfluenceRadius_ * grabInfluenceRadius_;
        std::uint32_t mask = 0U;
        physics::Real nearestVertexSquared =
            std::numeric_limits<physics::Real>::max();
        std::uint32_t nearestVertexIndex = 0U;
        for (std::uint32_t index = 0U; index < vertices.size(); ++index) {
            const physics::Real distanceSquared =
                physics::lengthSquared(vertices[index] - influenceCenter);
            if (distanceSquared <= radiusSquared) {
                mask |= 1U << index;
            }
            if (distanceSquared < nearestVertexSquared) {
                nearestVertexSquared = distanceSquared;
                nearestVertexIndex = index;
            }
        }
        if (mask == 0U) {
            mask = 1U << nearestVertexIndex;
        }

        hoveredBodyId_ = candidateBody->id();
        hoveredVertexMask_ = mask;
        hoverAnchorWorld_ = influenceCenter;
        hoverRayDistance_ = candidateRayDistance;
    }

    void beginGrab(
        const physics::Vec3& rayOrigin,
        const physics::Vec3& rayDirection
    ) {
        physics::RigidBody* body = world_.body(hoveredBodyId_);
        if (body == nullptr ||
            body->motionType() != physics::MotionType::Dynamic) {
            return;
        }

        const physics::Transform& transform = body->transform();
        const physics::Vec3 localAnchor =
            transform.orientation.inverseRotate(
                hoverAnchorWorld_ - transform.position
            ) + body->centerOfMassLocal();
        const physics::Real mass = std::max(body->mass(), 0.01F);
        const physics::Real angularFrequency =
            2.0F * kPi * std::max(grabFrequencyHz_, 0.1F);

        physics::SpringForce spring{};
        spring.body = body->id();
        spring.localAnchor = localAnchor;
        const physics::Vec3 initialRayPoint =
            rayOrigin + rayDirection * std::max(hoverRayDistance_, 0.05F);
        spring.worldTarget = hoverAnchorWorld_;
        spring.stiffness = mass * angularFrequency * angularFrequency;
        spring.damping = 2.0F * std::sqrt(spring.stiffness * mass);
        spring.maximumForce =
            mass * std::max(grabMaximumAcceleration_, 1.0F);
        spring.enabled = true;

        grabbedSpringId_ = world_.createSpring(spring);
        if (grabbedSpringId_ == physics::kInvalidSpringId) {
            return;
        }
        grabbedBodyId_ = body->id();
        grabbedLocalAnchor_ = localAnchor;
        grabWorldTarget_ = spring.worldTarget;
        grabRayDistance_ = std::max(hoverRayDistance_, 0.05F);
        grabRayToAnchorOffset_ = hoverAnchorWorld_ - initialRayPoint;
        grabbedVertexMask_ = hoveredVertexMask_;
        selectedBodyId_ = body->id();
    }

    void releaseGrab() noexcept {
        if (grabbedSpringId_ != physics::kInvalidSpringId) {
            world_.destroySpring(grabbedSpringId_);
        }
        grabbedSpringId_ = physics::kInvalidSpringId;
        grabbedBodyId_ = physics::kInvalidBodyId;
        grabbedVertexMask_ = 0U;
        grabbedLocalAnchor_ = {};
        grabWorldTarget_ = {};
        grabRayToAnchorOffset_ = {};
        grabRayDistance_ = 0.0F;
    }

    void updateViewportInteraction() {
        const bool windowFocused =
            glfwGetWindowAttrib(window_, GLFW_FOCUSED) == GLFW_TRUE;
        const bool controlHeld =
            keyPressed(window_, GLFW_KEY_LEFT_CONTROL) ||
            keyPressed(window_, GLFW_KEY_RIGHT_CONTROL);
        const bool leftMousePressed =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool escapePressed = keyPressed(window_, GLFW_KEY_ESCAPE);

        if (escapePressed && spawnerPlacementArmed_) {
            spawnerPlacementArmed_ = false;
            spawnerStatus_ = "Viewport placement cancelled.";
        }

        if (!windowFocused || escapePressed || !controlHeld ||
            !sceneViewportValid_ ||
            cameraLookActive_ ||
            (grabbedSpringId_ != physics::kInvalidSpringId &&
             !leftMousePressed)) {
            releaseGrab();
        }

        if (!sceneViewportValid_ || cameraLookActive_ || !windowFocused) {
            clearHover();
            leftMouseWasPressed_ = leftMousePressed;
            return;
        }

        const ImVec2 mouse = ImGui::GetMousePos();
        const float localX = mouse.x - sceneViewportPosition_.x;
        const float localY = mouse.y - sceneViewportPosition_.y;
        const bool insideViewport =
            localX >= 0.0F && localY >= 0.0F &&
            localX < sceneViewportSize_.x &&
            localY < sceneViewportSize_.y;

        const float framebufferScaleX =
            sceneViewportSize_.x > 0.0F
                ? sceneViewportFramebufferSize_.x / sceneViewportSize_.x
                : 1.0F;
        const float framebufferScaleY =
            sceneViewportSize_.y > 0.0F
                ? sceneViewportFramebufferSize_.y / sceneViewportSize_.y
                : 1.0F;
        render::CameraRay cameraRay = camera_.rayFromViewport(
            localX * framebufferScaleX,
            localY * framebufferScaleY,
            sceneViewportFramebufferSize_.x,
            sceneViewportFramebufferSize_.y
        );
        const physics::Vec3 rayOrigin = toPhysics(cameraRay.origin);
        const physics::Vec3 rayDirection =
            physics::normalized(toPhysics(cameraRay.direction));

        if (spawnerPlacementArmed_) {
            releaseGrab();
            clearHover();
            if (insideViewport &&
                !ImGui::GetIO().WantCaptureMouse &&
                leftMousePressed &&
                !leftMouseWasPressed_) {
                if (spawnCubeFromViewportRay(rayOrigin, rayDirection)) {
                    spawnerPlacementArmed_ = false;
                }
            }
            leftMouseWasPressed_ = leftMousePressed;
            return;
        }

        if (grabbedSpringId_ != physics::kInvalidSpringId) {
            grabWorldTarget_ =
                rayOrigin +
                rayDirection * std::max(grabRayDistance_, 0.05F) +
                grabRayToAnchorOffset_;
            if (!world_.setSpringTarget(grabbedSpringId_, grabWorldTarget_)) {
                releaseGrab();
            }
        } else if (
            controlHeld &&
            insideViewport &&
            !ImGui::GetIO().WantCaptureMouse
        ) {
            updateHoverFromRay(rayOrigin, rayDirection);
            if (leftMousePressed && !leftMouseWasPressed_ &&
                hoveredBodyId_ != physics::kInvalidBodyId) {
                beginGrab(rayOrigin, rayDirection);
            }
        } else {
            clearHover();
        }

        leftMouseWasPressed_ = leftMousePressed;
    }

    void updateSimulationShortcuts() {
        const bool keyboardCaptured = ImGui::GetIO().WantCaptureKeyboard;
        const bool spacePressed = keyPressed(window_, GLFW_KEY_SPACE);
        const bool periodPressed = keyPressed(window_, GLFW_KEY_PERIOD);
        const bool deletePressed = keyPressed(window_, GLFW_KEY_DELETE);
        const bool resetPressed =
            (keyPressed(window_, GLFW_KEY_LEFT_CONTROL) ||
             keyPressed(window_, GLFW_KEY_RIGHT_CONTROL)) &&
            keyPressed(window_, GLFW_KEY_R);

        if (!keyboardCaptured && spacePressed && !spaceKeyWasPressed_) {
            simulationPaused_ = !simulationPaused_;
        }
        if (!keyboardCaptured && periodPressed && !periodKeyWasPressed_ &&
            simulationPaused_) {
            singleStepRequested_ = true;
        }
        if (!keyboardCaptured && resetPressed && !resetShortcutWasPressed_) {
            resetSimulation();
        }
        if (!keyboardCaptured && deletePressed && !deleteKeyWasPressed_) {
            deleteSelectedPrimitive();
        }

        spaceKeyWasPressed_ = spacePressed;
        periodKeyWasPressed_ = periodPressed;
        deleteKeyWasPressed_ = deletePressed;
        resetShortcutWasPressed_ = resetPressed;
    }

    void updateDroneControls() {
        physics::Drone* drone = world_.drone(controlledDroneId_);
        if (drone == nullptr) {
            controlledDroneId_ = physics::kInvalidDroneId;
            droneBodyId_ = physics::kInvalidBodyId;
            followDroneCamera_ = false;
            return;
        }
        droneBodyId_ = drone->bodyId();

        physics::DronePilotCommand command{};
        const bool acceptsKeyboard =
            droneControlEnabled_ &&
            !cameraLookActive_ &&
            !ImGui::GetIO().WantCaptureKeyboard &&
            glfwGetWindowAttrib(window_, GLFW_FOCUSED) == GLFW_TRUE;
        if (acceptsKeyboard) {
            command.forward =
                (keyPressed(window_, GLFW_KEY_W) ? 1.0F : 0.0F) -
                (keyPressed(window_, GLFW_KEY_S) ? 1.0F : 0.0F);
            command.right =
                (keyPressed(window_, GLFW_KEY_A) ? 1.0F : 0.0F) -
                (keyPressed(window_, GLFW_KEY_D) ? 1.0F : 0.0F);
            command.yaw =
                (keyPressed(window_, GLFW_KEY_Q) ? 1.0F : 0.0F) -
                (keyPressed(window_, GLFW_KEY_E) ? 1.0F : 0.0F);
            command.climb =
                (keyPressed(window_, GLFW_KEY_UP) ? 1.0F : 0.0F) -
                (keyPressed(window_, GLFW_KEY_DOWN) ? 1.0F : 0.0F);
        }
        drone->setPilotCommand(command);
    }

    void advanceSimulation(float deltaSeconds) {
        updateSimulationShortcuts();
        updateDroneControls();
        if (simulationPaused_) {
            if (singleStepRequested_) {
                world_.stepFixed();
                // A manual step must show the completed tick even when the
                // scheduler phase is zero. Keep that pose through resume until
                // the next real tick establishes a continuous interpolation
                // interval whose previous endpoint is this exact pose.
                renderCurrentPoseUntilNextTick_ = true;
            }
        } else {
            const std::uint64_t tickBeforeAdvance =
                world_.debugStats().fixedTick;
            world_.advance(static_cast<double>(deltaSeconds));
            if (renderCurrentPoseUntilNextTick_ &&
                world_.debugStats().fixedTick > tickBeforeAdvance) {
                renderCurrentPoseUntilNextTick_ = false;
            }
        }
        singleStepRequested_ = false;
    }

    void appendDebugLine(
        const physics::Vec3& start,
        const physics::Vec3& end,
        const glm::vec4& color
    ) {
        render::DebugLineRenderData line{};
        line.start = toGlm(start);
        line.end = toGlm(end);
        line.color = color;
        debugLineSnapshot_.push_back(line);
    }

    void appendAabb(const physics::Aabb& bounds) {
        const physics::Vec3& minimum = bounds.minimum;
        const physics::Vec3& maximum = bounds.maximum;
        const std::array<physics::Vec3, 8U> corners{
            physics::Vec3{minimum.x, minimum.y, minimum.z},
            physics::Vec3{maximum.x, minimum.y, minimum.z},
            physics::Vec3{minimum.x, maximum.y, minimum.z},
            physics::Vec3{maximum.x, maximum.y, minimum.z},
            physics::Vec3{minimum.x, minimum.y, maximum.z},
            physics::Vec3{maximum.x, minimum.y, maximum.z},
            physics::Vec3{minimum.x, maximum.y, maximum.z},
            physics::Vec3{maximum.x, maximum.y, maximum.z}
        };
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
        for (const auto& edge : edges) {
            appendDebugLine(
                corners[edge[0]],
                corners[edge[1]],
                {0.26F, 0.72F, 1.0F, 0.78F}
            );
        }
    }

    void appendBoxCollider(
        const physics::RigidBody& body,
        const physics::Transform& transform
    ) {
        const auto corners = boxVerticesInRenderOrder(body, transform);
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
        for (const auto& edge : edges) {
            appendDebugLine(
                corners[edge[0]],
                corners[edge[1]],
                {0.42F, 0.92F, 0.58F, 0.82F}
            );
        }
    }

    void publishSceneData() {
        cubeSnapshot_.clear();
        debugLineSnapshot_.clear();
        // Retain the scheduler phase while paused so toggling pause never
        // snaps the scene forward by the remainder of a fixed tick.
        const float renderAlpha =
            renderCurrentPoseUntilNextTick_
                ? 1.0F
                : world_.interpolationAlpha();

        for (std::size_t slot = 1U; slot <= world_.bodySlotCount(); ++slot) {
            const physics::RigidBody* body =
                world_.body(static_cast<physics::BodyId>(slot));
            if (body == nullptr ||
                body->collider().type != physics::ColliderType::Box) {
                continue;
            }

            const physics::Transform renderTransform =
                interpolatedTransform(*body, renderAlpha);
            const physics::Drone* renderedDrone =
                world_.droneForBody(body->id());
            const glm::mat4 bodyMatrix =
                bodyToWorldMatrix(*body, renderTransform);
            render::CubeRenderData cube{};
            cube.bodyToWorld = bodyMatrix;
            cube.halfExtents = renderedDrone != nullptr
                ? glm::vec3{0.10F, 0.05F, 0.12F}
                : toGlm(body->collider().box.halfExtents);
            cube.roughness = renderedDrone != nullptr ? 0.34F : 0.48F;
            cube.baseColor = renderedDrone != nullptr
                ? glm::vec3{0.075F, 0.085F, 0.095F}
                : glm::vec3{0.19F, 0.42F, 0.72F};
            cube.flags = render::CubeRenderFlags::visible;
            if (body->id() == selectedBodyId_) {
                cube.flags |= render::CubeRenderFlags::selected;
            }
            if (body->id() == hoveredBodyId_) {
                cube.flags |= render::CubeRenderFlags::hovered;
                cube.highlightedVertexMask = hoveredVertexMask_;
            }
            if (body->id() == grabbedBodyId_) {
                cube.flags |= render::CubeRenderFlags::grabbed;
                cube.highlightedVertexMask = grabbedVertexMask_;
            }
            cube.objectId = body->id();
            cubeSnapshot_.push_back(cube);

            if (renderedDrone != nullptr) {
                const auto appendDronePart =
                    [&](const glm::mat4& localTransform,
                        const glm::vec3& halfExtents,
                        const glm::vec3& color,
                        float roughness) {
                        render::CubeRenderData part{};
                        part.bodyToWorld = bodyMatrix * localTransform;
                        part.halfExtents = halfExtents;
                        part.roughness = roughness;
                        part.baseColor = color;
                        part.flags = render::CubeRenderFlags::visible;
                        part.objectId = body->id();
                        cubeSnapshot_.push_back(part);
                    };
                appendDronePart(
                    glm::rotate(
                        glm::mat4{1.0F},
                        glm::radians(45.0F),
                        glm::vec3{0.0F, 1.0F, 0.0F}
                    ),
                    {0.018F, 0.018F, renderedDrone->description().armRadius},
                    {0.13F, 0.14F, 0.15F},
                    0.42F
                );
                appendDronePart(
                    glm::rotate(
                        glm::mat4{1.0F},
                        glm::radians(-45.0F),
                        glm::vec3{0.0F, 1.0F, 0.0F}
                    ),
                    {0.018F, 0.018F, renderedDrone->description().armRadius},
                    {0.13F, 0.14F, 0.15F},
                    0.42F
                );
                for (std::size_t motorIndex = 0U;
                     motorIndex < physics::kQuadMotorCount;
                     ++motorIndex) {
                    const physics::Vec3 localMotor =
                        renderedDrone->motorPositionLocal(motorIndex);
                    appendDronePart(
                        glm::translate(
                            glm::mat4{1.0F},
                            toGlm(localMotor)
                        ),
                        {0.045F, 0.028F, 0.045F},
                        {0.035F, 0.040F, 0.045F},
                        0.28F
                    );
                    const physics::Vec3 motorWorld =
                        renderTransform.position +
                        renderTransform.orientation.rotate(localMotor);
                    const physics::Vec3 thrustDirection =
                        renderTransform.orientation.rotate(
                            {0.0F, 1.0F, 0.0F}
                        );
                    appendDebugLine(
                        motorWorld,
                        motorWorld + thrustDirection *
                            (
                                renderedDrone->motors()[motorIndex]
                                    .thrustNewtons * 0.035F
                            ),
                        {0.55F, 0.88F, 0.62F, 0.72F}
                    );
                }
                appendDronePart(
                    glm::translate(
                        glm::mat4{1.0F},
                        {0.0F, 0.052F, 0.105F}
                    ),
                    {0.035F, 0.012F, 0.025F},
                    {0.60F, 0.63F, 0.66F},
                    0.30F
                );
            }

            if (showColliderDebug_) {
                appendBoxCollider(*body, renderTransform);
            }
            if (showVelocityDebug_) {
                const physics::Vec3 start = renderTransform.position;
                appendDebugLine(
                    start,
                    start + body->linearVelocity() * 0.25F,
                    {1.0F, 0.78F, 0.18F, 0.92F}
                );
            }
            if (showAabbDebug_) {
                const auto vertices =
                    boxVerticesInRenderOrder(*body, renderTransform);
                physics::Aabb bounds{vertices[0], vertices[0]};
                for (const physics::Vec3& vertex : vertices) {
                    bounds.minimum.x =
                        std::min(bounds.minimum.x, vertex.x);
                    bounds.minimum.y =
                        std::min(bounds.minimum.y, vertex.y);
                    bounds.minimum.z =
                        std::min(bounds.minimum.z, vertex.z);
                    bounds.maximum.x =
                        std::max(bounds.maximum.x, vertex.x);
                    bounds.maximum.y =
                        std::max(bounds.maximum.y, vertex.y);
                    bounds.maximum.z =
                        std::max(bounds.maximum.z, vertex.z);
                }
                appendAabb(bounds);
            }
            if (showCenterOfMassDebug_) {
                const physics::Vec3 center = renderTransform.position;
                constexpr physics::Real radius = 0.18F;
                appendDebugLine(
                    center - physics::Vec3{radius, 0.0F, 0.0F},
                    center + physics::Vec3{radius, 0.0F, 0.0F},
                    {1.0F, 0.25F, 0.20F, 1.0F}
                );
                appendDebugLine(
                    center - physics::Vec3{0.0F, radius, 0.0F},
                    center + physics::Vec3{0.0F, radius, 0.0F},
                    {0.25F, 1.0F, 0.35F, 1.0F}
                );
                appendDebugLine(
                    center - physics::Vec3{0.0F, 0.0F, radius},
                    center + physics::Vec3{0.0F, 0.0F, radius},
                    {0.30F, 0.55F, 1.0F, 1.0F}
                );
            }
        }

        if (showContactDebug_) {
            for (const physics::ContactDebugPoint& contact :
                 world_.debugContacts()) {
                const physics::Real normalLength =
                    0.20F + std::min(contact.normalImpulse * 0.001F, 0.60F);
                appendDebugLine(
                    contact.point,
                    contact.point + contact.normal * normalLength,
                    {1.0F, 0.22F, 0.18F, 1.0F}
                );
                constexpr physics::Real crossSize = 0.035F;
                appendDebugLine(
                    contact.point - physics::Vec3{crossSize, 0.0F, 0.0F},
                    contact.point + physics::Vec3{crossSize, 0.0F, 0.0F},
                    {1.0F, 0.72F, 0.18F, 1.0F}
                );
                appendDebugLine(
                    contact.point - physics::Vec3{0.0F, 0.0F, crossSize},
                    contact.point + physics::Vec3{0.0F, 0.0F, crossSize},
                    {1.0F, 0.72F, 0.18F, 1.0F}
                );
            }
        }

        if (showWindDebug_) {
            const double simulationTime = world_.debugStats().simulationTime;
            physics::Vec3 center{};
            const physics::RigidBody* windFocus = world_.body(selectedBodyId_);
            if (windFocus == nullptr ||
                windFocus->motionType() != physics::MotionType::Dynamic) {
                windFocus = nullptr;
                for (std::size_t slot = 1U;
                     slot <= world_.bodySlotCount();
                     ++slot) {
                    const physics::RigidBody* candidate =
                        world_.body(static_cast<physics::BodyId>(slot));
                    if (candidate != nullptr &&
                        candidate->motionType() ==
                            physics::MotionType::Dynamic) {
                        windFocus = candidate;
                        break;
                    }
                }
            }
            if (windFocus != nullptr) {
                center = windFocus->transform().position;
            }
            const physics::Real baseX =
                std::floor(center.x / 2.0F) * 2.0F;
            const physics::Real baseZ =
                std::floor(center.z / 2.0F) * 2.0F;
            for (int x = -4; x <= 4; x += 2) {
                for (int z = -4; z <= 4; z += 2) {
                    const physics::Vec3 sample{
                        baseX + static_cast<float>(x),
                        0.25F,
                        baseZ + static_cast<float>(z)
                    };
                    const physics::Vec3 wind =
                        world_.environment().windVelocity(sample, simulationTime);
                    appendDebugLine(
                        sample,
                        sample + wind * windDebugScale_,
                        {0.18F, 0.88F, 1.0F, 0.78F}
                    );
                }
            }
        }

        if (grabbedSpringId_ != physics::kInvalidSpringId) {
            if (const physics::RigidBody* body = world_.body(grabbedBodyId_)) {
                appendDebugLine(
                    body->worldPointFromLocal(grabbedLocalAnchor_),
                    grabWorldTarget_,
                    {1.0F, 0.38F, 0.08F, 1.0F}
                );
            }
        }

        render::SceneRenderDataView scene{};
        scene.cubes = cubeSnapshot_.data();
        scene.cubeCount = cubeSnapshot_.size();
        scene.debugLines = debugLineSnapshot_.data();
        scene.debugLineCount = debugLineSnapshot_.size();
        scene.spotLights = spotLights_.data();
        scene.spotLightCount = spotLights_.size();
        renderer_.setSceneData(scene);
    }

    void drawViewportOverlay(ImGuiID dockspaceId) const {
        if (!showViewportOverlay_) {
            return;
        }

        const ImGuiDockNode* centralNode = ImGui::DockBuilderGetCentralNode(dockspaceId);
        if (centralNode == nullptr || centralNode->Size.x < 260.0F ||
            centralNode->Size.y < 120.0F) {
            return;
        }

        const ImVec2 overlayPosition(
            centralNode->Pos.x + centralNode->Size.x - 12.0F,
            centralNode->Pos.y + centralNode->Size.y - 12.0F
        );
        ImGui::SetNextWindowPos(
            overlayPosition,
            ImGuiCond_Always,
            ImVec2(1.0F, 1.0F)
        );
        ImGui::SetNextWindowBgAlpha(0.76F);

        constexpr ImGuiWindowFlags overlayFlags =
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav |
            ImGuiWindowFlags_NoDocking |
            ImGuiWindowFlags_NoInputs;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0F, 7.0F));
        if (ImGui::Begin("##ViewportControls", nullptr, overlayFlags)) {
            if (world_.drone(controlledDroneId_) != nullptr &&
                droneControlEnabled_) {
                ImGui::TextDisabled("DRONE CONTROL");
                ImGui::TextUnformatted(
                    "W/S  Forward/back    A/D  Left/right"
                );
                ImGui::TextUnformatted(
                    "Q/E  Yaw    Up/Down  Height"
                );
                ImGui::TextDisabled(
                    followDroneCamera_
                        ? "Follow camera active   Home exits follow"
                        : "Use the Drone panel to enable follow camera"
                );
            } else {
                ImGui::TextDisabled("SCENE VIEWPORT");
                ImGui::TextUnformatted("RMB + WASD  Move and look");
                ImGui::TextUnformatted(
                    "Ctrl  Hover vertices    Ctrl + LMB  Pull"
                );
                ImGui::TextDisabled(
                    "Q / E  Down / up    Shift  Faster    Home  Reset camera"
                );
            }
            ImGui::TextDisabled(
                "%s   %u Hz   tick %llu",
                simulationPaused_ ? "Paused" : "Running",
                world_.settings().fixedUpdateHz,
                static_cast<unsigned long long>(
                    world_.debugStats().fixedTick
                )
            );
            if (grabbedSpringId_ != physics::kInvalidSpringId) {
                ImGui::TextColored(
                    uiColor(117, 174, 123),
                    "Point spring active"
                );
            } else if (spawnerPlacementArmed_) {
                ImGui::TextColored(
                    uiColor(190, 190, 190),
                    "Cube placement armed - click a surface"
                );
            } else if (hoveredBodyId_ != physics::kInvalidBodyId) {
                ImGui::TextColored(
                    uiColor(165, 170, 178),
                    "Physics vertex group ready"
                );
            }
            const auto& position = camera_.position();
            ImGui::TextDisabled(
                "Camera  %.1f  %.1f  %.1f",
                position.x,
                position.y,
                position.z
            );
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
    }

    void updateCamera(float deltaSeconds) {
        if (keyPressed(window_, GLFW_KEY_HOME) && !homeKeyWasPressed_ &&
            !ImGui::GetIO().WantCaptureKeyboard) {
            resetCamera();
        }
        homeKeyWasPressed_ = keyPressed(window_, GLFW_KEY_HOME);

        if (followDroneCamera_) {
            const physics::Drone* drone =
                world_.drone(controlledDroneId_);
            const physics::RigidBody* body = drone != nullptr
                ? world_.body(drone->bodyId())
                : nullptr;
            if (body != nullptr) {
                releaseCameraLook();
                const physics::Vec3 bodyForwardPhysics =
                    body->transform().orientation.rotate(
                        {0.0F, 0.0F, 1.0F}
                    );
                glm::vec3 horizontalForward{
                    bodyForwardPhysics.x,
                    0.0F,
                    bodyForwardPhysics.z
                };
                const float forwardLengthSquared =
                    glm::dot(horizontalForward, horizontalForward);
                horizontalForward = forwardLengthSquared > 1.0e-6F
                    ? horizontalForward *
                        glm::inversesqrt(forwardLengthSquared)
                    : glm::vec3{0.0F, 0.0F, 1.0F};
                const glm::vec3 target =
                    toGlm(body->transform().position) +
                    glm::vec3{0.0F, 0.18F, 0.0F};
                const glm::vec3 desiredPosition =
                    target - horizontalForward * 4.2F +
                    glm::vec3{0.0F, 1.8F, 0.0F};
                if (!followCameraInitialized_ ||
                    !std::isfinite(deltaSeconds) ||
                    deltaSeconds <= 0.0F) {
                    camera_.setPosition(desiredPosition);
                    followCameraInitialized_ = true;
                } else {
                    const float followAmount =
                        1.0F - std::exp(-6.5F * deltaSeconds);
                    camera_.setPosition(
                        camera_.position() +
                        (desiredPosition - camera_.position()) *
                            followAmount
                    );
                }
                const glm::vec3 viewDirection =
                    glm::normalize(target - camera_.position());
                const float yawDegrees =
                    std::atan2(viewDirection.z, viewDirection.x) *
                    (180.0F / kPi);
                const float pitchDegrees =
                    std::asin(
                        std::clamp(viewDirection.y, -1.0F, 1.0F)
                    ) * (180.0F / kPi);
                camera_.setOrientation(yawDegrees, pitchDegrees);
                return;
            }
            followDroneCamera_ = false;
            followCameraInitialized_ = false;
        }

        const bool windowFocused =
            glfwGetWindowAttrib(window_, GLFW_FOCUSED) == GLFW_TRUE;
        const bool rightMousePressed =
            glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool uiAcceptsLook =
            cameraLookActive_ || !ImGui::GetIO().WantCaptureMouse;
        const bool shouldLook = windowFocused && rightMousePressed && uiAcceptsLook;

        if (shouldLook && !cameraLookActive_) {
            cameraLookActive_ = true;
            firstMouseSample_ = true;
            glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            if (glfwRawMouseMotionSupported() == GLFW_TRUE) {
                glfwSetInputMode(window_, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
            }
        } else if (!shouldLook && cameraLookActive_) {
            releaseCameraLook();
        }

        render::CameraInput input{};
        input.lookActive = cameraLookActive_;
        if (cameraLookActive_) {
            double cursorX = 0.0;
            double cursorY = 0.0;
            glfwGetCursorPos(window_, &cursorX, &cursorY);
            if (firstMouseSample_) {
                previousCursorX_ = cursorX;
                previousCursorY_ = cursorY;
                firstMouseSample_ = false;
            } else {
                input.mouseDeltaX = static_cast<float>(cursorX - previousCursorX_);
                input.mouseDeltaY = static_cast<float>(cursorY - previousCursorY_);
                previousCursorX_ = cursorX;
                previousCursorY_ = cursorY;
            }

            input.forward = keyPressed(window_, GLFW_KEY_W);
            input.backward = keyPressed(window_, GLFW_KEY_S);
            input.left = keyPressed(window_, GLFW_KEY_A);
            input.right = keyPressed(window_, GLFW_KEY_D);
            input.down = keyPressed(window_, GLFW_KEY_Q);
            input.up = keyPressed(window_, GLFW_KEY_E);
            input.sprint =
                keyPressed(window_, GLFW_KEY_LEFT_SHIFT) ||
                keyPressed(window_, GLFW_KEY_RIGHT_SHIFT);
        }
        camera_.update(input, deltaSeconds);
    }

    void resetCamera() {
        followDroneCamera_ = false;
        followCameraInitialized_ = false;
        camera_.setPosition({7.5F, 4.8F, 7.5F});
        camera_.setOrientation(-135.0F, -22.0F);
        camera_.setFieldOfView(58.0F);
    }

    void releaseCameraLook() {
        if (window_ == nullptr || !cameraLookActive_) {
            return;
        }
        if (glfwRawMouseMotionSupported() == GLFW_TRUE) {
            glfwSetInputMode(window_, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
        }
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        cameraLookActive_ = false;
        firstMouseSample_ = true;
    }

    void destroyWindow() {
        if (window_ != nullptr) {
            glfwSetWindowUserPointer(window_, nullptr);
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        if (glfwInitialized_) {
            glfwTerminate();
            glfwInitialized_ = false;
        }
    }

    void destroyImGuiContext() noexcept {
        if (imguiContextCreated_) {
            ImGui::DestroyContext();
            imguiContextCreated_ = false;
        }
    }

    void shutdownNoThrow() noexcept {
        releaseGrab();
        if (window_ != nullptr) {
            releaseCameraLook();
        }
        if (renderer_.isInitialized()) {
            try {
                renderer_.waitIdle();
            } catch (...) {
            }
            try {
                renderer_.shutdown();
            } catch (...) {
            }
        }
        destroyImGuiContext();
        destroyWindow();
    }

    StudioApplicationConfig config_;
    std::filesystem::path executableDirectory_;
    GLFWwindow* window_{nullptr};
    bool glfwInitialized_{false};
    bool imguiContextCreated_{false};

    render::VulkanRenderer renderer_;
    render::Camera camera_;
    render::GroundRenderSettings groundSettings_;
    physics::PhysicsSettings physicsSettings_{};
    physics::World world_{physicsSettings_};
    CubeSpawnerSettings spawnerSettings_{};
    std::mt19937 spawnRandomEngine_{0x55415653U};
    std::string spawnerStatus_{"Ready."};
    physics::BodyId groundId_{physics::kInvalidBodyId};
    physics::BodyId cubeId_{physics::kInvalidBodyId};
    physics::DroneId droneId_{physics::kInvalidDroneId};
    physics::DroneId controlledDroneId_{physics::kInvalidDroneId};
    physics::BodyId droneBodyId_{physics::kInvalidBodyId};
    physics::BodyId selectedBodyId_{physics::kInvalidBodyId};
    physics::BodyId hoveredBodyId_{physics::kInvalidBodyId};
    physics::BodyId grabbedBodyId_{physics::kInvalidBodyId};
    physics::SpringId grabbedSpringId_{physics::kInvalidSpringId};
    physics::Vec3 hoverAnchorWorld_{};
    physics::Vec3 grabbedLocalAnchor_{};
    physics::Vec3 grabWorldTarget_{};
    physics::Vec3 grabRayToAnchorOffset_{};
    float hoverRayDistance_{0.0F};
    float grabRayDistance_{0.0F};
    std::uint32_t hoveredVertexMask_{0U};
    std::uint32_t grabbedVertexMask_{0U};

    std::vector<render::CubeRenderData> cubeSnapshot_;
    std::vector<render::DebugLineRenderData> debugLineSnapshot_;
    std::vector<render::SpotLightRenderData> spotLights_;
    std::vector<unsigned char> fontFileBytes_;
    ImVec2 sceneViewportPosition_{};
    ImVec2 sceneViewportSize_{};
    ImVec2 sceneViewportFramebufferSize_{};

    bool showScenePanel_{true};
    bool showInspectorPanel_{true};
    bool showRendererPanel_{true};
    bool showPhysicsPanel_{true};
    bool showDronePanel_{true};
    bool showSpawnerPanel_{true};
    bool showViewportOverlay_{true};
    bool showContactDebug_{true};
    bool showColliderDebug_{false};
    bool showVelocityDebug_{false};
    bool showAabbDebug_{false};
    bool showCenterOfMassDebug_{true};
    bool showWindDebug_{false};
    bool sceneViewportValid_{false};
    bool simulationPaused_{false};
    bool singleStepRequested_{false};
    bool interFontLoaded_{false};
    bool resetDockLayout_{false};
    bool openAboutPopup_{false};
    bool cameraLookActive_{false};
    bool droneControlEnabled_{true};
    bool followDroneCamera_{true};
    bool followCameraInitialized_{false};
    bool droneMotorVariationEnabled_{true};
    bool firstMouseSample_{true};
    bool homeKeyWasPressed_{false};
    bool leftMouseWasPressed_{false};
    bool spaceKeyWasPressed_{false};
    bool periodKeyWasPressed_{false};
    bool deleteKeyWasPressed_{false};
    bool resetShortcutWasPressed_{false};
    bool spawnerPlacementArmed_{false};
    bool renderCurrentPoseUntilNextTick_{false};
    std::uint32_t nextCubeOrdinal_{2U};
    float grabInfluenceRadius_{kDefaultGrabInfluenceRadiusMeters};
    float grabFrequencyHz_{kDefaultGrabFrequencyHz};
    float grabMaximumAcceleration_{kDefaultGrabMaximumAcceleration};
    float windDebugScale_{2.5F};
    float droneMotorVariationFraction_{0.015F};
    double previousCursorX_{0.0};
    double previousCursorY_{0.0};
};

StudioApplication::StudioApplication(StudioApplicationConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

StudioApplication::~StudioApplication() = default;

int StudioApplication::run() {
    return impl_->run();
}

} // namespace uaview::app
