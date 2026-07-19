#include <uaview/render/Camera.hpp>

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec4.hpp>

namespace uaview::render {
namespace {

constexpr glm::vec3 kWorldUp{0.0F, 1.0F, 0.0F};
constexpr float kMinimumPitchDegrees = -89.0F;
constexpr float kMaximumPitchDegrees = 89.0F;
constexpr float kMinimumFovDegrees = 1.0F;
constexpr float kMaximumFovDegrees = 179.0F;
constexpr float kNearPlane = 0.05F;
constexpr float kFarPlane = 2'000.0F;
constexpr float kDirectionEpsilon = 1.0e-8F;

[[nodiscard]] float wrapYawDegrees(float degrees) noexcept {
    if (!std::isfinite(degrees)) {
        return 0.0F;
    }

    float wrapped = std::fmod(degrees + 180.0F, 360.0F);
    if (wrapped < 0.0F) {
        wrapped += 360.0F;
    }
    return wrapped - 180.0F;
}

[[nodiscard]] float clampPitchDegrees(float degrees) noexcept {
    if (!std::isfinite(degrees)) {
        return 0.0F;
    }
    return std::clamp(degrees, kMinimumPitchDegrees, kMaximumPitchDegrees);
}

[[nodiscard]] float clampFovDegrees(float degrees) noexcept {
    if (!std::isfinite(degrees)) {
        return 58.0F;
    }
    return std::clamp(degrees, kMinimumFovDegrees, kMaximumFovDegrees);
}

} // namespace

Camera::Camera() = default;

void Camera::update(const CameraInput& input, float deltaSeconds) {
    // Editor camera input is intentionally captured only while RMB look mode is
    // active so normal keyboard and mouse use remains available to Dear ImGui.
    if (!input.lookActive) {
        return;
    }

    setOrientation(
        yawDegrees_ + input.mouseDeltaX * lookSensitivity_,
        pitchDegrees_ - input.mouseDeltaY * lookSensitivity_
    );

    if (!std::isfinite(deltaSeconds) || deltaSeconds <= 0.0F) {
        return;
    }

    const glm::vec3 cameraForward = forward();
    glm::vec3 cameraRight = glm::cross(cameraForward, kWorldUp);
    const float rightLengthSquared = glm::dot(cameraRight, cameraRight);
    if (rightLengthSquared > 0.0F) {
        cameraRight *= glm::inversesqrt(rightLengthSquared);
    } else {
        cameraRight = glm::vec3{1.0F, 0.0F, 0.0F};
    }

    glm::vec3 movement{0.0F};
    movement += cameraForward * (input.forward ? 1.0F : 0.0F);
    movement -= cameraForward * (input.backward ? 1.0F : 0.0F);
    movement -= cameraRight * (input.left ? 1.0F : 0.0F);
    movement += cameraRight * (input.right ? 1.0F : 0.0F);
    movement += kWorldUp * (input.up ? 1.0F : 0.0F);
    movement -= kWorldUp * (input.down ? 1.0F : 0.0F);

    const float movementLengthSquared = glm::dot(movement, movement);
    if (movementLengthSquared <= 0.0F) {
        return;
    }

    movement *= glm::inversesqrt(movementLengthSquared);
    const float speed = moveSpeed_ * (input.sprint ? sprintMultiplier_ : 1.0F);
    position_ += movement * speed * deltaSeconds;
}

void Camera::setPosition(const glm::vec3& position) noexcept {
    position_ = position;
}

void Camera::setOrientation(float yawDegrees, float pitchDegrees) noexcept {
    yawDegrees_ = wrapYawDegrees(yawDegrees);
    pitchDegrees_ = clampPitchDegrees(pitchDegrees);
}

void Camera::setFieldOfView(float verticalFovDegrees) noexcept {
    verticalFovDegrees_ = clampFovDegrees(verticalFovDegrees);
}

glm::mat4 Camera::viewMatrix() const {
    return glm::lookAtRH(position_, position_ + forward(), kWorldUp);
}

glm::mat4 Camera::projectionMatrix(float aspectRatio) const {
    const float safeAspect =
        std::isfinite(aspectRatio) && aspectRatio > 0.0F ? aspectRatio : 1.0F;

    // GLM's RH_ZO form supplies Vulkan's [0, 1] depth range. Vulkan framebuffer
    // coordinates have the opposite Y direction from GLM's clip convention, so
    // flip the projection rather than every mesh or camera transform.
    glm::mat4 projection = glm::perspectiveRH_ZO(
        glm::radians(verticalFovDegrees_),
        safeAspect,
        kNearPlane,
        kFarPlane
    );
    projection[1][1] *= -1.0F;
    return projection;
}

glm::mat4 Camera::viewProjectionMatrix(float aspectRatio) const {
    return projectionMatrix(aspectRatio) * viewMatrix();
}

CameraRay Camera::rayFromViewport(
    float cursorX,
    float cursorY,
    float viewportWidth,
    float viewportHeight
) const noexcept {
    if (!std::isfinite(cursorX) || !std::isfinite(cursorY) ||
        !std::isfinite(viewportWidth) || !std::isfinite(viewportHeight) ||
        viewportWidth <= 0.0F || viewportHeight <= 0.0F) {
        return {position_, forward()};
    }

    const float normalizedX =
        (2.0F * cursorX / viewportWidth) - 1.0F;
    const float normalizedY =
        1.0F - (2.0F * cursorY / viewportHeight);
    const float tangent =
        std::tan(glm::radians(verticalFovDegrees_) * 0.5F);
    const float aspectRatio = viewportWidth / viewportHeight;

    const glm::vec3 cameraForward = forward();
    glm::vec3 cameraRight = glm::cross(cameraForward, kWorldUp);
    const float rightLengthSquared = glm::dot(cameraRight, cameraRight);
    cameraRight = rightLengthSquared > kDirectionEpsilon
        ? cameraRight * glm::inversesqrt(rightLengthSquared)
        : glm::vec3{1.0F, 0.0F, 0.0F};
    const glm::vec3 cameraUp =
        glm::normalize(glm::cross(cameraRight, cameraForward));

    const glm::vec3 direction = glm::normalize(
        cameraForward +
        cameraRight * normalizedX * tangent * aspectRatio +
        cameraUp * normalizedY * tangent
    );
    return {position_, direction};
}

const glm::vec3& Camera::position() const noexcept {
    return position_;
}

glm::vec3 Camera::forward() const noexcept {
    const float yaw = glm::radians(yawDegrees_);
    const float pitch = glm::radians(pitchDegrees_);
    return glm::normalize(glm::vec3{
        std::cos(yaw) * std::cos(pitch),
        std::sin(pitch),
        std::sin(yaw) * std::cos(pitch),
    });
}

float Camera::yawDegrees() const noexcept {
    return yawDegrees_;
}

float Camera::pitchDegrees() const noexcept {
    return pitchDegrees_;
}

float Camera::fieldOfViewDegrees() const noexcept {
    return verticalFovDegrees_;
}

} // namespace uaview::render
