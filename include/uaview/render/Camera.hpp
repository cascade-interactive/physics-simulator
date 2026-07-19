#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace uaview::render {

struct CameraInput {
    bool forward{false};
    bool backward{false};
    bool left{false};
    bool right{false};
    bool up{false};
    bool down{false};
    bool sprint{false};
    bool lookActive{false};
    float mouseDeltaX{0.0F};
    float mouseDeltaY{0.0F};
};

struct CameraRay {
    glm::vec3 origin{0.0F};
    glm::vec3 direction{0.0F, 0.0F, -1.0F};
};

class Camera {
public:
    Camera();

    void update(const CameraInput& input, float deltaSeconds);
    void setPosition(const glm::vec3& position) noexcept;
    void setOrientation(float yawDegrees, float pitchDegrees) noexcept;
    void setFieldOfView(float verticalFovDegrees) noexcept;

    [[nodiscard]] glm::mat4 viewMatrix() const;
    [[nodiscard]] glm::mat4 projectionMatrix(float aspectRatio) const;
    [[nodiscard]] glm::mat4 viewProjectionMatrix(float aspectRatio) const;
    [[nodiscard]] CameraRay rayFromViewport(
        float cursorX,
        float cursorY,
        float viewportWidth,
        float viewportHeight
    ) const noexcept;
    [[nodiscard]] const glm::vec3& position() const noexcept;
    [[nodiscard]] glm::vec3 forward() const noexcept;
    [[nodiscard]] float yawDegrees() const noexcept;
    [[nodiscard]] float pitchDegrees() const noexcept;
    [[nodiscard]] float fieldOfViewDegrees() const noexcept;

private:
    glm::vec3 position_{7.5F, 4.8F, 7.5F};
    float yawDegrees_{-135.0F};
    float pitchDegrees_{-22.0F};
    float verticalFovDegrees_{58.0F};
    float moveSpeed_{5.0F};
    float sprintMultiplier_{3.0F};
    float lookSensitivity_{0.12F};
};

} // namespace uaview::render
