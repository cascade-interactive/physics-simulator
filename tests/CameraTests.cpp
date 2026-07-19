#include <uaview/render/Camera.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace {

using uaview::render::Camera;
using uaview::render::CameraInput;

int failureCount = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failureCount;
    }
}

[[nodiscard]] bool near(float actual, float expected, float tolerance = 1.0e-4F) {
    return std::abs(actual - expected) <= tolerance;
}

[[nodiscard]] bool near(
    const glm::vec3& actual,
    const glm::vec3& expected,
    float tolerance = 1.0e-4F
) {
    return glm::length(actual - expected) <= tolerance;
}

void testDefaultPoseFacesTheGroundOrigin() {
    Camera camera;

    expect(
        near(glm::length(camera.forward()), 1.0F),
        "forward vector must remain normalized"
    );
    expect(
        glm::dot(camera.forward(), glm::normalize(-camera.position())) > 0.995F,
        "default camera should face the center of the ground plane"
    );
}

void testLookModeGatesEditorInput() {
    Camera camera;
    const glm::vec3 initialPosition = camera.position();
    const float initialYaw = camera.yawDegrees();
    const float initialPitch = camera.pitchDegrees();

    CameraInput input;
    input.forward = true;
    input.right = true;
    input.mouseDeltaX = 300.0F;
    input.mouseDeltaY = -200.0F;
    camera.update(input, 1.0F);

    expect(near(camera.position(), initialPosition), "inactive RMB look must gate movement");
    expect(near(camera.yawDegrees(), initialYaw), "inactive RMB look must gate yaw");
    expect(near(camera.pitchDegrees(), initialPitch), "inactive RMB look must gate pitch");
}

void testMovementIsFrameRateIndependent() {
    Camera oneStep;
    Camera manySteps;
    oneStep.setPosition(glm::vec3{0.0F});
    manySteps.setPosition(glm::vec3{0.0F});
    oneStep.setOrientation(-90.0F, -20.0F);
    manySteps.setOrientation(-90.0F, -20.0F);

    CameraInput input;
    input.lookActive = true;
    input.forward = true;
    input.right = true;
    input.up = true;

    oneStep.update(input, 1.0F);
    for (int frame = 0; frame < 120; ++frame) {
        manySteps.update(input, 1.0F / 120.0F);
    }

    expect(
        near(oneStep.position(), manySteps.position(), 2.0e-4F),
        "movement over equal time must not depend on update frequency"
    );
}

void testDiagonalMovementIsNormalized() {
    Camera straight;
    Camera diagonal;
    straight.setPosition(glm::vec3{0.0F});
    diagonal.setPosition(glm::vec3{0.0F});
    straight.setOrientation(-90.0F, 0.0F);
    diagonal.setOrientation(-90.0F, 0.0F);

    CameraInput straightInput;
    straightInput.lookActive = true;
    straightInput.forward = true;
    CameraInput diagonalInput = straightInput;
    diagonalInput.right = true;

    straight.update(straightInput, 0.75F);
    diagonal.update(diagonalInput, 0.75F);

    expect(
        near(glm::length(straight.position()), glm::length(diagonal.position())),
        "diagonal input must not move faster than single-axis input"
    );
    expect(
        near(glm::length(straight.position()), 3.75F),
        "default camera speed should be five world units per second"
    );
}

void testSprintAndInvalidDeltaTime() {
    Camera normal;
    Camera sprint;
    normal.setPosition(glm::vec3{0.0F});
    sprint.setPosition(glm::vec3{0.0F});
    normal.setOrientation(-90.0F, 0.0F);
    sprint.setOrientation(-90.0F, 0.0F);

    CameraInput input;
    input.lookActive = true;
    input.forward = true;
    normal.update(input, 0.5F);
    input.sprint = true;
    sprint.update(input, 0.5F);

    expect(
        near(glm::length(sprint.position()), glm::length(normal.position()) * 3.0F),
        "sprint multiplier should be applied after normalized movement"
    );

    const glm::vec3 beforeInvalidUpdates = normal.position();
    normal.update(input, -1.0F);
    normal.update(input, std::numeric_limits<float>::quiet_NaN());
    expect(
        near(normal.position(), beforeInvalidUpdates),
        "invalid delta times must not change position"
    );
}

void testOrientationAndFieldOfViewBounds() {
    Camera camera;
    camera.setOrientation(765.0F, 140.0F);
    expect(near(camera.yawDegrees(), 45.0F), "yaw should wrap into a stable range");
    expect(near(camera.pitchDegrees(), 89.0F), "pitch should clamp below the pole");

    camera.setOrientation(-765.0F, -140.0F);
    expect(near(camera.yawDegrees(), -45.0F), "negative yaw should wrap symmetrically");
    expect(near(camera.pitchDegrees(), -89.0F), "negative pitch should clamp below the pole");

    camera.setFieldOfView(0.0F);
    expect(near(camera.fieldOfViewDegrees(), 1.0F), "field of view must remain positive");
    camera.setFieldOfView(200.0F);
    expect(
        near(camera.fieldOfViewDegrees(), 179.0F),
        "field of view must remain below a singular 180 degrees"
    );
}

void testMouseLookUsesPointerDeltaNotFrameTime() {
    Camera shortFrame;
    Camera longFrame;
    shortFrame.setPosition(glm::vec3{0.0F});
    longFrame.setPosition(glm::vec3{0.0F});
    shortFrame.setOrientation(0.0F, 0.0F);
    longFrame.setOrientation(0.0F, 0.0F);

    CameraInput input;
    input.lookActive = true;
    input.mouseDeltaX = 100.0F;
    input.mouseDeltaY = -50.0F;
    shortFrame.update(input, 1.0F / 240.0F);
    longFrame.update(input, 1.0F / 15.0F);

    expect(
        near(shortFrame.yawDegrees(), longFrame.yawDegrees())
            && near(shortFrame.pitchDegrees(), longFrame.pitchDegrees()),
        "mouse look must be based on accumulated pointer delta, not delta time"
    );
    expect(near(shortFrame.yawDegrees(), 12.0F), "yaw sensitivity should be deterministic");
    expect(near(shortFrame.pitchDegrees(), 6.0F), "pitch sensitivity should be deterministic");
}

void testViewMatrixUsesRightHandedCameraSpace() {
    Camera camera;
    camera.setPosition(glm::vec3{3.0F, 2.0F, 1.0F});
    camera.setOrientation(-90.0F, 0.0F);

    const glm::mat4 view = camera.viewMatrix();
    const glm::vec4 cameraInView = view * glm::vec4(camera.position(), 1.0F);
    const glm::vec4 pointAheadInView =
        view * glm::vec4(camera.position() + camera.forward() * 4.0F, 1.0F);

    expect(
        near(glm::vec3(cameraInView), glm::vec3{0.0F}),
        "view matrix must map the camera position to the origin"
    );
    expect(
        near(glm::vec3(pointAheadInView), glm::vec3{0.0F, 0.0F, -4.0F}),
        "view matrix must map forward to negative view-space Z"
    );
}

void testProjectionUsesVulkanClipSpace() {
    Camera camera;
    camera.setFieldOfView(90.0F);
    const glm::mat4 projection = camera.projectionMatrix(1.0F);

    const glm::vec4 nearClip = projection * glm::vec4{0.0F, 0.0F, -0.05F, 1.0F};
    const glm::vec4 farClip = projection * glm::vec4{0.0F, 0.0F, -2'000.0F, 1.0F};
    const glm::vec4 upperClip = projection * glm::vec4{0.0F, 1.0F, -1.0F, 1.0F};

    expect(
        near(nearClip.z / nearClip.w, 0.0F, 2.0e-5F),
        "Vulkan near plane must map to NDC depth zero"
    );
    expect(
        near(farClip.z / farClip.w, 1.0F, 2.0e-5F),
        "Vulkan far plane must map to NDC depth one"
    );
    expect(
        upperClip.y / upperClip.w < 0.0F,
        "projection must flip clip-space Y for a positive Vulkan viewport"
    );

    const glm::mat4 fallbackProjection = camera.projectionMatrix(0.0F);
    expect(
        std::isfinite(fallbackProjection[0][0]),
        "invalid aspect ratios must produce a finite fallback projection"
    );
}

void testViewportRaysMatchTheVisibleCameraFrustum() {
    Camera camera;
    camera.setPosition({2.0F, 3.0F, 4.0F});
    camera.setOrientation(-90.0F, 0.0F);
    camera.setFieldOfView(90.0F);

    const auto center = camera.rayFromViewport(800.0F, 450.0F, 1600.0F, 900.0F);
    const auto upperLeft = camera.rayFromViewport(0.0F, 0.0F, 1600.0F, 900.0F);
    const glm::vec3 right = glm::normalize(
        glm::cross(camera.forward(), glm::vec3{0.0F, 1.0F, 0.0F})
    );

    expect(near(center.origin, camera.position()), "camera rays must start at the eye");
    expect(
        near(center.direction, camera.forward()),
        "the viewport center ray must match camera forward"
    );
    expect(
        near(glm::length(upperLeft.direction), 1.0F),
        "viewport ray directions must remain normalized"
    );
    expect(
        glm::dot(upperLeft.direction, right) < 0.0F,
        "the left side of the viewport must raycast to camera-left"
    );
    expect(
        upperLeft.direction.y > 0.0F,
        "the top of the viewport must raycast above camera-forward"
    );

    const auto fallback = camera.rayFromViewport(0.0F, 0.0F, 0.0F, 0.0F);
    expect(
        near(fallback.direction, camera.forward()),
        "an invalid viewport must return a safe center ray"
    );
}

} // namespace

int main() {
    testDefaultPoseFacesTheGroundOrigin();
    testLookModeGatesEditorInput();
    testMovementIsFrameRateIndependent();
    testDiagonalMovementIsNormalized();
    testSprintAndInvalidDeltaTime();
    testOrientationAndFieldOfViewBounds();
    testMouseLookUsesPointerDeltaNotFrameTime();
    testViewMatrixUsesRightHandedCameraSpace();
    testProjectionUsesVulkanClipSpace();
    testViewportRaysMatchTheVisibleCameraFrustum();

    if (failureCount != 0) {
        std::cerr << failureCount << " camera test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All camera tests passed.\n";
    return EXIT_SUCCESS;
}
