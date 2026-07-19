#include <uaview/physics/Drone.hpp>
#include <uaview/physics/World.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

using namespace uaview::physics;

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "Drone dynamics test failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

[[nodiscard]] bool near(Real lhs, Real rhs, Real tolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

[[nodiscard]] World makeWorld(const Vec3& wind = {}) {
    PhysicsSettings settings{};
    settings.fixedUpdateHz = 120U;
    settings.solverSubsteps = 1U;
    settings.maximumAdaptiveSubsteps = 8U;
    World world{settings};
    WindFieldSettings windSettings{};
    windSettings.meanVelocity = wind;
    windSettings.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(windSettings);
    BodyDescription ground =
        BodyDescription::makeStaticPlane({0.0F, 1.0F, 0.0F}, 0.0F);
    world.createBody(ground);
    return world;
}

[[nodiscard]] DroneId createFlyingDrone(
    World& world,
    const Vec3& position = {0.0F, 3.0F, 0.0F},
    bool variation = false
) {
    DroneDescription description{};
    description.transform.position = position;
    description.startArmed = true;
    description.motors.variationEnabled = variation;
    const DroneId id = world.createDrone(description);
    require(id != kInvalidDroneId, "drone creation failed");
    return id;
}

void step(World& world, std::uint32_t ticks) {
    for (std::uint32_t tick = 0U; tick < ticks; ++tick) {
        world.stepFixed();
    }
}

void testMixerReconstructsRequestedWrench() {
    constexpr Real collective = 12.0F;
    constexpr Real pitch = 0.18F;
    constexpr Real yaw = -0.045F;
    constexpr Real roll = -0.12F;
    constexpr Real armRadius = 0.23F;
    constexpr Real reactionRatio = 0.015F;
    const QuadMotorThrusts mixed = QuadXMotorMixer::mix(
        collective,
        pitch,
        yaw,
        roll,
        armRadius,
        reactionRatio,
        20.0F
    );
    const Real coordinate = armRadius / std::sqrt(2.0F);
    const auto& thrust = mixed.newtons;
    const Real reconstructedCollective =
        thrust[0] + thrust[1] + thrust[2] + thrust[3];
    const Real reconstructedPitch =
        coordinate * (-thrust[0] - thrust[1] + thrust[2] + thrust[3]);
    const Real reconstructedRoll =
        coordinate * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]);
    const Real reconstructedYaw =
        reactionRatio * (thrust[0] - thrust[1] + thrust[2] - thrust[3]);
    require(
        near(reconstructedCollective, collective, 1.0e-4F),
        "mixer collective mismatch"
    );
    require(near(reconstructedPitch, pitch, 1.0e-4F), "mixer pitch mismatch");
    require(near(reconstructedYaw, yaw, 1.0e-4F), "mixer yaw mismatch");
    require(near(reconstructedRoll, roll, 1.0e-4F), "mixer roll mismatch");
}

void testStabilizedDroneHoldsHover() {
    World world = makeWorld();
    const DroneId id = createFlyingDrone(world);
    step(world, 1'200U);

    const Drone* drone = world.drone(id);
    require(drone != nullptr, "hover drone disappeared");
    const RigidBody* body = world.body(drone->bodyId());
    require(body != nullptr, "hover drone body disappeared");
    const Vec3 up = body->transform().orientation.rotate({0.0F, 1.0F, 0.0F});
    std::cout << "hover profile: altitude "
              << body->transform().position.y
              << " m, vertical speed " << body->linearVelocity().y
              << " m/s, up.y " << up.y << '\n';
    require(
        std::abs(body->transform().position.y - 3.0F) < 0.22F,
        "stabilized drone did not hold altitude"
    );
    require(
        std::abs(body->linearVelocity().y) < 0.18F,
        "stabilized drone retained excessive vertical speed"
    );
    require(up.y > 0.985F, "stabilized drone did not remain level");
}

void testPilotVelocityAltitudeAndYawCommands() {
    World world = makeWorld();
    const DroneId id = createFlyingDrone(world);
    step(world, 480U);

    Drone* drone = world.drone(id);
    require(drone != nullptr, "command drone disappeared");
    DronePilotCommand command{};
    command.forward = 0.70F;
    command.right = 0.30F;
    command.climb = 0.50F;
    drone->setPilotCommand(command);
    step(world, 240U);
    drone->setPilotCommand({});
    step(world, 240U);

    const RigidBody* body = world.body(drone->bodyId());
    require(body != nullptr, "command drone body disappeared");
    const Vec3 translatedPosition = body->transform().position;

    DronePilotCommand yawCommand{};
    yawCommand.yaw = 0.45F;
    drone->setPilotCommand(yawCommand);
    step(world, 180U);
    drone->setPilotCommand({});
    step(world, 180U);

    body = world.body(drone->bodyId());
    const Vec3 forward =
        body->transform().orientation.rotate({0.0F, 0.0F, 1.0F});
    const Real yaw = std::atan2(forward.x, forward.z);
    std::cout << "command profile: position "
              << body->transform().position.x << ", "
              << body->transform().position.y << ", "
              << body->transform().position.z
              << " yaw " << yaw << " rad\n";
    require(
        translatedPosition.z > 2.0F,
        "forward command did not produce forward flight"
    );
    require(
        translatedPosition.x > 0.5F,
        "right command did not produce lateral flight"
    );
    require(
        body->transform().position.y > 3.6F,
        "climb command did not increase held altitude"
    );
    require(std::abs(yaw) > 0.45F, "yaw command did not rotate the drone");
}

void testWindCreatesARealControlResponse() {
    World calm = makeWorld();
    World windy = makeWorld({3.5F, 0.0F, 0.0F});
    const DroneId calmId = createFlyingDrone(calm);
    const DroneId windyId = createFlyingDrone(windy);
    step(calm, 840U);
    step(windy, 840U);

    const RigidBody* calmBody =
        calm.body(calm.drone(calmId)->bodyId());
    const RigidBody* windyBody =
        windy.body(windy.drone(windyId)->bodyId());
    require(calmBody != nullptr && windyBody != nullptr, "wind drone missing");
    const Vec3 windyUp =
        windyBody->transform().orientation.rotate({0.0F, 1.0F, 0.0F});
    std::cout << "wind profile: calm x "
              << calmBody->transform().position.x
              << " m, windy x " << windyBody->transform().position.x
              << " m, up.x " << windyUp.x << '\n';
    require(
        std::abs(windyUp.x) > 0.01F ||
            std::abs(
                windyBody->transform().position.x -
                calmBody->transform().position.x
            ) > 0.05F,
        "wind produced no measurable physical or controller response"
    );
    require(
        std::abs(windyBody->transform().position.x) < 3.0F,
        "stabilized drone failed to resist steady wind"
    );
}

void testMotorVariationAndSensorFramesAreDeterministic() {
    World first = makeWorld();
    World second = makeWorld();
    const DroneId firstId = createFlyingDrone(first, {0.0F, 3.0F, 0.0F}, true);
    const DroneId secondId = createFlyingDrone(second, {0.0F, 3.0F, 0.0F}, true);
    step(first, 600U);
    step(second, 600U);

    const Drone* firstDrone = first.drone(firstId);
    const Drone* secondDrone = second.drone(secondId);
    require(firstDrone != nullptr && secondDrone != nullptr, "variation drone missing");
    bool anyVariation = false;
    for (std::size_t index = 0U; index < kQuadMotorCount; ++index) {
        const Real firstScale = firstDrone->motors()[index].thrustScale;
        const Real secondScale = secondDrone->motors()[index].thrustScale;
        anyVariation = anyVariation || std::abs(firstScale - 1.0F) > 1.0e-4F;
        require(
            near(firstScale, secondScale, 1.0e-7F),
            "seeded motor variation is not deterministic"
        );
        require(
            firstScale >= 0.985F && firstScale <= 1.015F,
            "motor variation exceeded configured bounds"
        );
    }
    require(anyVariation, "motor variation option did not vary any motor");

    const DroneSensorFrame& sensors = firstDrone->sensorFrame();
    require(sensors.sequence == 600U, "sensor frame did not update every fixed tick");
    require(
        (sensors.availableChannels & DroneSensorChannel::gyroscope) != 0U &&
        (sensors.availableChannels & DroneSensorChannel::accelerometer) != 0U &&
        (sensors.availableChannels & DroneSensorChannel::barometer) != 0U,
        "implemented sensor channels were not advertised"
    );
    require(
        (sensors.availableChannels & DroneSensorChannel::lidar) == 0U,
        "LiDAR was advertised before implementation"
    );
    require(
        isFinite(sensors.gyroscopeRadiansPerSecondBody) &&
        isFinite(sensors.accelerometerMetersPerSecondSquaredBody) &&
        std::isfinite(sensors.pressurePascals),
        "sensor frame contains non-finite values"
    );
}

void testExternalActuatorContractAndPidBypass() {
    World world = makeWorld();
    DroneDescription description{};
    description.transform.position = {0.0F, 2.5F, 0.0F};
    description.startArmed = true;
    const DroneId id = world.createDrone(description);
    Drone* drone = world.drone(id);
    require(drone != nullptr, "external actuator drone creation failed");
    drone->setControlMode(DroneControlMode::ExternalActuators);

    const Real hoverMotorCommand = std::sqrt(
        (description.mass * 9.80665F * 0.25F) /
        (
            description.motors.thrustCoefficient *
            description.motors.maximumAngularVelocity *
            description.motors.maximumAngularVelocity
        )
    );
    DroneActuatorFrame frame{};
    frame.sequence = 42U;
    frame.sourceTimeSeconds = 1.25;
    frame.normalizedMotorCommand.fill(hoverMotorCommand);
    frame.armed = true;
    require(
        drone->submitActuatorFrame(frame),
        "valid external actuator frame was rejected"
    );
    step(world, 240U);
    for (const DroneMotorState& motor : drone->motors()) {
        require(
            std::abs(motor.command - hoverMotorCommand) < 0.01F,
            "external actuator command did not reach a motor"
        );
    }

    drone->setControlMode(DroneControlMode::DirectMixer);
    DronePilotCommand direct{};
    direct.forward = 0.4F;
    drone->setPilotCommand(direct);
    step(world, 60U);
    const auto& motors = drone->motors();
    require(
        motors[2].command > motors[0].command &&
        motors[3].command > motors[1].command,
        "PID bypass did not feed the physical X mixer"
    );
}

} // namespace

int main() {
    testMixerReconstructsRequestedWrench();
    testStabilizedDroneHoldsHover();
    testPilotVelocityAltitudeAndYawCommands();
    testWindCreatesARealControlResponse();
    testMotorVariationAndSensorFramesAreDeterministic();
    testExternalActuatorContractAndPidBypass();
    std::cout << "All UAView Studio drone-dynamics tests passed.\n";
    return EXIT_SUCCESS;
}
