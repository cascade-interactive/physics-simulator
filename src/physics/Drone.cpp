#include <uaview/physics/Drone.hpp>

#include <uaview/physics/World.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace uaview::physics {
namespace {

constexpr Real kSeaLevelDensity = 1.225F;
constexpr Real kStandardGravity = 9.80665F;
constexpr Real kInverseSqrtTwo = 0.70710678118F;
constexpr std::array<Real, kQuadMotorCount> kRotorSpinDirection{
    -1.0F, 1.0F, -1.0F, 1.0F
};

[[nodiscard]] Real finiteOr(Real value, Real fallback) noexcept {
    return std::isfinite(value) ? value : fallback;
}

[[nodiscard]] Real clampUnit(Real value) noexcept {
    return std::clamp(finiteOr(value, 0.0F), -1.0F, 1.0F);
}

[[nodiscard]] Real wrapRadians(Real radiansValue) noexcept {
    if (!std::isfinite(radiansValue)) {
        return 0.0F;
    }
    Real wrapped = std::fmod(radiansValue + kPi, 2.0F * kPi);
    if (wrapped < 0.0F) {
        wrapped += 2.0F * kPi;
    }
    return wrapped - kPi;
}

[[nodiscard]] Real yawFromOrientation(
    const Quaternion& orientation
) noexcept {
    const Vec3 forward = orientation.rotate({0.0F, 0.0F, 1.0F});
    return std::atan2(forward.x, forward.z);
}

[[nodiscard]] PidGains sanitizedGains(
    const PidGains& input,
    const PidGains& fallback
) noexcept {
    PidGains result{};
    result.proportional = std::max(
        0.0F,
        finiteOr(input.proportional, fallback.proportional)
    );
    result.integral = std::max(
        0.0F,
        finiteOr(input.integral, fallback.integral)
    );
    result.derivative = std::max(
        0.0F,
        finiteOr(input.derivative, fallback.derivative)
    );
    result.integralLimit = std::max(
        0.0F,
        finiteOr(input.integralLimit, fallback.integralLimit)
    );
    result.outputLimit = std::max(
        0.0F,
        finiteOr(input.outputLimit, fallback.outputLimit)
    );
    return result;
}

[[nodiscard]] std::uint32_t nextRandom(
    std::uint32_t& state
) noexcept {
    if (state == 0U) {
        state = 0x9E3779B9U;
    }
    state ^= state << 13U;
    state ^= state >> 17U;
    state ^= state << 5U;
    return state;
}

[[nodiscard]] Real randomSignedUnit(std::uint32_t& state) noexcept {
    constexpr Real inverseMaximum =
        1.0F / static_cast<Real>(std::numeric_limits<std::uint32_t>::max());
    return static_cast<Real>(nextRandom(state)) * inverseMaximum * 2.0F - 1.0F;
}

} // namespace

QuadMotorThrusts QuadXMotorMixer::mix(
    Real collectiveThrustNewtons,
    Real pitchTorqueNewtonMeters,
    Real yawTorqueNewtonMeters,
    Real rollTorqueNewtonMeters,
    Real armRadiusMeters,
    Real reactionTorquePerThrustMeter,
    Real maximumMotorThrustNewtons
) noexcept {
    const Real collective = std::max(
        0.0F,
        finiteOr(collectiveThrustNewtons, 0.0F)
    );
    const Real coordinate = std::max(
        1.0e-4F,
        finiteOr(armRadiusMeters, 0.23F) * kInverseSqrtTwo
    );
    const Real reactionRatio = std::max(
        1.0e-5F,
        finiteOr(reactionTorquePerThrustMeter, 0.015F)
    );
    const Real maximum = std::max(
        0.0F,
        finiteOr(maximumMotorThrustNewtons, 0.0F)
    );
    const Real base = collective * 0.25F;
    const Real pitch = finiteOr(pitchTorqueNewtonMeters, 0.0F) /
        (4.0F * coordinate);
    const Real roll = finiteOr(rollTorqueNewtonMeters, 0.0F) /
        (4.0F * coordinate);
    const Real yaw = finiteOr(yawTorqueNewtonMeters, 0.0F) /
        (4.0F * reactionRatio);

    QuadMotorThrusts result{{
        base - pitch - roll + yaw,
        base - pitch + roll - yaw,
        base + pitch + roll + yaw,
        base + pitch - roll - yaw
    }};

    Real minimum = result.newtons[0];
    Real maximumRequested = result.newtons[0];
    for (Real thrust : result.newtons) {
        minimum = std::min(minimum, thrust);
        maximumRequested = std::max(maximumRequested, thrust);
    }
    if (minimum < 0.0F) {
        for (Real& thrust : result.newtons) {
            thrust -= minimum;
        }
        maximumRequested -= minimum;
    }
    if (maximum > 0.0F && maximumRequested > maximum) {
        const Real reduction = maximumRequested - maximum;
        for (Real& thrust : result.newtons) {
            thrust -= reduction;
        }
    }
    for (Real& thrust : result.newtons) {
        thrust = std::clamp(thrust, 0.0F, maximum);
    }
    return result;
}

Drone::Drone(
    DroneId id,
    BodyId bodyId,
    const DroneDescription& description
) noexcept
    : id_{id},
      bodyId_{bodyId},
      armed_{description.startArmed},
      description_{description} {
    const DroneDescription defaults{};
    description_.mass = std::clamp(
        finiteOr(description_.mass, defaults.mass),
        0.05F,
        100.0F
    );
    description_.bodyHalfExtents = {
        std::clamp(
            finiteOr(description_.bodyHalfExtents.x, defaults.bodyHalfExtents.x),
            0.01F,
            5.0F
        ),
        std::clamp(
            finiteOr(description_.bodyHalfExtents.y, defaults.bodyHalfExtents.y),
            0.01F,
            5.0F
        ),
        std::clamp(
            finiteOr(description_.bodyHalfExtents.z, defaults.bodyHalfExtents.z),
            0.01F,
            5.0F
        )
    };
    description_.armRadius = std::clamp(
        finiteOr(description_.armRadius, defaults.armRadius),
        0.02F,
        5.0F
    );

    DroneMotorSettings& motors = description_.motors;
    const DroneMotorSettings motorDefaults{};
    motors.maximumAngularVelocity = std::clamp(
        finiteOr(
            motors.maximumAngularVelocity,
            motorDefaults.maximumAngularVelocity
        ),
        10.0F,
        10'000.0F
    );
    motors.thrustCoefficient = std::clamp(
        finiteOr(motors.thrustCoefficient, motorDefaults.thrustCoefficient),
        1.0e-10F,
        1.0F
    );
    motors.reactionTorqueCoefficient = std::clamp(
        finiteOr(
            motors.reactionTorqueCoefficient,
            motorDefaults.reactionTorqueCoefficient
        ),
        1.0e-12F,
        1.0F
    );
    motors.rotorInertia = std::clamp(
        finiteOr(motors.rotorInertia, motorDefaults.rotorInertia),
        1.0e-9F,
        1.0F
    );
    motors.spoolTimeConstant = std::clamp(
        finiteOr(
            motors.spoolTimeConstant,
            motorDefaults.spoolTimeConstant
        ),
        0.002F,
        2.0F
    );
    motors.maximumMotorTorque = std::clamp(
        finiteOr(
            motors.maximumMotorTorque,
            motorDefaults.maximumMotorTorque
        ),
        0.001F,
        100.0F
    );
    motors.propellerRadius = std::clamp(
        finiteOr(motors.propellerRadius, motorDefaults.propellerRadius),
        0.01F,
        2.0F
    );
    motors.variationFraction = std::clamp(
        finiteOr(
            motors.variationFraction,
            motorDefaults.variationFraction
        ),
        0.0F,
        0.20F
    );

    setControllerSettings(description_.controller);

    std::uint32_t randomState = motors.variationSeed;
    for (DroneMotorState& motor : motors_) {
        motor.thrustScale = motors.variationEnabled
            ? 1.0F + randomSignedUnit(randomState) *
                motors.variationFraction
            : 1.0F;
    }
}

DroneId Drone::id() const noexcept {
    return id_;
}

BodyId Drone::bodyId() const noexcept {
    return bodyId_;
}

bool Drone::isAlive() const noexcept {
    return alive_;
}

bool Drone::isArmed() const noexcept {
    return armed_;
}

DroneControlMode Drone::controlMode() const noexcept {
    return controlMode_;
}

const DroneDescription& Drone::description() const noexcept {
    return description_;
}

const DronePilotCommand& Drone::pilotCommand() const noexcept {
    return pilotCommand_;
}

const DroneControllerSettings& Drone::controllerSettings() const noexcept {
    return description_.controller;
}

const std::array<DroneMotorState, kQuadMotorCount>&
Drone::motors() const noexcept {
    return motors_;
}

const DroneSensorFrame& Drone::sensorFrame() const noexcept {
    return sensorFrame_;
}

Vec3 Drone::motorPositionLocal(std::size_t index) const noexcept {
    const Real armCoordinate = description_.armRadius * kInverseSqrtTwo;
    switch (index) {
    case 0U:
        return {-armCoordinate, 0.0F, armCoordinate};
    case 1U:
        return {armCoordinate, 0.0F, armCoordinate};
    case 2U:
        return {armCoordinate, 0.0F, -armCoordinate};
    case 3U:
        return {-armCoordinate, 0.0F, -armCoordinate};
    default:
        return {};
    }
}

Real Drone::targetAltitudeMeters() const noexcept {
    return targetAltitudeMeters_;
}

void Drone::setArmed(bool armed) noexcept {
    if (armed_ == armed) {
        return;
    }
    armed_ = armed;
    resetController();
}

void Drone::setControlMode(DroneControlMode mode) noexcept {
    if (controlMode_ == mode) {
        return;
    }
    controlMode_ = mode;
    resetController();
}

void Drone::setPilotCommand(const DronePilotCommand& command) noexcept {
    pilotCommand_.forward = clampUnit(command.forward);
    pilotCommand_.right = clampUnit(command.right);
    pilotCommand_.yaw = clampUnit(command.yaw);
    pilotCommand_.climb = clampUnit(command.climb);
}

void Drone::setControllerSettings(
    const DroneControllerSettings& settings
) noexcept {
    const DroneControllerSettings defaults{};
    DroneControllerSettings sanitized{};
    sanitized.horizontalVelocity = sanitizedGains(
        settings.horizontalVelocity,
        defaults.horizontalVelocity
    );
    sanitized.altitude = sanitizedGains(
        settings.altitude,
        defaults.altitude
    );
    sanitized.attitude = sanitizedGains(
        settings.attitude,
        defaults.attitude
    );
    sanitized.bodyRate = sanitizedGains(
        settings.bodyRate,
        defaults.bodyRate
    );
    sanitized.maximumHorizontalSpeed = std::clamp(
        finiteOr(
            settings.maximumHorizontalSpeed,
            defaults.maximumHorizontalSpeed
        ),
        0.0F,
        100.0F
    );
    sanitized.maximumClimbSpeed = std::clamp(
        finiteOr(
            settings.maximumClimbSpeed,
            defaults.maximumClimbSpeed
        ),
        0.0F,
        50.0F
    );
    sanitized.maximumYawRate = std::clamp(
        finiteOr(settings.maximumYawRate, defaults.maximumYawRate),
        0.0F,
        20.0F
    );
    sanitized.maximumTiltRadians = std::clamp(
        finiteOr(
            settings.maximumTiltRadians,
            defaults.maximumTiltRadians
        ),
        0.0F,
        radians(80.0F)
    );
    description_.controller = sanitized;
    resetController();
}

void Drone::setMotorVariation(
    bool enabled,
    Real fraction,
    std::uint32_t seed
) noexcept {
    description_.motors.variationEnabled = enabled;
    description_.motors.variationFraction = std::clamp(
        finiteOr(fraction, 0.0F),
        0.0F,
        0.20F
    );
    description_.motors.variationSeed = seed;
    std::uint32_t randomState = seed;
    for (DroneMotorState& motor : motors_) {
        motor.thrustScale = enabled
            ? 1.0F + randomSignedUnit(randomState) *
                description_.motors.variationFraction
            : 1.0F;
    }
}

bool Drone::submitActuatorFrame(
    const DroneActuatorFrame& frame
) noexcept {
    if (!std::isfinite(frame.sourceTimeSeconds)) {
        return false;
    }
    DroneActuatorFrame sanitized = frame;
    for (Real& command : sanitized.normalizedMotorCommand) {
        if (!std::isfinite(command)) {
            return false;
        }
        command = std::clamp(command, 0.0F, 1.0F);
    }
    actuatorFrame_ = sanitized;
    return true;
}

void Drone::resetController() noexcept {
    horizontalVelocityPid_ = {};
    altitudePid_ = {};
    attitudePid_ = {};
    bodyRatePid_ = {};
    controllerInitialized_ = false;
}

bool Drone::initializeRotors(RigidBody& body) noexcept {
    try {
        for (std::size_t index = 0U;
             index < kQuadMotorCount;
             ++index) {
            GyroscopicRotorDescription rotor{};
            rotor.axisLocal = {0.0F, 1.0F, 0.0F};
            rotor.axialInertia = description_.motors.rotorInertia;
            rotor.relativeAngularVelocity = 0.0F;
            rotor.bearingDamping = 0.0F;
            rotorIds_[index] = body.createGyroscopicRotor(rotor);
            if (rotorIds_[index] == kInvalidRotorId) {
                return false;
            }
        }
    } catch (...) {
        return false;
    }
    return true;
}

Real Drone::updatePid(
    const PidGains& gains,
    PidState& state,
    Real error,
    Real deltaSeconds
) noexcept {
    if (!std::isfinite(error) || !std::isfinite(deltaSeconds) ||
        deltaSeconds <= 0.0F) {
        return 0.0F;
    }
    const Real derivative = state.initialized
        ? (error - state.previousError) / deltaSeconds
        : 0.0F;
    state.integral = std::clamp(
        state.integral + error * deltaSeconds,
        -gains.integralLimit,
        gains.integralLimit
    );
    state.previousError = error;
    state.initialized = true;
    return std::clamp(
        gains.proportional * error +
            gains.integral * state.integral +
            gains.derivative * derivative,
        -gains.outputLimit,
        gains.outputLimit
    );
}

void Drone::resetControllerAtBody(const RigidBody& body) noexcept {
    resetController();
    targetAltitudeMeters_ = body.transform().position.y;
    targetYawRadians_ = yawFromOrientation(body.transform().orientation);
    previousLinearVelocity_ = body.linearVelocity();
    controllerInitialized_ = true;
}

void Drone::advance(World& world, Real deltaSeconds) noexcept {
    RigidBody* body = world.body(bodyId_);
    if (!alive_ || body == nullptr ||
        body->motionType() != MotionType::Dynamic ||
        !std::isfinite(deltaSeconds) || deltaSeconds <= 0.0F) {
        return;
    }
    body->wake();
    if (!controllerInitialized_) {
        resetControllerAtBody(*body);
    }

    const DroneMotorSettings& motorSettings = description_.motors;
    const DroneControllerSettings& controller =
        description_.controller;
    const Real maximumMotorThrust =
        motorSettings.thrustCoefficient *
        motorSettings.maximumAngularVelocity *
        motorSettings.maximumAngularVelocity;
    const Real reactionPerThrust =
        motorSettings.reactionTorqueCoefficient /
        motorSettings.thrustCoefficient;

    std::array<Real, kQuadMotorCount> targetAngularVelocity{};
    if (armed_) {
        if (controlMode_ == DroneControlMode::ExternalActuators) {
            if (actuatorFrame_.armed) {
                for (std::size_t index = 0U;
                     index < kQuadMotorCount;
                     ++index) {
                    targetAngularVelocity[index] =
                        actuatorFrame_.normalizedMotorCommand[index] *
                        motorSettings.maximumAngularVelocity;
                }
            }
        } else {
            Real collective = description_.mass * kStandardGravity;
            Real pitchTorque = 0.0F;
            Real yawTorque = 0.0F;
            Real rollTorque = 0.0F;

            if (controlMode_ == DroneControlMode::DirectMixer) {
                collective += pilotCommand_.climb *
                    description_.mass * 7.0F;
                pitchTorque = pilotCommand_.forward * 0.38F;
                yawTorque = pilotCommand_.yaw * 0.20F;
                rollTorque = -pilotCommand_.right * 0.38F;
            } else {
                targetAltitudeMeters_ +=
                    pilotCommand_.climb *
                    controller.maximumClimbSpeed *
                    deltaSeconds;
                targetYawRadians_ = wrapRadians(
                    targetYawRadians_ +
                    pilotCommand_.yaw *
                        controller.maximumYawRate *
                        deltaSeconds
                );

                const Quaternion& orientation =
                    body->transform().orientation;
                const Quaternion yawOrientation =
                    Quaternion::fromAxisAngle(
                        {0.0F, 1.0F, 0.0F},
                        targetYawRadians_
                    );
                const Vec3 headingVelocity =
                    yawOrientation.inverseRotate(
                        body->linearVelocity()
                    );
                const Real desiredRightSpeed =
                    pilotCommand_.right *
                    controller.maximumHorizontalSpeed;
                const Real desiredForwardSpeed =
                    pilotCommand_.forward *
                    controller.maximumHorizontalSpeed;
                const Real rightAcceleration = updatePid(
                    controller.horizontalVelocity,
                    horizontalVelocityPid_[0],
                    desiredRightSpeed - headingVelocity.x,
                    deltaSeconds
                );
                const Real forwardAcceleration = updatePid(
                    controller.horizontalVelocity,
                    horizontalVelocityPid_[1],
                    desiredForwardSpeed - headingVelocity.z,
                    deltaSeconds
                );
                const Real verticalAcceleration = updatePid(
                    controller.altitude,
                    altitudePid_,
                    targetAltitudeMeters_ -
                        body->transform().position.y,
                    deltaSeconds
                );

                Real desiredPitch = std::atan2(
                    forwardAcceleration,
                    kStandardGravity + verticalAcceleration
                );
                Real desiredRoll = -std::atan2(
                    rightAcceleration,
                    kStandardGravity + verticalAcceleration
                );
                desiredPitch = std::clamp(
                    desiredPitch,
                    -controller.maximumTiltRadians,
                    controller.maximumTiltRadians
                );
                desiredRoll = std::clamp(
                    desiredRoll,
                    -controller.maximumTiltRadians,
                    controller.maximumTiltRadians
                );

                const Quaternion desiredOrientation =
                    yawOrientation *
                    Quaternion::fromAxisAngle(
                        {1.0F, 0.0F, 0.0F},
                        desiredPitch
                    ) *
                    Quaternion::fromAxisAngle(
                        {0.0F, 0.0F, 1.0F},
                        desiredRoll
                    );
                Quaternion error =
                    desiredOrientation * orientation.conjugate();
                if (error.w < 0.0F) {
                    error.w = -error.w;
                    error.x = -error.x;
                    error.y = -error.y;
                    error.z = -error.z;
                }
                const Vec3 attitudeErrorLocal =
                    orientation.inverseRotate(
                        Vec3{error.x, error.y, error.z} * 2.0F
                    );
                const Vec3 bodyRate =
                    orientation.inverseRotate(
                        body->angularVelocity()
                    );
                Vec3 desiredBodyRate{};
                desiredBodyRate.x = updatePid(
                    controller.attitude,
                    attitudePid_[0],
                    attitudeErrorLocal.x,
                    deltaSeconds
                );
                desiredBodyRate.y = updatePid(
                    controller.attitude,
                    attitudePid_[1],
                    attitudeErrorLocal.y,
                    deltaSeconds
                );
                desiredBodyRate.z = updatePid(
                    controller.attitude,
                    attitudePid_[2],
                    attitudeErrorLocal.z,
                    deltaSeconds
                );
                pitchTorque = updatePid(
                    controller.bodyRate,
                    bodyRatePid_[0],
                    desiredBodyRate.x - bodyRate.x,
                    deltaSeconds
                );
                yawTorque = updatePid(
                    controller.bodyRate,
                    bodyRatePid_[1],
                    desiredBodyRate.y - bodyRate.y,
                    deltaSeconds
                );
                rollTorque = updatePid(
                    controller.bodyRate,
                    bodyRatePid_[2],
                    desiredBodyRate.z - bodyRate.z,
                    deltaSeconds
                );

                const Vec3 currentUp =
                    orientation.rotate({0.0F, 1.0F, 0.0F});
                const Real verticalAuthority =
                    std::max(0.30F, currentUp.y);
                collective = description_.mass *
                    std::max(
                        0.0F,
                        kStandardGravity + verticalAcceleration
                    ) / verticalAuthority;
            }

            const QuadMotorThrusts mixed = QuadXMotorMixer::mix(
                collective,
                pitchTorque,
                yawTorque,
                rollTorque,
                description_.armRadius,
                reactionPerThrust,
                maximumMotorThrust
            );
            for (std::size_t index = 0U;
                 index < kQuadMotorCount;
                 ++index) {
                targetAngularVelocity[index] = std::sqrt(
                    mixed.newtons[index] /
                    motorSettings.thrustCoefficient
                );
            }
        }
    } else {
        resetControllerAtBody(*body);
    }

    const Quaternion& orientation = body->transform().orientation;
    const Vec3 bodyUp = orientation.rotate({0.0F, 1.0F, 0.0F});
    const AtmosphereSample atmosphere =
        world.environment().sampleAtmosphere(body->transform().position);
    const Real densityScale = std::max(0.0F, atmosphere.density) /
        kSeaLevelDensity;

    for (std::size_t index = 0U;
         index < kQuadMotorCount;
         ++index) {
        DroneMotorState& motor = motors_[index];
        motor.targetAngularVelocity = std::clamp(
            targetAngularVelocity[index],
            0.0F,
            motorSettings.maximumAngularVelocity
        );
        motor.command =
            motor.targetAngularVelocity /
            motorSettings.maximumAngularVelocity;

        const GyroscopicRotorState* rotor =
            body->gyroscopicRotor(rotorIds_[index]);
        Real signedAngularVelocity = 0.0F;
        if (rotor != nullptr && rotor->axialInertia > kEpsilon) {
            signedAngularVelocity =
                rotor->absoluteAxialAngularMomentum /
                rotor->axialInertia;
        }
        const Real signedTarget =
            kRotorSpinDirection[index] *
            motor.targetAngularVelocity;
        const Real motorTorque = std::clamp(
            (signedTarget - signedAngularVelocity) *
                motorSettings.rotorInertia /
                motorSettings.spoolTimeConstant,
            -motorSettings.maximumMotorTorque,
            motorSettings.maximumMotorTorque
        );
        body->setGyroscopicRotorMotorTorque(
            rotorIds_[index],
            motorTorque
        );

        motor.angularVelocity = std::abs(signedAngularVelocity);
        const Vec3 motorWorld =
            body->worldPointFromLocal(motorPositionLocal(index));
        const Vec3 wind = world.environment().windVelocity(
            motorWorld,
            world.debugStats().simulationTime
        );
        const Vec3 relativeAir =
            body->velocityAtWorldPoint(motorWorld) - wind;
        const Real tipSpeed = std::max(
            1.0F,
            motor.angularVelocity * motorSettings.propellerRadius
        );
        const Real axialInflow = dot(relativeAir, bodyUp);
        const Real inflowScale = std::clamp(
            1.0F - axialInflow / (2.0F * tipSpeed),
            0.72F,
            1.16F
        );
        motor.thrustNewtons =
            motorSettings.thrustCoefficient *
            motor.angularVelocity * motor.angularVelocity *
            densityScale * inflowScale * motor.thrustScale;
        motor.reactionTorqueNewtonMeters =
            -kRotorSpinDirection[index] *
            motorSettings.reactionTorqueCoefficient *
            motor.angularVelocity * motor.angularVelocity *
            densityScale * motor.thrustScale;

        body->applyForceAtWorldPoint(
            bodyUp * motor.thrustNewtons,
            motorWorld
        );
        body->applyTorque(
            orientation.rotate({
                0.0F,
                motor.reactionTorqueNewtonMeters,
                0.0F
            })
        );
    }
}

void Drone::sampleSensors(
    const World& world,
    Real deltaSeconds,
    double simulationTimeSeconds
) noexcept {
    const RigidBody* body = world.body(bodyId_);
    if (!alive_ || body == nullptr ||
        !std::isfinite(deltaSeconds) || deltaSeconds <= 0.0F) {
        return;
    }

    const Vec3 linearAcceleration = sensorInitialized_
        ? (body->linearVelocity() - previousLinearVelocity_) /
            deltaSeconds
        : Vec3{};
    previousLinearVelocity_ = body->linearVelocity();
    sensorInitialized_ = true;

    const Quaternion& orientation = body->transform().orientation;
    sensorFrame_.gyroscopeRadiansPerSecondBody =
        orientation.inverseRotate(body->angularVelocity());
    sensorFrame_.accelerometerMetersPerSecondSquaredBody =
        orientation.inverseRotate(
            linearAcceleration - world.environment().gravity()
        );
    const AtmosphereSample atmosphere =
        world.environment().sampleAtmosphere(
            body->transform().position
        );
    sensorFrame_.pressurePascals = atmosphere.pressurePascals;
    sensorFrame_.temperatureKelvin = atmosphere.temperatureKelvin;
    sensorFrame_.barometricAltitudeMeters =
        body->transform().position.y;
    sensorFrame_.simulationTimeSeconds = simulationTimeSeconds;
    ++sensorFrame_.sequence;
}

} // namespace uaview::physics
