#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <uaview/physics/RigidBody.hpp>

namespace uaview::physics {

class World;

using DroneId = std::uint32_t;
constexpr DroneId kInvalidDroneId = 0U;
constexpr std::size_t kQuadMotorCount = 4U;

enum class DroneControlMode : std::uint8_t {
    Stabilized,
    DirectMixer,
    ExternalActuators,
};

namespace DroneSensorChannel {

inline constexpr std::uint32_t gyroscope = 1U << 0U;
inline constexpr std::uint32_t accelerometer = 1U << 1U;
inline constexpr std::uint32_t barometer = 1U << 2U;
// Reserved in the transport-neutral API. No LiDAR model is implemented yet.
inline constexpr std::uint32_t lidar = 1U << 3U;

} // namespace DroneSensorChannel

struct PidGains {
    Real proportional{0.0F};
    Real integral{0.0F};
    Real derivative{0.0F};
    Real integralLimit{0.0F};
    Real outputLimit{0.0F};
};

struct DroneControllerSettings {
    PidGains horizontalVelocity{
        1.8F, 0.16F, 0.32F, 2.5F, 6.5F
    };
    PidGains altitude{
        4.2F, 1.25F, 2.4F, 2.0F, 8.5F
    };
    PidGains attitude{
        7.5F, 0.0F, 0.20F, 0.0F, 5.0F
    };
    PidGains bodyRate{
        0.18F, 0.035F, 0.0035F, 0.8F, 0.72F
    };
    Real maximumHorizontalSpeed{5.0F};
    Real maximumClimbSpeed{3.0F};
    Real maximumYawRate{2.4F};
    Real maximumTiltRadians{0.5235988F}; // 30 degrees
};

struct DroneMotorSettings {
    Real maximumAngularVelocity{1'260.0F}; // rad/s, about 12,000 RPM
    Real thrustCoefficient{5.8e-6F};       // N / (rad/s)^2
    Real reactionTorqueCoefficient{8.7e-8F}; // N m / (rad/s)^2
    Real rotorInertia{1.6e-5F};            // kg m^2
    Real spoolTimeConstant{0.040F};         // s
    Real maximumMotorTorque{0.42F};         // N m
    Real propellerRadius{0.127F};           // m, 10-inch diameter
    bool variationEnabled{false};
    Real variationFraction{0.015F};
    std::uint32_t variationSeed{0x55415644U};
};

struct DroneDescription {
    Transform transform{{0.0F, 2.0F, 0.0F}, Quaternion::identity()};
    Real mass{1.20F};
    Vec3 bodyHalfExtents{0.24F, 0.055F, 0.24F};
    Real armRadius{0.23F};
    PhysicsMaterial material{0.72F, 0.58F, 0.04F, 0.012F};
    AerodynamicProperties aerodynamics{
        true, 1.10F, 0.12F, 1.0F, {}
    };
    DroneMotorSettings motors{};
    DroneControllerSettings controller{};
    bool startArmed{false};
    const char* debugName{"Quadcopter"};
};

struct DronePilotCommand {
    Real forward{0.0F}; // W/S, normalized
    Real right{0.0F};   // A/D, normalized
    Real yaw{0.0F};     // Q/E, normalized
    Real climb{0.0F};   // Up/Down arrows, normalized
};

// Fixed-size transport-neutral actuator frame. A future UDP/UART bridge can
// validate/deserialize its own protocol and submit one of these without making
// the physics core depend on sockets, serial ports, or an operating system.
struct DroneActuatorFrame {
    std::uint64_t sequence{0U};
    double sourceTimeSeconds{0.0};
    std::array<Real, kQuadMotorCount> normalizedMotorCommand{};
    bool armed{false};
};

struct DroneSensorFrame {
    std::uint64_t sequence{0U};
    double simulationTimeSeconds{0.0};
    std::uint32_t availableChannels{
        DroneSensorChannel::gyroscope |
        DroneSensorChannel::accelerometer |
        DroneSensorChannel::barometer
    };
    Vec3 gyroscopeRadiansPerSecondBody{};
    Vec3 accelerometerMetersPerSecondSquaredBody{};
    Real pressurePascals{101'325.0F};
    Real temperatureKelvin{288.15F};
    Real barometricAltitudeMeters{0.0F};
};

static_assert(
    std::is_trivially_copyable_v<DroneActuatorFrame> &&
    std::is_standard_layout_v<DroneActuatorFrame>,
    "Actuator frames must remain easy to copy into a transport adapter."
);
static_assert(
    std::is_trivially_copyable_v<DroneSensorFrame> &&
    std::is_standard_layout_v<DroneSensorFrame>,
    "Sensor frames must remain easy to copy out through a transport adapter."
);

struct DroneMotorState {
    Real command{0.0F};
    Real targetAngularVelocity{0.0F};
    Real angularVelocity{0.0F};
    Real thrustNewtons{0.0F};
    Real reactionTorqueNewtonMeters{0.0F};
    Real thrustScale{1.0F};
};

struct QuadMotorThrusts {
    std::array<Real, kQuadMotorCount> newtons{};
};

class QuadXMotorMixer {
public:
    [[nodiscard]] static QuadMotorThrusts mix(
        Real collectiveThrustNewtons,
        Real pitchTorqueNewtonMeters,
        Real yawTorqueNewtonMeters,
        Real rollTorqueNewtonMeters,
        Real armRadiusMeters,
        Real reactionTorquePerThrustMeter,
        Real maximumMotorThrustNewtons
    ) noexcept;
};

class Drone {
public:
    Drone(const Drone&) = delete;
    Drone& operator=(const Drone&) = delete;
    Drone(Drone&&) noexcept = default;
    Drone& operator=(Drone&&) noexcept = default;

    [[nodiscard]] DroneId id() const noexcept;
    [[nodiscard]] BodyId bodyId() const noexcept;
    [[nodiscard]] bool isAlive() const noexcept;
    [[nodiscard]] bool isArmed() const noexcept;
    [[nodiscard]] DroneControlMode controlMode() const noexcept;
    [[nodiscard]] const DroneDescription& description() const noexcept;
    [[nodiscard]] const DronePilotCommand& pilotCommand() const noexcept;
    [[nodiscard]] const DroneControllerSettings& controllerSettings() const noexcept;
    [[nodiscard]] const std::array<DroneMotorState, kQuadMotorCount>&
        motors() const noexcept;
    [[nodiscard]] const DroneSensorFrame& sensorFrame() const noexcept;
    [[nodiscard]] Vec3 motorPositionLocal(std::size_t index) const noexcept;
    [[nodiscard]] Real targetAltitudeMeters() const noexcept;

    void setArmed(bool armed) noexcept;
    void setControlMode(DroneControlMode mode) noexcept;
    void setPilotCommand(const DronePilotCommand& command) noexcept;
    void setControllerSettings(
        const DroneControllerSettings& settings
    ) noexcept;
    void setMotorVariation(
        bool enabled,
        Real fraction,
        std::uint32_t seed
    ) noexcept;
    bool submitActuatorFrame(const DroneActuatorFrame& frame) noexcept;
    void resetController() noexcept;

private:
    friend class World;

    struct PidState {
        Real integral{0.0F};
        Real previousError{0.0F};
        bool initialized{false};
    };

    Drone(
        DroneId id,
        BodyId bodyId,
        const DroneDescription& description
    ) noexcept;

    bool initializeRotors(RigidBody& body) noexcept;
    void advance(World& world, Real deltaSeconds) noexcept;
    void sampleSensors(
        const World& world,
        Real deltaSeconds,
        double simulationTimeSeconds
    ) noexcept;
    void resetControllerAtBody(const RigidBody& body) noexcept;
    [[nodiscard]] static Real updatePid(
        const PidGains& gains,
        PidState& state,
        Real error,
        Real deltaSeconds
    ) noexcept;

    DroneId id_{kInvalidDroneId};
    BodyId bodyId_{kInvalidBodyId};
    bool alive_{true};
    bool armed_{false};
    DroneControlMode controlMode_{DroneControlMode::Stabilized};
    DroneDescription description_{};
    DronePilotCommand pilotCommand_{};
    DroneActuatorFrame actuatorFrame_{};
    std::array<RotorId, kQuadMotorCount> rotorIds_{};
    std::array<DroneMotorState, kQuadMotorCount> motors_{};
    std::array<PidState, 2U> horizontalVelocityPid_{};
    PidState altitudePid_{};
    std::array<PidState, 3U> attitudePid_{};
    std::array<PidState, 3U> bodyRatePid_{};
    DroneSensorFrame sensorFrame_{};
    Vec3 previousLinearVelocity_{};
    Real targetAltitudeMeters_{0.0F};
    Real targetYawRadians_{0.0F};
    bool controllerInitialized_{false};
    bool sensorInitialized_{false};
};

} // namespace uaview::physics
