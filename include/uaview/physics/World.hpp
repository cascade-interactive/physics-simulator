#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <uaview/physics/Drone.hpp>
#include <uaview/physics/Environment.hpp>
#include <uaview/physics/RigidBody.hpp>

namespace uaview::physics {

struct PhysicsSettings {
    std::uint32_t fixedUpdateHz{120};
    std::uint32_t solverSubsteps{2};
    std::uint32_t maximumAdaptiveSubsteps{32};
    Real maximumTravelFraction{0.35F};
    // Torque-free orientation drift receives its own inexpensive rotational
    // microsteps. This keeps very high-RPM rotors accurate without multiplying
    // broadphase/contact work for the entire world.
    Real maximumRotationStepRadians{0.2617994F}; // 15 degrees
    std::uint32_t maximumRotationMicrosteps{64};
    std::uint32_t velocityIterations{12};
    Real contactSlop{0.0015F};
    Real restitutionVelocityThreshold{0.75F};
    Real penetrationVelocityFactor{0.16F};
    Real maximumDepenetrationVelocity{3.0F};
    Real positionCorrectionFraction{0.82F};
    Real sleepLinearSpeed{0.035F};
    Real sleepAngularSpeed{0.05F};
    Real sleepDelaySeconds{0.65F};
    // A body may sleep only when the gravity-projected center of mass lies
    // this far inside a non-degenerate support polygon. Corner and edge
    // balances therefore remain active until they physically topple.
    Real sleepSupportMargin{0.002F};
    std::uint32_t maximumCatchUpSteps{8};
};

struct SpringForce {
    BodyId body{kInvalidBodyId};
    Vec3 localAnchor{};
    Vec3 worldTarget{};
    Real stiffness{18'000.0F}; // N / m
    Real damping{3'000.0F};    // N s / m
    Real maximumForce{80'000.0F};
    bool enabled{true};
};

// A deterministic force generator whose lifetime is measured in simulation
// seconds. Body-frame vectors are rotated into world space at every force
// evaluation, so they follow the body throughout a fixed tick. A final partial
// tick is force-weighted to preserve the requested total impulse.
struct TimedBodyForce {
    BodyId body{kInvalidBodyId};
    Vec3 force{};                    // N
    Vec3 torque{};                   // N m
    Real remainingSeconds{0.0F};     // enabled simulation seconds
    bool forceInBodyFrame{false};
    bool torqueInBodyFrame{false};
    // Disabled generators retain, rather than consume, remainingSeconds.
    bool enabled{true};
};

struct ContactDebugPoint {
    BodyId bodyA{kInvalidBodyId};
    BodyId bodyB{kInvalidBodyId};
    Vec3 point{};
    Vec3 normal{};
    Real penetration{0.0F};
    Real normalImpulse{0.0F};
};

struct PhysicsDebugStats {
    std::uint64_t fixedTick{0};
    double simulationTime{0.0};
    std::size_t bodyCount{0};
    std::size_t awakeBodyCount{0};
    std::size_t broadphasePairs{0};
    std::size_t narrowphaseTests{0};
    std::size_t contactCount{0};
    std::size_t rotationalCcdHits{0};
    std::size_t rotationalCcdAdvances{0};
    std::size_t rotationalCcdConvergenceHits{0};
    std::size_t rotationalCcdAdvanceCapHits{0};
    std::uint32_t internalSubsteps{0};
    std::size_t rotationMicrosteps{0};
    std::uint32_t maximumRotationMicrostepsUsed{0};
    std::size_t rotationMicrostepCapHits{0};
    std::size_t rotationMidpointNonConvergenceCount{0};
    Real maximumRotationMidpointResidual{0.0F};
    std::uint32_t velocityIterations{0};
    double droppedRealTime{0.0};
};

struct RaycastHit {
    BodyId body{kInvalidBodyId};
    Vec3 point{};
    Vec3 normal{};
    Real distance{0.0F};
};

class World {
public:
    explicit World(const PhysicsSettings& settings = {});
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept;
    World& operator=(World&&) noexcept;

    [[nodiscard]] const PhysicsSettings& settings() const noexcept;
    void setSettings(const PhysicsSettings& settings) noexcept;

    [[nodiscard]] Environment& environment() noexcept;
    [[nodiscard]] const Environment& environment() const noexcept;

    BodyId createBody(const BodyDescription& description);
    bool destroyBody(BodyId id) noexcept;
    [[nodiscard]] RigidBody* body(BodyId id) noexcept;
    [[nodiscard]] const RigidBody* body(BodyId id) const noexcept;
    [[nodiscard]] std::size_t bodySlotCount() const noexcept;
    void wakeAllDynamic() noexcept;
    void reset() noexcept;

    DroneId createDrone(const DroneDescription& description = {});
    bool destroyDrone(DroneId id) noexcept;
    [[nodiscard]] Drone* drone(DroneId id) noexcept;
    [[nodiscard]] const Drone* drone(DroneId id) const noexcept;
    [[nodiscard]] Drone* droneForBody(BodyId bodyId) noexcept;
    [[nodiscard]] const Drone* droneForBody(BodyId bodyId) const noexcept;
    [[nodiscard]] std::size_t droneSlotCount() const noexcept;

    [[nodiscard]] bool raycast(
        const Vec3& origin,
        const Vec3& direction,
        Real maximumDistance,
        RaycastHit& hit
    ) const noexcept;

    SpringId createSpring(const SpringForce& spring);
    bool updateSpring(SpringId id, const SpringForce& spring) noexcept;
    bool setSpringTarget(SpringId id, const Vec3& worldTarget) noexcept;
    bool destroySpring(SpringId id) noexcept;

    TimedForceId createTimedForce(const TimedBodyForce& force);
    bool updateTimedForce(
        TimedForceId id,
        const TimedBodyForce& force
    ) noexcept;
    bool destroyTimedForce(TimedForceId id) noexcept;

    // Advances exactly one configured fixed tick. Forces submitted directly to
    // a body remain active through every internal substep and are cleared only
    // after this call. A Ctrl-drag adapter therefore applies its force/spring
    // once per fixed tick, not once per substep. This is the deterministic entry
    // point for simulation, replay, tests, and future embedded/HIL use.
    void stepFixed();

    // Desktop convenience scheduler. It never changes the fixed dt and caps
    // catch-up work, reporting discarded wall time in debugStats().
    std::uint32_t advance(double realDeltaSeconds);

    // Fraction of the next fixed interval already accumulated by advance().
    // Always finite and clamped to [0, 1], including before the first update.
    [[nodiscard]] Real interpolationAlpha() const noexcept;
    [[nodiscard]] const PhysicsDebugStats& debugStats() const noexcept;
    [[nodiscard]] const std::vector<ContactDebugPoint>& debugContacts() const noexcept;

private:
    struct SpringSlot {
        SpringForce force{};
        bool alive{false};
    };

    struct TimedForceSlot {
        TimedBodyForce force{};
        double remainingSeconds{0.0};
        Real tickScale{0.0F};
        bool alive{false};
    };

    struct Contact {
        BodyId bodyA{kInvalidBodyId};
        BodyId bodyB{kInvalidBodyId};
        Vec3 point{};
        Vec3 normal{};
        Real penetration{0.0F};
        Real accumulatedNormalImpulse{0.0F};
        Vec3 accumulatedTangentImpulse{};
        Vec3 accumulatedRollingImpulse{};
        Real restitutionVelocity{0.0F};
        std::uint8_t manifoldPointCount{1};
    };

    void simulateSubstep(Real deltaSeconds);
    void accumulateForces(Real deltaSeconds);
    void integrateKick(Real halfDeltaSeconds);
    void integrateDrift(Real deltaSeconds);
    void detectContacts();
    void solveContacts(Real deltaSeconds);
    void updateSleeping(Real deltaSeconds);
    void clearExternalForces() noexcept;
    void prepareTimedForces(Real fixedDeltaSeconds) noexcept;
    void expireTimedForces(Real fixedDeltaSeconds) noexcept;
    void advanceDrones(Real fixedDeltaSeconds) noexcept;
    void sampleDroneSensors(Real fixedDeltaSeconds) noexcept;

    PhysicsSettings settings_{};
    Environment environment_{};
    std::vector<RigidBody> bodies_{};
    std::vector<Drone> drones_{};
    std::vector<SpringSlot> springs_{};
    std::vector<TimedForceSlot> timedForces_{};
    std::vector<Contact> contacts_{};
    std::vector<ContactDebugPoint> debugContacts_{};
    PhysicsDebugStats debugStats_{};
    double accumulatorSeconds_{0.0};
};

} // namespace uaview::physics
