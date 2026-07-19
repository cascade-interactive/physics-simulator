#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <uaview/physics/Math.hpp>

namespace uaview::physics {

using BodyId = std::uint32_t;
using SpringId = std::uint32_t;
using TimedForceId = std::uint32_t;
using RotorId = std::uint32_t;

constexpr BodyId kInvalidBodyId = 0;
constexpr SpringId kInvalidSpringId = 0;
constexpr TimedForceId kInvalidTimedForceId = 0;
constexpr RotorId kInvalidRotorId = 0;

enum class MotionType : std::uint8_t {
    Static,
    Dynamic,
};

enum class ColliderType : std::uint8_t {
    Box,
    Plane,
};

struct BoxCollider {
    Vec3 halfExtents{0.5F, 0.5F, 0.5F};
};

struct PlaneCollider {
    Vec3 normal{0.0F, 1.0F, 0.0F};
    Real offset{0.0F}; // dot(normal, worldPoint) == offset
};

struct Collider {
    ColliderType type{ColliderType::Box};
    BoxCollider box{};
    PlaneCollider plane{};

    static Collider makeBox(const Vec3& halfExtents = {0.5F, 0.5F, 0.5F}) noexcept;
    static Collider makePlane(
        const Vec3& normal = {0.0F, 1.0F, 0.0F},
        Real offset = 0.0F
    ) noexcept;
};

struct Transform {
    Vec3 position{};
    Quaternion orientation{};
};

struct PhysicsMaterial {
    Real staticFriction{0.72F};
    Real dynamicFriction{0.56F};
    Real restitution{0.03F};
    Real rollingFriction{0.015F};
};

struct AerodynamicProperties {
    bool enabled{true};
    Real dragCoefficient{1.05F};
    Real angularDragCoefficient{0.08F};
    Real projectedAreaScale{1.0F};
    Vec3 centerOfPressureLocal{};
};

// A reduced, physically coupled rotor degree of freedom mounted at the host
// body's center of mass. The host locked-assembly tensor already contains the
// wheel mass/inertia; this coordinate releases its axial spin momentum. Motor
// torque is internal, so it produces an equal-and-opposite chassis reaction.
struct GyroscopicRotorDescription {
    Vec3 axisLocal{0.0F, 0.0F, 1.0F};
    // The host body's locked-assembly inertia must already include the wheel's
    // complete mass/inertia. This axial value defines the relative rotor DOF.
    Real axialInertia{0.01F};       // kg m^2 about the spin axis
    Real relativeAngularVelocity{0.0F}; // rad/s relative to the chassis
    Real bearingDamping{0.0F};      // N m s/rad
};

struct GyroscopicRotorState {
    RotorId id{kInvalidRotorId};
    Vec3 axisLocal{0.0F, 0.0F, 1.0F};
    Real axialInertia{0.01F};
    Real relativeAngularVelocity{0.0F};
    // J * absolute rotor-axis angular velocity. This is the canonical rotor
    // state advanced by internal motor and bearing torque.
    Real absoluteAxialAngularMomentum{0.0F};
    Real motorTorqueCommand{0.0F};
    Real bearingDamping{0.0F};
};

struct BodyDescription {
    MotionType motionType{MotionType::Dynamic};
    Collider collider{Collider::makeBox()};
    Transform transform{};
    Vec3 linearVelocity{};
    Vec3 angularVelocity{};
    Real mass{1'000.0F}; // kg: dense, non-floaty 1 m default cube
    Vec3 centerOfMassLocal{};
    // Optional inertia about the integration origin / center of mass, in the
    // body's local frame. It must be finite, symmetric, and positive definite.
    bool useCustomLocalInertiaTensor{false};
    Mat3 customLocalInertiaTensor{Mat3::identity()};
    PhysicsMaterial material{};
    AerodynamicProperties aerodynamics{};
    bool allowSleep{true};
    const char* debugName{"Body"};

    static BodyDescription makeDenseCube(const Vec3& position = {0.0F, 2.0F, 0.0F}) noexcept;
    static BodyDescription makeStaticPlane(
        const Vec3& normal = {0.0F, 1.0F, 0.0F},
        Real offset = 0.0F
    ) noexcept;
};

class World;

class RigidBody {
public:
    [[nodiscard]] BodyId id() const noexcept;
    [[nodiscard]] bool isAlive() const noexcept;
    [[nodiscard]] MotionType motionType() const noexcept;
    [[nodiscard]] const Collider& collider() const noexcept;
    [[nodiscard]] const Transform& transform() const noexcept;
    [[nodiscard]] const Transform& previousTransform() const noexcept;
    [[nodiscard]] const Vec3& linearVelocity() const noexcept;
    [[nodiscard]] const Vec3& angularVelocity() const noexcept;
    [[nodiscard]] Vec3 worldAngularMomentum() const noexcept;
    [[nodiscard]] Real rotationalKineticEnergy() const noexcept;
    [[nodiscard]] const PhysicsMaterial& material() const noexcept;
    [[nodiscard]] const AerodynamicProperties& aerodynamics() const noexcept;
    [[nodiscard]] Real mass() const noexcept;
    [[nodiscard]] Real inverseMass() const noexcept;
    // Compatibility view used by the editor: the three diagonal elements of
    // localInertiaTensor(). Off-diagonal products of inertia are available
    // through the tensor accessors below.
    [[nodiscard]] const Vec3& localInertia() const noexcept;
    // Effective carrier tensor used while rotor axial momenta are fixed.
    [[nodiscard]] const Mat3& localInertiaTensor() const noexcept;
    [[nodiscard]] const Mat3& inverseLocalInertiaTensor() const noexcept;
    [[nodiscard]] const Mat3& lockedAssemblyInertiaTensor() const noexcept;
    [[nodiscard]] bool usesCustomLocalInertiaTensor() const noexcept;
    [[nodiscard]] const Vec3& centerOfMassLocal() const noexcept;
    [[nodiscard]] Real volume() const noexcept;
    [[nodiscard]] Real bulkDensity() const noexcept;
    [[nodiscard]] bool isSleeping() const noexcept;
    [[nodiscard]] bool allowsSleep() const noexcept;
    [[nodiscard]] const char* debugName() const noexcept;
    [[nodiscard]] std::size_t gyroscopicRotorCount() const noexcept;
    [[nodiscard]] std::size_t gyroscopicRotorSlotCount() const noexcept;
    [[nodiscard]] const GyroscopicRotorState* gyroscopicRotor(
        RotorId id
    ) const noexcept;
    [[nodiscard]] Vec3 gyroscopicRotorAngularMomentumLocal() const noexcept;

    void setTransform(const Transform& transform) noexcept;
    void setLinearVelocity(const Vec3& velocity) noexcept;
    void setAngularVelocity(const Vec3& velocity) noexcept;
    void setWorldAngularMomentum(const Vec3& angularMomentum) noexcept;
    void setMass(Real kilograms) noexcept;
    [[nodiscard]] bool setCustomLocalInertiaTensor(
        const Mat3& inertiaTensorKilogramMetersSquared
    ) noexcept;
    void useAutomaticLocalInertiaTensor() noexcept;
    void setCenterOfMassLocal(const Vec3& localOffsetMeters) noexcept;
    void setBoxHalfExtents(const Vec3& halfExtentsMeters) noexcept;
    void setMaterial(const PhysicsMaterial& material) noexcept;
    void setAerodynamics(const AerodynamicProperties& properties) noexcept;
    void setAllowSleep(bool allowSleep) noexcept;
    void wake() noexcept;

    RotorId createGyroscopicRotor(
        const GyroscopicRotorDescription& description
    );
    bool destroyGyroscopicRotor(RotorId id) noexcept;
    bool setGyroscopicRotorRelativeAngularVelocity(
        RotorId id,
        Real radiansPerSecond
    ) noexcept;
    bool setGyroscopicRotorMotorTorque(
        RotorId id,
        Real torqueNewtonMeters
    ) noexcept;
    void applyGyroscopicRotorTorque(
        RotorId id,
        Real torqueNewtonMeters
    ) noexcept;

    void applyForce(const Vec3& forceNewtons) noexcept;
    void applyForceAtWorldPoint(const Vec3& forceNewtons, const Vec3& worldPoint) noexcept;
    void applyTorque(const Vec3& torqueNewtonMeters) noexcept;
    void applyAngularImpulse(const Vec3& angularImpulseNewtonMeterSeconds) noexcept;
    void applyImpulseAtWorldPoint(
        const Vec3& impulseNewtonSeconds,
        const Vec3& worldPoint
    ) noexcept;

    [[nodiscard]] Vec3 worldPointFromLocal(const Vec3& localPoint) const noexcept;
    [[nodiscard]] Vec3 velocityAtWorldPoint(const Vec3& worldPoint) const noexcept;
    [[nodiscard]] Real projectedBoxArea(const Vec3& worldDirection) const noexcept;
    [[nodiscard]] Aabb worldAabb() const noexcept;
    [[nodiscard]] std::array<Vec3, 8> boxWorldVertices() const noexcept;

private:
    friend class World;

    explicit RigidBody(BodyId id, const BodyDescription& description) noexcept;

    [[nodiscard]] Vec3 multiplyWorldInverseInertia(const Vec3& value) const noexcept;
    [[nodiscard]] Vec3 multiplyWorldInertia(const Vec3& value) const noexcept;
    void synchronizeAngularMomentumFromVelocity() noexcept;
    void synchronizeAngularVelocityFromMomentum() noexcept;
    [[nodiscard]] bool recomputeMassProperties() noexcept;
    void clearAccumulators() noexcept;

    struct RotorSlot {
        GyroscopicRotorState state{};
        Real torqueAccumulator{0.0F};
        Real evaluatedTorque{0.0F};
        bool alive{false};
    };

    BodyId id_{kInvalidBodyId};
    bool alive_{true};
    MotionType motionType_{MotionType::Static};
    Collider collider_{};
    Transform transform_{};
    Transform previousTransform_{};
    Vec3 linearVelocity_{};
    Vec3 angularVelocity_{};
    // World-space angular momentum is the conserved rotational state.
    // Angular velocity is derived from it through the orientation-dependent
    // full inertia tensor. Keeping both synchronized avoids the accumulated
    // I*w -> inverse(I)*L round-trip error that destabilizes fast rotors.
    Vec3 worldAngularMomentum_{};
    Vec3 forceAccumulator_{};
    Vec3 torqueAccumulator_{};
    Vec3 evaluatedForce_{};
    Vec3 evaluatedTorque_{};
    PhysicsMaterial material_{};
    AerodynamicProperties aerodynamics_{};
    Real mass_{0.0F};
    Real inverseMass_{0.0F};
    Vec3 centerOfMassLocal_{};
    Vec3 localInertia_{};
    Mat3 localInertiaTensor_{};
    Mat3 inverseLocalInertiaTensor_{};
    Mat3 lockedAssemblyInertiaTensor_{};
    bool useCustomLocalInertiaTensor_{false};
    Mat3 customLocalInertiaTensor_{Mat3::identity()};
    std::vector<RotorSlot> rotors_{};
    bool allowSleep_{true};
    bool sleeping_{false};
    bool touchedThisStep_{false};
    Real quietTime_{0.0F};
    std::array<char, 48> debugName_{};
};

} // namespace uaview::physics
