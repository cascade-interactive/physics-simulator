#include <uaview/physics/World.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

using namespace uaview::physics;

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

[[nodiscard]] bool nearlyEqual(
    Real lhs,
    Real rhs,
    Real tolerance = 1.0e-4F
) {
    return std::abs(lhs - rhs) <= tolerance;
}

[[nodiscard]] bool nearlyEqual(
    const Vec3& lhs,
    const Vec3& rhs,
    Real tolerance = 1.0e-4F
) {
    return nearlyEqual(lhs.x, rhs.x, tolerance) &&
           nearlyEqual(lhs.y, rhs.y, tolerance) &&
           nearlyEqual(lhs.z, rhs.z, tolerance);
}

[[nodiscard]] Real angleBetween(const Vec3& lhs, const Vec3& rhs) {
    return std::acos(std::clamp(
        dot(normalized(lhs), normalized(rhs)),
        -1.0F,
        1.0F
    ));
}

void makeVacuum(World& world) {
    world.environment().setGravity({});
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.meanVelocity = {};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
}

RigidBody* createSphericalCarrier(World& world) {
    BodyDescription description = BodyDescription::makeDenseCube();
    description.mass = 10.0F;
    description.aerodynamics.enabled = false;
    description.allowSleep = false;
    description.useCustomLocalInertiaTensor = true;
    description.customLocalInertiaTensor =
        Mat3::diagonal({2.0F, 2.0F, 2.0F});
    return world.body(world.createBody(description));
}

void testMotorTorqueProducesExactCarrierReaction() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axisLocal = {0.0F, 0.0F, 1.0F};
    rotorDescription.axialInertia = 0.1F;
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    check(rotorId != kInvalidRotorId, "valid internal rotor is created");

    constexpr Real motorTorque = 0.2F;
    check(
        body->setGyroscopicRotorMotorTorque(
            rotorId,
            motorTorque
        ),
        "persistent motor command is accepted"
    );
    for (int tick = 0; tick < 120; ++tick) {
        world.stepFixed();
    }
    (void)body->setGyroscopicRotorMotorTorque(rotorId, 0.0F);

    const GyroscopicRotorState* rotor =
        body->gyroscopicRotor(rotorId);
    check(rotor != nullptr, "rotor state remains addressable");
    if (rotor == nullptr) {
        return;
    }
    check(
        nearlyEqual(body->worldAngularMomentum(), {}, 2.0e-5F),
        "internal motor torque conserves total world angular momentum"
    );
    check(
        nearlyEqual(
            rotor->absoluteAxialAngularMomentum,
            0.2F,
            2.0e-5F
        ),
        "motor torque integrates rotor axial momentum as torque times time"
    );
    check(
        nearlyEqual(
            body->angularVelocity().z,
            -0.2F / 1.9F,
            2.0e-5F
        ),
        "carrier receives the analytic equal-and-opposite angular response"
    );
    check(
        nearlyEqual(
            rotor->relativeAngularVelocity,
            2.0F + 0.2F / 1.9F,
            3.0e-5F
        ),
        "reported rotor speed is relative to the reacting carrier"
    );
    check(
        nearlyEqual(
            body->rotationalKineticEnergy(),
            0.21052632F,
            5.0e-5F
        ),
        "carrier plus rotor energy matches the analytic motor-work result"
    );
}

struct DisturbanceRun {
    Real bodyAxisTilt{0.0F};
    Vec3 finalMomentum{};
    Real rotorMomentum{0.0F};
};

DisturbanceRun runTransverseDisturbance(Real rotorSpeed) {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axisLocal = {0.0F, 0.0F, 1.0F};
    rotorDescription.axialInertia = 0.1F;
    rotorDescription.relativeAngularVelocity = rotorSpeed;
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    const Real initialRotorMomentum =
        body->gyroscopicRotor(rotorId)
            ->absoluteAxialAngularMomentum;

    constexpr Vec3 externalTorque{1.0F, 0.0F, 0.0F};
    for (int tick = 0; tick < 120; ++tick) {
        body->applyTorque(externalTorque);
        world.stepFixed();
    }
    const Vec3 bodyAxis =
        body->transform().orientation.rotate({0.0F, 0.0F, 1.0F});
    return {
        angleBetween(bodyAxis, {0.0F, 0.0F, 1.0F}),
        body->worldAngularMomentum(),
        body->gyroscopicRotor(rotorId)
            ->absoluteAxialAngularMomentum -
            initialRotorMomentum,
    };
}

void testPassiveRotorStabilizationEmergesWithoutController() {
    const DisturbanceRun stopped = runTransverseDisturbance(0.0F);
    const DisturbanceRun spinning = runTransverseDisturbance(1'000.0F);
    check(
        spinning.bodyAxisTilt < stopped.bodyAxisTilt * 0.25F,
        "stored rotor momentum strongly resists transverse chassis tilt"
    );
    check(
        nearlyEqual(
            spinning.finalMomentum,
            {1.0F, 0.0F, 100.0F},
            0.002F
        ),
        "external torque changes total gyro momentum by torque times time"
    );
    check(
        std::abs(spinning.rotorMomentum) < 1.0e-6F,
        "passive disturbance leaves the rotor canonical momentum unchanged"
    );
}

void testBearingLossTransfersMomentumAndDissipatesEnergy() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    body->setAllowSleep(true);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axialInertia = 0.1F;
    rotorDescription.relativeAngularVelocity = 100.0F;
    rotorDescription.bearingDamping = 0.01F;
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    const Vec3 initialMomentum = body->worldAngularMomentum();
    const Real initialRotorMomentum =
        body->gyroscopicRotor(rotorId)
            ->absoluteAxialAngularMomentum;
    Real previousEnergy = body->rotationalKineticEnergy();

    for (int tick = 0; tick < 240; ++tick) {
        world.stepFixed();
        const Real energy = body->rotationalKineticEnergy();
        check(
            energy <= previousEnergy + 2.0e-4F,
            "bearing damping never injects rotational energy"
        );
        previousEnergy = energy;
    }

    check(
        nearlyEqual(
            body->worldAngularMomentum(),
            initialMomentum,
            2.0e-4F
        ),
        "bearing loss conserves total assembly angular momentum"
    );
    check(
        std::abs(
            body->gyroscopicRotor(rotorId)
                ->absoluteAxialAngularMomentum
        ) < std::abs(initialRotorMomentum),
        "bearing loss transfers rotor momentum into the carrier"
    );
    const Real finalEnergy = body->rotationalKineticEnergy();
    const Real initialEnergy = 0.5F * 0.1F * 100.0F * 100.0F;
    check(
        finalEnergy < initialEnergy,
        "bearing loss reduces total rotational energy (initial=" +
            std::to_string(initialEnergy) +
            ", final=" + std::to_string(finalEnergy) + ")"
    );
    check(
        !body->isSleeping(),
        "a body with stored rotor momentum is never zeroed by sleeping"
    );
}

void testInvalidRotorCannotMakeCarrierInertiaSingular() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription invalid{};
    invalid.axialInertia = 2.0F;
    const RotorId rejected = body->createGyroscopicRotor(invalid);
    check(
        rejected == kInvalidRotorId,
        "rotor configuration that makes carrier inertia singular is rejected"
    );
    check(
        body->gyroscopicRotorCount() == 0,
        "rejected rotor leaves the body configuration transactional"
    );
    check(
        nearlyEqual(
            body->localInertiaTensor().diagonalValue(),
            {2.0F, 2.0F, 2.0F}
        ),
        "rejected rotor leaves the previous inertia tensor unchanged"
    );
}

void testLargeBearingDampingCannotOvershoot() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axialInertia = 0.1F;
    rotorDescription.relativeAngularVelocity = 100.0F;
    rotorDescription.bearingDamping = 1.0e6F;
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    const Vec3 initialMomentum = body->worldAngularMomentum();
    const Real initialEnergy = body->rotationalKineticEnergy();

    world.stepFixed();
    const GyroscopicRotorState* rotor =
        body->gyroscopicRotor(rotorId);
    check(
        rotor != nullptr &&
            std::abs(rotor->relativeAngularVelocity) < 0.001F,
        "arbitrarily stiff bearing damping approaches lock without overshoot"
    );
    check(
        isFinite(body->angularVelocity()) &&
            std::isfinite(body->rotationalKineticEnergy()) &&
            body->rotationalKineticEnergy() <= initialEnergy,
        "stiff bearing damping remains finite and dissipative"
    );
    check(
        nearlyEqual(
            body->worldAngularMomentum(),
            initialMomentum,
            2.0e-5F
        ),
        "stiff bearing lock still conserves total angular momentum"
    );
}

void testMotorAndBearingReachPhysicalSteadySlip() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axialInertia = 0.1F;
    rotorDescription.bearingDamping = 100.0F;
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    constexpr Real motorTorque = 20.0F;
    (void)body->setGyroscopicRotorMotorTorque(
        rotorId,
        motorTorque
    );
    for (int tick = 0; tick < 5; ++tick) {
        world.stepFixed();
    }

    const GyroscopicRotorState* rotor =
        body->gyroscopicRotor(rotorId);
    check(
        rotor != nullptr &&
            nearlyEqual(
                rotor->relativeAngularVelocity,
                motorTorque / rotorDescription.bearingDamping,
                2.0e-4F
            ),
        "combined motor and bearing solve reaches the physical u/c steady slip"
    );
    check(
        nearlyEqual(body->worldAngularMomentum(), {}, 2.0e-5F),
        "steady internal motor/bearing exchange preserves total momentum"
    );
}

void testTinyBearingDampingHasFiniteMotorLimit() {
    World world{};
    makeVacuum(world);
    RigidBody* body = createSphericalCarrier(world);
    GyroscopicRotorDescription rotorDescription{};
    rotorDescription.axialInertia = 0.1F;
    rotorDescription.bearingDamping =
        std::numeric_limits<Real>::denorm_min();
    const RotorId rotorId =
        body->createGyroscopicRotor(rotorDescription);
    (void)body->setGyroscopicRotorMotorTorque(rotorId, 1.0F);
    world.stepFixed();

    const GyroscopicRotorState* rotor =
        body->gyroscopicRotor(rotorId);
    check(
        rotor != nullptr &&
            std::isfinite(
                rotor->absoluteAxialAngularMomentum
            ) &&
            nearlyEqual(
                rotor->absoluteAxialAngularMomentum,
                1.0F / 120.0F,
                2.0e-5F
            ),
        "denormal-small bearing damping approaches the undamped motor impulse"
    );
    check(
        isFinite(body->angularVelocity()) &&
            nearlyEqual(body->worldAngularMomentum(), {}, 2.0e-5F),
        "tiny damping cannot poison or erase conserved assembly momentum"
    );
}

} // namespace

int main() {
    testMotorTorqueProducesExactCarrierReaction();
    testPassiveRotorStabilizationEmergesWithoutController();
    testBearingLossTransfersMomentumAndDissipatesEnergy();
    testInvalidRotorCannotMakeCarrierInertiaSingular();
    testLargeBearingDampingCannotOvershoot();
    testMotorAndBearingReachPhysicalSteadySlip();
    testTinyBearingDampingHasFiniteMotorLimit();

    if (failures != 0) {
        std::cerr << failures << " gyroscopic-rotor test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All UAView Studio gyroscopic-rotor tests passed.\n";
    return EXIT_SUCCESS;
}
