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

[[nodiscard]] Real relativeError(Real actual, Real expected) {
    return std::abs(actual - expected) /
           std::max(std::abs(expected), 1.0e-6F);
}

[[nodiscard]] Real angleBetween(const Vec3& lhs, const Vec3& rhs) {
    const Vec3 lhsUnit = normalized(lhs);
    const Vec3 rhsUnit = normalized(rhs);
    return std::acos(std::clamp(dot(lhsUnit, rhsUnit), -1.0F, 1.0F));
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

[[nodiscard]] Vec3 worldAngularMomentum(const RigidBody& body) {
    const Quaternion& orientation = body.transform().orientation;
    const Vec3 localAngularVelocity =
        orientation.inverseRotate(body.angularVelocity());
    return orientation.rotate(
        body.localInertiaTensor() * localAngularVelocity
    );
}

[[nodiscard]] Real rotationalEnergy(const RigidBody& body) {
    return 0.5F * dot(body.angularVelocity(), worldAngularMomentum(body));
}

BodyId createSpinningBox(
    World& world,
    const Vec3& halfExtents,
    Real mass,
    const Vec3& angularVelocity
) {
    BodyDescription body = BodyDescription::makeDenseCube();
    body.collider = Collider::makeBox(halfExtents);
    body.mass = mass;
    body.angularVelocity = angularVelocity;
    body.aerodynamics.enabled = false;
    body.allowSleep = false;
    return world.createBody(body);
}

void testIsotropicHighSpinIsExactAndNormalized() {
    World world{};
    makeVacuum(world);
    const Vec3 initialAngularVelocity{240.0F, -320.0F, 180.0F};
    const BodyId id = createSpinningBox(
        world,
        {0.5F, 0.5F, 0.5F},
        20.0F,
        initialAngularVelocity
    );

    for (int tick = 0; tick < 240; ++tick) {
        world.stepFixed();
    }

    const RigidBody& body = *world.body(id);
    const Real angularVelocityError =
        length(body.angularVelocity() - initialAngularVelocity);
    check(
        angularVelocityError < 0.01F,
        "an isotropic body keeps constant world angular velocity at high spin "
        "(error=" + std::to_string(angularVelocityError) + ")"
    );
    const Quaternion& orientation = body.transform().orientation;
    const Real quaternionNorm = std::sqrt(
        orientation.w * orientation.w +
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z
    );
    check(
        std::abs(quaternionNorm - 1.0F) < 2.0e-5F,
        "high-spin quaternion integration remains normalized"
    );
    check(
        world.debugStats().internalSubsteps >
            world.settings().solverSubsteps,
        "fast angular motion activates native adaptive substeps"
    );
}

void testTorqueFreeAsymmetricBodyConservesInvariants() {
    World world{};
    makeVacuum(world);
    const BodyId id = createSpinningBox(
        world,
        {0.35F, 0.55F, 0.90F},
        12.0F,
        {9.0F, 14.0F, 22.0F}
    );

    const Vec3 initialMomentum = worldAngularMomentum(*world.body(id));
    const Real initialEnergy = rotationalEnergy(*world.body(id));
    for (int tick = 0; tick < 600; ++tick) {
        world.stepFixed();
    }

    const RigidBody& body = *world.body(id);
    const Vec3 finalMomentum = worldAngularMomentum(body);
    const Real finalEnergy = rotationalEnergy(body);
    const Real momentumError =
        length(finalMomentum - initialMomentum) / length(initialMomentum);
    const Real energyError = relativeError(finalEnergy, initialEnergy);
    check(
        momentumError < 0.01F,
        "torque-free asymmetric spin conserves world angular momentum (error=" +
            std::to_string(momentumError) + ")"
    );
    check(
        energyError < 0.01F,
        "torque-free asymmetric spin conserves rotational energy (error=" +
            std::to_string(energyError) + ")"
    );
}

void testFastAxisymmetricRotorConservesMomentumAndEnergy() {
    World world{};
    makeVacuum(world);
    const BodyId id = createSpinningBox(
        world,
        {0.08F, 0.08F, 0.80F},
        10.0F,
        {0.55F, -0.35F, 500.0F}
    );

    const Vec3 initialMomentum = worldAngularMomentum(*world.body(id));
    const Real initialEnergy = rotationalEnergy(*world.body(id));
    for (int tick = 0; tick < 240; ++tick) {
        world.stepFixed();
    }

    const RigidBody& body = *world.body(id);
    const Vec3 finalMomentum = worldAngularMomentum(body);
    const Real finalEnergy = rotationalEnergy(body);
    const Real momentumError =
        length(finalMomentum - initialMomentum) / length(initialMomentum);
    const Real energyError = relativeError(finalEnergy, initialEnergy);
    check(
        momentumError < 0.015F,
        "500 rad/s rotor conserves angular momentum (error=" +
            std::to_string(momentumError) + ")"
    );
    check(
        energyError < 0.015F,
        "500 rad/s rotor conserves rotational energy (error=" +
            std::to_string(energyError) + ")"
    );
}

void testExtremeRotorUsesAccurateRotationalMicrosteps() {
    World world{};
    makeVacuum(world);
    constexpr Real revolutionsPerMinute = 100'000.0F;
    constexpr Real radiansPerSecond =
        revolutionsPerMinute * (2.0F * kPi / 60.0F);
    const BodyId id = createSpinningBox(
        world,
        {0.10F, 1.00F, 0.10F},
        10.0F,
        {1.0F, radiansPerSecond, -0.7F}
    );

    const Vec3 initialMomentum = worldAngularMomentum(*world.body(id));
    const Real initialEnergy = rotationalEnergy(*world.body(id));
    for (int tick = 0; tick < 30; ++tick) {
        world.stepFixed();
    }

    const RigidBody& body = *world.body(id);
    const Vec3 finalMomentum = worldAngularMomentum(body);
    const Real finalEnergy = rotationalEnergy(body);
    const Real momentumError =
        length(finalMomentum - initialMomentum) / length(initialMomentum);
    const Real energyError = relativeError(finalEnergy, initialEnergy);
    check(
        momentumError < 1.0e-4F,
        "100,000 RPM spin conserves world angular momentum (error=" +
            std::to_string(momentumError) + ")"
    );
    check(
        energyError < 5.0e-4F,
        "100,000 RPM spin keeps rotational energy bounded (error=" +
            std::to_string(energyError) + ")"
    );
    check(
        world.debugStats().maximumRotationMicrostepsUsed > 1,
        "extreme RPM activates independent rotational microsteps"
    );
    check(
        world.debugStats().rotationMicrostepCapHits == 0,
        "100,000 RPM remains below the configured rotor microstep cap"
    );
    check(
        world.debugStats().rotationMidpointNonConvergenceCount == 0,
        "100,000 RPM midpoint solves converge"
    );
}

void testPureRotationCannotSweepThroughGroundPlane() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 1;
    settings.maximumAdaptiveSubsteps = 1;
    settings.velocityIterations = 24;
    World world{settings};
    makeVacuum(world);
    (void)world.createBody(
        BodyDescription::makeStaticPlane(
            {0.0F, 1.0F, 0.0F},
            0.0F
        )
    );

    BodyDescription rod = BodyDescription::makeDenseCube(
        {0.0F, 0.30F, 0.0F}
    );
    rod.collider = Collider::makeBox({1.0F, 0.02F, 0.02F});
    rod.mass = 10.0F;
    // Exactly half a revolution in one deliberately under-substepped tick:
    // both endpoint poses are clear of the plane, but the intermediate rod
    // orientation crosses deeply through it.
    rod.angularVelocity = {
        0.0F,
        0.0F,
        kPi * static_cast<Real>(settings.fixedUpdateHz)
    };
    rod.aerodynamics.enabled = false;
    rod.allowSleep = false;
    const BodyId rodId = world.createBody(rod);
    const Real initialSpeed =
        length(world.body(rodId)->angularVelocity());

    world.stepFixed();
    const RigidBody& body = *world.body(rodId);
    Real minimumVertexHeight =
        std::numeric_limits<Real>::max();
    for (const Vec3& vertex : body.boxWorldVertices()) {
        minimumVertexHeight =
            std::min(minimumVertexHeight, vertex.y);
    }
    std::cout
        << "rotational CCD probe: hits "
        << world.debugStats().rotationalCcdHits
        << ", advances " << world.debugStats().rotationalCcdAdvances
        << ", convergence " << world.debugStats().rotationalCcdConvergenceHits
        << ", caps " << world.debugStats().rotationalCcdAdvanceCapHits
        << ", contacts " << world.debugStats().contactCount
        << ", min y " << minimumVertexHeight
        << ", speed " << length(body.angularVelocity())
        << '\n';
    check(
        world.debugStats().rotationalCcdHits > 0,
        "pure rotational plane crossing is caught by conservative CCD"
    );
    check(
        world.debugStats().contactCount > 0,
        "rotational CCD feeds the ordinary manifold/contact solver"
    );
    check(
        minimumVertexHeight > -0.01F,
        "fast rotating rod is stopped at the ground instead of tunneling"
    );
    check(
        length(body.angularVelocity()) < initialSpeed,
        "rotational impact removes the inward endpoint velocity"
    );
}

struct TorqueRun {
    Vec3 angularMomentum{};
    Vec3 bodyAxis{};
    Real axisTilt{0.0F};
};

TorqueRun runTransverseTorque(Real axialSpin) {
    World world{};
    makeVacuum(world);
    const BodyId id = createSpinningBox(
        world,
        {0.08F, 0.08F, 0.80F},
        10.0F,
        {0.0F, 0.0F, axialSpin}
    );

    constexpr Vec3 appliedTorque{2.0F, 0.0F, 0.0F};
    for (int tick = 0; tick < 120; ++tick) {
        world.body(id)->applyTorque(appliedTorque);
        world.stepFixed();
    }

    const RigidBody& body = *world.body(id);
    const Vec3 axis = body.transform().orientation.rotate({0.0F, 0.0F, 1.0F});
    return {
        worldAngularMomentum(body),
        axis,
        angleBetween(axis, {0.0F, 0.0F, 1.0F}),
    };
}

void testTransverseTorqueProducesNativeGyroscopicStabilization() {
    World referenceWorld{};
    makeVacuum(referenceWorld);
    const BodyId referenceId = createSpinningBox(
        referenceWorld,
        {0.08F, 0.08F, 0.80F},
        10.0F,
        {0.0F, 0.0F, 300.0F}
    );
    const Vec3 initialMomentum =
        worldAngularMomentum(*referenceWorld.body(referenceId));

    const TorqueRun fast = runTransverseTorque(300.0F);
    const TorqueRun stopped = runTransverseTorque(0.0F);
    const Vec3 expectedMomentum =
        initialMomentum + Vec3{2.0F, 0.0F, 0.0F};
    const Real momentumError =
        length(fast.angularMomentum - expectedMomentum) /
        length(expectedMomentum);

    check(
        momentumError < 0.02F,
        "integrated world torque changes angular momentum by torque times time "
        "(error=" + std::to_string(momentumError) + ")"
    );
    check(
        fast.axisTilt < stopped.axisTilt * 0.75F,
        "a fast rotor resists transverse reorientation through native "
        "gyroscopic dynamics"
    );
    check(
        angleBetween(fast.bodyAxis, fast.angularMomentum) < radians(12.0F),
        "the high-spin symmetry axis remains close to angular momentum"
    );
}

void testIntermediateAxisInstabilityIsPhysical() {
    World world{};
    makeVacuum(world);
    const BodyId id = createSpinningBox(
        world,
        {0.30F, 0.60F, 1.00F},
        8.0F,
        {0.02F, 30.0F, 0.01F}
    );
    for (int tick = 0; tick < 360; ++tick) {
        world.stepFixed();
    }

    const Vec3 localAngularVelocity =
        world.body(id)->transform().orientation.inverseRotate(
            world.body(id)->angularVelocity()
        );
    check(
        std::abs(localAngularVelocity.x) > 0.2F ||
            std::abs(localAngularVelocity.z) > 0.2F,
        "spin near the intermediate principal axis naturally tumbles"
    );
}

void testFastSpinReplayIsDeterministic() {
    World first{};
    World second{};
    makeVacuum(first);
    makeVacuum(second);
    const BodyId firstId = createSpinningBox(
        first,
        {0.12F, 0.20F, 0.75F},
        9.0F,
        {35.0F, -28.0F, 420.0F}
    );
    const BodyId secondId = createSpinningBox(
        second,
        {0.12F, 0.20F, 0.75F},
        9.0F,
        {35.0F, -28.0F, 420.0F}
    );
    for (int tick = 0; tick < 240; ++tick) {
        first.stepFixed();
        second.stepFixed();
    }

    const RigidBody& lhs = *first.body(firstId);
    const RigidBody& rhs = *second.body(secondId);
    check(
        lhs.angularVelocity().x == rhs.angularVelocity().x &&
            lhs.angularVelocity().y == rhs.angularVelocity().y &&
            lhs.angularVelocity().z == rhs.angularVelocity().z,
        "fast-spin angular velocity replay is bit-identical"
    );
    check(
        lhs.transform().orientation.w == rhs.transform().orientation.w &&
            lhs.transform().orientation.x == rhs.transform().orientation.x &&
            lhs.transform().orientation.y == rhs.transform().orientation.y &&
            lhs.transform().orientation.z == rhs.transform().orientation.z,
        "fast-spin orientation replay is bit-identical"
    );
}

} // namespace

int main() {
    testIsotropicHighSpinIsExactAndNormalized();
    testTorqueFreeAsymmetricBodyConservesInvariants();
    testFastAxisymmetricRotorConservesMomentumAndEnergy();
    testExtremeRotorUsesAccurateRotationalMicrosteps();
    testPureRotationCannotSweepThroughGroundPlane();
    testTransverseTorqueProducesNativeGyroscopicStabilization();
    testIntermediateAxisInstabilityIsPhysical();
    testFastSpinReplayIsDeterministic();

    if (failures != 0) {
        std::cerr << failures << " rotational-dynamics test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All UAView Studio rotational-dynamics tests passed.\n";
    return EXIT_SUCCESS;
}
