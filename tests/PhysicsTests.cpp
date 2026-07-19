#include <uaview/physics/World.hpp>

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

bool nearlyEqual(Real lhs, Real rhs, Real tolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

void makeStillAir(World& world, Real density = 0.0F) {
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = density;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.meanVelocity = {};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
}

void testVelocityVerletFreeFall() {
    World world{};
    makeStillAir(world);

    BodyDescription falling = BodyDescription::makeDenseCube({0.0F, 20.0F, 0.0F});
    falling.mass = 120.0F;
    const BodyId bodyId = world.createBody(falling);
    for (int step = 0; step < 120; ++step) {
        world.stepFixed();
    }

    const RigidBody* body = world.body(bodyId);
    const Real expectedVelocity = -9.80665F;
    const Real expectedHeight = 20.0F - 0.5F * 9.80665F;
    check(body != nullptr, "free-fall body remains valid");
    if (body != nullptr) {
        check(
            nearlyEqual(body->linearVelocity().y, expectedVelocity, 0.0025F),
            "Velocity Verlet matches analytic constant-acceleration velocity"
        );
        check(
            nearlyEqual(body->transform().position.y, expectedHeight, 0.0025F),
            "Velocity Verlet matches analytic constant-acceleration position"
        );
    }
    check(world.debugStats().fixedTick == 120, "fixed tick counter is exact");
}

void testDenseCubeIsNotFloatyAndSettles() {
    World world{};
    makeStillAir(world, 1.225F);
    world.createBody(BodyDescription::makeStaticPlane());

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 5.0F, 0.0F});
    cube.mass = 120.0F;
    cube.material.restitution = 0.01F;
    const BodyId cubeId = world.createBody(cube);

    for (int step = 0; step < 60; ++step) {
        world.stepFixed();
    }
    const RigidBody* body = world.body(cubeId);
    const Real ballisticHeight = 5.0F - 0.5F * 9.80665F * 0.25F;
    check(body != nullptr, "dense cube remains valid");
    if (body != nullptr) {
        check(
            std::abs(body->transform().position.y - ballisticHeight) < 0.035F,
            "120 kg, 1 m cube is near-ballistic during the first 0.5 seconds"
        );
        check(
            body->linearVelocity().y < -4.75F,
            "dense cube gains realistic downward speed instead of floating"
        );
    }

    for (int step = 0; step < 720; ++step) {
        world.stepFixed();
    }
    body = world.body(cubeId);
    if (body != nullptr) {
        check(
            nearlyEqual(body->transform().position.y, 0.5F, 0.012F),
            "dense cube settles at its physical half-height"
        );
        check(
            std::sqrt(
                body->transform().position.x * body->transform().position.x +
                body->transform().position.z * body->transform().position.z
            ) < 0.02F,
            "a symmetric vertical drop does not create horizontal drift"
        );
        check(
            length(body->linearVelocity()) < 0.05F,
            "settled cube has negligible residual linear speed"
        );
        check(body->isSleeping(), "stable resting cube enters sleep");
    }
}

void testSubtleWindDoesNotSweepTheDenseCubeAcrossTheGround() {
    World world{};

    BodyDescription ground = BodyDescription::makeStaticPlane();
    ground.material.staticFriction = 0.82F;
    ground.material.dynamicFriction = 0.68F;
    ground.material.restitution = 0.02F;
    ground.material.rollingFriction = 0.02F;
    world.createBody(ground);

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 3.0F, 0.0F});
    cube.mass = 120.0F;
    cube.material.staticFriction = 0.65F;
    cube.material.dynamicFriction = 0.50F;
    cube.material.restitution = 0.05F;
    cube.material.rollingFriction = 0.015F;
    cube.aerodynamics.enabled = true;
    cube.aerodynamics.dragCoefficient = 1.05F;
    cube.aerodynamics.angularDragCoefficient = 0.08F;
    const BodyId cubeId = world.createBody(cube);

    for (int step = 0; step < 2'000; ++step) {
        world.stepFixed();
    }

    const RigidBody* body = world.body(cubeId);
    const Real horizontalDistance = std::sqrt(
        body->transform().position.x * body->transform().position.x +
        body->transform().position.z * body->transform().position.z
    );
    check(
        horizontalDistance < 0.15F,
        "subtle default wind cannot sweep a settled 120 kg cube across the ground (distance=" +
            std::to_string(horizontalDistance) + " m)"
    );
}

Real slidingSpeedAfterTwoSeconds(Real staticFriction, Real dynamicFriction) {
    World world{};
    makeStillAir(world);

    BodyDescription ground = BodyDescription::makeStaticPlane();
    ground.material.staticFriction = staticFriction;
    ground.material.dynamicFriction = dynamicFriction;
    ground.material.rollingFriction = 0.0F;
    world.createBody(ground);

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 0.501F, 0.0F});
    cube.mass = 120.0F;
    cube.linearVelocity = {4.0F, 0.0F, 0.0F};
    cube.material.staticFriction = staticFriction;
    cube.material.dynamicFriction = dynamicFriction;
    cube.material.rollingFriction = 0.0F;
    const BodyId cubeId = world.createBody(cube);

    for (int step = 0; step < 240; ++step) {
        world.stepFixed();
    }
    return std::abs(world.body(cubeId)->linearVelocity().x);
}

void testStaticAndDynamicFriction() {
    const Real frictionSpeed = slidingSpeedAfterTwoSeconds(0.82F, 0.65F);
    const Real frictionlessSpeed = slidingSpeedAfterTwoSeconds(0.0F, 0.0F);
    check(frictionSpeed < 0.15F, "dynamic friction stops a sliding dense cube");
    check(
        frictionlessSpeed > 3.9F,
        "zero-friction material preserves horizontal speed"
    );
}

Real rollingSpeedAfterTenthSecond(Real rollingFriction) {
    World world{};
    makeStillAir(world);

    BodyDescription ground = BodyDescription::makeStaticPlane();
    ground.material.staticFriction = 0.0F;
    ground.material.dynamicFriction = 0.0F;
    ground.material.rollingFriction = rollingFriction;
    world.createBody(ground);

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 0.501F, 0.0F});
    cube.mass = 120.0F;
    cube.angularVelocity = {0.0F, 0.0F, 8.0F};
    cube.material.staticFriction = 0.0F;
    cube.material.dynamicFriction = 0.0F;
    cube.material.rollingFriction = rollingFriction;
    const BodyId cubeId = world.createBody(cube);
    for (int step = 0; step < 12; ++step) {
        world.stepFixed();
    }
    return length(world.body(cubeId)->angularVelocity());
}

void testRollingResistance() {
    const Real withRollingResistance = rollingSpeedAfterTenthSecond(0.12F);
    const Real withoutRollingResistance = rollingSpeedAfterTenthSecond(0.0F);
    check(
        withRollingResistance < withoutRollingResistance - 0.25F,
        "bounded contact rolling resistance reduces angular speed (with=" +
            std::to_string(withRollingResistance) + ", without=" +
            std::to_string(withoutRollingResistance) + ")"
    );
}

void testProjectedAreaAndRelativeWindDrag() {
    World faceWorld{};
    World edgeWorld{};
    makeStillAir(faceWorld, 1.225F);
    makeStillAir(edgeWorld, 1.225F);
    faceWorld.environment().setGravity({});
    edgeWorld.environment().setGravity({});

    BodyDescription plate = BodyDescription::makeDenseCube({0.0F, 10.0F, 0.0F});
    plate.collider = Collider::makeBox({0.5F, 0.04F, 0.5F});
    plate.mass = 10.0F;
    plate.linearVelocity = {0.0F, -20.0F, 0.0F};
    plate.allowSleep = false;

    const BodyId faceId = faceWorld.createBody(plate);
    plate.transform.orientation =
        Quaternion::fromAxisAngle({1.0F, 0.0F, 0.0F}, radians(90.0F));
    const BodyId edgeId = edgeWorld.createBody(plate);

    const RigidBody* faceBody = faceWorld.body(faceId);
    const RigidBody* edgeBody = edgeWorld.body(edgeId);
    check(
        nearlyEqual(
            faceBody->projectedBoxArea({0.0F, -1.0F, 0.0F}),
            1.0F,
            0.001F
        ),
        "face-on thin box exposes one square meter"
    );
    check(
        nearlyEqual(
            edgeBody->projectedBoxArea({0.0F, -1.0F, 0.0F}),
            0.08F,
            0.002F
        ),
        "edge-on thin box exposes only its thin side"
    );

    for (int step = 0; step < 60; ++step) {
        faceWorld.stepFixed();
        edgeWorld.stepFixed();
    }
    const Real faceSpeed = length(faceWorld.body(faceId)->linearVelocity());
    const Real edgeSpeed = length(edgeWorld.body(edgeId)->linearVelocity());
    check(
        faceSpeed < edgeSpeed - 0.7F,
        "face-on plate loses substantially more speed to aerodynamic drag"
    );

    World matchingWindWorld{};
    matchingWindWorld.environment().setGravity({});
    AtmosphereSample atmosphere = matchingWindWorld.environment().atmosphere();
    atmosphere.density = 1.225F;
    matchingWindWorld.environment().setAtmosphere(atmosphere);
    WindFieldSettings matchingWind{};
    matchingWind.meanVelocity = {10.0F, 0.0F, 0.0F};
    matchingWind.turbulenceAmplitude = 0.0F;
    matchingWindWorld.environment().setWindSettings(matchingWind);
    BodyDescription carried = BodyDescription::makeDenseCube();
    carried.mass = 1.0F;
    carried.linearVelocity = matchingWind.meanVelocity;
    carried.allowSleep = false;
    const BodyId carriedId = matchingWindWorld.createBody(carried);
    for (int step = 0; step < 120; ++step) {
        matchingWindWorld.stepFixed();
    }
    check(
        nearlyEqual(
            matchingWindWorld.body(carriedId)->linearVelocity().x,
            10.0F,
            0.0005F
        ),
        "drag uses air-relative velocity, not world velocity"
    );
}

void testRestitutionAndHighSpeedAntiTunneling() {
    World bounceWorld{};
    makeStillAir(bounceWorld);
    BodyDescription bouncyGround = BodyDescription::makeStaticPlane();
    bouncyGround.material.restitution = 0.78F;
    bounceWorld.createBody(bouncyGround);
    BodyDescription bouncyCube = BodyDescription::makeDenseCube({0.0F, 3.0F, 0.0F});
    bouncyCube.mass = 120.0F;
    bouncyCube.material.restitution = 0.78F;
    bouncyCube.allowSleep = false;
    const BodyId bounceId = bounceWorld.createBody(bouncyCube);
    Real maximumUpwardSpeed = 0.0F;
    for (int step = 0; step < 180; ++step) {
        bounceWorld.stepFixed();
        maximumUpwardSpeed = std::max(
            maximumUpwardSpeed,
            bounceWorld.body(bounceId)->linearVelocity().y
        );
    }
    check(
        maximumUpwardSpeed > 4.0F,
        "cached restitution bias preserves a physically visible bounce"
    );

    World fastWorld{};
    makeStillAir(fastWorld);
    fastWorld.createBody(BodyDescription::makeStaticPlane());
    BodyDescription fastCube = BodyDescription::makeDenseCube({0.0F, 10.0F, 0.0F});
    fastCube.mass = 120.0F;
    fastCube.linearVelocity = {0.0F, -600.0F, 0.0F};
    fastCube.material.restitution = 0.0F;
    fastCube.allowSleep = false;
    const BodyId fastId = fastWorld.createBody(fastCube);
    std::uint32_t maximumObservedSubsteps = 0;
    for (int step = 0; step < 8; ++step) {
        fastWorld.stepFixed();
        maximumObservedSubsteps = std::max(
            maximumObservedSubsteps,
            fastWorld.debugStats().internalSubsteps
        );
    }
    const RigidBody* fastBody = fastWorld.body(fastId);
    check(maximumObservedSubsteps > 2, "fast motion activates deterministic adaptive substeps");
    check(
        fastBody->transform().position.y > 0.45F,
        "600 m/s cube does not tunnel through the ground plane"
    );
    check(isFinite(fastBody->linearVelocity()), "high-speed impact remains finite");
}

void testOrientedBoxCollisionAndRaycast() {
    World world{};
    makeStillAir(world);
    world.environment().setGravity({});

    BodyDescription obstacle = BodyDescription::makeDenseCube({0.0F, 1.0F, 0.0F});
    obstacle.motionType = MotionType::Static;
    obstacle.transform.orientation =
        Quaternion::fromAxisAngle({0.0F, 1.0F, 0.0F}, radians(28.0F));
    obstacle.aerodynamics.enabled = false;
    const BodyId obstacleId = world.createBody(obstacle);

    BodyDescription mover = BodyDescription::makeDenseCube({-3.0F, 1.0F, 0.15F});
    mover.mass = 120.0F;
    mover.linearVelocity = {10.0F, 0.0F, 0.0F};
    mover.aerodynamics.enabled = false;
    mover.allowSleep = false;
    const BodyId moverId = world.createBody(mover);
    for (int step = 0; step < 90; ++step) {
        world.stepFixed();
    }
    check(
        world.body(moverId)->transform().position.x < 1.8F,
        "15-axis OBB SAT prevents passage through a rotated static box"
    );

    RaycastHit hit{};
    check(
        world.raycast({0.0F, 1.0F, 5.0F}, {0.0F, 0.0F, -1.0F}, 20.0F, hit),
        "raycast finds an oriented box"
    );
    check(hit.body == obstacleId || hit.body == moverId, "raycast returns a stable body handle");
    check(hit.distance >= 0.0F && hit.distance < 20.0F, "raycast distance is bounded");
}

void testSpringAndExternalForceLifetime() {
    World forceWorld{};
    makeStillAir(forceWorld);
    forceWorld.environment().setGravity({});
    BodyDescription forced = BodyDescription::makeDenseCube();
    forced.mass = 120.0F;
    forced.allowSleep = false;
    const BodyId forcedId = forceWorld.createBody(forced);
    forceWorld.body(forcedId)->applyForce({120.0F, 0.0F, 0.0F});
    forceWorld.stepFixed();
    const Real expectedDeltaVelocity = 1.0F / 120.0F;
    check(
        nearlyEqual(
            forceWorld.body(forcedId)->linearVelocity().x,
            expectedDeltaVelocity,
            0.00005F
        ),
        "one external force application persists across substeps without multiplying"
    );
    forceWorld.stepFixed();
    check(
        nearlyEqual(
            forceWorld.body(forcedId)->linearVelocity().x,
            expectedDeltaVelocity,
            0.00005F
        ),
        "external force accumulator clears after one fixed tick"
    );

    World springWorld{};
    makeStillAir(springWorld);
    springWorld.environment().setGravity({});
    BodyDescription grabbed = BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    grabbed.mass = 10.0F;
    grabbed.allowSleep = false;
    const BodyId grabbedId = springWorld.createBody(grabbed);
    SpringForce spring{};
    spring.body = grabbedId;
    spring.worldTarget = {2.0F, 0.0F, 0.0F};
    spring.stiffness = 500.0F;
    spring.damping = 80.0F;
    spring.maximumForce = 2'000.0F;
    const SpringId springId = springWorld.createSpring(spring);
    for (int step = 0; step < 120; ++step) {
        springWorld.stepFixed();
    }
    check(
        springWorld.body(grabbedId)->transform().position.x > 1.5F,
        "managed spring moves a grabbed body toward its target"
    );
    check(springWorld.destroySpring(springId), "managed spring can be released");
}

void testDeterministicFixedStep() {
    World first{};
    World second{};

    auto populate = [](World& world) {
        world.createBody(BodyDescription::makeStaticPlane());
        BodyDescription cube = BodyDescription::makeDenseCube({0.25F, 4.0F, -0.5F});
        cube.mass = 120.0F;
        cube.linearVelocity = {1.2F, 0.0F, -0.35F};
        cube.angularVelocity = {0.2F, 0.4F, -0.1F};
        cube.centerOfMassLocal = {0.03F, -0.02F, 0.01F};
        cube.aerodynamics.centerOfPressureLocal = {0.1F, 0.0F, -0.05F};
        return world.createBody(cube);
    };

    const BodyId firstId = populate(first);
    const BodyId secondId = populate(second);
    for (int step = 0; step < 1'000; ++step) {
        first.stepFixed();
        second.stepFixed();
    }

    const RigidBody* lhs = first.body(firstId);
    const RigidBody* rhs = second.body(secondId);
    check(
        lhs->transform().position.x == rhs->transform().position.x &&
            lhs->transform().position.y == rhs->transform().position.y &&
            lhs->transform().position.z == rhs->transform().position.z,
        "identical fixed-step worlds produce bit-identical positions"
    );
    check(
        lhs->linearVelocity().x == rhs->linearVelocity().x &&
            lhs->linearVelocity().y == rhs->linearVelocity().y &&
            lhs->linearVelocity().z == rhs->linearVelocity().z,
        "identical fixed-step worlds produce bit-identical velocities"
    );
}

void testLiveFrequencyChangePreservesAccumulatorPhase() {
    World world{};
    makeStillAir(world);
    world.environment().setGravity({});

    check(
        nearlyEqual(world.interpolationAlpha(), 0.0F, 0.0F),
        "a new world has zero interpolation phase"
    );
    const double oldHalfTick = 0.5 / 120.0;
    check(
        world.advance(oldHalfTick) == 0,
        "half of the original fixed tick remains accumulated"
    );
    check(
        nearlyEqual(world.interpolationAlpha(), 0.5F, 0.00001F),
        "interpolation alpha exposes the accumulated fixed-tick phase"
    );

    PhysicsSettings settings = world.settings();
    settings.fixedUpdateHz = 480;
    world.setSettings(settings);
    check(
        nearlyEqual(world.interpolationAlpha(), 0.5F, 0.00001F),
        "frequency changes preserve interpolation phase"
    );

    const double newHalfTick = 0.5 / 480.0;
    check(
        world.advance(newHalfTick) == 1,
        "changing physics frequency preserves normalized accumulator phase"
    );
    check(
        world.debugStats().fixedTick == 1,
        "frequency edits do not burst stale accumulator time into extra ticks"
    );
    check(
        world.interpolationAlpha() >= 0.0F &&
            world.interpolationAlpha() <= 0.00001F,
        "interpolation phase wraps safely after a fixed tick"
    );
}

void testSweptThinWallAndAccelerationFromRest() {
    PhysicsSettings fastSettings{};
    fastSettings.solverSubsteps = 2;
    fastSettings.maximumAdaptiveSubsteps = 32;
    World fastWorld{fastSettings};
    makeStillAir(fastWorld);
    fastWorld.environment().setGravity({});

    BodyDescription thinWall =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    thinWall.motionType = MotionType::Static;
    thinWall.collider = Collider::makeBox({0.01F, 2.0F, 2.0F});
    thinWall.aerodynamics.enabled = false;
    thinWall.material.staticFriction = 0.0F;
    thinWall.material.dynamicFriction = 0.0F;
    thinWall.material.restitution = 0.0F;
    fastWorld.createBody(thinWall);

    BodyDescription projectile =
        BodyDescription::makeDenseCube({-1.0F, 0.0F, 0.0F});
    projectile.collider = Collider::makeBox({0.05F, 0.05F, 0.05F});
    projectile.mass = 1.0F;
    projectile.linearVelocity = {600.0F, 0.0F, 0.0F};
    projectile.aerodynamics.enabled = false;
    projectile.allowSleep = false;
    projectile.material.staticFriction = 0.0F;
    projectile.material.dynamicFriction = 0.0F;
    projectile.material.restitution = 0.0F;
    const BodyId projectileId = fastWorld.createBody(projectile);
    fastWorld.stepFixed();

    const RigidBody* stoppedProjectile = fastWorld.body(projectileId);
    check(
        stoppedProjectile->transform().position.x <= -0.055F,
        "swept SAT prevents a 0.1 m cube at 600 m/s crossing a 0.02 m wall (x=" +
            std::to_string(stoppedProjectile->transform().position.x) + ")"
    );
    check(
        std::abs(stoppedProjectile->linearVelocity().x) < 1.0F,
        "thin-wall impact removes the incoming normal speed (vx=" +
            std::to_string(stoppedProjectile->linearVelocity().x) + ")"
    );
    check(
        fastWorld.debugStats().internalSubsteps == 32,
        "thin-wall stress case deterministically reaches the adaptive cap"
    );
    check(
        isFinite(stoppedProjectile->transform().position) &&
            isFinite(stoppedProjectile->linearVelocity()),
        "thin-wall CCD remains finite"
    );

    World acceleratedWorld{};
    makeStillAir(acceleratedWorld);
    acceleratedWorld.environment().setGravity({});

    BodyDescription wall =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    wall.motionType = MotionType::Static;
    wall.collider = Collider::makeBox({0.05F, 2.0F, 2.0F});
    wall.aerodynamics.enabled = false;
    wall.material.staticFriction = 0.0F;
    wall.material.dynamicFriction = 0.0F;
    wall.material.restitution = 0.0F;
    acceleratedWorld.createBody(wall);

    BodyDescription accelerated =
        BodyDescription::makeDenseCube({-2.0F, 0.0F, 0.0F});
    accelerated.collider = Collider::makeBox({0.5F, 0.5F, 0.5F});
    accelerated.mass = 1.0F;
    accelerated.aerodynamics.enabled = false;
    accelerated.allowSleep = false;
    accelerated.material.staticFriction = 0.0F;
    accelerated.material.dynamicFriction = 0.0F;
    accelerated.material.restitution = 0.0F;
    const BodyId acceleratedId = acceleratedWorld.createBody(accelerated);
    acceleratedWorld.body(acceleratedId)->applyForce(
        {120'000.0F, 0.0F, 0.0F}
    );
    acceleratedWorld.stepFixed();

    const RigidBody* stoppedAccelerated =
        acceleratedWorld.body(acceleratedId);
    check(
        acceleratedWorld.debugStats().internalSubsteps > 2,
        "adaptive travel prediction includes acceleration from rest"
    );
    check(
        stoppedAccelerated->transform().position.x <= -0.5F,
        "force-accelerated cube cannot cross the static wall in one tick"
    );
    check(
        isFinite(stoppedAccelerated->transform().position) &&
            isFinite(stoppedAccelerated->linearVelocity()),
        "acceleration-from-rest impact remains finite"
    );
}

void testClippedCrossedBoxManifold() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 1;
    settings.maximumAdaptiveSubsteps = 1;
    settings.velocityIterations = 24;
    World world{settings};
    makeStillAir(world);
    world.environment().setGravity({});

    BodyDescription first =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    first.motionType = MotionType::Static;
    first.collider = Collider::makeBox({2.0F, 0.05F, 0.05F});
    first.aerodynamics.enabled = false;
    first.material.staticFriction = 0.0F;
    first.material.dynamicFriction = 0.0F;
    first.material.restitution = 0.0F;
    world.createBody(first);

    BodyDescription second =
        BodyDescription::makeDenseCube({0.0F, 0.08F, 0.0F});
    second.collider = Collider::makeBox({2.0F, 0.05F, 0.05F});
    second.transform.orientation =
        Quaternion::fromAxisAngle({0.0F, 1.0F, 0.0F}, radians(90.0F));
    second.mass = 1.0F;
    second.aerodynamics.enabled = false;
    second.allowSleep = false;
    second.material.staticFriction = 0.0F;
    second.material.dynamicFriction = 0.0F;
    second.material.restitution = 0.0F;
    const BodyId secondId = world.createBody(second);
    world.stepFixed();

    const auto& contacts = world.debugContacts();
    check(
        contacts.size() >= 2 && contacts.size() <= 4,
        "crossed box faces produce a clipped multi-point manifold"
    );
    bool contactsStayOnIntersection = !contacts.empty();
    for (const ContactDebugPoint& contact : contacts) {
        contactsStayOnIntersection &=
            std::abs(contact.point.x) <= 0.06F &&
            std::abs(contact.point.z) <= 0.06F &&
            std::abs(contact.point.y - 0.04F) <= 0.012F;
    }
    check(
        contactsStayOnIntersection,
        "clipped contacts remain on the physical crossed-face intersection"
    );
    check(
        length(world.body(secondId)->angularVelocity()) < 0.05F,
        "symmetric crossed-face resolution does not create fallback torque"
    );
}

void testClosestEdgeBoxManifold() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 1;
    settings.maximumAdaptiveSubsteps = 1;
    settings.velocityIterations = 24;
    World world{settings};
    makeStillAir(world);
    world.environment().setGravity({});

    const Vec3 firstDirection{1.0F, 0.0F, 0.0F};
    const Vec3 secondDirection =
        normalized(Vec3{1.0F, 1.0F, 1.0F});
    const Vec3 edgeNormal =
        normalized(cross(firstDirection, secondDirection));

    BodyDescription first =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    first.motionType = MotionType::Static;
    first.collider = Collider::makeBox({2.0F, 0.05F, 0.05F});
    first.aerodynamics.enabled = false;
    first.material.staticFriction = 0.0F;
    first.material.dynamicFriction = 0.0F;
    first.material.restitution = 0.0F;
    world.createBody(first);

    BodyDescription second =
        BodyDescription::makeDenseCube(edgeNormal * 0.13F);
    second.collider = Collider::makeBox({2.0F, 0.05F, 0.05F});
    second.transform.orientation = Quaternion::fromAxisAngle(
        edgeNormal,
        std::acos(dot(firstDirection, secondDirection))
    );
    second.mass = 1.0F;
    second.aerodynamics.enabled = false;
    second.allowSleep = false;
    second.material.staticFriction = 0.0F;
    second.material.dynamicFriction = 0.0F;
    second.material.restitution = 0.0F;
    const BodyId secondId = world.createBody(second);
    world.stepFixed();

    const auto& contacts = world.debugContacts();
    check(
        contacts.size() == 1,
        "a genuine edge-edge SAT feature produces one closest-segment contact"
    );
    if (!contacts.empty()) {
        check(
            length(contacts.front().point - edgeNormal * 0.065F) < 0.15F,
            "edge-edge contact stays near the two finite supporting edges"
        );
    }
    check(
        length(world.body(secondId)->angularVelocity()) < 0.05F,
        "centered edge-edge resolution avoids arbitrary support-corner torque"
    );
}

void testPreviousTransformAndWorldInputSanitization() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 4;
    settings.maximumAdaptiveSubsteps = 16;
    World world{settings};
    makeStillAir(world);
    world.environment().setGravity({});

    BodyDescription moving =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    moving.mass = 1.0F;
    moving.linearVelocity = {12.0F, 0.0F, 0.0F};
    moving.aerodynamics.enabled = false;
    moving.allowSleep = false;
    const BodyId movingId = world.createBody(moving);
    world.stepFixed();
    check(
        nearlyEqual(
            world.body(movingId)->previousTransform().position.x,
            0.0F,
            0.00001F
        ),
        "previous transform remains the beginning of the fixed tick"
    );
    const Real firstTickPosition =
        world.body(movingId)->transform().position.x;
    world.stepFixed();
    check(
        nearlyEqual(
            world.body(movingId)->previousTransform().position.x,
            firstTickPosition,
            0.00001F
        ),
        "previous transform advances exactly once per fixed tick"
    );

    PhysicsSettings invalid = world.settings();
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    invalid.maximumTravelFraction = nan;
    invalid.maximumRotationStepRadians = nan;
    invalid.contactSlop = nan;
    invalid.restitutionVelocityThreshold = nan;
    invalid.penetrationVelocityFactor = nan;
    invalid.maximumDepenetrationVelocity = nan;
    invalid.positionCorrectionFraction = nan;
    invalid.sleepLinearSpeed = nan;
    invalid.sleepAngularSpeed = nan;
    invalid.sleepDelaySeconds = nan;
    invalid.sleepSupportMargin = nan;
    world.setSettings(invalid);
    check(
        std::isfinite(world.settings().maximumTravelFraction) &&
            std::isfinite(world.settings().maximumRotationStepRadians) &&
            std::isfinite(world.settings().contactSlop) &&
            std::isfinite(world.settings().penetrationVelocityFactor) &&
            std::isfinite(world.settings().sleepSupportMargin),
        "non-finite world settings are replaced with finite defaults"
    );

    SpringForce invalidSpring{};
    invalidSpring.body = movingId;
    invalidSpring.localAnchor = {nan, 0.0F, 0.0F};
    invalidSpring.worldTarget = {nan, nan, nan};
    invalidSpring.stiffness = nan;
    invalidSpring.damping = nan;
    invalidSpring.maximumForce = nan;
    world.createSpring(invalidSpring);
    world.stepFixed();
    check(
        isFinite(world.body(movingId)->transform().position) &&
            isFinite(world.body(movingId)->linearVelocity()),
        "non-finite spring input cannot poison body state"
    );
}

void testTimedBodyForceLifetimeAndAdaptiveSubsteps() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 8;
    settings.maximumAdaptiveSubsteps = 32;
    World world{settings};
    makeStillAir(world);
    world.environment().setGravity({});

    BodyDescription driven =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    driven.mass = 2.0F;
    driven.aerodynamics.enabled = false;
    driven.allowSleep = false;
    const BodyId drivenId = world.createBody(driven);

    constexpr Real forceDuration = 30.5F / 120.0F;
    TimedBodyForce drive{};
    drive.body = drivenId;
    drive.force = {10.0F, 0.0F, 0.0F};
    drive.torque = {0.0F, 0.0F, 4.0F};
    drive.remainingSeconds = forceDuration;
    const TimedForceId driveId = world.createTimedForce(drive);
    check(
        driveId != kInvalidTimedForceId,
        "a valid timed body force returns a usable handle"
    );

    for (int tick = 0; tick < 31; ++tick) {
        world.stepFixed();
    }
    const RigidBody* drivenBody = world.body(drivenId);
    const Real expectedLinearSpeed =
        drive.force.x * forceDuration / driven.mass;
    const Real expectedAngularMomentum =
        drive.torque.z * forceDuration;
    check(
        nearlyEqual(
            drivenBody->linearVelocity().x,
            expectedLinearSpeed,
            0.0003F
        ),
        "timed force preserves exact impulse through eight solver substeps"
    );
    check(
        nearlyEqual(
            drivenBody->worldAngularMomentum().z,
            expectedAngularMomentum,
            0.0003F
        ),
        "timed torque preserves exact angular impulse through substeps"
    );
    check(
        !world.destroyTimedForce(driveId),
        "a timed force handle expires after its simulation duration"
    );
    TimedBodyForce laterForce = drive;
    laterForce.remainingSeconds = 1.0F;
    laterForce.enabled = false;
    const TimedForceId laterId = world.createTimedForce(laterForce);
    check(
        laterId != kInvalidTimedForceId &&
            laterId != driveId &&
            !world.destroyTimedForce(driveId) &&
            world.destroyTimedForce(laterId),
        "expired timed-force handles never alias later force generators"
    );

    const Vec3 velocityAfterExpiry = drivenBody->linearVelocity();
    const Vec3 momentumAfterExpiry = drivenBody->worldAngularMomentum();
    world.stepFixed();
    check(
        nearlyEqual(
            world.body(drivenId)->linearVelocity().x,
            velocityAfterExpiry.x,
            0.00001F
        ) &&
            nearlyEqual(
                world.body(drivenId)->worldAngularMomentum().z,
                momentumAfterExpiry.z,
                0.00001F
            ),
        "expired timed force and torque do not leak into later ticks"
    );

    PhysicsSettings adaptiveSettings{};
    adaptiveSettings.solverSubsteps = 1;
    adaptiveSettings.maximumAdaptiveSubsteps = 32;
    World adaptiveWorld{adaptiveSettings};
    makeStillAir(adaptiveWorld);
    adaptiveWorld.environment().setGravity({});

    BodyDescription accelerated =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    accelerated.collider =
        Collider::makeBox({0.05F, 0.05F, 0.05F});
    accelerated.mass = 1.0F;
    accelerated.aerodynamics.enabled = false;
    accelerated.allowSleep = false;
    const BodyId acceleratedId =
        adaptiveWorld.createBody(accelerated);

    TimedBodyForce adaptiveForce{};
    adaptiveForce.body = acceleratedId;
    adaptiveForce.force = {120'000.0F, 0.0F, 0.0F};
    adaptiveForce.remainingSeconds = 1.0F / 120.0F;
    check(
        adaptiveWorld.createTimedForce(adaptiveForce) !=
            kInvalidTimedForceId,
        "adaptive stress force is accepted"
    );
    adaptiveWorld.stepFixed();
    check(
        adaptiveWorld.debugStats().internalSubsteps > 1,
        "timed force participates in adaptive travel prediction"
    );
    check(
        nearlyEqual(
            adaptiveWorld.body(acceleratedId)->linearVelocity().x,
            1'000.0F,
            0.05F
        ),
        "adaptive substeps neither multiply nor dilute timed force impulse"
    );
}

void testBodyLocalTimedForceLifecycleAndValidation() {
    World world{};
    makeStillAir(world);
    world.environment().setGravity({});
    world.createBody(BodyDescription::makeStaticPlane());

    BodyDescription bodyDescription =
        BodyDescription::makeDenseCube({0.0F, 0.5F, 0.0F});
    bodyDescription.transform.orientation =
        Quaternion::fromAxisAngle(
            {0.0F, 0.0F, 1.0F},
            radians(90.0F)
        );
    bodyDescription.mass = 2.0F;
    bodyDescription.aerodynamics.enabled = false;
    bodyDescription.allowSleep = true;
    const BodyId bodyId = world.createBody(bodyDescription);

    for (int tick = 0; tick < 90; ++tick) {
        world.stepFixed();
    }
    check(
        world.body(bodyId)->isSleeping(),
        "quiet body sleeps before a timed force is attached"
    );

    TimedBodyForce local{};
    local.body = bodyId;
    local.force = {12.0F, 0.0F, 0.0F};
    local.torque = {2.0F, 0.0F, 0.0F};
    local.remainingSeconds = 1.0F / 120.0F;
    local.forceInBodyFrame = true;
    local.torqueInBodyFrame = true;
    const TimedForceId localId = world.createTimedForce(local);
    check(
        localId != kInvalidTimedForceId &&
            !world.body(bodyId)->isSleeping(),
        "creating a timed force wakes its dynamic body"
    );

    world.stepFixed();
    const RigidBody* localBody = world.body(bodyId);
    check(
        std::abs(localBody->linearVelocity().x) < 0.0001F &&
            nearlyEqual(
                localBody->linearVelocity().y,
                0.05F,
                0.0002F
            ),
        "body-local force rotates into the current world frame"
    );
    check(
        std::abs(localBody->worldAngularMomentum().x) < 0.0001F &&
            nearlyEqual(
                localBody->worldAngularMomentum().y,
                2.0F / 120.0F,
                0.0002F
            ),
        "body-local torque rotates into the current world frame"
    );

    TimedBodyForce paused{};
    paused.body = bodyId;
    paused.force = {24.0F, 0.0F, 0.0F};
    paused.remainingSeconds = 2.0F / 120.0F;
    paused.enabled = false;
    const TimedForceId pausedId = world.createTimedForce(paused);
    const Real speedBeforePause = world.body(bodyId)->linearVelocity().x;
    for (int tick = 0; tick < 4; ++tick) {
        world.stepFixed();
    }
    check(
        nearlyEqual(
            world.body(bodyId)->linearVelocity().x,
            speedBeforePause,
            0.00001F
        ),
        "disabled timed force pauses without consuming its duration"
    );

    paused.enabled = true;
    check(
        world.updateTimedForce(pausedId, paused),
        "an existing timed force can be updated and enabled"
    );
    world.stepFixed();
    world.stepFixed();
    check(
        nearlyEqual(
            world.body(bodyId)->linearVelocity().x - speedBeforePause,
            0.2F,
            0.0003F
        ),
        "updated world-frame force runs for its requested duration"
    );
    check(
        !world.updateTimedForce(pausedId, paused),
        "an expired timed force cannot be updated through a stale handle"
    );

    TimedBodyForce cancellable{};
    cancellable.body = bodyId;
    cancellable.force = {1.0F, 0.0F, 0.0F};
    cancellable.remainingSeconds = 10.0F;
    const TimedForceId cancellableId =
        world.createTimedForce(cancellable);
    check(
        world.destroyTimedForce(cancellableId) &&
            !world.destroyTimedForce(cancellableId),
        "timed force destruction is explicit and idempotence-safe"
    );

    const TimedForceId bodyCleanupId =
        world.createTimedForce(cancellable);
    check(world.destroyBody(bodyId), "timed-force target body can be destroyed");
    check(
        !world.destroyTimedForce(bodyCleanupId),
        "destroying a body cleans up its timed forces"
    );

    BodyDescription replacement =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    replacement.aerodynamics.enabled = false;
    const BodyId replacementId = world.createBody(replacement);
    cancellable.body = replacementId;
    const TimedForceId resetCleanupId =
        world.createTimedForce(cancellable);
    world.reset();
    check(
        !world.destroyTimedForce(resetCleanupId),
        "world reset clears every timed force handle"
    );

    World validationWorld{};
    BodyDescription validBody =
        BodyDescription::makeDenseCube({0.0F, 0.0F, 0.0F});
    const BodyId validBodyId = validationWorld.createBody(validBody);
    TimedBodyForce invalid{};
    invalid.body = validBodyId;
    invalid.force = {
        std::numeric_limits<Real>::quiet_NaN(),
        0.0F,
        0.0F
    };
    invalid.remainingSeconds = 1.0F;
    check(
        validationWorld.createTimedForce(invalid) ==
            kInvalidTimedForceId,
        "non-finite timed force input is rejected"
    );
    invalid.force = {};
    invalid.remainingSeconds = 0.0F;
    check(
        validationWorld.createTimedForce(invalid) ==
            kInvalidTimedForceId,
        "non-positive timed force duration is rejected"
    );
}

} // namespace

int main() {
    testVelocityVerletFreeFall();
    testDenseCubeIsNotFloatyAndSettles();
    testSubtleWindDoesNotSweepTheDenseCubeAcrossTheGround();
    testStaticAndDynamicFriction();
    testRollingResistance();
    testProjectedAreaAndRelativeWindDrag();
    testRestitutionAndHighSpeedAntiTunneling();
    testOrientedBoxCollisionAndRaycast();
    testSpringAndExternalForceLifetime();
    testDeterministicFixedStep();
    testLiveFrequencyChangePreservesAccumulatorPhase();
    testSweptThinWallAndAccelerationFromRest();
    testClippedCrossedBoxManifold();
    testClosestEdgeBoxManifold();
    testPreviousTransformAndWorldInputSanitization();
    testTimedBodyForceLifetimeAndAdaptiveSubsteps();
    testBodyLocalTimedForceLifecycleAndValidation();

    if (failures != 0) {
        std::cerr << failures << " physics test(s) failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "All UAView Studio physics tests passed.\n";
    return EXIT_SUCCESS;
}
