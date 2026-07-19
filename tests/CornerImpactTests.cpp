#include <uaview/physics/World.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

namespace {

using namespace uaview::physics;
using Clock = std::chrono::steady_clock;

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

bool finiteBody(const RigidBody& body) {
    return isFinite(body.transform().position) &&
           isFinite(body.transform().orientation) &&
           isFinite(body.linearVelocity()) &&
           isFinite(body.angularVelocity()) &&
           isFinite(body.worldAngularMomentum());
}

struct ScenarioResult {
    double maximumStepMilliseconds{0.0};
    double impactStepMilliseconds{0.0};
    std::uint32_t maximumSubsteps{0U};
    std::size_t maximumRotationMicrosteps{0U};
    std::size_t rotationCapHits{0U};
    std::size_t rotationalCcdHits{0U};
    std::size_t maximumRotationalCcdAdvances{0U};
    std::size_t rotationalCcdConvergenceHits{0U};
    std::size_t rotationalCcdAdvanceCapHits{0U};
    bool sawContact{false};
    bool remainedFinite{true};
    Real minimumVertexHeight{std::numeric_limits<Real>::max()};
};

ScenarioResult runCornerImpactScenario(
    Real tiltXDegrees,
    Real tiltZDegrees,
    Real initialSpinRadiansPerSecond,
    Real downwardSpeed
) {
    PhysicsSettings settings{};
    settings.fixedUpdateHz = 120U;
    settings.solverSubsteps = 2U;
    settings.maximumAdaptiveSubsteps = 32U;
    settings.maximumRotationMicrosteps = 64U;
    World world{settings};

    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);

    BodyDescription ground = BodyDescription::makeStaticPlane();
    ground.material.staticFriction = 0.82F;
    ground.material.dynamicFriction = 0.68F;
    ground.material.restitution = 0.02F;
    world.createBody(ground);

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 2.2F, 0.0F});
    cube.mass = 120.0F;
    cube.allowSleep = false;
    cube.aerodynamics.enabled = false;
    cube.material.staticFriction = 0.65F;
    cube.material.dynamicFriction = 0.50F;
    cube.material.restitution = 0.05F;
    cube.transform.orientation =
        (
            Quaternion::fromAxisAngle(
                {1.0F, 0.0F, 0.0F},
                radians(tiltXDegrees)
            ) *
            Quaternion::fromAxisAngle(
                {0.0F, 0.0F, 1.0F},
                radians(tiltZDegrees)
            )
        ).normalizedValue();
    cube.linearVelocity = {0.0F, downwardSpeed, 0.0F};
    cube.angularVelocity = {
        initialSpinRadiansPerSecond * 0.37F,
        initialSpinRadiansPerSecond * 0.21F,
        initialSpinRadiansPerSecond
    };
    const BodyId cubeId = world.createBody(cube);

    ScenarioResult result{};
    for (int tick = 0; tick < 480; ++tick) {
        const auto begin = Clock::now();
        world.stepFixed();
        const double milliseconds =
            std::chrono::duration<double, std::milli>(
                Clock::now() - begin
            ).count();
        result.maximumStepMilliseconds =
            std::max(result.maximumStepMilliseconds, milliseconds);

        const PhysicsDebugStats& stats = world.debugStats();
        result.maximumSubsteps =
            std::max(result.maximumSubsteps, stats.internalSubsteps);
        result.maximumRotationMicrosteps = std::max(
            result.maximumRotationMicrosteps,
            stats.rotationMicrosteps
        );
        result.rotationCapHits += stats.rotationMicrostepCapHits;
        result.rotationalCcdHits += stats.rotationalCcdHits;
        result.maximumRotationalCcdAdvances = std::max(
            result.maximumRotationalCcdAdvances,
            stats.rotationalCcdAdvances
        );
        result.rotationalCcdConvergenceHits +=
            stats.rotationalCcdConvergenceHits;
        result.rotationalCcdAdvanceCapHits +=
            stats.rotationalCcdAdvanceCapHits;
        if (stats.contactCount > 0U) {
            if (!result.sawContact) {
                result.impactStepMilliseconds = milliseconds;
            }
            result.sawContact = true;
        }

        const RigidBody* body = world.body(cubeId);
        if (body == nullptr || !finiteBody(*body)) {
            result.remainedFinite = false;
            break;
        }
        for (const Vec3& vertex : body->boxWorldVertices()) {
            result.minimumVertexHeight =
                std::min(result.minimumVertexHeight, vertex.y);
        }
    }
    return result;
}

void testCornerImpactsRemainFiniteAndBounded() {
    constexpr Real tilts[][2]{
        {35.264F, 45.0F},
        {17.0F, 44.5F},
        {44.9F, 1.0F},
        {61.0F, 29.0F},
    };
    constexpr Real spins[]{0.0F, 8.0F, 24.0F, 60.0F};
    constexpr Real downwardSpeeds[]{-2.0F, -8.0F, -25.0F};

    double slowestStep = 0.0;
    double slowestImpact = 0.0;
    std::uint32_t peakSubsteps = 0U;
    std::size_t peakRotationMicrosteps = 0U;
    std::size_t totalRotationCapHits = 0U;
    std::size_t totalRotationalCcdHits = 0U;

    for (const auto& tilt : tilts) {
        for (Real spin : spins) {
            for (Real downwardSpeed : downwardSpeeds) {
                const ScenarioResult result = runCornerImpactScenario(
                    tilt[0],
                    tilt[1],
                    spin,
                    downwardSpeed
                );
                check(
                    result.remainedFinite,
                    "corner impact remains finite"
                );
                check(
                    result.sawContact,
                    "corner-first drop reaches the ground"
                );
                check(
                    result.minimumVertexHeight > -0.08F,
                    "corner impact does not tunnel materially through the plane"
                );
                slowestStep =
                    std::max(slowestStep, result.maximumStepMilliseconds);
                slowestImpact =
                    std::max(slowestImpact, result.impactStepMilliseconds);
                peakSubsteps =
                    std::max(peakSubsteps, result.maximumSubsteps);
                peakRotationMicrosteps = std::max(
                    peakRotationMicrosteps,
                    result.maximumRotationMicrosteps
                );
                totalRotationCapHits += result.rotationCapHits;
                totalRotationalCcdHits += result.rotationalCcdHits;
            }
        }
    }

    std::cout
        << "corner-impact profile: max step " << slowestStep
        << " ms, impact " << slowestImpact
        << " ms, substeps " << peakSubsteps
        << ", rotation microsteps " << peakRotationMicrosteps
        << ", rotation cap hits " << totalRotationCapHits
        << ", rotational CCD hits " << totalRotationalCcdHits
        << '\n';
}

void testGrazingCornerCcdHasBoundedWork() {
    const ScenarioResult result =
        runCornerImpactScenario(55.0F, 75.0F, 0.0F, -20.0F);
    check(result.remainedFinite, "grazing corner CCD remains finite");
    check(result.sawContact, "grazing corner CCD reaches contact");
    check(
        result.minimumVertexHeight > -0.08F,
        "grazing corner CCD remains above the anti-tunneling bound"
    );
    check(
        result.maximumRotationalCcdAdvances <= 224U,
        "grazing corner CCD obeys its bounded advance budget"
    );
    check(
        result.rotationalCcdAdvanceCapHits <= 64U,
        "grazing contact keeps conservative-to-sampled fallbacks bounded"
    );
    check(
        result.rotationalCcdConvergenceHits > 0U,
        "grazing contact exercises the finite-distance convergence path"
    );
    std::cout
        << "grazing-CCD profile: max advances "
        << result.maximumRotationalCcdAdvances
        << ", convergence hits " << result.rotationalCcdConvergenceHits
        << ", advance caps " << result.rotationalCcdAdvanceCapHits
        << '\n';
}

Quaternion exactCornerOrientation() {
    const Vec3 localCorner = normalized(Vec3{-1.0F, -1.0F, -1.0F});
    const Vec3 worldDown{0.0F, -1.0F, 0.0F};
    const Vec3 rotationAxis = normalized(cross(localCorner, worldDown));
    const Real rotationAngle = std::acos(
        std::clamp(dot(localCorner, worldDown), -1.0F, 1.0F)
    );
    return Quaternion::fromAxisAngle(rotationAxis, rotationAngle);
}

void testExactCornerCannotEnterSleep() {
    World world{};
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    world.createBody(BodyDescription::makeStaticPlane());

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 2.0F, 0.0F});
    cube.mass = 120.0F;
    cube.aerodynamics.enabled = false;
    cube.transform.orientation = exactCornerOrientation();
    const BodyId cubeId = world.createBody(cube);

    std::size_t singlePointContactTicks = 0U;
    for (int tick = 0; tick < 1'200; ++tick) {
        world.stepFixed();
        if (world.debugStats().contactCount == 1U) {
            ++singlePointContactTicks;
        }
    }
    const RigidBody* body = world.body(cubeId);
    check(body != nullptr, "exact-corner body remains valid");
    if (body != nullptr) {
        check(
            !body->isSleeping(),
            "a zero-area exact-corner support cannot enter sleep"
        );
        std::cout
            << "exact-corner profile: sleeping "
            << (body->isSleeping() ? "yes" : "no")
            << ", single-contact ticks " << singlePointContactTicks
            << ", speed " << length(body->linearVelocity())
            << ", angular " << length(body->angularVelocity())
            << ", center y " << body->transform().position.y
            << '\n';
    }
}

void testMarginalCornerTopplesBeforeSleeping() {
    World world{};
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
    world.createBody(BodyDescription::makeStaticPlane());

    BodyDescription cube = BodyDescription::makeDenseCube({0.0F, 2.0F, 0.0F});
    cube.mass = 120.0F;
    cube.aerodynamics.enabled = false;
    cube.transform.orientation =
        (
            Quaternion::fromAxisAngle(
                {1.0F, 0.0F, 0.0F},
                radians(0.5F)
            ) *
            exactCornerOrientation()
        ).normalizedValue();
    const BodyId cubeId = world.createBody(cube);

    bool sleptOnDegenerateSupport = false;
    std::size_t maximumContacts = 0U;
    for (int tick = 0; tick < 2'000; ++tick) {
        world.stepFixed();
        const RigidBody* body = world.body(cubeId);
        check(body != nullptr && finiteBody(*body), "marginal corner remains finite");
        if (body == nullptr || !finiteBody(*body)) {
            break;
        }
        const std::size_t contacts = world.debugStats().contactCount;
        maximumContacts = std::max(maximumContacts, contacts);
        if (contacts < 3U && body->isSleeping()) {
            sleptOnDegenerateSupport = true;
        }
    }

    const RigidBody* body = world.body(cubeId);
    check(
        !sleptOnDegenerateSupport,
        "a marginal corner cannot freeze before it topples"
    );
    check(
        body != nullptr && maximumContacts >= 3U,
        "a marginal corner eventually reaches an area support"
    );
    check(
        body != nullptr &&
            body->transform().position.y > 0.45F &&
            body->transform().position.y < 0.55F,
        "the toppled cube settles on a face at its physical half-height"
    );
    check(
        body != nullptr && body->isSleeping(),
        "the cube may sleep after its support polygon is stable"
    );
}

void testRestingContactIslandCanSleepAndWake() {
    World world{};
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
    world.createBody(BodyDescription::makeStaticPlane());

    BodyDescription lower =
        BodyDescription::makeDenseCube({0.0F, 0.5F, 0.0F});
    lower.mass = 120.0F;
    lower.aerodynamics.enabled = false;
    const BodyId lowerId = world.createBody(lower);
    BodyDescription upper =
        BodyDescription::makeDenseCube({0.0F, 1.5F, 0.0F});
    upper.mass = 120.0F;
    upper.aerodynamics.enabled = false;
    const BodyId upperId = world.createBody(upper);

    for (int tick = 0; tick < 600; ++tick) {
        world.stepFixed();
    }
    check(
        world.body(lowerId)->isSleeping() &&
            world.body(upperId)->isSleeping(),
        "a quiet two-cube contact island can enter sleep"
    );

    RigidBody* upperBody = world.body(upperId);
    upperBody->applyImpulseAtWorldPoint(
        {30.0F, 0.0F, 0.0F},
        upperBody->transform().position
    );
    world.stepFixed();
    check(
        !world.body(lowerId)->isSleeping() &&
            !world.body(upperId)->isSleeping(),
        "an impact wakes every connected body in a two-cube island"
    );
}

Real faceTiltDegrees(const RigidBody& body) {
    const Vec3 faceNormal =
        body.transform().orientation.rotate({0.0F, 1.0F, 0.0F});
    return std::acos(std::clamp(
        dot(faceNormal, Vec3{0.0F, 1.0F, 0.0F}),
        -1.0F,
        1.0F
    )) * (180.0F / kPi);
}

void testUserThinPlateFallsFullyOntoItsFace() {
    World world{};
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 1.225F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
    world.createBody(BodyDescription::makeStaticPlane());

    constexpr Vec3 halfExtents{1.2F, 0.005F, 1.0F};
    constexpr Real initialTilt = 80.0F;
    const Real tiltRadians = radians(initialTilt);
    const Real projectedHalfHeight =
        halfExtents.x * std::abs(std::sin(tiltRadians)) +
        halfExtents.y * std::abs(std::cos(tiltRadians));

    BodyDescription plate = BodyDescription::makeDenseCube(
        {0.0F, projectedHalfHeight + 0.0005F, 0.0F}
    );
    plate.collider = Collider::makeBox(halfExtents);
    plate.mass = 1.0F;
    plate.aerodynamics.enabled = true;
    plate.transform.orientation =
        Quaternion::fromAxisAngle(
            {0.0F, 0.0F, 1.0F},
            tiltRadians
        );
    const BodyId plateId = world.createBody(plate);

    Real minimumTilt = initialTilt;
    std::size_t peakCcdAdvances = 0U;
    std::uint32_t peakSubsteps = 0U;
    std::size_t totalCcdCaps = 0U;
    double slowestStepMilliseconds = 0.0;
    for (int tick = 0; tick < 1'200; ++tick) {
        const auto begin = Clock::now();
        world.stepFixed();
        slowestStepMilliseconds = std::max(
            slowestStepMilliseconds,
            std::chrono::duration<double, std::milli>(
                Clock::now() - begin
            ).count()
        );
        const RigidBody* body = world.body(plateId);
        check(body != nullptr && finiteBody(*body), "thin plate remains finite");
        if (body == nullptr || !finiteBody(*body)) {
            break;
        }
        minimumTilt = std::min(minimumTilt, faceTiltDegrees(*body));
        const PhysicsDebugStats& stats = world.debugStats();
        peakCcdAdvances = std::max(
            peakCcdAdvances,
            stats.rotationalCcdAdvances
        );
        peakSubsteps = std::max(peakSubsteps, stats.internalSubsteps);
        totalCcdCaps += stats.rotationalCcdAdvanceCapHits;
    }

    const RigidBody* body = world.body(plateId);
    check(
        body != nullptr && minimumTilt < 2.0F,
        "2.4 x 0.010 x 2 m plate reaches its face instead of hovering"
    );
    check(
        body != nullptr &&
            body->transform().position.y > 0.003F &&
            body->transform().position.y < 0.02F,
        "thin plate settles at its physical half-thickness"
    );
    check(
        body != nullptr && body->isSleeping(),
        "thin plate sleeps only after reaching a stable face contact"
    );
    check(
        peakSubsteps <= 8U,
        "a supported 10 mm plate does not saturate the adaptive solver"
    );
    check(
        peakCcdAdvances <= 224U,
        "thin-plate CCD work remains bounded within the 120 Hz tick"
    );
    std::cout
        << "thin-plate profile: min/final tilt "
        << minimumTilt << '/' << (body != nullptr ? faceTiltDegrees(*body) : -1.0F)
        << " deg, center y "
        << (body != nullptr ? body->transform().position.y : -1.0F)
        << " m, peak substeps " << peakSubsteps
        << ", peak CCD advances " << peakCcdAdvances
        << ", CCD caps " << totalCcdCaps
        << ", max step " << slowestStepMilliseconds << " ms\n";
}

BodyId createClearancePlate(
    World& world,
    Real angularVelocityZ
) {
    constexpr Vec3 halfExtents{1.2F, 0.005F, 1.0F};
    constexpr Real tiltDegrees = 15.0F;
    const Real angle = radians(tiltDegrees);
    const Real support =
        halfExtents.x * std::abs(std::sin(angle)) +
        halfExtents.y * std::abs(std::cos(angle));

    BodyDescription plate = BodyDescription::makeDenseCube({
        0.0F,
        support + world.settings().contactSlop + 1.0e-7F,
        0.0F
    });
    plate.collider = Collider::makeBox(halfExtents);
    plate.mass = 1.0F;
    plate.allowSleep = false;
    plate.aerodynamics.enabled = false;
    plate.transform.orientation =
        Quaternion::fromAxisAngle(
            {0.0F, 0.0F, 1.0F},
            angle
        );
    plate.linearVelocity = {0.0F, -0.1F, 0.0F};
    plate.angularVelocity = {0.0F, 0.0F, angularVelocityZ};
    return world.createBody(plate);
}

void makeVacuumGround(World& world) {
    AtmosphereSample atmosphere = world.environment().atmosphere();
    atmosphere.density = 0.0F;
    world.environment().setAtmosphere(atmosphere);
    WindFieldSettings wind{};
    wind.meanVelocity = {};
    wind.turbulenceAmplitude = 0.0F;
    world.environment().setWindSettings(wind);
    world.createBody(BodyDescription::makeStaticPlane());
}

void testThinPlateClearMotionIsNotAFalseCcdHit() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 1U;
    settings.maximumAdaptiveSubsteps = 1U;
    World world{settings};
    makeVacuumGround(world);
    const BodyId plateId = createClearancePlate(world, -1.0F);

    std::size_t ccdHitsWithoutContact = 0U;
    for (int tick = 0; tick < 12; ++tick) {
        world.stepFixed();
        if (world.debugStats().rotationalCcdHits > 0U &&
            world.debugStats().contactCount == 0U) {
            ++ccdHitsWithoutContact;
        }
    }
    const RigidBody* plate = world.body(plateId);
    std::cout
        << "thin clear-path profile: false hits "
        << ccdHitsWithoutContact
        << ", final tilt "
        << (plate != nullptr ? faceTiltDegrees(*plate) : -1.0F)
        << " deg\n";
    check(
        ccdHitsWithoutContact == 0U,
        "a clear shrinking-support path is never reported as a CCD collision"
    );
    check(
        plate != nullptr && faceTiltDegrees(*plate) < 13.0F,
        "a thin plate rotating toward its face cannot freeze at 15 degrees"
    );
}

void testThinPlateCcdHitProducesImmediateContact() {
    PhysicsSettings settings{};
    settings.solverSubsteps = 1U;
    settings.maximumAdaptiveSubsteps = 1U;
    World world{settings};
    makeVacuumGround(world);
    const BodyId plateId = createClearancePlate(world, 1.0F);

    world.stepFixed();
    const RigidBody* plate = world.body(plateId);
    Real minimumVertexHeight = std::numeric_limits<Real>::max();
    if (plate != nullptr) {
        for (const Vec3& vertex : plate->boxWorldVertices()) {
            minimumVertexHeight =
                std::min(minimumVertexHeight, vertex.y);
        }
    }
    std::cout
        << "thin hit-handoff profile: hits "
        << world.debugStats().rotationalCcdHits
        << ", contacts " << world.debugStats().contactCount
        << ", advances " << world.debugStats().rotationalCcdAdvances
        << ", caps " << world.debugStats().rotationalCcdAdvanceCapHits
        << ", min y " << minimumVertexHeight
        << ", tilt "
        << (plate != nullptr ? faceTiltDegrees(*plate) : -1.0F)
        << " deg\n";
    check(
        world.debugStats().rotationalCcdHits > 0U,
        "a converging thin-plate path is caught by rotational CCD"
    );
    check(
        world.debugStats().contactCount > 0U,
        "every reported thin-plate CCD hit creates a same-substep manifold"
    );
    check(
        plate != nullptr &&
            minimumVertexHeight >= -0.01F &&
            minimumVertexHeight <= world.settings().contactSlop,
        "the CCD handoff installs the actual contact transform"
    );
}

} // namespace

int main() {
    testCornerImpactsRemainFiniteAndBounded();
    testGrazingCornerCcdHasBoundedWork();
    testExactCornerCannotEnterSleep();
    testMarginalCornerTopplesBeforeSleeping();
    testRestingContactIslandCanSleepAndWake();
    testUserThinPlateFallsFullyOntoItsFace();
    testThinPlateClearMotionIsNotAFalseCcdHit();
    testThinPlateCcdHitProducesImmediateContact();
    if (failures != 0) {
        std::cerr << failures << " corner-impact test(s) failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "All UAView Studio corner-impact tests passed.\n";
    return EXIT_SUCCESS;
}
