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

bool nearlyEqual(Real lhs, Real rhs, Real tolerance = 1.0e-5F) {
    return std::abs(lhs - rhs) <= tolerance;
}

bool nearlyEqual(const Vec3& lhs, const Vec3& rhs, Real tolerance = 1.0e-5F) {
    return nearlyEqual(lhs.x, rhs.x, tolerance) &&
           nearlyEqual(lhs.y, rhs.y, tolerance) &&
           nearlyEqual(lhs.z, rhs.z, tolerance);
}

void checkIdentity(const Mat3& matrix, Real tolerance, const std::string& message) {
    const bool matches =
        nearlyEqual(matrix.m00, 1.0F, tolerance) &&
        nearlyEqual(matrix.m01, 0.0F, tolerance) &&
        nearlyEqual(matrix.m02, 0.0F, tolerance) &&
        nearlyEqual(matrix.m10, 0.0F, tolerance) &&
        nearlyEqual(matrix.m11, 1.0F, tolerance) &&
        nearlyEqual(matrix.m12, 0.0F, tolerance) &&
        nearlyEqual(matrix.m20, 0.0F, tolerance) &&
        nearlyEqual(matrix.m21, 0.0F, tolerance) &&
        nearlyEqual(matrix.m22, 1.0F, tolerance);
    check(matches, message);
}

bool nearlyEqual(const Mat3& lhs, const Mat3& rhs, Real tolerance = 1.0e-5F) {
    return nearlyEqual(lhs.m00, rhs.m00, tolerance) &&
           nearlyEqual(lhs.m01, rhs.m01, tolerance) &&
           nearlyEqual(lhs.m02, rhs.m02, tolerance) &&
           nearlyEqual(lhs.m10, rhs.m10, tolerance) &&
           nearlyEqual(lhs.m11, rhs.m11, tolerance) &&
           nearlyEqual(lhs.m12, rhs.m12, tolerance) &&
           nearlyEqual(lhs.m20, rhs.m20, tolerance) &&
           nearlyEqual(lhs.m21, rhs.m21, tolerance) &&
           nearlyEqual(lhs.m22, rhs.m22, tolerance);
}

BodyId createOffsetBox(World& world, const Quaternion& orientation = {}) {
    BodyDescription description = BodyDescription::makeDenseCube({1.0F, 2.0F, 3.0F});
    description.collider = Collider::makeBox({1.0F, 2.0F, 3.0F});
    description.mass = 12.0F;
    description.centerOfMassLocal = {0.4F, -0.3F, 0.2F};
    description.transform.orientation = orientation;
    description.aerodynamics.enabled = false;
    description.allowSleep = false;
    return world.createBody(description);
}

void testFullParallelAxisTensor() {
    World world{};
    const RigidBody* body = world.body(createOffsetBox(world));
    check(body != nullptr, "offset box is created");
    if (body == nullptr) {
        return;
    }

    const Mat3& tensor = body->localInertiaTensor();
    check(isFinite(tensor), "local inertia tensor is finite");
    check(nearlyEqual(tensor.m00, 53.56F), "parallel-axis Ixx is correct");
    check(nearlyEqual(tensor.m11, 42.40F), "parallel-axis Iyy is correct");
    check(nearlyEqual(tensor.m22, 23.00F), "parallel-axis Izz is correct");
    check(nearlyEqual(tensor.m01, 1.44F), "parallel-axis Ixy is correct");
    check(nearlyEqual(tensor.m02, -0.96F), "parallel-axis Ixz is correct");
    check(nearlyEqual(tensor.m12, 0.72F), "parallel-axis Iyz is correct");
    check(
        tensor.m01 == tensor.m10 &&
            tensor.m02 == tensor.m20 &&
            tensor.m12 == tensor.m21,
        "inertia tensor is exactly symmetric"
    );

    const Vec3 diagonal = body->localInertia();
    check(
        nearlyEqual(diagonal, {tensor.m00, tensor.m11, tensor.m22}),
        "legacy localInertia accessor remains the tensor diagonal"
    );
    check(
        isFinite(body->inverseLocalInertiaTensor()),
        "inverse local inertia tensor is finite"
    );
    checkIdentity(
        tensor * body->inverseLocalInertiaTensor(),
        2.0e-5F,
        "analytic inverse multiplies back to identity"
    );
}

void testTensorCouplingAndWorldRotation() {
    World localWorld{};
    RigidBody* localBody = localWorld.body(createOffsetBox(localWorld));
    const Vec3 localPoint = localBody->transform().position + Vec3{0.0F, 1.0F, 0.0F};
    const Vec3 impulse{0.0F, 0.0F, 1.0F};
    const Vec3 localAngularImpulse = cross(
        localPoint - localBody->transform().position,
        impulse
    );
    const Vec3 expectedLocalAngularVelocity =
        localBody->inverseLocalInertiaTensor() * localAngularImpulse;
    localBody->applyImpulseAtWorldPoint(impulse, localPoint);
    check(
        nearlyEqual(localBody->angularVelocity(), expectedLocalAngularVelocity),
        "impulse response uses the complete local inverse tensor"
    );
    check(
        std::abs(localBody->angularVelocity().y) > 1.0e-4F ||
            std::abs(localBody->angularVelocity().z) > 1.0e-4F,
        "products of inertia couple a local x angular impulse into other axes"
    );

    World rotatedWorld{};
    const Quaternion orientation =
        Quaternion::fromAxisAngle(normalized({0.3F, 0.8F, -0.4F}), radians(37.0F));
    RigidBody* rotatedBody =
        rotatedWorld.body(createOffsetBox(rotatedWorld, orientation));
    const Vec3 leverArm{0.6F, -0.25F, 0.45F};
    const Vec3 worldImpulse{-0.2F, 0.75F, 1.1F};
    const Vec3 angularImpulse = cross(leverArm, worldImpulse);
    const Vec3 expectedWorldAngularVelocity = orientation.rotate(
        rotatedBody->inverseLocalInertiaTensor() *
        orientation.inverseRotate(angularImpulse)
    );
    rotatedBody->applyImpulseAtWorldPoint(
        worldImpulse,
        rotatedBody->transform().position + leverArm
    );
    check(
        nearlyEqual(
            rotatedBody->angularVelocity(),
            expectedWorldAngularVelocity,
            2.0e-5F
        ),
        "world inverse inertia applies R times I-local-inverse times R-transpose"
    );
}

void testExplicitInertiaTensorIsIndependentAndTransactional() {
    World world{};
    BodyDescription description = BodyDescription::makeDenseCube();
    description.mass = 10.0F;
    description.aerodynamics.enabled = false;
    description.allowSleep = false;
    description.useCustomLocalInertiaTensor = true;
    // Principal moments (2, 3, 4) rotated 45 degrees about local Z.
    description.customLocalInertiaTensor = {
        2.5F, -0.5F, 0.0F,
        -0.5F, 2.5F, 0.0F,
        0.0F, 0.0F, 4.0F,
    };
    RigidBody* body = world.body(world.createBody(description));
    check(body != nullptr, "explicit-inertia body is created");
    if (body == nullptr) {
        return;
    }
    const Mat3 expected = description.customLocalInertiaTensor;
    check(
        body->usesCustomLocalInertiaTensor(),
        "valid explicit inertia mode remains active"
    );
    check(
        nearlyEqual(body->lockedAssemblyInertiaTensor(), expected),
        "explicit inertia is stored as the locked assembly tensor"
    );

    body->applyAngularImpulse({1.0F, 0.0F, 0.0F});
    check(
        nearlyEqual(
            body->angularVelocity(),
            {0.4166667F, 0.0833333F, 0.0F},
            2.0e-5F
        ),
        "off-diagonal explicit inertia produces the analytic coupled response"
    );

    body->setBoxHalfExtents({3.0F, 0.25F, 7.0F});
    check(
        nearlyEqual(body->lockedAssemblyInertiaTensor(), expected),
        "collider resizing does not alter explicit mass properties"
    );

    const Mat3 beforeInvalidEdit = body->lockedAssemblyInertiaTensor();
    const Mat3 nonsymmetric{
        2.0F, 0.4F, 0.0F,
        0.0F, 2.0F, 0.0F,
        0.0F, 0.0F, 2.0F,
    };
    check(
        !body->setCustomLocalInertiaTensor(nonsymmetric),
        "nonsymmetric explicit inertia is rejected"
    );
    check(
        !body->setCustomLocalInertiaTensor(
            Mat3::diagonal({1.0F, 1.0F, 3.0F})
        ),
        "positive-definite but physically unrealizable inertia is rejected"
    );
    check(
        !body->setCustomLocalInertiaTensor(
            Mat3::diagonal({1.0F, -1.0F, 1.0F})
        ),
        "non-positive-definite inertia is rejected"
    );
    check(
        nearlyEqual(
            body->lockedAssemblyInertiaTensor(),
            beforeInvalidEdit
        ) &&
            body->usesCustomLocalInertiaTensor(),
        "invalid inertia edits leave the prior valid state untouched"
    );

    body->useAutomaticLocalInertiaTensor();
    check(
        !body->usesCustomLocalInertiaTensor() &&
            !nearlyEqual(
                body->lockedAssemblyInertiaTensor(),
                expected
            ),
        "automatic mode recomputes inertia from current box geometry"
    );
}

void testNonFiniteSetterContainment() {
    constexpr Real infinity = std::numeric_limits<Real>::infinity();
    constexpr Real nan = std::numeric_limits<Real>::quiet_NaN();

    World world{};
    RigidBody* body = world.body(createOffsetBox(world));
    const Real originalMass = body->mass();
    const Vec3 originalCenterOfMass = body->centerOfMassLocal();
    const Vec3 originalPosition = body->transform().position;
    const PhysicsMaterial originalMaterial = body->material();
    const AerodynamicProperties originalAerodynamics = body->aerodynamics();

    body->setMass(infinity);
    body->setMass(nan);
    body->setCenterOfMassLocal({nan, 0.0F, 0.0F});
    check(body->mass() == originalMass, "non-finite mass edits are rejected");
    check(
        nearlyEqual(body->centerOfMassLocal(), originalCenterOfMass),
        "non-finite center-of-mass edits are rejected"
    );

    body->setBoxHalfExtents({infinity, 4.0F, nan});
    check(
        isFinite(body->collider().box.halfExtents),
        "non-finite extent components cannot enter body state"
    );
    check(
        nearlyEqual(body->collider().box.halfExtents, {1.0F, 4.0F, 3.0F}),
        "invalid extent components preserve their prior values"
    );

    PhysicsMaterial invalidMaterial = originalMaterial;
    invalidMaterial.staticFriction = infinity;
    invalidMaterial.dynamicFriction = nan;
    invalidMaterial.restitution = -infinity;
    invalidMaterial.rollingFriction = nan;
    body->setMaterial(invalidMaterial);
    check(
        body->material().staticFriction == originalMaterial.staticFriction &&
            body->material().dynamicFriction == originalMaterial.dynamicFriction &&
            body->material().restitution == originalMaterial.restitution &&
            body->material().rollingFriction == originalMaterial.rollingFriction,
        "invalid material fields preserve the last finite values"
    );

    PhysicsMaterial clampedMaterial{};
    clampedMaterial.staticFriction = -1.0F;
    clampedMaterial.dynamicFriction = 4.0F;
    clampedMaterial.restitution = 2.0F;
    clampedMaterial.rollingFriction = -3.0F;
    body->setMaterial(clampedMaterial);
    check(
        body->material().staticFriction == 0.0F &&
            body->material().dynamicFriction == 0.0F &&
            body->material().restitution == 1.0F &&
            body->material().rollingFriction == 0.0F,
        "finite material fields retain the documented physical clamps"
    );

    AerodynamicProperties invalidAerodynamics = originalAerodynamics;
    invalidAerodynamics.enabled = false;
    invalidAerodynamics.dragCoefficient = infinity;
    invalidAerodynamics.angularDragCoefficient = nan;
    invalidAerodynamics.projectedAreaScale = -infinity;
    invalidAerodynamics.centerOfPressureLocal = {0.0F, nan, 0.0F};
    body->setAerodynamics(invalidAerodynamics);
    check(!body->aerodynamics().enabled, "finite aerodynamic toggles remain editable");
    check(
        body->aerodynamics().dragCoefficient ==
                originalAerodynamics.dragCoefficient &&
            body->aerodynamics().angularDragCoefficient ==
                originalAerodynamics.angularDragCoefficient &&
            body->aerodynamics().projectedAreaScale ==
                originalAerodynamics.projectedAreaScale &&
            nearlyEqual(
                body->aerodynamics().centerOfPressureLocal,
                originalAerodynamics.centerOfPressureLocal
            ),
        "invalid aerodynamic fields preserve the last finite values"
    );

    Transform invalidTransform = body->transform();
    invalidTransform.position = {infinity, 4.0F, 5.0F};
    invalidTransform.orientation = {nan, 0.0F, 0.0F, 0.0F};
    body->setTransform(invalidTransform);
    check(
        nearlyEqual(body->transform().position, originalPosition),
        "non-finite transform position is rejected"
    );
    check(
        isFinite(body->transform().orientation) &&
            isFinite(body->previousTransform().orientation),
        "non-finite orientation is contained in current and previous transforms"
    );
    check(
        isFinite(body->localInertiaTensor()) &&
            isFinite(body->inverseLocalInertiaTensor()),
        "invalid public edits cannot poison mass properties"
    );

    const Collider invalidBox = Collider::makeBox({infinity, nan, 0.0F});
    const Collider invalidPlane =
        Collider::makePlane({nan, 0.0F, 0.0F}, infinity);
    check(
        isFinite(invalidBox.box.halfExtents),
        "box factory contains non-finite dimensions"
    );
    check(
        isFinite(invalidPlane.plane.normal) &&
            std::isfinite(invalidPlane.plane.offset),
        "plane factory contains non-finite parameters"
    );
}

void testNonFiniteDescriptionContainment() {
    constexpr Real infinity = std::numeric_limits<Real>::infinity();
    constexpr Real nan = std::numeric_limits<Real>::quiet_NaN();

    World world{};
    BodyDescription description = BodyDescription::makeDenseCube();
    description.mass = infinity;
    description.collider.box.halfExtents = {infinity, nan, 0.5F};
    description.transform.position = {nan, 2.0F, 3.0F};
    description.transform.orientation = {infinity, 0.0F, 0.0F, 0.0F};
    description.linearVelocity = {0.0F, infinity, 0.0F};
    description.angularVelocity = {nan, 0.0F, 0.0F};
    description.centerOfMassLocal = {0.0F, nan, 0.0F};
    description.material.staticFriction = infinity;
    description.aerodynamics.dragCoefficient = nan;

    const RigidBody* body = world.body(world.createBody(description));
    check(std::isfinite(body->mass()), "description mass is made finite");
    check(isFinite(body->collider().box.halfExtents), "description extents are finite");
    check(isFinite(body->transform().position), "description position is finite");
    check(isFinite(body->transform().orientation), "description orientation is finite");
    check(isFinite(body->previousTransform().position), "previous position is finite");
    check(isFinite(body->previousTransform().orientation), "previous orientation is finite");
    check(isFinite(body->linearVelocity()), "description linear velocity is finite");
    check(isFinite(body->angularVelocity()), "description angular velocity is finite");
    check(isFinite(body->localInertiaTensor()), "description inertia is finite");
    check(isFinite(body->inverseLocalInertiaTensor()), "description inverse inertia is finite");
    check(
        std::isfinite(body->material().staticFriction),
        "description material is finite"
    );
    check(
        std::isfinite(body->aerodynamics().dragCoefficient),
        "description aerodynamics are finite"
    );
}

} // namespace

int main() {
    testFullParallelAxisTensor();
    testTensorCouplingAndWorldRotation();
    testExplicitInertiaTensorIsIndependentAndTransactional();
    testNonFiniteSetterContainment();
    testNonFiniteDescriptionContainment();

    if (failures != 0) {
        std::cerr << failures << " mass-properties test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All UAView Studio mass-properties tests passed.\n";
    return EXIT_SUCCESS;
}
