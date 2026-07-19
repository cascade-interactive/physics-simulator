#include <uaview/physics/RigidBody.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace uaview::physics {
namespace {

constexpr Real kMinimumHalfExtent = 0.0025F;
constexpr Real kMinimumDynamicMass = 0.001F;

Real finiteNonnegativeOr(Real value, Real fallback) noexcept {
    const Real safeFallback =
        std::isfinite(fallback) ? std::max(0.0F, fallback) : 0.0F;
    return std::isfinite(value) ? std::max(0.0F, value) : safeFallback;
}

Real finiteUnitOr(Real value, Real fallback) noexcept {
    const Real safeFallback =
        std::isfinite(fallback) ? std::clamp(fallback, 0.0F, 1.0F) : 0.0F;
    return std::isfinite(value) ? std::clamp(value, 0.0F, 1.0F) : safeFallback;
}

Real sanitizedHalfExtent(Real value, Real fallback) noexcept {
    if (!std::isfinite(value)) {
        value = std::isfinite(fallback) ? fallback : 0.5F;
    }
    return std::max(kMinimumHalfExtent, std::abs(value));
}

Vec3 sanitizedHalfExtents(
    const Vec3& halfExtents,
    const Vec3& fallback = {0.5F, 0.5F, 0.5F}
) noexcept {
    return {
        sanitizedHalfExtent(halfExtents.x, fallback.x),
        sanitizedHalfExtent(halfExtents.y, fallback.y),
        sanitizedHalfExtent(halfExtents.z, fallback.z),
    };
}

PhysicsMaterial sanitizedMaterial(
    PhysicsMaterial material,
    const PhysicsMaterial& fallback = {}
) noexcept {
    material.staticFriction =
        finiteNonnegativeOr(material.staticFriction, fallback.staticFriction);
    material.dynamicFriction = std::min(
        finiteNonnegativeOr(material.dynamicFriction, fallback.dynamicFriction),
        material.staticFriction
    );
    material.restitution =
        finiteUnitOr(material.restitution, fallback.restitution);
    material.rollingFriction =
        finiteNonnegativeOr(material.rollingFriction, fallback.rollingFriction);
    return material;
}

AerodynamicProperties sanitizedAerodynamics(
    AerodynamicProperties properties,
    const AerodynamicProperties& fallback = {}
) noexcept {
    properties.dragCoefficient =
        finiteNonnegativeOr(properties.dragCoefficient, fallback.dragCoefficient);
    properties.angularDragCoefficient = finiteNonnegativeOr(
        properties.angularDragCoefficient,
        fallback.angularDragCoefficient
    );
    properties.projectedAreaScale = finiteNonnegativeOr(
        properties.projectedAreaScale,
        fallback.projectedAreaScale
    );
    if (!isFinite(properties.centerOfPressureLocal)) {
        properties.centerOfPressureLocal =
            isFinite(fallback.centerOfPressureLocal)
                ? fallback.centerOfPressureLocal
                : Vec3{};
    }
    return properties;
}

Real saturatedReal(double value) noexcept {
    constexpr double maximum = static_cast<double>(std::numeric_limits<Real>::max());
    if (std::isnan(value)) {
        return 0.0F;
    }
    return static_cast<Real>(std::clamp(value, -maximum, maximum));
}

bool invertSymmetricPositiveDefinite(
    double m00,
    double m01,
    double m02,
    double m11,
    double m12,
    double m22,
    Mat3& inverse
) noexcept {
    const double scale = std::max({
        std::abs(m00),
        std::abs(m01),
        std::abs(m02),
        std::abs(m11),
        std::abs(m12),
        std::abs(m22),
    });
    if (!(scale > 0.0) || !std::isfinite(scale)) {
        inverse = {};
        return false;
    }

    const double a = m00 / scale;
    const double b = m01 / scale;
    const double c = m02 / scale;
    const double d = m11 / scale;
    const double e = m12 / scale;
    const double f = m22 / scale;

    const double cofactor00 = d * f - e * e;
    const double cofactor01 = c * e - b * f;
    const double cofactor02 = b * e - c * d;
    const double cofactor11 = a * f - c * c;
    const double cofactor12 = b * c - a * e;
    const double cofactor22 = a * d - b * b;
    const double determinant =
        a * cofactor00 + b * cofactor01 + c * cofactor02;
    const double leadingMinor2 = a * d - b * b;

    // Sylvester's criterion is required here because custom tensors enter
    // through the public API. A positive determinant alone does not make a
    // symmetric matrix physically valid.
    if (!(a > 0.0) ||
        !(leadingMinor2 > 0.0) ||
        !(determinant > 0.0) ||
        !std::isfinite(determinant)) {
        inverse = {};
        return false;
    }

    const double inverseScaleDeterminant = 1.0 / (scale * determinant);
    inverse = {
        saturatedReal(cofactor00 * inverseScaleDeterminant),
        saturatedReal(cofactor01 * inverseScaleDeterminant),
        saturatedReal(cofactor02 * inverseScaleDeterminant),
        saturatedReal(cofactor01 * inverseScaleDeterminant),
        saturatedReal(cofactor11 * inverseScaleDeterminant),
        saturatedReal(cofactor12 * inverseScaleDeterminant),
        saturatedReal(cofactor02 * inverseScaleDeterminant),
        saturatedReal(cofactor12 * inverseScaleDeterminant),
        saturatedReal(cofactor22 * inverseScaleDeterminant),
    };
    return isFinite(inverse);
}

bool canonicalizePositiveDefiniteInertia(
    const Mat3& candidate,
    Mat3& canonical,
    Mat3& inverse
) noexcept {
    if (!isFinite(candidate)) {
        return false;
    }
    const Real scale = std::max({
        1.0F,
        std::abs(candidate.m00),
        std::abs(candidate.m01),
        std::abs(candidate.m02),
        std::abs(candidate.m10),
        std::abs(candidate.m11),
        std::abs(candidate.m12),
        std::abs(candidate.m20),
        std::abs(candidate.m21),
        std::abs(candidate.m22),
    });
    constexpr Real kSymmetryRelativeTolerance = 1.0e-5F;
    if (std::abs(candidate.m01 - candidate.m10) >
            scale * kSymmetryRelativeTolerance ||
        std::abs(candidate.m02 - candidate.m20) >
            scale * kSymmetryRelativeTolerance ||
        std::abs(candidate.m12 - candidate.m21) >
            scale * kSymmetryRelativeTolerance) {
        return false;
    }

    const double i00 = static_cast<double>(candidate.m00);
    const double i01 =
        0.5 * static_cast<double>(candidate.m01 + candidate.m10);
    const double i02 =
        0.5 * static_cast<double>(candidate.m02 + candidate.m20);
    const double i11 = static_cast<double>(candidate.m11);
    const double i12 =
        0.5 * static_cast<double>(candidate.m12 + candidate.m21);
    const double i22 = static_cast<double>(candidate.m22);
    if (!invertSymmetricPositiveDefinite(
            i00,
            i01,
            i02,
            i11,
            i12,
            i22,
            inverse
        )) {
        return false;
    }
    canonical = {
        saturatedReal(i00),
        saturatedReal(i01),
        saturatedReal(i02),
        saturatedReal(i01),
        saturatedReal(i11),
        saturatedReal(i12),
        saturatedReal(i02),
        saturatedReal(i12),
        saturatedReal(i22),
    };

    // Principal moments of a realizable mass distribution obey the triangle
    // inequalities. A short deterministic Jacobi diagonalization also lets us
    // reject tensors too ill-conditioned for the float simulation state.
    double eigenMatrix[3][3]{
        {i00, i01, i02},
        {i01, i11, i12},
        {i02, i12, i22},
    };
    for (int sweep = 0; sweep < 12; ++sweep) {
        int p = 0;
        int q = 1;
        double largest = std::abs(eigenMatrix[0][1]);
        if (std::abs(eigenMatrix[0][2]) > largest) {
            p = 0;
            q = 2;
            largest = std::abs(eigenMatrix[0][2]);
        }
        if (std::abs(eigenMatrix[1][2]) > largest) {
            p = 1;
            q = 2;
            largest = std::abs(eigenMatrix[1][2]);
        }
        if (largest <= scale * 1.0e-12) {
            break;
        }
        const double angle = 0.5 * std::atan2(
            2.0 * eigenMatrix[p][q],
            eigenMatrix[q][q] - eigenMatrix[p][p]
        );
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const double app = eigenMatrix[p][p];
        const double aqq = eigenMatrix[q][q];
        const double apq = eigenMatrix[p][q];
        eigenMatrix[p][p] =
            cosine * cosine * app -
            2.0 * sine * cosine * apq +
            sine * sine * aqq;
        eigenMatrix[q][q] =
            sine * sine * app +
            2.0 * sine * cosine * apq +
            cosine * cosine * aqq;
        eigenMatrix[p][q] = 0.0;
        eigenMatrix[q][p] = 0.0;
        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) {
                continue;
            }
            const double arp = eigenMatrix[r][p];
            const double arq = eigenMatrix[r][q];
            eigenMatrix[r][p] =
                cosine * arp - sine * arq;
            eigenMatrix[p][r] = eigenMatrix[r][p];
            eigenMatrix[r][q] =
                sine * arp + cosine * arq;
            eigenMatrix[q][r] = eigenMatrix[r][q];
        }
    }
    std::array<double, 3> principalMoments{
        eigenMatrix[0][0],
        eigenMatrix[1][1],
        eigenMatrix[2][2],
    };
    std::sort(principalMoments.begin(), principalMoments.end());
    constexpr double kMaximumConditionNumber = 1.0e7;
    constexpr double kRealizabilityTolerance = 2.0e-5;
    if (!(principalMoments[0] > 0.0) ||
        principalMoments[2] / principalMoments[0] >
            kMaximumConditionNumber ||
        principalMoments[2] >
            (principalMoments[0] + principalMoments[1]) *
                (1.0 + kRealizabilityTolerance)) {
        inverse = {};
        canonical = {};
        return false;
    }
    return true;
}

} // namespace

Collider Collider::makeBox(const Vec3& halfExtents) noexcept {
    Collider collider{};
    collider.type = ColliderType::Box;
    collider.box.halfExtents = sanitizedHalfExtents(halfExtents);
    return collider;
}

Collider Collider::makePlane(const Vec3& normal, Real offset) noexcept {
    Collider collider{};
    collider.type = ColliderType::Plane;
    const Real normalLength = length(normal);
    if (normalLength > kEpsilon && std::isfinite(normalLength)) {
        collider.plane.normal = normal / normalLength;
        collider.plane.offset = std::isfinite(offset)
                                    ? saturatedReal(
                                          static_cast<double>(offset) /
                                          static_cast<double>(normalLength)
                                      )
                                    : 0.0F;
    } else {
        collider.plane.normal = {0.0F, 1.0F, 0.0F};
        collider.plane.offset = std::isfinite(offset) ? offset : 0.0F;
    }
    return collider;
}

BodyDescription BodyDescription::makeDenseCube(const Vec3& position) noexcept {
    BodyDescription description{};
    description.motionType = MotionType::Dynamic;
    description.collider = Collider::makeBox({0.5F, 0.5F, 0.5F});
    description.transform.position = position;
    description.mass = 1'000.0F;
    description.debugName = "Dense Cube";
    return description;
}

BodyDescription BodyDescription::makeStaticPlane(const Vec3& normal, Real offset) noexcept {
    BodyDescription description{};
    description.motionType = MotionType::Static;
    description.collider = Collider::makePlane(normal, offset);
    description.mass = 0.0F;
    description.aerodynamics.enabled = false;
    description.allowSleep = false;
    description.debugName = "Static Plane";
    return description;
}

RigidBody::RigidBody(BodyId id, const BodyDescription& description) noexcept
    : id_{id},
      motionType_{description.motionType},
      collider_{description.collider},
      transform_{description.transform},
      previousTransform_{description.transform},
      linearVelocity_{description.linearVelocity},
      angularVelocity_{description.angularVelocity},
      material_{sanitizedMaterial(description.material)},
      aerodynamics_{sanitizedAerodynamics(description.aerodynamics)},
      mass_{description.mass},
      centerOfMassLocal_{description.centerOfMassLocal},
      useCustomLocalInertiaTensor_{
          description.useCustomLocalInertiaTensor
      },
      customLocalInertiaTensor_{
          description.customLocalInertiaTensor
      },
      allowSleep_{description.allowSleep} {
    transform_.orientation = transform_.orientation.normalizedValue();
    if (!isFinite(transform_.position)) {
        transform_.position = {};
    }
    if (!isFinite(linearVelocity_)) {
        linearVelocity_ = {};
    }
    if (!isFinite(angularVelocity_)) {
        angularVelocity_ = {};
    }
    if (!isFinite(centerOfMassLocal_)) {
        centerOfMassLocal_ = {};
    }
    if (collider_.type == ColliderType::Box) {
        collider_.box.halfExtents = sanitizedHalfExtents(collider_.box.halfExtents);
    } else {
        collider_ = Collider::makePlane(collider_.plane.normal, collider_.plane.offset);
        motionType_ = MotionType::Static;
    }
    previousTransform_ = transform_;

    debugName_.fill('\0');
    const char* sourceName = description.debugName != nullptr ? description.debugName : "Body";
    std::size_t nameIndex = 0;
    while (nameIndex + 1 < debugName_.size() && sourceName[nameIndex] != '\0') {
        debugName_[nameIndex] = sourceName[nameIndex];
        ++nameIndex;
    }
    (void)recomputeMassProperties();
    synchronizeAngularMomentumFromVelocity();
}

BodyId RigidBody::id() const noexcept {
    return id_;
}

bool RigidBody::isAlive() const noexcept {
    return alive_;
}

MotionType RigidBody::motionType() const noexcept {
    return motionType_;
}

const Collider& RigidBody::collider() const noexcept {
    return collider_;
}

const Transform& RigidBody::transform() const noexcept {
    return transform_;
}

const Transform& RigidBody::previousTransform() const noexcept {
    return previousTransform_;
}

const Vec3& RigidBody::linearVelocity() const noexcept {
    return linearVelocity_;
}

const Vec3& RigidBody::angularVelocity() const noexcept {
    return angularVelocity_;
}

Vec3 RigidBody::worldAngularMomentum() const noexcept {
    return worldAngularMomentum_;
}

Real RigidBody::rotationalKineticEnergy() const noexcept {
    const Vec3 localAngularVelocity =
        transform_.orientation.inverseRotate(angularVelocity_);
    Real energy =
        0.5F *
        dot(
            localAngularVelocity,
            localInertiaTensor_ * localAngularVelocity
        );
    for (const RotorSlot& rotor : rotors_) {
        if (!rotor.alive) {
            continue;
        }
        energy +=
            0.5F *
            rotor.state.absoluteAxialAngularMomentum *
            rotor.state.absoluteAxialAngularMomentum /
            rotor.state.axialInertia;
    }
    return energy;
}

const PhysicsMaterial& RigidBody::material() const noexcept {
    return material_;
}

const AerodynamicProperties& RigidBody::aerodynamics() const noexcept {
    return aerodynamics_;
}

Real RigidBody::mass() const noexcept {
    return mass_;
}

Real RigidBody::inverseMass() const noexcept {
    return inverseMass_;
}

const Vec3& RigidBody::localInertia() const noexcept {
    return localInertia_;
}

const Mat3& RigidBody::localInertiaTensor() const noexcept {
    return localInertiaTensor_;
}

const Mat3& RigidBody::inverseLocalInertiaTensor() const noexcept {
    return inverseLocalInertiaTensor_;
}

const Mat3& RigidBody::lockedAssemblyInertiaTensor() const noexcept {
    return lockedAssemblyInertiaTensor_;
}

bool RigidBody::usesCustomLocalInertiaTensor() const noexcept {
    return useCustomLocalInertiaTensor_;
}

const Vec3& RigidBody::centerOfMassLocal() const noexcept {
    return centerOfMassLocal_;
}

Real RigidBody::volume() const noexcept {
    if (collider_.type != ColliderType::Box) {
        return 0.0F;
    }
    const Vec3 size = collider_.box.halfExtents * 2.0F;
    return size.x * size.y * size.z;
}

Real RigidBody::bulkDensity() const noexcept {
    const Real bodyVolume = volume();
    return bodyVolume > kEpsilon ? mass_ / bodyVolume : 0.0F;
}

bool RigidBody::isSleeping() const noexcept {
    return sleeping_;
}

bool RigidBody::allowsSleep() const noexcept {
    return allowSleep_;
}

const char* RigidBody::debugName() const noexcept {
    return debugName_.data();
}

std::size_t RigidBody::gyroscopicRotorCount() const noexcept {
    return static_cast<std::size_t>(std::count_if(
        rotors_.begin(),
        rotors_.end(),
        [](const RotorSlot& rotor) { return rotor.alive; }
    ));
}

std::size_t RigidBody::gyroscopicRotorSlotCount() const noexcept {
    return rotors_.size();
}

const GyroscopicRotorState* RigidBody::gyroscopicRotor(
    RotorId id
) const noexcept {
    if (id == kInvalidRotorId || id > rotors_.size()) {
        return nullptr;
    }
    const RotorSlot& rotor =
        rotors_[static_cast<std::size_t>(id - 1)];
    return rotor.alive ? &rotor.state : nullptr;
}

Vec3 RigidBody::gyroscopicRotorAngularMomentumLocal() const noexcept {
    Vec3 momentum{};
    for (const RotorSlot& rotor : rotors_) {
        if (!rotor.alive) {
            continue;
        }
        momentum +=
            rotor.state.axisLocal *
            rotor.state.absoluteAxialAngularMomentum;
    }
    return momentum;
}

void RigidBody::setTransform(const Transform& transform) noexcept {
    if (isFinite(transform.position)) {
        transform_.position = transform.position;
        previousTransform_.position = transform.position;
    }
    transform_.orientation = transform.orientation.normalizedValue();
    previousTransform_.orientation = transform_.orientation;
    // Editor teleports retain the user-visible angular velocity. The
    // corresponding momentum changes because the inertia tensor moved.
    synchronizeAngularMomentumFromVelocity();
    wake();
}

void RigidBody::setLinearVelocity(const Vec3& velocity) noexcept {
    if (isFinite(velocity)) {
        linearVelocity_ = velocity;
        wake();
    }
}

void RigidBody::setAngularVelocity(const Vec3& velocity) noexcept {
    if (isFinite(velocity)) {
        angularVelocity_ = velocity;
        synchronizeAngularMomentumFromVelocity();
        wake();
    }
}

void RigidBody::setWorldAngularMomentum(
    const Vec3& angularMomentum
) noexcept {
    if (motionType_ != MotionType::Dynamic ||
        !isFinite(angularMomentum)) {
        return;
    }
    worldAngularMomentum_ = angularMomentum;
    synchronizeAngularVelocityFromMomentum();
    wake();
}

void RigidBody::setMass(Real kilograms) noexcept {
    if (motionType_ != MotionType::Dynamic || !std::isfinite(kilograms)) {
        return;
    }
    const Real previousMass = mass_;
    mass_ = std::max(kMinimumDynamicMass, kilograms);
    if (!recomputeMassProperties()) {
        mass_ = previousMass;
        (void)recomputeMassProperties();
        return;
    }
    // Editing mass properties preserves the editor's angular-velocity value.
    synchronizeAngularMomentumFromVelocity();
    wake();
}

bool RigidBody::setCustomLocalInertiaTensor(
    const Mat3& inertiaTensorKilogramMetersSquared
) noexcept {
    if (motionType_ != MotionType::Dynamic) {
        return false;
    }
    Mat3 canonical{};
    Mat3 inverse{};
    if (!canonicalizePositiveDefiniteInertia(
            inertiaTensorKilogramMetersSquared,
            canonical,
            inverse
        )) {
        return false;
    }
    const Mat3 previousCustom = customLocalInertiaTensor_;
    const bool previousMode = useCustomLocalInertiaTensor_;
    customLocalInertiaTensor_ = canonical;
    useCustomLocalInertiaTensor_ = true;
    if (!recomputeMassProperties()) {
        customLocalInertiaTensor_ = previousCustom;
        useCustomLocalInertiaTensor_ = previousMode;
        (void)recomputeMassProperties();
        return false;
    }
    synchronizeAngularMomentumFromVelocity();
    wake();
    return true;
}

void RigidBody::useAutomaticLocalInertiaTensor() noexcept {
    if (motionType_ != MotionType::Dynamic) {
        return;
    }
    const bool previousMode = useCustomLocalInertiaTensor_;
    useCustomLocalInertiaTensor_ = false;
    if (!recomputeMassProperties()) {
        useCustomLocalInertiaTensor_ = previousMode;
        (void)recomputeMassProperties();
        return;
    }
    synchronizeAngularMomentumFromVelocity();
    wake();
}

void RigidBody::setCenterOfMassLocal(const Vec3& localOffsetMeters) noexcept {
    if (!isFinite(localOffsetMeters)) {
        return;
    }
    const Vec3 previousCenterOfMass = centerOfMassLocal_;
    centerOfMassLocal_ = localOffsetMeters;
    if (!recomputeMassProperties()) {
        centerOfMassLocal_ = previousCenterOfMass;
        (void)recomputeMassProperties();
        return;
    }
    synchronizeAngularMomentumFromVelocity();
    wake();
}

void RigidBody::setBoxHalfExtents(const Vec3& halfExtentsMeters) noexcept {
    if (collider_.type != ColliderType::Box) {
        return;
    }
    const Vec3 previousHalfExtents = collider_.box.halfExtents;
    collider_.box.halfExtents =
        sanitizedHalfExtents(halfExtentsMeters, collider_.box.halfExtents);
    if (!recomputeMassProperties()) {
        collider_.box.halfExtents = previousHalfExtents;
        (void)recomputeMassProperties();
        return;
    }
    synchronizeAngularMomentumFromVelocity();
    wake();
}

void RigidBody::setMaterial(const PhysicsMaterial& material) noexcept {
    material_ = sanitizedMaterial(material, material_);
    wake();
}

void RigidBody::setAerodynamics(const AerodynamicProperties& properties) noexcept {
    aerodynamics_ = sanitizedAerodynamics(properties, aerodynamics_);
    wake();
}

void RigidBody::setAllowSleep(bool allowSleep) noexcept {
    allowSleep_ = allowSleep;
    if (!allowSleep_) {
        wake();
    }
}

void RigidBody::wake() noexcept {
    if (motionType_ == MotionType::Dynamic) {
        sleeping_ = false;
        quietTime_ = 0.0F;
    }
}

RotorId RigidBody::createGyroscopicRotor(
    const GyroscopicRotorDescription& description
) {
    if (motionType_ != MotionType::Dynamic ||
        !isFinite(description.axisLocal) ||
        !std::isfinite(description.axialInertia) ||
        !std::isfinite(description.relativeAngularVelocity) ||
        !std::isfinite(description.bearingDamping) ||
        description.axialInertia <= kEpsilon ||
        description.bearingDamping < 0.0F) {
        return kInvalidRotorId;
    }
    const Real axisLength = length(description.axisLocal);
    if (!(axisLength > kEpsilon) || !std::isfinite(axisLength)) {
        return kInvalidRotorId;
    }

    RotorSlot slot{};
    slot.state.id = static_cast<RotorId>(rotors_.size() + 1);
    slot.state.axisLocal = description.axisLocal / axisLength;
    slot.state.axialInertia = description.axialInertia;
    slot.state.relativeAngularVelocity =
        description.relativeAngularVelocity;
    const Vec3 localAngularVelocity =
        transform_.orientation.inverseRotate(angularVelocity_);
    slot.state.absoluteAxialAngularMomentum =
        description.axialInertia *
        (dot(localAngularVelocity, slot.state.axisLocal) +
         description.relativeAngularVelocity);
    slot.state.bearingDamping = description.bearingDamping;
    slot.alive = true;
    rotors_.push_back(slot);

    // Component creation is an initial-condition edit: retain the chassis
    // angular velocity and establish the matching total momentum.
    if (!recomputeMassProperties()) {
        rotors_.pop_back();
        (void)recomputeMassProperties();
        return kInvalidRotorId;
    }
    synchronizeAngularMomentumFromVelocity();
    wake();
    return slot.state.id;
}

bool RigidBody::destroyGyroscopicRotor(RotorId id) noexcept {
    if (id == kInvalidRotorId || id > rotors_.size()) {
        return false;
    }
    RotorSlot& rotor = rotors_[static_cast<std::size_t>(id - 1)];
    if (!rotor.alive) {
        return false;
    }
    rotor.alive = false;
    rotor.torqueAccumulator = 0.0F;
    rotor.evaluatedTorque = 0.0F;
    (void)recomputeMassProperties();
    synchronizeAngularMomentumFromVelocity();
    wake();
    return true;
}

bool RigidBody::setGyroscopicRotorRelativeAngularVelocity(
    RotorId id,
    Real radiansPerSecond
) noexcept {
    if (!std::isfinite(radiansPerSecond) ||
        id == kInvalidRotorId ||
        id > rotors_.size()) {
        return false;
    }
    RotorSlot& rotor = rotors_[static_cast<std::size_t>(id - 1)];
    if (!rotor.alive) {
        return false;
    }
    // Solve for the absolute rotor-axis momentum that realizes the requested
    // relative speed while total assembly angular momentum stays unchanged.
    // This is an internal editor/motor action, so the carrier receives the
    // exact equal-and-opposite response.
    const Vec3 localTotalMomentum =
        transform_.orientation.inverseRotate(worldAngularMomentum_);
    Vec3 otherRotorMomentum{};
    for (const RotorSlot& candidate : rotors_) {
        if (!candidate.alive ||
            candidate.state.id == rotor.state.id) {
            continue;
        }
        otherRotorMomentum +=
            candidate.state.axisLocal *
            candidate.state.absoluteAxialAngularMomentum;
    }
    const Vec3 remainingMomentum =
        localTotalMomentum - otherRotorMomentum;
    const Vec3 inverseTimesRemaining =
        inverseLocalInertiaTensor_ * remainingMomentum;
    const Vec3 inverseTimesAxis =
        inverseLocalInertiaTensor_ * rotor.state.axisLocal;
    const Real denominator =
        1.0F / rotor.state.axialInertia +
        dot(rotor.state.axisLocal, inverseTimesAxis);
    if (!(denominator > kEpsilon) ||
        !std::isfinite(denominator)) {
        return false;
    }
    rotor.state.absoluteAxialAngularMomentum =
        (radiansPerSecond +
         dot(rotor.state.axisLocal, inverseTimesRemaining)) /
        denominator;
    synchronizeAngularVelocityFromMomentum();
    wake();
    return true;
}

bool RigidBody::setGyroscopicRotorMotorTorque(
    RotorId id,
    Real torqueNewtonMeters
) noexcept {
    if (!std::isfinite(torqueNewtonMeters) ||
        id == kInvalidRotorId ||
        id > rotors_.size()) {
        return false;
    }
    RotorSlot& rotor = rotors_[static_cast<std::size_t>(id - 1)];
    if (!rotor.alive) {
        return false;
    }
    rotor.state.motorTorqueCommand = torqueNewtonMeters;
    wake();
    return true;
}

void RigidBody::applyGyroscopicRotorTorque(
    RotorId id,
    Real torqueNewtonMeters
) noexcept {
    if (!std::isfinite(torqueNewtonMeters) ||
        id == kInvalidRotorId ||
        id > rotors_.size()) {
        return;
    }
    RotorSlot& rotor = rotors_[static_cast<std::size_t>(id - 1)];
    if (!rotor.alive) {
        return;
    }
    rotor.torqueAccumulator += torqueNewtonMeters;
    wake();
}

void RigidBody::applyForce(const Vec3& forceNewtons) noexcept {
    if (motionType_ != MotionType::Dynamic || !isFinite(forceNewtons)) {
        return;
    }
    forceAccumulator_ += forceNewtons;
    wake();
}

void RigidBody::applyForceAtWorldPoint(
    const Vec3& forceNewtons,
    const Vec3& worldPoint
) noexcept {
    if (motionType_ != MotionType::Dynamic || !isFinite(forceNewtons) ||
        !isFinite(worldPoint)) {
        return;
    }
    forceAccumulator_ += forceNewtons;
    torqueAccumulator_ += cross(worldPoint - transform_.position, forceNewtons);
    wake();
}

void RigidBody::applyTorque(const Vec3& torqueNewtonMeters) noexcept {
    if (motionType_ != MotionType::Dynamic || !isFinite(torqueNewtonMeters)) {
        return;
    }
    torqueAccumulator_ += torqueNewtonMeters;
    wake();
}

void RigidBody::applyAngularImpulse(
    const Vec3& angularImpulseNewtonMeterSeconds
) noexcept {
    if (motionType_ != MotionType::Dynamic ||
        !isFinite(angularImpulseNewtonMeterSeconds)) {
        return;
    }
    worldAngularMomentum_ += angularImpulseNewtonMeterSeconds;
    synchronizeAngularVelocityFromMomentum();
    wake();
}

void RigidBody::applyImpulseAtWorldPoint(
    const Vec3& impulseNewtonSeconds,
    const Vec3& worldPoint
) noexcept {
    if (motionType_ != MotionType::Dynamic || !isFinite(impulseNewtonSeconds) ||
        !isFinite(worldPoint)) {
        return;
    }
    linearVelocity_ += impulseNewtonSeconds * inverseMass_;
    const Vec3 angularImpulse = cross(worldPoint - transform_.position, impulseNewtonSeconds);
    worldAngularMomentum_ += angularImpulse;
    synchronizeAngularVelocityFromMomentum();
    wake();
}

Vec3 RigidBody::worldPointFromLocal(const Vec3& localPoint) const noexcept {
    return transform_.position +
           transform_.orientation.rotate(localPoint - centerOfMassLocal_);
}

Vec3 RigidBody::velocityAtWorldPoint(const Vec3& worldPoint) const noexcept {
    return linearVelocity_ +
           cross(angularVelocity_, worldPoint - transform_.position);
}

Real RigidBody::projectedBoxArea(const Vec3& worldDirection) const noexcept {
    if (collider_.type != ColliderType::Box || lengthSquared(worldDirection) <= kEpsilon) {
        return 0.0F;
    }

    const Vec3 direction = normalized(worldDirection);
    const Vec3 axisX = transform_.orientation.rotate({1.0F, 0.0F, 0.0F});
    const Vec3 axisY = transform_.orientation.rotate({0.0F, 1.0F, 0.0F});
    const Vec3 axisZ = transform_.orientation.rotate({0.0F, 0.0F, 1.0F});
    const Vec3 size = collider_.box.halfExtents * 2.0F;
    return std::abs(dot(direction, axisX)) * size.y * size.z +
           std::abs(dot(direction, axisY)) * size.x * size.z +
           std::abs(dot(direction, axisZ)) * size.x * size.y;
}

Aabb RigidBody::worldAabb() const noexcept {
    if (collider_.type != ColliderType::Box) {
        const Real infinity = std::numeric_limits<Real>::infinity();
        return {{-infinity, -infinity, -infinity}, {infinity, infinity, infinity}};
    }

    const Vec3 axisX = absolute(transform_.orientation.rotate({1.0F, 0.0F, 0.0F}));
    const Vec3 axisY = absolute(transform_.orientation.rotate({0.0F, 1.0F, 0.0F}));
    const Vec3 axisZ = absolute(transform_.orientation.rotate({0.0F, 0.0F, 1.0F}));
    const Vec3 half = collider_.box.halfExtents;
    const Vec3 worldExtent = axisX * half.x + axisY * half.y + axisZ * half.z;

    const Vec3 geometryCenter =
        transform_.position - transform_.orientation.rotate(centerOfMassLocal_);
    return {geometryCenter - worldExtent, geometryCenter + worldExtent};
}

std::array<Vec3, 8> RigidBody::boxWorldVertices() const noexcept {
    std::array<Vec3, 8> vertices{};
    if (collider_.type != ColliderType::Box) {
        vertices.fill(transform_.position);
        return vertices;
    }

    const Vec3 half = collider_.box.halfExtents;
    std::size_t index = 0;
    for (int xSign : {-1, 1}) {
        for (int ySign : {-1, 1}) {
            for (int zSign : {-1, 1}) {
                vertices[index++] = worldPointFromLocal({
                    half.x * static_cast<Real>(xSign),
                    half.y * static_cast<Real>(ySign),
                    half.z * static_cast<Real>(zSign),
                });
            }
        }
    }
    return vertices;
}

Vec3 RigidBody::multiplyWorldInverseInertia(const Vec3& value) const noexcept {
    if (inverseMass_ <= 0.0F) {
        return {};
    }
    const Vec3 local = transform_.orientation.inverseRotate(value);
    return transform_.orientation.rotate(inverseLocalInertiaTensor_ * local);
}

Vec3 RigidBody::multiplyWorldInertia(const Vec3& value) const noexcept {
    const Vec3 local = transform_.orientation.inverseRotate(value);
    return transform_.orientation.rotate(localInertiaTensor_ * local);
}

void RigidBody::synchronizeAngularMomentumFromVelocity() noexcept {
    if (motionType_ != MotionType::Dynamic || inverseMass_ <= 0.0F) {
        worldAngularMomentum_ = {};
        angularVelocity_ = {};
        return;
    }
    const Vec3 localAngularVelocity =
        transform_.orientation.inverseRotate(angularVelocity_);
    for (RotorSlot& rotor : rotors_) {
        if (!rotor.alive) {
            continue;
        }
        rotor.state.absoluteAxialAngularMomentum =
            rotor.state.axialInertia *
            (dot(localAngularVelocity, rotor.state.axisLocal) +
             rotor.state.relativeAngularVelocity);
    }
    worldAngularMomentum_ = transform_.orientation.rotate(
        localInertiaTensor_ * localAngularVelocity +
        gyroscopicRotorAngularMomentumLocal()
    );
    if (!isFinite(worldAngularMomentum_)) {
        worldAngularMomentum_ = {};
        angularVelocity_ = {};
    }
}

void RigidBody::synchronizeAngularVelocityFromMomentum() noexcept {
    if (motionType_ != MotionType::Dynamic || inverseMass_ <= 0.0F) {
        worldAngularMomentum_ = {};
        angularVelocity_ = {};
        return;
    }
    const Vec3 rotorMomentumWorld =
        transform_.orientation.rotate(
            gyroscopicRotorAngularMomentumLocal()
        );
    angularVelocity_ = multiplyWorldInverseInertia(
        worldAngularMomentum_ - rotorMomentumWorld
    );
    if (!isFinite(angularVelocity_)) {
        worldAngularMomentum_ = {};
        angularVelocity_ = {};
        for (RotorSlot& rotor : rotors_) {
            rotor.state.absoluteAxialAngularMomentum = 0.0F;
            rotor.state.relativeAngularVelocity = 0.0F;
        }
        return;
    }
    const Vec3 localAngularVelocity =
        transform_.orientation.inverseRotate(angularVelocity_);
    for (RotorSlot& rotor : rotors_) {
        if (!rotor.alive) {
            continue;
        }
        rotor.state.relativeAngularVelocity =
            rotor.state.absoluteAxialAngularMomentum /
                rotor.state.axialInertia -
            dot(localAngularVelocity, rotor.state.axisLocal);
    }
}

bool RigidBody::recomputeMassProperties() noexcept {
    if (motionType_ != MotionType::Dynamic || collider_.type != ColliderType::Box) {
        mass_ = 0.0F;
        inverseMass_ = 0.0F;
        localInertia_ = {};
        localInertiaTensor_ = {};
        inverseLocalInertiaTensor_ = {};
        lockedAssemblyInertiaTensor_ = {};
        return true;
    }

    const Real sanitizedMass =
        std::isfinite(mass_)
            ? std::max(kMinimumDynamicMass, std::abs(mass_))
            : kMinimumDynamicMass;
    double ixx = 0.0;
    double iyy = 0.0;
    double izz = 0.0;
    double ixy = 0.0;
    double ixz = 0.0;
    double iyz = 0.0;

    Mat3 canonicalCustom{};
    Mat3 unusedCustomInverse{};
    if (useCustomLocalInertiaTensor_ &&
        canonicalizePositiveDefiniteInertia(
            customLocalInertiaTensor_,
            canonicalCustom,
            unusedCustomInverse
        )) {
        customLocalInertiaTensor_ = canonicalCustom;
        ixx = canonicalCustom.m00;
        iyy = canonicalCustom.m11;
        izz = canonicalCustom.m22;
        ixy = canonicalCustom.m01;
        ixz = canonicalCustom.m02;
        iyz = canonicalCustom.m12;
    } else {
        useCustomLocalInertiaTensor_ = false;
        const double bodyMass = static_cast<double>(sanitizedMass);
        const double sizeX =
            2.0 * static_cast<double>(collider_.box.halfExtents.x);
        const double sizeY =
            2.0 * static_cast<double>(collider_.box.halfExtents.y);
        const double sizeZ =
            2.0 * static_cast<double>(collider_.box.halfExtents.z);
        const double offsetX =
            static_cast<double>(centerOfMassLocal_.x);
        const double offsetY =
            static_cast<double>(centerOfMassLocal_.y);
        const double offsetZ =
            static_cast<double>(centerOfMassLocal_.z);

        const double centeredIxx =
            bodyMass * (sizeY * sizeY + sizeZ * sizeZ) / 12.0;
        const double centeredIyy =
            bodyMass * (sizeX * sizeX + sizeZ * sizeZ) / 12.0;
        const double centeredIzz =
            bodyMass * (sizeX * sizeX + sizeY * sizeY) / 12.0;

        // Full parallel-axis theorem. The editable CoM moves the integration
        // origin away from the geometric box center, so non-axis-aligned
        // offsets create products of inertia and coupled angular response.
        ixx =
            centeredIxx +
            bodyMass * (offsetY * offsetY + offsetZ * offsetZ);
        iyy =
            centeredIyy +
            bodyMass * (offsetX * offsetX + offsetZ * offsetZ);
        izz =
            centeredIzz +
            bodyMass * (offsetX * offsetX + offsetY * offsetY);
        ixy = -bodyMass * offsetX * offsetY;
        ixz = -bodyMass * offsetX * offsetZ;
        iyz = -bodyMass * offsetY * offsetZ;
    }

    const Mat3 lockedTensor = {
        saturatedReal(ixx),
        saturatedReal(ixy),
        saturatedReal(ixz),
        saturatedReal(ixy),
        saturatedReal(iyy),
        saturatedReal(iyz),
        saturatedReal(ixz),
        saturatedReal(iyz),
        saturatedReal(izz),
    };
    Mat3 unusedLockedInverse{};
    if (!invertSymmetricPositiveDefinite(
            ixx,
            ixy,
            ixz,
            iyy,
            iyz,
            izz,
            unusedLockedInverse
        )) {
        return false;
    }

    // Rotor coordinates store absolute axial momentum. Holding those momenta
    // fixed leaves the carrier with I_eff = I_locked - sum(J a a^T).
    double effectiveIxx = ixx;
    double effectiveIyy = iyy;
    double effectiveIzz = izz;
    double effectiveIxy = ixy;
    double effectiveIxz = ixz;
    double effectiveIyz = iyz;
    for (const RotorSlot& rotor : rotors_) {
        if (!rotor.alive) {
            continue;
        }
        const Vec3 axis = rotor.state.axisLocal;
        const double axial =
            static_cast<double>(rotor.state.axialInertia);
        effectiveIxx -= axial * axis.x * axis.x;
        effectiveIyy -= axial * axis.y * axis.y;
        effectiveIzz -= axial * axis.z * axis.z;
        effectiveIxy -= axial * axis.x * axis.y;
        effectiveIxz -= axial * axis.x * axis.z;
        effectiveIyz -= axial * axis.y * axis.z;
    }

    Mat3 effectiveInverse{};
    if (!invertSymmetricPositiveDefinite(
            effectiveIxx,
            effectiveIxy,
            effectiveIxz,
            effectiveIyy,
            effectiveIyz,
            effectiveIzz,
            effectiveInverse
        )) {
        return false;
    }
    const Mat3 effectiveTensor = {
        saturatedReal(effectiveIxx),
        saturatedReal(effectiveIxy),
        saturatedReal(effectiveIxz),
        saturatedReal(effectiveIxy),
        saturatedReal(effectiveIyy),
        saturatedReal(effectiveIyz),
        saturatedReal(effectiveIxz),
        saturatedReal(effectiveIyz),
        saturatedReal(effectiveIzz),
    };

    mass_ = sanitizedMass;
    inverseMass_ = 1.0F / mass_;
    lockedAssemblyInertiaTensor_ = lockedTensor;
    localInertiaTensor_ = effectiveTensor;
    inverseLocalInertiaTensor_ = effectiveInverse;
    localInertia_ = localInertiaTensor_.diagonalValue();
    return true;
}

void RigidBody::clearAccumulators() noexcept {
    forceAccumulator_ = {};
    torqueAccumulator_ = {};
    for (RotorSlot& rotor : rotors_) {
        rotor.torqueAccumulator = 0.0F;
    }
}

} // namespace uaview::physics
