#pragma once

#include <uaview/physics/Math.hpp>

namespace uaview::physics {

struct AtmosphereSample {
    Real density{1.225F};           // kg / m^3, ISA sea-level default
    Real dynamicViscosity{1.81e-5F}; // Pa s
    Real temperatureKelvin{288.15F};
    Real pressurePascals{101'325.0F};
};

struct WindFieldSettings {
    Vec3 meanVelocity{0.12F, 0.0F, 0.03F}; // m / s
    Real turbulenceAmplitude{0.16F};        // m / s
    Real spatialFrequency{0.045F};          // rad / m
    Real temporalFrequency{0.11F};          // rad / s
};

class Environment {
public:
    [[nodiscard]] const Vec3& gravity() const noexcept;
    void setGravity(const Vec3& gravityMetersPerSecondSquared) noexcept;

    [[nodiscard]] const AtmosphereSample& atmosphere() const noexcept;
    void setAtmosphere(const AtmosphereSample& atmosphere) noexcept;

    [[nodiscard]] const WindFieldSettings& windSettings() const noexcept;
    void setWindSettings(const WindFieldSettings& settings) noexcept;

    // The altitude parameter is intentionally part of this API even though
    // Milestone 2 uses a constant atmosphere. A layered/ISA model can replace
    // this implementation without changing rigid-body code.
    [[nodiscard]] AtmosphereSample sampleAtmosphere(const Vec3& worldPosition) const noexcept;

    // Deterministic, analytic, divergence-free 3D field. It has no random state
    // and therefore produces identical results for identical position/time.
    [[nodiscard]] Vec3 windVelocity(const Vec3& worldPosition, double simulationTime) const noexcept;

private:
    Vec3 gravity_{0.0F, -9.80665F, 0.0F};
    AtmosphereSample atmosphere_{};
    WindFieldSettings wind_{};
};

} // namespace uaview::physics
