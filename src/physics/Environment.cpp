#include <uaview/physics/Environment.hpp>

#include <algorithm>
#include <cmath>

namespace uaview::physics {

const Vec3& Environment::gravity() const noexcept {
    return gravity_;
}

void Environment::setGravity(const Vec3& gravityMetersPerSecondSquared) noexcept {
    if (isFinite(gravityMetersPerSecondSquared)) {
        gravity_ = gravityMetersPerSecondSquared;
    }
}

const AtmosphereSample& Environment::atmosphere() const noexcept {
    return atmosphere_;
}

void Environment::setAtmosphere(const AtmosphereSample& atmosphere) noexcept {
    atmosphere_ = atmosphere;
    atmosphere_.density = std::max(0.0F, atmosphere_.density);
    atmosphere_.dynamicViscosity = std::max(0.0F, atmosphere_.dynamicViscosity);
    atmosphere_.temperatureKelvin = std::max(1.0F, atmosphere_.temperatureKelvin);
    atmosphere_.pressurePascals = std::max(0.0F, atmosphere_.pressurePascals);
}

const WindFieldSettings& Environment::windSettings() const noexcept {
    return wind_;
}

void Environment::setWindSettings(const WindFieldSettings& settings) noexcept {
    wind_ = settings;
    if (!isFinite(wind_.meanVelocity)) {
        wind_.meanVelocity = {};
    }
    wind_.turbulenceAmplitude = std::max(0.0F, wind_.turbulenceAmplitude);
    wind_.spatialFrequency = std::max(0.0F, wind_.spatialFrequency);
    wind_.temporalFrequency = std::max(0.0F, wind_.temporalFrequency);
}

AtmosphereSample Environment::sampleAtmosphere(const Vec3&) const noexcept {
    return atmosphere_;
}

Vec3 Environment::windVelocity(
    const Vec3& worldPosition,
    double simulationTime
) const noexcept {
    const Real k = wind_.spatialFrequency;
    const Real phase = static_cast<Real>(simulationTime) * wind_.temporalFrequency;
    const Real amplitude = wind_.turbulenceAmplitude * 0.5F;

    // Each component is independent of its matching coordinate, so the
    // analytic divergence is exactly zero. The phase offsets avoid obvious
    // repetition while keeping the field cheap and deterministic.
    const Vec3 curlLikeField{
        amplitude *
            (std::cos(k * worldPosition.y + phase + 0.71F) -
             std::sin(k * worldPosition.z - phase * 0.83F + 1.37F)),
        amplitude *
            (std::cos(k * worldPosition.z + phase * 0.91F + 2.11F) -
             std::sin(k * worldPosition.x - phase + 0.29F)),
        amplitude *
            (std::cos(k * worldPosition.x + phase * 1.07F + 1.63F) -
             std::sin(k * worldPosition.y - phase * 0.89F + 2.73F)),
    };
    return wind_.meanVelocity + curlLikeField;
}

} // namespace uaview::physics
