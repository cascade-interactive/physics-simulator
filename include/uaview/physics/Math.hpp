#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace uaview::physics {

using Real = float;

constexpr Real kPi = 3.14159265358979323846F;
constexpr Real kEpsilon = 1.0e-6F;

struct Vec3 {
    Real x{0.0F};
    Real y{0.0F};
    Real z{0.0F};

    constexpr Vec3() = default;
    constexpr Vec3(Real xValue, Real yValue, Real zValue)
        : x{xValue}, y{yValue}, z{zValue} {}

    constexpr Vec3 operator+() const noexcept { return *this; }
    constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }

    constexpr Vec3& operator+=(const Vec3& rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(Real scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    constexpr Vec3& operator/=(Real scalar) noexcept {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }
};

constexpr Vec3 operator+(Vec3 lhs, const Vec3& rhs) noexcept {
    return lhs += rhs;
}

constexpr Vec3 operator-(Vec3 lhs, const Vec3& rhs) noexcept {
    return lhs -= rhs;
}

constexpr Vec3 operator*(Vec3 vector, Real scalar) noexcept {
    return vector *= scalar;
}

constexpr Vec3 operator*(Real scalar, Vec3 vector) noexcept {
    return vector *= scalar;
}

constexpr Vec3 operator/(Vec3 vector, Real scalar) noexcept {
    return vector /= scalar;
}

constexpr Vec3 hadamard(const Vec3& lhs, const Vec3& rhs) noexcept {
    return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

constexpr Real dot(const Vec3& lhs, const Vec3& rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

constexpr Vec3 cross(const Vec3& lhs, const Vec3& rhs) noexcept {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

constexpr Real lengthSquared(const Vec3& value) noexcept {
    return dot(value, value);
}

inline Real length(const Vec3& value) noexcept {
    return std::sqrt(lengthSquared(value));
}

inline Vec3 normalized(const Vec3& value, const Vec3& fallback = {1.0F, 0.0F, 0.0F}) noexcept {
    const Real squaredLength = lengthSquared(value);
    if (squaredLength <= kEpsilon * kEpsilon || !std::isfinite(squaredLength)) {
        return fallback;
    }
    return value / std::sqrt(squaredLength);
}

inline Vec3 clampLength(const Vec3& value, Real maximumLength) noexcept {
    const Real squaredLength = lengthSquared(value);
    const Real maximumSquared = maximumLength * maximumLength;
    if (squaredLength <= maximumSquared || squaredLength <= kEpsilon * kEpsilon) {
        return value;
    }
    return value * (maximumLength / std::sqrt(squaredLength));
}

inline bool isFinite(const Vec3& value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Row-major 3x3 matrix used by the portable physics core. Inertia tensors are
// symmetric, but retaining every element keeps multiplication explicit and
// avoids encoding assumptions in consumers such as the impulse solver.
struct Mat3 {
    Real m00{0.0F};
    Real m01{0.0F};
    Real m02{0.0F};
    Real m10{0.0F};
    Real m11{0.0F};
    Real m12{0.0F};
    Real m20{0.0F};
    Real m21{0.0F};
    Real m22{0.0F};

    constexpr Mat3() = default;
    constexpr Mat3(
        Real row0Column0,
        Real row0Column1,
        Real row0Column2,
        Real row1Column0,
        Real row1Column1,
        Real row1Column2,
        Real row2Column0,
        Real row2Column1,
        Real row2Column2
    ) noexcept
        : m00{row0Column0},
          m01{row0Column1},
          m02{row0Column2},
          m10{row1Column0},
          m11{row1Column1},
          m12{row1Column2},
          m20{row2Column0},
          m21{row2Column1},
          m22{row2Column2} {}

    [[nodiscard]] static constexpr Mat3 identity() noexcept {
        return {
            1.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 1.0F,
        };
    }

    [[nodiscard]] static constexpr Mat3 diagonal(const Vec3& value) noexcept {
        return {
            value.x, 0.0F, 0.0F,
            0.0F, value.y, 0.0F,
            0.0F, 0.0F, value.z,
        };
    }

    [[nodiscard]] constexpr Vec3 diagonalValue() const noexcept {
        return {m00, m11, m22};
    }
};

constexpr Vec3 operator*(const Mat3& matrix, const Vec3& vector) noexcept {
    return {
        matrix.m00 * vector.x + matrix.m01 * vector.y + matrix.m02 * vector.z,
        matrix.m10 * vector.x + matrix.m11 * vector.y + matrix.m12 * vector.z,
        matrix.m20 * vector.x + matrix.m21 * vector.y + matrix.m22 * vector.z,
    };
}

constexpr Mat3 operator*(const Mat3& lhs, const Mat3& rhs) noexcept {
    return {
        lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20,
        lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21,
        lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22,
        lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20,
        lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21,
        lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22,
        lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20,
        lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21,
        lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22,
    };
}

inline bool isFinite(const Mat3& value) noexcept {
    return std::isfinite(value.m00) && std::isfinite(value.m01) &&
           std::isfinite(value.m02) && std::isfinite(value.m10) &&
           std::isfinite(value.m11) && std::isfinite(value.m12) &&
           std::isfinite(value.m20) && std::isfinite(value.m21) &&
           std::isfinite(value.m22);
}

constexpr Vec3 absolute(const Vec3& value) noexcept {
    return {
        value.x < 0.0F ? -value.x : value.x,
        value.y < 0.0F ? -value.y : value.y,
        value.z < 0.0F ? -value.z : value.z,
    };
}

struct Quaternion {
    Real w{1.0F};
    Real x{0.0F};
    Real y{0.0F};
    Real z{0.0F};

    constexpr Quaternion() = default;
    constexpr Quaternion(Real wValue, Real xValue, Real yValue, Real zValue)
        : w{wValue}, x{xValue}, y{yValue}, z{zValue} {}

    static constexpr Quaternion identity() noexcept {
        return {};
    }

    static Quaternion fromAxisAngle(const Vec3& axis, Real radians) noexcept {
        const Real halfAngle = radians * 0.5F;
        const Real sine = std::sin(halfAngle);
        const Vec3 unitAxis = normalized(axis);
        return {std::cos(halfAngle), unitAxis.x * sine, unitAxis.y * sine, unitAxis.z * sine};
    }

    [[nodiscard]] Quaternion conjugate() const noexcept {
        return {w, -x, -y, -z};
    }

    [[nodiscard]] Quaternion normalizedValue() const noexcept {
        const Real magnitudeSquared = w * w + x * x + y * y + z * z;
        if (magnitudeSquared <= kEpsilon * kEpsilon || !std::isfinite(magnitudeSquared)) {
            return identity();
        }
        const Real inverseMagnitude = 1.0F / std::sqrt(magnitudeSquared);
        return {
            w * inverseMagnitude,
            x * inverseMagnitude,
            y * inverseMagnitude,
            z * inverseMagnitude,
        };
    }

    [[nodiscard]] Vec3 rotate(const Vec3& vector) const noexcept {
        const Vec3 imaginary{x, y, z};
        const Vec3 twiceCross = 2.0F * cross(imaginary, vector);
        return vector + w * twiceCross + cross(imaginary, twiceCross);
    }

    [[nodiscard]] Vec3 inverseRotate(const Vec3& vector) const noexcept {
        return conjugate().rotate(vector);
    }
};

inline bool isFinite(const Quaternion& value) noexcept {
    return std::isfinite(value.w) && std::isfinite(value.x) &&
           std::isfinite(value.y) && std::isfinite(value.z);
}

constexpr Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) noexcept {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

inline Quaternion integrateWorldAngularVelocity(
    const Quaternion& orientation,
    const Vec3& angularVelocity,
    Real deltaSeconds
) noexcept {
    const Real angularSpeed = length(angularVelocity);
    if (angularSpeed <= kEpsilon || deltaSeconds <= 0.0F) {
        return orientation;
    }
    const Quaternion delta =
        Quaternion::fromAxisAngle(angularVelocity / angularSpeed, angularSpeed * deltaSeconds);
    return (delta * orientation).normalizedValue();
}

struct Aabb {
    Vec3 minimum{};
    Vec3 maximum{};

    [[nodiscard]] bool overlaps(const Aabb& rhs, Real margin = 0.0F) const noexcept {
        return minimum.x <= rhs.maximum.x + margin &&
               maximum.x + margin >= rhs.minimum.x &&
               minimum.y <= rhs.maximum.y + margin &&
               maximum.y + margin >= rhs.minimum.y &&
               minimum.z <= rhs.maximum.z + margin &&
               maximum.z + margin >= rhs.minimum.z;
    }
};

inline Real radians(Real degrees) noexcept {
    return degrees * (kPi / 180.0F);
}

} // namespace uaview::physics
