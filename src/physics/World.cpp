#include <uaview/physics/World.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace uaview::physics {
namespace {

struct Obb {
    Vec3 center{};
    std::array<Vec3, 3> axis{};
    Vec3 half{};
};

struct ManifoldPoint {
    Vec3 point{};
    Real penetration{0.0F};
};

struct CollisionManifold {
    Vec3 normal{};
    std::array<ManifoldPoint, 4> points{};
    std::size_t pointCount{0};
};

enum class SatFeature : std::uint8_t {
    FaceA,
    FaceB,
    EdgeEdge,
};

struct SatResult {
    Vec3 normal{1.0F, 0.0F, 0.0F};
    Real overlap{std::numeric_limits<Real>::max()};
    SatFeature feature{SatFeature::FaceA};
    std::size_t axisA{0};
    std::size_t axisB{0};
};

struct Segment {
    Vec3 first{};
    Vec3 second{};
};

struct SweepHit {
    Real fraction{1.0F};
    Vec3 normal{};
    Transform transform{};
    bool hasTransform{false};
};

struct RotationalCcdWork {
    std::uint32_t advances{0};
    bool convergedToContact{false};
    bool hitAdvanceCap{false};
};

struct SupportPoint2 {
    Real x{0.0F};
    Real y{0.0F};
};

Real cross2(
    const SupportPoint2& origin,
    const SupportPoint2& first,
    const SupportPoint2& second
) noexcept {
    return
        (first.x - origin.x) * (second.y - origin.y) -
        (first.y - origin.y) * (second.x - origin.x);
}

bool supportPolygonContainsOrigin(
    std::vector<SupportPoint2> points,
    Real requiredMargin
) {
    if (points.size() < 3U) {
        return false;
    }
    std::sort(
        points.begin(),
        points.end(),
        [](const SupportPoint2& lhs, const SupportPoint2& rhs) {
            return lhs.x < rhs.x ||
                   (lhs.x == rhs.x && lhs.y < rhs.y);
        }
    );

    const Real duplicateTolerance =
        std::max(1.0e-6F, requiredMargin * 0.05F);
    points.erase(
        std::unique(
            points.begin(),
            points.end(),
            [duplicateTolerance](
                const SupportPoint2& lhs,
                const SupportPoint2& rhs
            ) {
                return
                    std::abs(lhs.x - rhs.x) <= duplicateTolerance &&
                    std::abs(lhs.y - rhs.y) <= duplicateTolerance;
            }
        ),
        points.end()
    );
    if (points.size() < 3U) {
        return false;
    }

    std::vector<SupportPoint2> hull(points.size() * 2U);
    std::size_t hullSize = 0U;
    for (const SupportPoint2& point : points) {
        while (hullSize >= 2U &&
               cross2(hull[hullSize - 2U], hull[hullSize - 1U], point) <=
                   0.0F) {
            --hullSize;
        }
        hull[hullSize++] = point;
    }
    const std::size_t lowerHullSize = hullSize;
    for (std::size_t index = points.size() - 1U; index > 0U; --index) {
        const SupportPoint2& point = points[index - 1U];
        while (hullSize > lowerHullSize &&
               cross2(hull[hullSize - 2U], hull[hullSize - 1U], point) <=
                   0.0F) {
            --hullSize;
        }
        hull[hullSize++] = point;
    }
    if (hullSize <= 3U) {
        return false;
    }
    --hullSize; // The monotonic-chain algorithm repeats the first point.

    Real twiceArea = 0.0F;
    Real maximumRadiusSquared = 0.0F;
    for (std::size_t index = 0; index < hullSize; ++index) {
        const SupportPoint2& first = hull[index];
        const SupportPoint2& second = hull[(index + 1U) % hullSize];
        twiceArea += first.x * second.y - first.y * second.x;
        maximumRadiusSquared = std::max(
            maximumRadiusSquared,
            first.x * first.x + first.y * first.y
        );
    }
    if (!(twiceArea > maximumRadiusSquared * 1.0e-6F)) {
        return false;
    }

    for (std::size_t index = 0; index < hullSize; ++index) {
        const SupportPoint2& first = hull[index];
        const SupportPoint2& second = hull[(index + 1U) % hullSize];
        const Real edgeX = second.x - first.x;
        const Real edgeY = second.y - first.y;
        const Real edgeLength = std::sqrt(edgeX * edgeX + edgeY * edgeY);
        if (!(edgeLength > kEpsilon)) {
            return false;
        }
        // The hull is counter-clockwise. Its origin must remain on the left
        // side of every edge by the configured physical support margin.
        const Real originDistance =
            (edgeY * first.x - edgeX * first.y) / edgeLength;
        if (originDistance < requiredMargin) {
            return false;
        }
    }
    return true;
}

Vec3 geometryCenterAt(
    const RigidBody& body,
    const Transform& transform
) noexcept {
    return transform.position -
           transform.orientation.rotate(body.centerOfMassLocal());
}

Obb makeObbAtTransform(
    const RigidBody& body,
    const Transform& transform
) noexcept {
    return {
        geometryCenterAt(body, transform),
        {
            transform.orientation.rotate({1.0F, 0.0F, 0.0F}),
            transform.orientation.rotate({0.0F, 1.0F, 0.0F}),
            transform.orientation.rotate({0.0F, 0.0F, 1.0F}),
        },
        body.collider().box.halfExtents,
    };
}

Obb makeObb(const RigidBody& body) noexcept {
    return makeObbAtTransform(body, body.transform());
}

Real component(const Vec3& vector, std::size_t index) noexcept {
    if (index == 0) {
        return vector.x;
    }
    if (index == 1) {
        return vector.y;
    }
    return vector.z;
}

void addUniqueCandidate(
    std::array<ManifoldPoint, 16>& candidates,
    std::size_t& count,
    const ManifoldPoint& candidate
) noexcept {
    for (std::size_t index = 0; index < count; ++index) {
        if (lengthSquared(candidates[index].point - candidate.point) < 1.0e-8F) {
            return;
        }
    }
    if (count < candidates.size()) {
        candidates[count++] = candidate;
    }
}

void chooseContactCandidates(
    const std::array<ManifoldPoint, 16>& candidates,
    std::size_t candidateCount,
    CollisionManifold& manifold
) noexcept {
    if (candidateCount == 0) {
        return;
    }

    std::array<std::size_t, 4> selected{};
    std::size_t selectedCount = 0;

    // Lexicographic first point gives deterministic tie-breaking.
    std::size_t first = 0;
    for (std::size_t index = 1; index < candidateCount; ++index) {
        const Vec3& lhs = candidates[index].point;
        const Vec3& rhs = candidates[first].point;
        if (lhs.x < rhs.x ||
            (lhs.x == rhs.x &&
             (lhs.y < rhs.y || (lhs.y == rhs.y && lhs.z < rhs.z)))) {
            first = index;
        }
    }
    selected[selectedCount++] = first;

    while (selectedCount < std::min<std::size_t>(4, candidateCount)) {
        Real bestMinimumDistance = -1.0F;
        std::size_t bestIndex = 0;
        for (std::size_t candidateIndex = 0; candidateIndex < candidateCount;
             ++candidateIndex) {
            bool alreadySelected = false;
            for (std::size_t selectedIndex = 0; selectedIndex < selectedCount;
                 ++selectedIndex) {
                alreadySelected |= selected[selectedIndex] == candidateIndex;
            }
            if (alreadySelected) {
                continue;
            }

            Real minimumDistance = std::numeric_limits<Real>::max();
            for (std::size_t selectedIndex = 0; selectedIndex < selectedCount;
                 ++selectedIndex) {
                minimumDistance = std::min(
                    minimumDistance,
                    lengthSquared(
                        candidates[candidateIndex].point -
                        candidates[selected[selectedIndex]].point
                    )
                );
            }
            if (minimumDistance > bestMinimumDistance) {
                bestMinimumDistance = minimumDistance;
                bestIndex = candidateIndex;
            }
        }
        selected[selectedCount++] = bestIndex;
    }

    manifold.pointCount = selectedCount;
    for (std::size_t index = 0; index < selectedCount; ++index) {
        manifold.points[index] = candidates[selected[index]];
    }
}

std::size_t clipPolygonAgainstPlane(
    const std::array<Vec3, 16>& input,
    std::size_t inputCount,
    const Vec3& planeNormal,
    Real planeOffset,
    Real tolerance,
    std::array<Vec3, 16>& output
) noexcept {
    if (inputCount == 0) {
        return 0;
    }

    std::size_t outputCount = 0;
    Vec3 previous = input[inputCount - 1];
    Real previousDistance = dot(planeNormal, previous) - planeOffset;
    bool previousInside = previousDistance <= tolerance;

    for (std::size_t index = 0; index < inputCount; ++index) {
        const Vec3 current = input[index];
        const Real currentDistance = dot(planeNormal, current) - planeOffset;
        const bool currentInside = currentDistance <= tolerance;

        if (currentInside != previousInside) {
            const Real denominator = previousDistance - currentDistance;
            const Real fraction =
                std::abs(denominator) > kEpsilon
                    ? std::clamp(previousDistance / denominator, 0.0F, 1.0F)
                    : 0.0F;
            if (outputCount < output.size()) {
                output[outputCount++] =
                    previous + (current - previous) * fraction;
            }
        }
        if (currentInside && outputCount < output.size()) {
            output[outputCount++] = current;
        }

        previous = current;
        previousDistance = currentDistance;
        previousInside = currentInside;
    }
    return outputCount;
}

Segment supportingEdge(
    const Obb& obb,
    std::size_t edgeAxis,
    const Vec3& supportDirection
) noexcept {
    Vec3 edgeCenter = obb.center;
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (axisIndex == edgeAxis) {
            continue;
        }
        const Real sign =
            dot(obb.axis[axisIndex], supportDirection) >= 0.0F ? 1.0F : -1.0F;
        edgeCenter +=
            obb.axis[axisIndex] * component(obb.half, axisIndex) * sign;
    }
    const Vec3 halfEdge =
        obb.axis[edgeAxis] * component(obb.half, edgeAxis);
    return {edgeCenter - halfEdge, edgeCenter + halfEdge};
}

void closestPointsOnSegments(
    const Segment& first,
    const Segment& second,
    Vec3& pointFirst,
    Vec3& pointSecond
) noexcept {
    const Vec3 directionFirst = first.second - first.first;
    const Vec3 directionSecond = second.second - second.first;
    const Vec3 offset = first.first - second.first;
    const Real lengthFirstSquared = dot(directionFirst, directionFirst);
    const Real lengthSecondSquared = dot(directionSecond, directionSecond);
    const Real directionDot = dot(directionFirst, directionSecond);
    const Real firstOffsetDot = dot(directionFirst, offset);
    const Real secondOffsetDot = dot(directionSecond, offset);

    Real firstFraction = 0.0F;
    Real secondFraction = 0.0F;
    if (lengthFirstSquared <= kEpsilon &&
        lengthSecondSquared <= kEpsilon) {
        pointFirst = first.first;
        pointSecond = second.first;
        return;
    }
    if (lengthFirstSquared <= kEpsilon) {
        secondFraction =
            std::clamp(secondOffsetDot / lengthSecondSquared, 0.0F, 1.0F);
    } else if (lengthSecondSquared <= kEpsilon) {
        firstFraction =
            std::clamp(-firstOffsetDot / lengthFirstSquared, 0.0F, 1.0F);
    } else {
        const Real denominator =
            lengthFirstSquared * lengthSecondSquared -
            directionDot * directionDot;
        if (std::abs(denominator) > kEpsilon) {
            firstFraction = std::clamp(
                (directionDot * secondOffsetDot -
                 firstOffsetDot * lengthSecondSquared) /
                    denominator,
                0.0F,
                1.0F
            );
        }
        secondFraction =
            (directionDot * firstFraction + secondOffsetDot) /
            lengthSecondSquared;
        if (secondFraction < 0.0F) {
            secondFraction = 0.0F;
            firstFraction =
                std::clamp(-firstOffsetDot / lengthFirstSquared, 0.0F, 1.0F);
        } else if (secondFraction > 1.0F) {
            secondFraction = 1.0F;
            firstFraction = std::clamp(
                (directionDot - firstOffsetDot) / lengthFirstSquared,
                0.0F,
                1.0F
            );
        }
    }

    pointFirst = first.first + directionFirst * firstFraction;
    pointSecond = second.first + directionSecond * secondFraction;
}

bool boxAgainstPlane(
    const RigidBody& box,
    const RigidBody& plane,
    Real contactSlop,
    CollisionManifold& manifold
) noexcept {
    const PlaneCollider& planeShape = plane.collider().plane;
    const auto vertices = box.boxWorldVertices();
    struct VertexDistance {
        Vec3 vertex{};
        Real distance{0.0F};
    };
    std::array<VertexDistance, 8> penetrating{};
    std::size_t penetratingCount = 0;

    for (const Vec3& vertex : vertices) {
        const Real distance = dot(planeShape.normal, vertex) - planeShape.offset;
        if (distance <= contactSlop) {
            penetrating[penetratingCount++] = {vertex, distance};
        }
    }
    if (penetratingCount == 0) {
        return false;
    }

    const auto lessVertexDistance = [](const VertexDistance& lhs, const VertexDistance& rhs) {
            if (lhs.distance == rhs.distance) {
                if (lhs.vertex.x != rhs.vertex.x) {
                    return lhs.vertex.x < rhs.vertex.x;
                }
                if (lhs.vertex.z != rhs.vertex.z) {
                    return lhs.vertex.z < rhs.vertex.z;
                }
                return lhs.vertex.y < rhs.vertex.y;
            }
            return lhs.distance < rhs.distance;
        };
    for (std::size_t index = 1; index < penetratingCount; ++index) {
        const VertexDistance value = penetrating[index];
        std::size_t insertionIndex = index;
        while (insertionIndex > 0 &&
               lessVertexDistance(value, penetrating[insertionIndex - 1])) {
            penetrating[insertionIndex] = penetrating[insertionIndex - 1];
            --insertionIndex;
        }
        penetrating[insertionIndex] = value;
    }

    manifold.normal = planeShape.normal;
    manifold.pointCount = std::min<std::size_t>(4, penetratingCount);
    for (std::size_t index = 0; index < manifold.pointCount; ++index) {
        const VertexDistance& candidate = penetrating[index];
        manifold.points[index].point =
            candidate.vertex - planeShape.normal * candidate.distance;
        manifold.points[index].penetration = std::max(0.0F, -candidate.distance);
    }
    return true;
}

bool boxAgainstBox(
    const RigidBody& bodyA,
    const RigidBody& bodyB,
    Real contactSlop,
    CollisionManifold& manifold
) noexcept {
    const Obb obbA = makeObb(bodyA);
    const Obb obbB = makeObb(bodyB);
    const Vec3 centerDelta = obbB.center - obbA.center;
    SatResult result{};

    const auto testAxis = [&](
                              const Vec3& rawAxis,
                              SatFeature feature,
                              std::size_t axisA,
                              std::size_t axisB
                          ) {
        const Real axisLengthSquared = lengthSquared(rawAxis);
        if (axisLengthSquared < 1.0e-10F) {
            return true;
        }
        Vec3 axis = rawAxis / std::sqrt(axisLengthSquared);
        const Real radiusA =
            obbA.half.x * std::abs(dot(axis, obbA.axis[0])) +
            obbA.half.y * std::abs(dot(axis, obbA.axis[1])) +
            obbA.half.z * std::abs(dot(axis, obbA.axis[2]));
        const Real radiusB =
            obbB.half.x * std::abs(dot(axis, obbB.axis[0])) +
            obbB.half.y * std::abs(dot(axis, obbB.axis[1])) +
            obbB.half.z * std::abs(dot(axis, obbB.axis[2]));
        const Real signedDistance = dot(centerDelta, axis);
        const Real overlap = radiusA + radiusB - std::abs(signedDistance);
        if (overlap < -contactSlop) {
            return false;
        }
        constexpr Real kFeatureTieTolerance = 1.0e-5F;
        const bool materiallyBetter =
            overlap < result.overlap - kFeatureTieTolerance;
        const bool facePreferredAtTie =
            std::abs(overlap - result.overlap) <= kFeatureTieTolerance &&
            feature != SatFeature::EdgeEdge &&
            result.feature == SatFeature::EdgeEdge;
        if (materiallyBetter || facePreferredAtTie) {
            result.overlap = overlap;
            result.normal = signedDistance >= 0.0F ? axis : -axis;
            result.feature = feature;
            result.axisA = axisA;
            result.axisB = axisB;
        }
        return true;
    };

    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (!testAxis(
                obbA.axis[axisIndex],
                SatFeature::FaceA,
                axisIndex,
                0
            )) {
            return false;
        }
    }
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (!testAxis(
                obbB.axis[axisIndex],
                SatFeature::FaceB,
                0,
                axisIndex
            )) {
            return false;
        }
    }
    for (std::size_t axisA = 0; axisA < 3; ++axisA) {
        for (std::size_t axisB = 0; axisB < 3; ++axisB) {
            if (!testAxis(
                    cross(obbA.axis[axisA], obbB.axis[axisB]),
                    SatFeature::EdgeEdge,
                    axisA,
                    axisB
                )) {
                return false;
            }
        }
    }

    manifold.normal = result.normal;
    if (result.feature == SatFeature::EdgeEdge) {
        const Segment edgeA =
            supportingEdge(obbA, result.axisA, result.normal);
        const Segment edgeB =
            supportingEdge(obbB, result.axisB, -result.normal);
        Vec3 pointA{};
        Vec3 pointB{};
        closestPointsOnSegments(edgeA, edgeB, pointA, pointB);
        manifold.points[0] = {
            (pointA + pointB) * 0.5F,
            std::max(0.0F, result.overlap),
        };
        manifold.pointCount = 1;
        return true;
    }

    const bool referenceIsA = result.feature == SatFeature::FaceA;
    const Obb& reference = referenceIsA ? obbA : obbB;
    const Obb& incident = referenceIsA ? obbB : obbA;
    const std::size_t referenceAxis =
        referenceIsA ? result.axisA : result.axisB;
    const Vec3 referenceNormal =
        referenceIsA ? result.normal : -result.normal;
    const Vec3 referenceFaceCenter =
        reference.center +
        referenceNormal * component(reference.half, referenceAxis);

    std::size_t incidentAxis = 0;
    Real bestIncidentAlignment =
        std::abs(dot(incident.axis[0], referenceNormal));
    for (std::size_t axisIndex = 1; axisIndex < 3; ++axisIndex) {
        const Real alignment =
            std::abs(dot(incident.axis[axisIndex], referenceNormal));
        if (alignment > bestIncidentAlignment) {
            bestIncidentAlignment = alignment;
            incidentAxis = axisIndex;
        }
    }
    const Real incidentSign =
        dot(incident.axis[incidentAxis], referenceNormal) > 0.0F
            ? -1.0F
            : 1.0F;
    const Vec3 incidentFaceCenter =
        incident.center +
        incident.axis[incidentAxis] *
            component(incident.half, incidentAxis) * incidentSign;

    std::array<std::size_t, 2> incidentSideAxes{};
    std::size_t incidentSideCount = 0;
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (axisIndex != incidentAxis) {
            incidentSideAxes[incidentSideCount++] = axisIndex;
        }
    }
    const Vec3 incidentSideFirst =
        incident.axis[incidentSideAxes[0]] *
        component(incident.half, incidentSideAxes[0]);
    const Vec3 incidentSideSecond =
        incident.axis[incidentSideAxes[1]] *
        component(incident.half, incidentSideAxes[1]);

    std::array<Vec3, 16> polygonA{};
    std::array<Vec3, 16> polygonB{};
    polygonA[0] = incidentFaceCenter - incidentSideFirst - incidentSideSecond;
    polygonA[1] = incidentFaceCenter + incidentSideFirst - incidentSideSecond;
    polygonA[2] = incidentFaceCenter + incidentSideFirst + incidentSideSecond;
    polygonA[3] = incidentFaceCenter - incidentSideFirst + incidentSideSecond;
    std::size_t polygonCount = 4;

    std::array<std::size_t, 2> referenceSideAxes{};
    std::size_t referenceSideCount = 0;
    for (std::size_t axisIndex = 0; axisIndex < 3; ++axisIndex) {
        if (axisIndex != referenceAxis) {
            referenceSideAxes[referenceSideCount++] = axisIndex;
        }
    }
    bool inputIsA = true;
    for (std::size_t sideIndex = 0; sideIndex < 2; ++sideIndex) {
        const Vec3 sideAxis = reference.axis[referenceSideAxes[sideIndex]];
        const Real sideHalf =
            component(reference.half, referenceSideAxes[sideIndex]);
        const Real positiveOffset = dot(sideAxis, reference.center) + sideHalf;
        const Real negativeOffset = dot(-sideAxis, reference.center) + sideHalf;

        if (inputIsA) {
            polygonCount = clipPolygonAgainstPlane(
                polygonA,
                polygonCount,
                sideAxis,
                positiveOffset,
                contactSlop,
                polygonB
            );
        } else {
            polygonCount = clipPolygonAgainstPlane(
                polygonB,
                polygonCount,
                sideAxis,
                positiveOffset,
                contactSlop,
                polygonA
            );
        }
        inputIsA = !inputIsA;
        if (polygonCount == 0) {
            break;
        }

        if (inputIsA) {
            polygonCount = clipPolygonAgainstPlane(
                polygonA,
                polygonCount,
                -sideAxis,
                negativeOffset,
                contactSlop,
                polygonB
            );
        } else {
            polygonCount = clipPolygonAgainstPlane(
                polygonB,
                polygonCount,
                -sideAxis,
                negativeOffset,
                contactSlop,
                polygonA
            );
        }
        inputIsA = !inputIsA;
        if (polygonCount == 0) {
            break;
        }
    }

    const std::array<Vec3, 16>& clippedPolygon =
        inputIsA ? polygonA : polygonB;
    std::array<ManifoldPoint, 16> candidates{};
    std::size_t candidateCount = 0;
    for (std::size_t index = 0; index < polygonCount; ++index) {
        const Vec3 incidentPoint = clippedPolygon[index];
        const Real separation =
            dot(referenceNormal, incidentPoint - referenceFaceCenter);
        if (separation <= contactSlop) {
            addUniqueCandidate(
                candidates,
                candidateCount,
                {
                    incidentPoint - referenceNormal * (0.5F * separation),
                    std::max(0.0F, -separation),
                }
            );
        }
    }

    if (candidateCount == 0) {
        const Real separation =
            dot(referenceNormal, incidentFaceCenter - referenceFaceCenter);
        addUniqueCandidate(
            candidates,
            candidateCount,
            {
                (referenceFaceCenter + incidentFaceCenter) * 0.5F,
                std::max(
                    0.0F,
                    std::max(result.overlap, -separation)
                ),
            }
        );
    }
    chooseContactCandidates(candidates, candidateCount, manifold);
    return manifold.pointCount > 0;
}

bool sweptBoxAgainstStaticBox(
    const RigidBody& dynamicBody,
    const Transform& startTransform,
    const Transform& endTransform,
    const RigidBody& staticBody,
    Real contactSlop,
    SweepHit& hit
) noexcept {
    const Obb movingStart =
        makeObbAtTransform(dynamicBody, startTransform);
    const Obb movingEnd =
        makeObbAtTransform(dynamicBody, endTransform);
    const Obb fixed = makeObb(staticBody);
    const Vec3 centerDelta = fixed.center - movingStart.center;
    const Vec3 relativeDisplacement =
        -(movingEnd.center - movingStart.center);

    Real entryTime = 0.0F;
    Real exitTime = 1.0F;
    Vec3 entryNormal{1.0F, 0.0F, 0.0F};
    bool startedSeparated = false;

    const auto sweepAxis = [&](const Vec3& rawAxis) {
        const Real axisLengthSquared = lengthSquared(rawAxis);
        if (axisLengthSquared < 1.0e-10F) {
            return true;
        }
        const Vec3 axis = rawAxis / std::sqrt(axisLengthSquared);
        const Real movingRadius =
            movingStart.half.x * std::abs(dot(axis, movingStart.axis[0])) +
            movingStart.half.y * std::abs(dot(axis, movingStart.axis[1])) +
            movingStart.half.z * std::abs(dot(axis, movingStart.axis[2]));
        const Real fixedRadius =
            fixed.half.x * std::abs(dot(axis, fixed.axis[0])) +
            fixed.half.y * std::abs(dot(axis, fixed.axis[1])) +
            fixed.half.z * std::abs(dot(axis, fixed.axis[2]));
        const Real radius = movingRadius + fixedRadius + contactSlop;
        const Real initialDistance = dot(centerDelta, axis);
        const Real projectedMotion = dot(relativeDisplacement, axis);
        if (std::abs(initialDistance) > radius) {
            startedSeparated = true;
        }

        if (std::abs(projectedMotion) <= kEpsilon) {
            return std::abs(initialDistance) <= radius;
        }

        Real axisEntry =
            (-radius - initialDistance) / projectedMotion;
        Real axisExit =
            (radius - initialDistance) / projectedMotion;
        if (axisEntry > axisExit) {
            std::swap(axisEntry, axisExit);
        }
        if (axisEntry > entryTime) {
            entryTime = axisEntry;
            const Real distanceAtEntry =
                initialDistance + projectedMotion * axisEntry;
            entryNormal = distanceAtEntry >= 0.0F ? axis : -axis;
        }
        exitTime = std::min(exitTime, axisExit);
        return entryTime <= exitTime;
    };

    for (const Vec3& axis : movingStart.axis) {
        if (!sweepAxis(axis)) {
            return false;
        }
    }
    for (const Vec3& axis : fixed.axis) {
        if (!sweepAxis(axis)) {
            return false;
        }
    }
    for (const Vec3& movingAxis : movingStart.axis) {
        for (const Vec3& fixedAxis : fixed.axis) {
            if (!sweepAxis(cross(movingAxis, fixedAxis))) {
                return false;
            }
        }
    }

    if (!startedSeparated || exitTime < 0.0F ||
        entryTime > 1.0F || entryTime > exitTime) {
        return false;
    }
    hit.fraction = std::clamp(entryTime, 0.0F, 1.0F);
    hit.normal = entryNormal;
    return true;
}

bool sweptBoxAgainstPlane(
    const RigidBody& dynamicBody,
    const Transform& startTransform,
    const Transform& endTransform,
    const RigidBody& planeBody,
    Real contactSlop,
    SweepHit& hit
) noexcept {
    const Obb movingStart =
        makeObbAtTransform(dynamicBody, startTransform);
    const Obb movingEnd =
        makeObbAtTransform(dynamicBody, endTransform);
    const PlaneCollider& plane = planeBody.collider().plane;
    const auto projectedRadius = [&](const Obb& obb) {
        return obb.half.x * std::abs(dot(plane.normal, obb.axis[0])) +
               obb.half.y * std::abs(dot(plane.normal, obb.axis[1])) +
               obb.half.z * std::abs(dot(plane.normal, obb.axis[2]));
    };
    // This analytic path is used only when rotational surface travel is
    // negligible. Use the physical plane rather than the speculative slop
    // boundary so the stored hit transform is comfortably inside the
    // ordinary box-plane manifold band.
    const Real conservativeRadius = projectedRadius(movingStart);
    const Real startDistance =
        dot(plane.normal, movingStart.center) -
        plane.offset - conservativeRadius;
    const Real endDistance =
        dot(plane.normal, movingEnd.center) -
        plane.offset - conservativeRadius;
    if (startDistance <= contactSlop ||
        endDistance > 0.0F ||
        endDistance >= startDistance) {
        return false;
    }

    hit.fraction = std::clamp(
        startDistance / (startDistance - endDistance),
        0.0F,
        1.0F
    );
    hit.normal = plane.normal;
    hit.transform.position =
        startTransform.position +
        (endTransform.position - startTransform.position) *
            hit.fraction;
    hit.transform.orientation = startTransform.orientation;
    hit.hasTransform = true;
    return true;
}

Real combinedStaticFriction(
    const PhysicsMaterial& lhs,
    const PhysicsMaterial& rhs
) noexcept {
    return std::sqrt(lhs.staticFriction * rhs.staticFriction);
}

Real combinedDynamicFriction(
    const PhysicsMaterial& lhs,
    const PhysicsMaterial& rhs
) noexcept {
    return std::sqrt(lhs.dynamicFriction * rhs.dynamicFriction);
}

Real finiteOr(Real value, Real fallback) noexcept {
    return std::isfinite(value) ? value : fallback;
}

Vec3 inverseInertiaAtOrientation(
    const RigidBody& body,
    const Quaternion& orientation,
    const Vec3& worldAngularMomentum
) noexcept {
    const Vec3 localMomentum =
        orientation.inverseRotate(worldAngularMomentum) -
        body.gyroscopicRotorAngularMomentumLocal();
    return orientation.rotate(
        body.inverseLocalInertiaTensor() * localMomentum
    );
}

struct RotationIntegrationResult {
    Quaternion orientation{};
    std::uint32_t microsteps{0};
    bool hitMicrostepCap{false};
    std::uint32_t nonConvergedMicrosteps{0};
    Real maximumMidpointResidual{0.0F};
};

Real matrixFrobeniusNorm(const Mat3& matrix) noexcept {
    const double sum =
        static_cast<double>(matrix.m00) * matrix.m00 +
        static_cast<double>(matrix.m01) * matrix.m01 +
        static_cast<double>(matrix.m02) * matrix.m02 +
        static_cast<double>(matrix.m10) * matrix.m10 +
        static_cast<double>(matrix.m11) * matrix.m11 +
        static_cast<double>(matrix.m12) * matrix.m12 +
        static_cast<double>(matrix.m20) * matrix.m20 +
        static_cast<double>(matrix.m21) * matrix.m21 +
        static_cast<double>(matrix.m22) * matrix.m22;
    return static_cast<Real>(std::sqrt(sum));
}

RotationIntegrationResult integrateOrientationFromAngularMomentum(
    const RigidBody& body,
    const Quaternion& startOrientation,
    const Vec3& worldAngularMomentum,
    Real deltaSeconds,
    Real maximumRotationStepRadians,
    std::uint32_t maximumRotationMicrosteps
) noexcept {
    RotationIntegrationResult result{};
    result.orientation = startOrientation;
    if (deltaSeconds <= 0.0F) {
        return result;
    }
    const Vec3 startingAngularVelocity =
        inverseInertiaAtOrientation(
            body,
            startOrientation,
            worldAngularMomentum
        );
    const Real startingAngularSpeed =
        length(startingAngularVelocity);
    if (!(startingAngularSpeed > kEpsilon) ||
        !std::isfinite(startingAngularSpeed)) {
        return result;
    }

    // Lie-group implicit midpoint iteration. World angular momentum is held
    // fixed during a torque-free drift, while angular velocity is recomputed
    // from the full inertia tensor at the estimated midpoint orientation.
    // This makes gyroscopic precession/nutation emerge from rigid-body
    // dynamics and avoids the energy blow-up of explicit Euler equations.
    // Internal wheel RPM is not carrier attitude motion. Base rotational
    // microsteps on carrier angular speed so a 100k-RPM reaction wheel can
    // remain inexpensive while a 100k-RPM rocket body is still subdivided.
    const double maximumAngularTravel =
        static_cast<double>(startingAngularSpeed) *
        static_cast<double>(deltaSeconds);
    const double requestedMicrosteps = std::ceil(
        maximumAngularTravel /
        static_cast<double>(maximumRotationStepRadians)
    );
    const std::uint32_t safeMicrostepCap =
        std::max<std::uint32_t>(1, maximumRotationMicrosteps);
    if (!std::isfinite(requestedMicrosteps) ||
        requestedMicrosteps >
            static_cast<double>(safeMicrostepCap)) {
        result.microsteps = safeMicrostepCap;
        result.hitMicrostepCap = true;
    } else {
        result.microsteps = std::clamp<std::uint32_t>(
            static_cast<std::uint32_t>(
                std::max(1.0, requestedMicrosteps)
            ),
            1,
            safeMicrostepCap
        );
    }

    constexpr int kMaximumMidpointIterations = 8;
    constexpr Real kMidpointRelativeTolerance = 2.0e-5F;
    const Real microstepDelta =
        deltaSeconds / static_cast<Real>(result.microsteps);
    Quaternion orientation = startOrientation;
    for (std::uint32_t microstep = 0;
         microstep < result.microsteps;
         ++microstep) {
        Vec3 midpointAngularVelocity = inverseInertiaAtOrientation(
            body,
            orientation,
            worldAngularMomentum
        );
        Real residual = 0.0F;
        for (int iteration = 0;
             iteration < kMaximumMidpointIterations;
             ++iteration) {
            const Quaternion midpointOrientation =
                integrateWorldAngularVelocity(
                    orientation,
                    midpointAngularVelocity,
                    microstepDelta * 0.5F
                );
            const Vec3 nextAngularVelocity =
                inverseInertiaAtOrientation(
                    body,
                    midpointOrientation,
                    worldAngularMomentum
                );
            residual =
                length(nextAngularVelocity - midpointAngularVelocity) /
                std::max(1.0F, length(nextAngularVelocity));
            midpointAngularVelocity = nextAngularVelocity;
            if (residual <= kMidpointRelativeTolerance) {
                break;
            }
        }
        result.maximumMidpointResidual =
            std::max(result.maximumMidpointResidual, residual);
        if (residual > kMidpointRelativeTolerance) {
            ++result.nonConvergedMicrosteps;
        }
        orientation = integrateWorldAngularVelocity(
            orientation,
            midpointAngularVelocity,
            microstepDelta
        );
        if (!isFinite(orientation)) {
            result.orientation = startOrientation;
            result.hitMicrostepCap = true;
            ++result.nonConvergedMicrosteps;
            return result;
        }
    }
    result.orientation = orientation;
    return result;
}

Real boxPlaneSeparation(
    const RigidBody& body,
    const Transform& transform,
    const PlaneCollider& plane
) noexcept {
    const Obb obb = makeObbAtTransform(body, transform);
    const Real projectedRadius =
        obb.half.x * std::abs(dot(plane.normal, obb.axis[0])) +
        obb.half.y * std::abs(dot(plane.normal, obb.axis[1])) +
        obb.half.z * std::abs(dot(plane.normal, obb.axis[2]));
    return dot(plane.normal, obb.center) -
           plane.offset - projectedRadius;
}

bool sweptRotatingBoxAgainstPlane(
    const RigidBody& body,
    const Transform& startTransform,
    const Transform& endTransform,
    const Vec3& worldAngularMomentum,
    Real deltaSeconds,
    Real contactSlop,
    Real maximumRotationStepRadians,
    std::uint32_t maximumRotationMicrosteps,
    const RigidBody& planeBody,
    SweepHit& hit,
    RotationalCcdWork& work
) noexcept {
    const PlaneCollider& plane = planeBody.collider().plane;
    if (deltaSeconds <= 0.0F) {
        return false;
    }

    const Vec3 startCenter =
        geometryCenterAt(body, startTransform);
    const Vec3 endCenter =
        geometryCenterAt(body, endTransform);
    const Real boundingRadius = length(body.collider().box.halfExtents);
    // Torque-free kinetic energy gives a much tighter safe speed bound than
    // ||I^-1||*||H|| for slender bodies spinning about their high-inertia
    // axis, while still covering any nutation within this drift.
    const Real angularSpeedBound = std::sqrt(std::max(
        0.0F,
        2.0F * body.rotationalKineticEnergy() *
            matrixFrobeniusNorm(
                body.inverseLocalInertiaTensor()
            )
    ));
    const Real separationTravelBound =
        std::abs(dot(plane.normal, endCenter - startCenter)) +
        boundingRadius * angularSpeedBound * deltaSeconds;
    if (!(separationTravelBound > kEpsilon) ||
        !std::isfinite(separationTravelBound)) {
        return false;
    }

    struct PathSample {
        Real fraction{0.0F};
        Transform transform{};
        Real separation{0.0F};
        bool finite{false};
    };
    const Real handoffMargin = std::max(
        1.0e-6F,
        std::min(contactSlop * 0.05F, 1.0e-4F)
    );
    const Real manifoldHandoffDistance =
        std::max(0.0F, contactSlop - handoffMargin);

    // Every probe is evaluated on the same direct-from-start torque-free
    // path. Chaining short orientation integrations here produced a different
    // pose than integrateDrift's final direct integration, which could report
    // a TOI but hand the contact solver a transform with no manifold.
    const auto samplePath = [&](Real rawFraction) {
        PathSample sample{};
        sample.fraction = std::clamp(rawFraction, 0.0F, 1.0F);
        if (sample.fraction <= 0.0F) {
            sample.transform = startTransform;
        } else if (sample.fraction >= 1.0F) {
            sample.transform = endTransform;
        } else {
            sample.transform.position =
                startTransform.position +
                (endTransform.position - startTransform.position) *
                    sample.fraction;
            sample.transform.orientation =
                integrateOrientationFromAngularMomentum(
                    body,
                    startTransform.orientation,
                    worldAngularMomentum,
                    deltaSeconds * sample.fraction,
                    maximumRotationStepRadians,
                    maximumRotationMicrosteps
                ).orientation;
        }
        sample.separation =
            boxPlaneSeparation(body, sample.transform, plane) -
            manifoldHandoffDistance;
        sample.finite =
            isFinite(sample.transform.position) &&
            isFinite(sample.transform.orientation) &&
            std::isfinite(sample.separation);
        ++work.advances;
        return sample;
    };

    constexpr std::uint32_t kMaximumProbeCount = 32;
    constexpr std::uint32_t kMaximumConservativeAdvances = 8;
    constexpr std::uint32_t kBisectionIterations = 12;
    constexpr Real kSafetyFactor = 0.95F;
    constexpr Real kMinimumProgressFraction = 1.0e-6F;

    const auto commitBracket = [&](PathSample outside, PathSample inside) {
        if (!outside.finite || !inside.finite ||
            !(outside.separation > 0.0F) ||
            !(inside.separation <= 0.0F)) {
            return false;
        }
        for (std::uint32_t iteration = 0;
             iteration < kBisectionIterations &&
             work.advances < kMaximumProbeCount;
             ++iteration) {
            const PathSample middle =
                samplePath((outside.fraction + inside.fraction) * 0.5F);
            if (!middle.finite) {
                break;
            }
            if (middle.separation <= 0.0F) {
                inside = middle;
            } else {
                outside = middle;
            }
        }
        // Retain the exact inside sample. This uses the same contactSlop
        // boundary as the ordinary manifold, so it creates a same-substep
        // contact without an arbitrary fraction nudge.
        hit.fraction = inside.fraction;
        hit.normal = plane.normal;
        hit.transform = inside.transform;
        hit.hasTransform = true;
        work.convergedToContact = true;
        return true;
    };

    PathSample current = samplePath(0.0F);
    if (!current.finite ||
        current.separation <= handoffMargin) {
        // A body already in the speculative band is handled by the ordinary
        // manifold after this short adaptive drift. Clamping it at fraction
        // zero pins legitimate edge-pivot motion.
        return false;
    }

    const PathSample end = samplePath(1.0F);
    if (!end.finite) {
        work.hitAdvanceCap = true;
        return false;
    }
    if (end.separation <= 0.0F) {
        return commitBracket(current, end);
    }
    // The tight lower envelope of two samples of an L-Lipschitz function.
    // A positive value certifies that the whole drift remains outside.
    const Real wholePathLowerBound =
        0.5F *
        (current.separation + end.separation -
         separationTravelBound);
    if (wholePathLowerBound > 0.0F) {
        return false;
    }

    std::uint32_t conservativeAdvances = 0;
    for (; conservativeAdvances < kMaximumConservativeAdvances &&
           current.fraction < 1.0F &&
           work.advances + kBisectionIterations < kMaximumProbeCount;
         ++conservativeAdvances) {
        const Real remainingFraction = 1.0F - current.fraction;
        const Real requestedAdvance =
            kSafetyFactor *
            current.separation / separationTravelBound;
        const Real safeAdvance =
            std::min(requestedAdvance, remainingFraction);
        if (!(safeAdvance > kMinimumProgressFraction) ||
            current.fraction + safeAdvance <= current.fraction) {
            break;
        }
        const Real nextFraction =
            std::min(1.0F, current.fraction + safeAdvance);
        const PathSample next =
            nextFraction >= 1.0F
                ? end
                : samplePath(nextFraction);
        if (!next.finite) {
            work.hitAdvanceCap = true;
            return false;
        }
        if (next.separation <= 0.0F) {
            return commitBracket(current, next);
        }
        if (nextFraction >= 1.0F) {
            return false;
        }
        current = next;
    }

    // The fallback walks the remaining exact path with a finite spatial
    // budget. It may only report a collision after observing an
    // outside-to-inside pair; a clear scan is clear, never a speculative hit.
    const Vec3 half = body.collider().box.halfExtents;
    const Real minimumHalfExtent =
        std::min({half.x, half.y, half.z});
    const Real sweepResolution = std::max(
        1.0e-5F,
        std::min(
            contactSlop * 0.25F,
            minimumHalfExtent * 0.05F
        )
    );
    const Real remainingTravelBound =
        separationTravelBound * (1.0F - current.fraction);
    const std::uint32_t availableFallbackSamples =
        work.advances + kBisectionIterations < kMaximumProbeCount
            ? kMaximumProbeCount - work.advances - kBisectionIterations
            : 0U;
    if (availableFallbackSamples == 0U) {
        return false;
    }
    const double requestedFallbackSamples = std::ceil(
        static_cast<double>(remainingTravelBound) /
        static_cast<double>(sweepResolution)
    );
    const std::uint32_t fallbackSamples =
        std::clamp<std::uint32_t>(
            std::isfinite(requestedFallbackSamples)
                ? static_cast<std::uint32_t>(std::min(
                    requestedFallbackSamples,
                    static_cast<double>(
                        std::numeric_limits<std::uint32_t>::max()
                    )
                ))
                : availableFallbackSamples,
            1U,
            availableFallbackSamples
        );
    const bool fallbackWasCapped =
        requestedFallbackSamples >
        static_cast<double>(availableFallbackSamples);
    PathSample outside = current;
    for (std::uint32_t sample = 1U;
         sample <= fallbackSamples;
         ++sample) {
        const Real high =
            current.fraction +
            (1.0F - current.fraction) *
                (static_cast<Real>(sample) /
                 static_cast<Real>(fallbackSamples));
        const PathSample insideCandidate = samplePath(high);
        if (!insideCandidate.finite) {
            return false;
        }
        if (insideCandidate.separation <= 0.0F) {
            return commitBracket(outside, insideCandidate);
        }
        outside = insideCandidate;
    }

    if (fallbackWasCapped) {
        work.hitAdvanceCap = true;
    }
    return false;
}

SpringForce sanitizedSpring(SpringForce spring) noexcept {
    const bool validAnchor = isFinite(spring.localAnchor);
    const bool validTarget = isFinite(spring.worldTarget);
    if (!validAnchor) {
        spring.localAnchor = {};
    }
    if (!validTarget) {
        spring.worldTarget = {};
    }
    spring.stiffness =
        std::max(0.0F, finiteOr(spring.stiffness, 0.0F));
    spring.damping =
        std::max(0.0F, finiteOr(spring.damping, 0.0F));
    spring.maximumForce =
        std::max(0.0F, finiteOr(spring.maximumForce, 0.0F));
    spring.enabled = spring.enabled && validAnchor && validTarget;
    return spring;
}

bool validTimedBodyForce(const TimedBodyForce& force) noexcept {
    return force.body != kInvalidBodyId &&
           isFinite(force.force) &&
           isFinite(force.torque) &&
           std::isfinite(force.remainingSeconds) &&
           force.remainingSeconds > 0.0F;
}

} // namespace

World::~World() = default;
World::World(World&&) noexcept = default;
World& World::operator=(World&&) noexcept = default;

World::World(const PhysicsSettings& settings) {
    setSettings(settings);
    bodies_.reserve(128);
    drones_.reserve(16);
    springs_.reserve(16);
    timedForces_.reserve(16);
    contacts_.reserve(512);
    debugContacts_.reserve(512);
}

const PhysicsSettings& World::settings() const noexcept {
    return settings_;
}

void World::setSettings(const PhysicsSettings& settings) noexcept {
    const PhysicsSettings defaults{};
    const std::uint32_t previousFixedUpdateHz = settings_.fixedUpdateHz;
    const double previousPhase =
        previousFixedUpdateHz > 0U
            ? accumulatorSeconds_ *
                  static_cast<double>(previousFixedUpdateHz)
            : 0.0;

    settings_ = settings;
    settings_.fixedUpdateHz = std::clamp<std::uint32_t>(
        settings_.fixedUpdateHz,
        30,
        1'000
    );
    settings_.solverSubsteps =
        std::clamp<std::uint32_t>(settings_.solverSubsteps, 1, 16);
    settings_.maximumAdaptiveSubsteps = std::clamp<std::uint32_t>(
        settings_.maximumAdaptiveSubsteps,
        settings_.solverSubsteps,
        64
    );
    settings_.maximumTravelFraction = std::clamp(
        finiteOr(
            settings_.maximumTravelFraction,
            defaults.maximumTravelFraction
        ),
        0.1F,
        1.0F
    );
    settings_.maximumRotationStepRadians = std::clamp(
        finiteOr(
            settings_.maximumRotationStepRadians,
            defaults.maximumRotationStepRadians
        ),
        radians(0.25F),
        radians(90.0F)
    );
    settings_.maximumRotationMicrosteps =
        std::clamp<std::uint32_t>(
            settings_.maximumRotationMicrosteps,
            1,
            256
        );
    settings_.velocityIterations =
        std::clamp<std::uint32_t>(settings_.velocityIterations, 1, 64);
    settings_.contactSlop = std::clamp(
        finiteOr(settings_.contactSlop, defaults.contactSlop),
        0.00001F,
        0.05F
    );
    settings_.restitutionVelocityThreshold = std::max(
        0.0F,
        finiteOr(
            settings_.restitutionVelocityThreshold,
            defaults.restitutionVelocityThreshold
        )
    );
    settings_.penetrationVelocityFactor = std::clamp(
        finiteOr(
            settings_.penetrationVelocityFactor,
            defaults.penetrationVelocityFactor
        ),
        0.0F,
        1.0F
    );
    settings_.maximumDepenetrationVelocity = std::max(
        0.0F,
        finiteOr(
            settings_.maximumDepenetrationVelocity,
            defaults.maximumDepenetrationVelocity
        )
    );
    settings_.positionCorrectionFraction = std::clamp(
        finiteOr(
            settings_.positionCorrectionFraction,
            defaults.positionCorrectionFraction
        ),
        0.0F,
        1.0F
    );
    settings_.sleepLinearSpeed = std::max(
        0.0F,
        finiteOr(settings_.sleepLinearSpeed, defaults.sleepLinearSpeed)
    );
    settings_.sleepAngularSpeed = std::max(
        0.0F,
        finiteOr(settings_.sleepAngularSpeed, defaults.sleepAngularSpeed)
    );
    settings_.sleepDelaySeconds = std::max(
        0.0F,
        finiteOr(settings_.sleepDelaySeconds, defaults.sleepDelaySeconds)
    );
    settings_.sleepSupportMargin = std::clamp(
        finiteOr(
            settings_.sleepSupportMargin,
            defaults.sleepSupportMargin
        ),
        0.0F,
        0.1F
    );
    settings_.maximumCatchUpSteps =
        std::clamp<std::uint32_t>(settings_.maximumCatchUpSteps, 1, 64);

    if (previousFixedUpdateHz != settings_.fixedUpdateHz) {
        const double normalizedPhase =
            std::clamp(previousPhase, 0.0, 1.0);
        accumulatorSeconds_ =
            normalizedPhase / static_cast<double>(settings_.fixedUpdateHz);
    }
}

Environment& World::environment() noexcept {
    return environment_;
}

const Environment& World::environment() const noexcept {
    return environment_;
}

BodyId World::createBody(const BodyDescription& description) {
    const BodyId id = static_cast<BodyId>(bodies_.size() + 1);
    bodies_.push_back(RigidBody{id, description});
    bodies_.back().previousTransform_ = bodies_.back().transform_;
    return id;
}

bool World::destroyBody(BodyId id) noexcept {
    RigidBody* selectedBody = body(id);
    if (selectedBody == nullptr) {
        return false;
    }
    selectedBody->alive_ = false;
    selectedBody->sleeping_ = true;
    selectedBody->clearAccumulators();
    for (SpringSlot& spring : springs_) {
        if (spring.alive && spring.force.body == id) {
            spring.alive = false;
        }
    }
    for (TimedForceSlot& timedForce : timedForces_) {
        if (timedForce.alive && timedForce.force.body == id) {
            timedForce.alive = false;
            timedForce.tickScale = 0.0F;
            timedForce.remainingSeconds = 0.0;
        }
    }
    for (Drone& selectedDrone : drones_) {
        if (selectedDrone.alive_ &&
            selectedDrone.bodyId_ == id) {
            selectedDrone.alive_ = false;
            selectedDrone.armed_ = false;
        }
    }
    return true;
}

RigidBody* World::body(BodyId id) noexcept {
    if (id == kInvalidBodyId || id > bodies_.size()) {
        return nullptr;
    }
    RigidBody& selectedBody = bodies_[static_cast<std::size_t>(id - 1)];
    return selectedBody.alive_ ? &selectedBody : nullptr;
}

const RigidBody* World::body(BodyId id) const noexcept {
    if (id == kInvalidBodyId || id > bodies_.size()) {
        return nullptr;
    }
    const RigidBody& selectedBody = bodies_[static_cast<std::size_t>(id - 1)];
    return selectedBody.alive_ ? &selectedBody : nullptr;
}

std::size_t World::bodySlotCount() const noexcept {
    return bodies_.size();
}

void World::wakeAllDynamic() noexcept {
    for (RigidBody& rigidBody : bodies_) {
        if (rigidBody.alive_ &&
            rigidBody.motionType_ == MotionType::Dynamic) {
            rigidBody.wake();
        }
    }
}

DroneId World::createDrone(const DroneDescription& description) {
    const DroneId createdDrone =
        static_cast<DroneId>(drones_.size() + 1U);
    Drone candidate{
        createdDrone,
        kInvalidBodyId,
        description
    };
    const DroneDescription& sanitized = candidate.description();
    BodyDescription bodyDescription{};
    bodyDescription.motionType = MotionType::Dynamic;
    bodyDescription.collider =
        Collider::makeBox(sanitized.bodyHalfExtents);
    bodyDescription.transform = sanitized.transform;
    bodyDescription.mass = sanitized.mass;
    bodyDescription.material = sanitized.material;
    bodyDescription.aerodynamics = sanitized.aerodynamics;
    bodyDescription.allowSleep = false;
    bodyDescription.debugName = sanitized.debugName;

    const BodyId createdBody = createBody(bodyDescription);
    candidate.bodyId_ = createdBody;
    RigidBody* rigidBody = body(createdBody);
    if (rigidBody == nullptr || !candidate.initializeRotors(*rigidBody)) {
        destroyBody(createdBody);
        return kInvalidDroneId;
    }
    candidate.resetControllerAtBody(*rigidBody);
    try {
        drones_.push_back(std::move(candidate));
    } catch (...) {
        destroyBody(createdBody);
        throw;
    }
    return createdDrone;
}

bool World::destroyDrone(DroneId id) noexcept {
    Drone* selectedDrone = drone(id);
    if (selectedDrone == nullptr) {
        return false;
    }
    const BodyId selectedBody = selectedDrone->bodyId_;
    selectedDrone->alive_ = false;
    selectedDrone->armed_ = false;
    if (body(selectedBody) != nullptr) {
        destroyBody(selectedBody);
    }
    return true;
}

Drone* World::drone(DroneId id) noexcept {
    if (id == kInvalidDroneId || id > drones_.size()) {
        return nullptr;
    }
    Drone& selected =
        drones_[static_cast<std::size_t>(id - 1U)];
    return selected.alive_ ? &selected : nullptr;
}

const Drone* World::drone(DroneId id) const noexcept {
    if (id == kInvalidDroneId || id > drones_.size()) {
        return nullptr;
    }
    const Drone& selected =
        drones_[static_cast<std::size_t>(id - 1U)];
    return selected.alive_ ? &selected : nullptr;
}

Drone* World::droneForBody(BodyId bodyId) noexcept {
    for (Drone& selected : drones_) {
        if (selected.alive_ && selected.bodyId_ == bodyId) {
            return &selected;
        }
    }
    return nullptr;
}

const Drone* World::droneForBody(BodyId bodyId) const noexcept {
    for (const Drone& selected : drones_) {
        if (selected.alive_ && selected.bodyId_ == bodyId) {
            return &selected;
        }
    }
    return nullptr;
}

std::size_t World::droneSlotCount() const noexcept {
    return drones_.size();
}

void World::reset() noexcept {
    bodies_.clear();
    drones_.clear();
    springs_.clear();
    timedForces_.clear();
    contacts_.clear();
    debugContacts_.clear();
    debugStats_ = {};
    accumulatorSeconds_ = 0.0;
}

bool World::raycast(
    const Vec3& origin,
    const Vec3& direction,
    Real maximumDistance,
    RaycastHit& hit
) const noexcept {
    if (!isFinite(origin) || !isFinite(direction) ||
        maximumDistance <= 0.0F || lengthSquared(direction) <= kEpsilon) {
        return false;
    }

    const Vec3 rayDirection = normalized(direction);
    bool found = false;
    Real closestDistance = maximumDistance;
    RaycastHit closest{};

    for (const RigidBody& rigidBody : bodies_) {
        if (!rigidBody.alive_) {
            continue;
        }

        Real distance = 0.0F;
        Vec3 normal{};
        bool intersects = false;

        if (rigidBody.collider_.type == ColliderType::Plane) {
            const PlaneCollider& plane = rigidBody.collider_.plane;
            const Real denominator = dot(plane.normal, rayDirection);
            if (std::abs(denominator) > kEpsilon) {
                distance =
                    (plane.offset - dot(plane.normal, origin)) / denominator;
                if (distance >= 0.0F && distance <= closestDistance) {
                    normal = denominator < 0.0F ? plane.normal : -plane.normal;
                    intersects = true;
                }
            }
        } else {
            const Vec3 geometryCenter = rigidBody.worldPointFromLocal({});
            const Quaternion inverseOrientation =
                rigidBody.transform_.orientation.conjugate();
            const Vec3 localOrigin =
                inverseOrientation.rotate(origin - geometryCenter);
            const Vec3 localDirection =
                inverseOrientation.rotate(rayDirection);
            const Vec3 half = rigidBody.collider_.box.halfExtents;

            Real entry = 0.0F;
            Real exit = closestDistance;
            Vec3 entryNormalLocal{};
            bool missed = false;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const Real originComponent = component(localOrigin, axis);
                const Real directionComponent = component(localDirection, axis);
                const Real halfExtent = component(half, axis);
                if (std::abs(directionComponent) <= kEpsilon) {
                    if (originComponent < -halfExtent ||
                        originComponent > halfExtent) {
                        missed = true;
                        break;
                    }
                    continue;
                }

                Real nearDistance =
                    (-halfExtent - originComponent) / directionComponent;
                Real farDistance =
                    (halfExtent - originComponent) / directionComponent;
                Real nearNormalSign = -1.0F;
                if (nearDistance > farDistance) {
                    std::swap(nearDistance, farDistance);
                    nearNormalSign = 1.0F;
                }
                if (nearDistance > entry) {
                    entry = nearDistance;
                    entryNormalLocal = {};
                    if (axis == 0) {
                        entryNormalLocal.x = nearNormalSign;
                    } else if (axis == 1) {
                        entryNormalLocal.y = nearNormalSign;
                    } else {
                        entryNormalLocal.z = nearNormalSign;
                    }
                }
                exit = std::min(exit, farDistance);
                if (entry > exit) {
                    missed = true;
                    break;
                }
            }

            if (!missed && exit >= 0.0F && entry <= closestDistance) {
                distance = std::max(0.0F, entry);
                normal = rigidBody.transform_.orientation.rotate(entryNormalLocal);
                if (lengthSquared(normal) <= kEpsilon) {
                    normal = -rayDirection;
                }
                intersects = true;
            }
        }

        if (intersects && distance <= closestDistance) {
            closestDistance = distance;
            closest = {
                rigidBody.id_,
                origin + rayDirection * distance,
                normalized(normal, -rayDirection),
                distance,
            };
            found = true;
        }
    }

    if (found) {
        hit = closest;
    }
    return found;
}

SpringId World::createSpring(const SpringForce& spring) {
    const SpringForce cleanSpring = sanitizedSpring(spring);
    for (std::size_t index = 0; index < springs_.size(); ++index) {
        if (!springs_[index].alive) {
            springs_[index] = {cleanSpring, true};
            return static_cast<SpringId>(index + 1);
        }
    }
    springs_.push_back({cleanSpring, true});
    return static_cast<SpringId>(springs_.size());
}

bool World::updateSpring(SpringId id, const SpringForce& spring) noexcept {
    if (id == kInvalidSpringId || id > springs_.size()) {
        return false;
    }
    SpringSlot& slot = springs_[static_cast<std::size_t>(id - 1)];
    if (!slot.alive) {
        return false;
    }
    slot.force = sanitizedSpring(spring);
    if (RigidBody* selectedBody = body(slot.force.body)) {
        selectedBody->wake();
    }
    return true;
}

bool World::setSpringTarget(SpringId id, const Vec3& worldTarget) noexcept {
    if (id == kInvalidSpringId || id > springs_.size() || !isFinite(worldTarget)) {
        return false;
    }
    SpringSlot& slot = springs_[static_cast<std::size_t>(id - 1)];
    if (!slot.alive) {
        return false;
    }
    slot.force.worldTarget = worldTarget;
    if (RigidBody* selectedBody = body(slot.force.body)) {
        selectedBody->wake();
    }
    return true;
}

bool World::destroySpring(SpringId id) noexcept {
    if (id == kInvalidSpringId || id > springs_.size()) {
        return false;
    }
    SpringSlot& slot = springs_[static_cast<std::size_t>(id - 1)];
    if (!slot.alive) {
        return false;
    }
    slot.alive = false;
    return true;
}

TimedForceId World::createTimedForce(const TimedBodyForce& force) {
    if (!validTimedBodyForce(force)) {
        return kInvalidTimedForceId;
    }
    RigidBody* selectedBody = body(force.body);
    if (selectedBody == nullptr ||
        selectedBody->motionType_ != MotionType::Dynamic) {
        return kInvalidTimedForceId;
    }

    const auto makeSlot = [&force]() {
        TimedForceSlot slot{};
        slot.force = force;
        slot.remainingSeconds =
            static_cast<double>(force.remainingSeconds);
        slot.tickScale = 0.0F;
        slot.alive = true;
        return slot;
    };

    // Timed-force handles remain monotonic for the lifetime of a world, like
    // BodyId. Do not recycle an expired slot: a stale editor handle must never
    // alias a later force.
    timedForces_.push_back(makeSlot());
    selectedBody->wake();
    return static_cast<TimedForceId>(timedForces_.size());
}

bool World::updateTimedForce(
    TimedForceId id,
    const TimedBodyForce& force
) noexcept {
    if (id == kInvalidTimedForceId || id > timedForces_.size() ||
        !validTimedBodyForce(force)) {
        return false;
    }
    TimedForceSlot& slot =
        timedForces_[static_cast<std::size_t>(id - 1)];
    if (!slot.alive) {
        return false;
    }
    RigidBody* selectedBody = body(force.body);
    if (selectedBody == nullptr ||
        selectedBody->motionType_ != MotionType::Dynamic) {
        return false;
    }

    slot.force = force;
    slot.remainingSeconds =
        static_cast<double>(force.remainingSeconds);
    slot.tickScale = 0.0F;
    selectedBody->wake();
    return true;
}

bool World::destroyTimedForce(TimedForceId id) noexcept {
    if (id == kInvalidTimedForceId || id > timedForces_.size()) {
        return false;
    }
    TimedForceSlot& slot =
        timedForces_[static_cast<std::size_t>(id - 1)];
    if (!slot.alive) {
        return false;
    }
    slot.alive = false;
    slot.tickScale = 0.0F;
    slot.remainingSeconds = 0.0;
    return true;
}

void World::advanceDrones(Real fixedDeltaSeconds) noexcept {
    for (Drone& selected : drones_) {
        if (!selected.alive_) {
            continue;
        }
        if (body(selected.bodyId_) == nullptr) {
            selected.alive_ = false;
            selected.armed_ = false;
            continue;
        }
        selected.advance(*this, fixedDeltaSeconds);
    }
}

void World::sampleDroneSensors(Real fixedDeltaSeconds) noexcept {
    for (Drone& selected : drones_) {
        if (!selected.alive_) {
            continue;
        }
        selected.sampleSensors(
            *this,
            fixedDeltaSeconds,
            debugStats_.simulationTime
        );
    }
}

void World::stepFixed() {
    debugStats_.broadphasePairs = 0;
    debugStats_.narrowphaseTests = 0;
    debugStats_.contactCount = 0;
    debugStats_.rotationalCcdHits = 0;
    debugStats_.rotationalCcdAdvances = 0;
    debugStats_.rotationalCcdConvergenceHits = 0;
    debugStats_.rotationalCcdAdvanceCapHits = 0;
    debugStats_.rotationMicrosteps = 0;
    debugStats_.maximumRotationMicrostepsUsed = 0;
    debugStats_.rotationMicrostepCapHits = 0;
    debugStats_.rotationMidpointNonConvergenceCount = 0;
    debugStats_.maximumRotationMidpointResidual = 0.0F;
    debugStats_.velocityIterations = settings_.velocityIterations;

    const Real fixedDelta =
        1.0F / static_cast<Real>(settings_.fixedUpdateHz);
    advanceDrones(fixedDelta);
    prepareTimedForces(fixedDelta);

    for (RigidBody& rigidBody : bodies_) {
        if (rigidBody.alive_) {
            rigidBody.previousTransform_ = rigidBody.transform_;
        }
    }

    // Evaluate the current force field before choosing the adaptive count.
    // This makes the conservative travel estimate include gravity, drag,
    // springs, and one-tick editor forces rather than only the initial speed.
    accumulateForces(0.0F);
    std::vector<Vec3> predictedLinearAcceleration(bodies_.size());
    std::vector<Vec3> predictedAngularAcceleration(bodies_.size());
    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        RigidBody& rigidBody = bodies_[index];
        if (!rigidBody.alive_ ||
            rigidBody.motionType_ != MotionType::Dynamic ||
            rigidBody.sleeping_) {
            continue;
        }
        predictedLinearAcceleration[index] =
            rigidBody.evaluatedForce_ * rigidBody.inverseMass_;
        Vec3 internalRotorTorqueLocal{};
        for (const RigidBody::RotorSlot& rotor : rigidBody.rotors_) {
            if (rotor.alive) {
                internalRotorTorqueLocal +=
                    rotor.state.axisLocal * rotor.evaluatedTorque;
            }
        }
        const Vec3 gyroscopicTorque =
            cross(
                rigidBody.angularVelocity_,
                rigidBody.worldAngularMomentum_
            );
        predictedAngularAcceleration[index] =
            rigidBody.multiplyWorldInverseInertia(
                rigidBody.evaluatedTorque_ -
                rigidBody.transform_.orientation.rotate(
                    internalRotorTorqueLocal
                ) -
                gyroscopicTorque
            );
    }

    std::uint32_t actualSubsteps = settings_.solverSubsteps;
    const auto includeTravelRequirement =
        [&](Real expectedTravel, Real allowedTravel) {
            if (!std::isfinite(expectedTravel) ||
                !std::isfinite(allowedTravel) ||
                allowedTravel <= 0.0F) {
                actualSubsteps = settings_.maximumAdaptiveSubsteps;
                return;
            }
            const Real ratio =
                std::max(0.0F, expectedTravel) / allowedTravel;
            if (ratio >=
                static_cast<Real>(settings_.maximumAdaptiveSubsteps)) {
                actualSubsteps = settings_.maximumAdaptiveSubsteps;
                return;
            }
            const auto requiredSubsteps = static_cast<std::uint32_t>(
                std::ceil(ratio)
            );
            actualSubsteps = std::max(actualSubsteps, requiredSubsteps);
        };

    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        const RigidBody& rigidBody = bodies_[index];
        if (!rigidBody.alive_ || rigidBody.motionType_ != MotionType::Dynamic ||
            rigidBody.sleeping_ || rigidBody.collider_.type != ColliderType::Box) {
            continue;
        }
        const Vec3 half = rigidBody.collider_.box.halfExtents;
        const Real minimumHalfExtent = std::min({half.x, half.y, half.z});
        const Real maximumHalfExtent = std::max({half.x, half.y, half.z});
        const Real medianHalfExtent =
            half.x + half.y + half.z -
            minimumHalfExtent - maximumHalfExtent;
        const bool plateLike =
            medianHalfExtent >= minimumHalfExtent * 8.0F &&
            maximumHalfExtent <= medianHalfExtent * 4.0F;
        const Real adaptiveFeature =
            plateLike ? medianHalfExtent : minimumHalfExtent;
        const Real boundingRadius = length(half);
        const Real expectedTravel =
            length(rigidBody.linearVelocity_) * fixedDelta +
            0.5F * length(predictedLinearAcceleration[index]) *
                fixedDelta * fixedDelta +
            boundingRadius *
                (length(rigidBody.angularVelocity_) * fixedDelta +
                 0.5F * length(predictedAngularAcceleration[index]) *
                     fixedDelta * fixedDelta);
        // One very small dimension describes a plate's thickness, not the
        // distance over which its whole 2D face needs repeated broadphase and
        // contact solves. Plane CCD and rotational microsteps already protect
        // that thin axis. Using the median feature keeps small cubes and rods
        // conservative while preventing a 10 mm plate from forcing 32 full
        // solver substeps merely because it pivots about a supported edge.
        const Real allowedTravel =
            std::max(
                0.001F,
                adaptiveFeature * settings_.maximumTravelFraction
            );
        includeTravelRequirement(expectedTravel, allowedTravel);
    }

    // Individual travel is insufficient when two bodies close on each other.
    // Add a deterministic pair-relative requirement for pairs whose swept
    // bounding spheres can reach during this tick.
    for (std::size_t firstIndex = 0;
         firstIndex < bodies_.size();
         ++firstIndex) {
        const RigidBody& first = bodies_[firstIndex];
        if (!first.alive_ || first.collider_.type != ColliderType::Box) {
            continue;
        }
        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < bodies_.size();
             ++secondIndex) {
            const RigidBody& second = bodies_[secondIndex];
            if (!second.alive_ ||
                second.collider_.type != ColliderType::Box ||
                (first.motionType_ == MotionType::Static &&
                 second.motionType_ == MotionType::Static)) {
                continue;
            }

            const Vec3 velocityFirst =
                first.motionType_ == MotionType::Dynamic &&
                        !first.sleeping_
                    ? first.linearVelocity_
                    : Vec3{};
            const Vec3 velocitySecond =
                second.motionType_ == MotionType::Dynamic &&
                        !second.sleeping_
                    ? second.linearVelocity_
                    : Vec3{};
            const Vec3 relativeAcceleration =
                predictedLinearAcceleration[secondIndex] -
                predictedLinearAcceleration[firstIndex];
            const Real radiusFirst = length(first.collider_.box.halfExtents);
            const Real radiusSecond = length(second.collider_.box.halfExtents);
            const Real angularTravel =
                radiusFirst *
                    (length(first.angularVelocity_) * fixedDelta +
                     0.5F *
                         length(predictedAngularAcceleration[firstIndex]) *
                         fixedDelta * fixedDelta) +
                radiusSecond *
                    (length(second.angularVelocity_) * fixedDelta +
                     0.5F *
                         length(predictedAngularAcceleration[secondIndex]) *
                         fixedDelta * fixedDelta);
            const Real relativeTravel =
                length(velocitySecond - velocityFirst) * fixedDelta +
                0.5F * length(relativeAcceleration) *
                    fixedDelta * fixedDelta +
                angularTravel;
            const Real centerDistance =
                length(makeObb(second).center - makeObb(first).center);
            if (centerDistance >
                radiusFirst + radiusSecond + relativeTravel +
                    settings_.contactSlop) {
                continue;
            }

            const Vec3 halfFirst = first.collider_.box.halfExtents;
            const Vec3 halfSecond = second.collider_.box.halfExtents;
            const Real pairFeature =
                std::min({halfFirst.x, halfFirst.y, halfFirst.z}) +
                std::min({halfSecond.x, halfSecond.y, halfSecond.z});
            includeTravelRequirement(
                relativeTravel,
                std::max(
                    0.001F,
                    pairFeature * settings_.maximumTravelFraction
                )
            );
        }
    }
    actualSubsteps =
        std::min(actualSubsteps, settings_.maximumAdaptiveSubsteps);
    debugStats_.internalSubsteps = actualSubsteps;

    const Real substepDelta =
        fixedDelta / static_cast<Real>(actualSubsteps);
    for (std::uint32_t substep = 0; substep < actualSubsteps; ++substep) {
        simulateSubstep(substepDelta);
        debugStats_.simulationTime += static_cast<double>(substepDelta);
    }

    sampleDroneSensors(fixedDelta);
    expireTimedForces(fixedDelta);
    clearExternalForces();
    ++debugStats_.fixedTick;

    debugStats_.bodyCount = 0;
    debugStats_.awakeBodyCount = 0;
    for (const RigidBody& rigidBody : bodies_) {
        if (!rigidBody.alive_) {
            continue;
        }
        ++debugStats_.bodyCount;
        if (rigidBody.motionType_ == MotionType::Dynamic && !rigidBody.sleeping_) {
            ++debugStats_.awakeBodyCount;
        }
    }
}

std::uint32_t World::advance(double realDeltaSeconds) {
    if (!std::isfinite(realDeltaSeconds) || realDeltaSeconds <= 0.0) {
        return 0;
    }

    const double fixedDelta =
        1.0 / static_cast<double>(settings_.fixedUpdateHz);
    accumulatorSeconds_ += std::min(realDeltaSeconds, 0.25);

    std::uint32_t steps = 0;
    while (accumulatorSeconds_ + 1.0e-12 >= fixedDelta &&
           steps < settings_.maximumCatchUpSteps) {
        stepFixed();
        accumulatorSeconds_ -= fixedDelta;
        ++steps;
    }

    if (steps == settings_.maximumCatchUpSteps && accumulatorSeconds_ >= fixedDelta) {
        const double retained = std::fmod(accumulatorSeconds_, fixedDelta);
        debugStats_.droppedRealTime += accumulatorSeconds_ - retained;
        accumulatorSeconds_ = retained;
    }
    return steps;
}

Real World::interpolationAlpha() const noexcept {
    if (!std::isfinite(accumulatorSeconds_) ||
        accumulatorSeconds_ <= 0.0 ||
        settings_.fixedUpdateHz == 0U) {
        return 0.0F;
    }
    const double phase =
        accumulatorSeconds_ *
        static_cast<double>(settings_.fixedUpdateHz);
    if (!std::isfinite(phase)) {
        return 0.0F;
    }
    return static_cast<Real>(std::clamp(phase, 0.0, 1.0));
}

const PhysicsDebugStats& World::debugStats() const noexcept {
    return debugStats_;
}

const std::vector<ContactDebugPoint>& World::debugContacts() const noexcept {
    return debugContacts_;
}

void World::simulateSubstep(Real deltaSeconds) {
    for (RigidBody& rigidBody : bodies_) {
        rigidBody.touchedThisStep_ = false;
    }

    // Kick-drift-kick is the velocity form of Verlet. Environmental and spring
    // forces are evaluated at both ends of the drift.
    accumulateForces(0.0F);
    integrateKick(deltaSeconds * 0.5F);
    integrateDrift(deltaSeconds);
    accumulateForces(deltaSeconds);
    integrateKick(deltaSeconds * 0.5F);

    detectContacts();
    solveContacts(deltaSeconds);
    updateSleeping(deltaSeconds);
}

void World::accumulateForces(Real timeOffsetSeconds) {
    for (RigidBody& rigidBody : bodies_) {
        rigidBody.evaluatedForce_ = rigidBody.forceAccumulator_;
        rigidBody.evaluatedTorque_ = rigidBody.torqueAccumulator_;
        for (RigidBody::RotorSlot& rotor : rigidBody.rotors_) {
            if (!rotor.alive) {
                continue;
            }
            rotor.evaluatedTorque =
                rotor.state.motorTorqueCommand +
                rotor.torqueAccumulator -
                rotor.state.bearingDamping *
                    rotor.state.relativeAngularVelocity;
        }
        if (!rigidBody.alive_ || rigidBody.motionType_ != MotionType::Dynamic ||
            rigidBody.sleeping_) {
            continue;
        }

        rigidBody.evaluatedForce_ += environment_.gravity() * rigidBody.mass_;
        if (!rigidBody.aerodynamics_.enabled ||
            rigidBody.collider_.type != ColliderType::Box) {
            continue;
        }

        const Vec3 centerOfPressure =
            rigidBody.worldPointFromLocal(
                rigidBody.aerodynamics_.centerOfPressureLocal
            );
        const Vec3 windVelocity =
            environment_.windVelocity(
                centerOfPressure,
                debugStats_.simulationTime +
                    static_cast<double>(timeOffsetSeconds)
            );
        const Vec3 relativeVelocity =
            rigidBody.velocityAtWorldPoint(centerOfPressure) - windVelocity;
        const Real relativeSpeed = length(relativeVelocity);
        const AtmosphereSample atmosphere =
            environment_.sampleAtmosphere(centerOfPressure);

        if (relativeSpeed > 1.0e-4F && atmosphere.density > 0.0F) {
            const Real projectedArea =
                rigidBody.projectedBoxArea(relativeVelocity) *
                rigidBody.aerodynamics_.projectedAreaScale;
            const Vec3 dragForce =
                -0.5F * atmosphere.density *
                rigidBody.aerodynamics_.dragCoefficient * projectedArea *
                relativeSpeed * relativeVelocity;
            rigidBody.evaluatedForce_ += dragForce;
            rigidBody.evaluatedTorque_ +=
                cross(centerOfPressure - rigidBody.transform_.position, dragForce);
        }

        const Real angularSpeed = length(rigidBody.angularVelocity_);
        if (angularSpeed > 1.0e-4F && atmosphere.density > 0.0F) {
            const Vec3 fullSize = rigidBody.collider_.box.halfExtents * 2.0F;
            const Real characteristicLength =
                (fullSize.x + fullSize.y + fullSize.z) / 3.0F;
            const Real angularScale =
                0.5F * atmosphere.density *
                rigidBody.aerodynamics_.angularDragCoefficient *
                rigidBody.volume() * characteristicLength * characteristicLength;
            rigidBody.evaluatedTorque_ -=
                rigidBody.angularVelocity_ * (angularScale * angularSpeed);
        }
    }

    for (const TimedForceSlot& slot : timedForces_) {
        if (!slot.alive || !slot.force.enabled ||
            slot.tickScale <= 0.0F) {
            continue;
        }
        RigidBody* selectedBody = body(slot.force.body);
        if (selectedBody == nullptr ||
            selectedBody->motionType_ != MotionType::Dynamic) {
            continue;
        }

        selectedBody->wake();
        const Quaternion& orientation =
            selectedBody->transform_.orientation;
        const Vec3 worldForce =
            slot.force.forceInBodyFrame
                ? orientation.rotate(slot.force.force)
                : slot.force.force;
        const Vec3 worldTorque =
            slot.force.torqueInBodyFrame
                ? orientation.rotate(slot.force.torque)
                : slot.force.torque;
        selectedBody->evaluatedForce_ +=
            worldForce * slot.tickScale;
        selectedBody->evaluatedTorque_ +=
            worldTorque * slot.tickScale;
    }

    for (const SpringSlot& slot : springs_) {
        if (!slot.alive || !slot.force.enabled) {
            continue;
        }
        RigidBody* selectedBody = body(slot.force.body);
        if (selectedBody == nullptr ||
            selectedBody->motionType_ != MotionType::Dynamic) {
            continue;
        }

        const Vec3 anchor = selectedBody->worldPointFromLocal(slot.force.localAnchor);
        const Vec3 anchorVelocity = selectedBody->velocityAtWorldPoint(anchor);
        Vec3 springForce =
            (slot.force.worldTarget - anchor) * std::max(0.0F, slot.force.stiffness) -
            anchorVelocity * std::max(0.0F, slot.force.damping);
        springForce =
            clampLength(springForce, std::max(0.0F, slot.force.maximumForce));
        selectedBody->evaluatedForce_ += springForce;
        selectedBody->evaluatedTorque_ +=
            cross(anchor - selectedBody->transform_.position, springForce);
        selectedBody->wake();
    }
}

void World::integrateKick(Real halfDeltaSeconds) {
    for (RigidBody& rigidBody : bodies_) {
        if (!rigidBody.alive_ || rigidBody.motionType_ != MotionType::Dynamic ||
            rigidBody.sleeping_) {
            continue;
        }
        const Vec3 linearAcceleration =
            rigidBody.evaluatedForce_ * rigidBody.inverseMass_;
        rigidBody.worldAngularMomentum_ +=
            rigidBody.evaluatedTorque_ * halfDeltaSeconds;
        rigidBody.linearVelocity_ += linearAcceleration * halfDeltaSeconds;
        rigidBody.synchronizeAngularVelocityFromMomentum();

        // Each motor/bearing coordinate is an internal momentum exchange.
        // Solve p_dot = u - c*s analytically for the current coordinate, with
        // s affine in p. This preserves the correct u/c steady slip and keeps
        // arbitrarily stiff bearings monotone instead of explicitly
        // overshooting.
        for (RigidBody::RotorSlot& rotor : rigidBody.rotors_) {
            if (!rotor.alive) {
                continue;
            }
            const Real motorTorque =
                rotor.state.motorTorqueCommand +
                rotor.torqueAccumulator;
            if (!(rotor.state.bearingDamping > 0.0F)) {
                rotor.state.absoluteAxialAngularMomentum +=
                    motorTorque * halfDeltaSeconds;
                rigidBody.synchronizeAngularVelocityFromMomentum();
                continue;
            }
            const Vec3 inverseTimesAxis =
                rigidBody.inverseLocalInertiaTensor_ *
                rotor.state.axisLocal;
            const Real compliance =
                1.0F / rotor.state.axialInertia +
                dot(rotor.state.axisLocal, inverseTimesAxis);
            if (!(compliance > kEpsilon) ||
                !std::isfinite(compliance)) {
                continue;
            }
            // Stable phi_1 form of the exact impulse. Unlike u/c followed by
            // multiplication by a vanishing decay factor, this remains finite
            // for both denormal-small and extremely large positive damping.
            const double damping =
                static_cast<double>(rotor.state.bearingDamping);
            const double coordinateCompliance =
                static_cast<double>(compliance);
            const double kickSeconds =
                static_cast<double>(halfDeltaSeconds);
            const double exponent =
                damping * coordinateCompliance * kickSeconds;
            const double phiOne =
                exponent > 1.0e-8
                    ? -std::expm1(-exponent) / exponent
                    : 1.0 - 0.5 * exponent +
                          exponent * exponent / 6.0;
            const double driveTorque =
                static_cast<double>(motorTorque) -
                damping * static_cast<double>(
                    rotor.state.relativeAngularVelocity
                );
            const double momentumDelta =
                driveTorque * kickSeconds * phiOne;
            constexpr double maximumReal =
                static_cast<double>(
                    std::numeric_limits<Real>::max()
                );
            const double updatedMomentum = std::clamp(
                static_cast<double>(
                    rotor.state.absoluteAxialAngularMomentum
                ) + momentumDelta,
                -maximumReal,
                maximumReal
            );
            rotor.state.absoluteAxialAngularMomentum =
                static_cast<Real>(updatedMomentum);
            rigidBody.synchronizeAngularVelocityFromMomentum();
        }
    }
}

void World::integrateDrift(Real deltaSeconds) {
    for (RigidBody& rigidBody : bodies_) {
        if (!rigidBody.alive_ || rigidBody.motionType_ != MotionType::Dynamic ||
            rigidBody.sleeping_) {
            continue;
        }

        const Transform startTransform = rigidBody.transform_;
        const Vec3 worldAngularMomentum =
            rigidBody.worldAngularMomentum_;
        Transform intendedTransform = startTransform;
        intendedTransform.position +=
            rigidBody.linearVelocity_ * deltaSeconds;
        const RotationIntegrationResult intendedRotation =
            integrateOrientationFromAngularMomentum(
                rigidBody,
                startTransform.orientation,
                worldAngularMomentum,
                deltaSeconds,
                settings_.maximumRotationStepRadians,
                settings_.maximumRotationMicrosteps
            );
        intendedTransform.orientation = intendedRotation.orientation;

        SweepHit earliestHit{};
        bool sweptCollision = false;
        if (rigidBody.collider_.type == ColliderType::Box) {
            for (const RigidBody& obstacle : bodies_) {
                if (!obstacle.alive_ ||
                    obstacle.id_ == rigidBody.id_ ||
                    obstacle.motionType_ != MotionType::Static) {
                    continue;
                }

                SweepHit candidate{};
                bool hit = false;
                if (obstacle.collider_.type == ColliderType::Box) {
                    hit = sweptBoxAgainstStaticBox(
                        rigidBody,
                        startTransform,
                        intendedTransform,
                        obstacle,
                        settings_.contactSlop,
                        candidate
                    );
                } else if (obstacle.collider_.type == ColliderType::Plane) {
                    const Real rotationalSurfaceTravel =
                        length(rigidBody.collider_.box.halfExtents) *
                        length(rigidBody.angularVelocity_) *
                        deltaSeconds;
                    const Real negligibleRotationTravel = std::max(
                        1.0e-7F,
                        settings_.contactSlop * 0.001F
                    );
                    if (rotationalSurfaceTravel <=
                        negligibleRotationTravel) {
                        hit = sweptBoxAgainstPlane(
                            rigidBody,
                            startTransform,
                            intendedTransform,
                            obstacle,
                            settings_.contactSlop,
                            candidate
                        );
                    } else {
                        RotationalCcdWork ccdWork{};
                        hit = sweptRotatingBoxAgainstPlane(
                            rigidBody,
                            startTransform,
                            intendedTransform,
                            worldAngularMomentum,
                            deltaSeconds,
                            settings_.contactSlop,
                            settings_.maximumRotationStepRadians,
                            settings_.maximumRotationMicrosteps,
                            obstacle,
                            candidate,
                            ccdWork
                        );
                        debugStats_.rotationalCcdAdvances +=
                            ccdWork.advances;
                        if (ccdWork.convergedToContact) {
                            ++debugStats_.rotationalCcdConvergenceHits;
                        }
                        if (ccdWork.hitAdvanceCap) {
                            ++debugStats_.rotationalCcdAdvanceCapHits;
                        }
                        if (hit) {
                            ++debugStats_.rotationalCcdHits;
                        }
                    }
                }
                if (hit &&
                    (!sweptCollision ||
                     candidate.fraction < earliestHit.fraction)) {
                    earliestHit = candidate;
                    sweptCollision = true;
                }
            }
        }

        Real driftFraction = 1.0F;
        if (sweptCollision && !earliestHit.hasTransform) {
            // Move a deterministic microscopic amount into the speculative
            // contact band for legacy box-box sweeps. Plane sweeps carry
            // their exact inside transform and do not use this approximation.
            constexpr Real kContactAdvanceFraction = 1.0e-4F;
            driftFraction = std::min(
                1.0F,
                earliestHit.fraction + kContactAdvanceFraction
            );
        } else if (sweptCollision) {
            driftFraction = earliestHit.fraction;
        }
        RotationIntegrationResult actualRotation = intendedRotation;
        if (driftFraction < 1.0F) {
            actualRotation = integrateOrientationFromAngularMomentum(
                rigidBody,
                startTransform.orientation,
                worldAngularMomentum,
                deltaSeconds * driftFraction,
                settings_.maximumRotationStepRadians,
                settings_.maximumRotationMicrosteps
            );
        }
        if (sweptCollision && earliestHit.hasTransform) {
            // Install the exact pose whose plane-separation probe was inside.
            // Re-integrating via a different sequence can move a slender box
            // back outside the manifold and repeat the same expensive TOI.
            rigidBody.transform_ = earliestHit.transform;
        } else {
            rigidBody.transform_.position =
                startTransform.position +
                (intendedTransform.position - startTransform.position) *
                    driftFraction;
            rigidBody.transform_.orientation = actualRotation.orientation;
        }
        debugStats_.rotationMicrosteps += actualRotation.microsteps;
        debugStats_.maximumRotationMicrostepsUsed = std::max(
            debugStats_.maximumRotationMicrostepsUsed,
            actualRotation.microsteps
        );
        if (actualRotation.hitMicrostepCap) {
            ++debugStats_.rotationMicrostepCapHits;
        }
        debugStats_.rotationMidpointNonConvergenceCount +=
            actualRotation.nonConvergedMicrosteps;
        debugStats_.maximumRotationMidpointResidual = std::max(
            debugStats_.maximumRotationMidpointResidual,
            actualRotation.maximumMidpointResidual
        );
        rigidBody.synchronizeAngularVelocityFromMomentum();
    }
}

void World::detectContacts() {
    contacts_.clear();
    debugContacts_.clear();

    for (std::size_t firstIndex = 0; firstIndex < bodies_.size(); ++firstIndex) {
        RigidBody& first = bodies_[firstIndex];
        if (!first.alive_) {
            continue;
        }
        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < bodies_.size();
             ++secondIndex) {
            RigidBody& second = bodies_[secondIndex];
            if (!second.alive_ ||
                (first.motionType_ == MotionType::Static &&
                 second.motionType_ == MotionType::Static)) {
                continue;
            }

            ++debugStats_.broadphasePairs;
            CollisionManifold manifold{};
            BodyId bodyA = first.id_;
            BodyId bodyB = second.id_;
            bool collided = false;

            if (first.collider_.type == ColliderType::Plane &&
                second.collider_.type == ColliderType::Box) {
                ++debugStats_.narrowphaseTests;
                collided = boxAgainstPlane(
                    second,
                    first,
                    settings_.contactSlop,
                    manifold
                );
            } else if (first.collider_.type == ColliderType::Box &&
                       second.collider_.type == ColliderType::Plane) {
                ++debugStats_.narrowphaseTests;
                collided = boxAgainstPlane(
                    first,
                    second,
                    settings_.contactSlop,
                    manifold
                );
                std::swap(bodyA, bodyB);
            } else if (first.collider_.type == ColliderType::Box &&
                       second.collider_.type == ColliderType::Box &&
                       first.worldAabb().overlaps(
                           second.worldAabb(),
                           settings_.contactSlop
                       )) {
                ++debugStats_.narrowphaseTests;
                collided = boxAgainstBox(
                    first,
                    second,
                    settings_.contactSlop,
                    manifold
                );
            }

            if (!collided) {
                continue;
            }

            RigidBody* contactA = body(bodyA);
            RigidBody* contactB = body(bodyB);
            if (contactA == nullptr || contactB == nullptr) {
                continue;
            }
            contactA->touchedThisStep_ = true;
            contactB->touchedThisStep_ = true;

            const bool firstCanWake =
                contactA->motionType_ == MotionType::Dynamic &&
                contactA->sleeping_ &&
                contactB->motionType_ == MotionType::Dynamic &&
                !contactB->sleeping_;
            const bool secondCanWake =
                contactB->motionType_ == MotionType::Dynamic &&
                contactB->sleeping_ &&
                contactA->motionType_ == MotionType::Dynamic &&
                !contactA->sleeping_;
            if (firstCanWake) {
                contactA->wake();
            }
            if (secondCanWake) {
                contactB->wake();
            }

            for (std::size_t pointIndex = 0;
                 pointIndex < manifold.pointCount;
                 ++pointIndex) {
                contacts_.push_back({
                    bodyA,
                    bodyB,
                    manifold.points[pointIndex].point,
                    manifold.normal,
                    manifold.points[pointIndex].penetration,
                    0.0F,
                    {},
                    {},
                    0.0F,
                    static_cast<std::uint8_t>(manifold.pointCount),
                });
            }
        }
    }
    debugStats_.contactCount = contacts_.size();
}

void World::solveContacts(Real deltaSeconds) {
    const auto applyImpulse = [](RigidBody& rigidBody, const Vec3& impulse, const Vec3& r) {
        if (rigidBody.motionType_ != MotionType::Dynamic || rigidBody.sleeping_) {
            return;
        }
        rigidBody.linearVelocity_ += impulse * rigidBody.inverseMass_;
        rigidBody.worldAngularMomentum_ += cross(r, impulse);
        rigidBody.synchronizeAngularVelocityFromMomentum();
    };

    const auto effectiveMass = [](const RigidBody& rigidBody, const Vec3& r, const Vec3& axis) {
        if (rigidBody.motionType_ != MotionType::Dynamic || rigidBody.sleeping_) {
            return 0.0F;
        }
        const Vec3 rotational =
            rigidBody.multiplyWorldInverseInertia(cross(r, axis));
        return rigidBody.inverseMass_ + dot(axis, cross(rotational, r));
    };

    const auto applyAngularImpulse = [](
                                         RigidBody& rigidBody,
                                         const Vec3& angularImpulse
                                     ) {
        if (rigidBody.motionType_ != MotionType::Dynamic || rigidBody.sleeping_) {
            return;
        }
        rigidBody.worldAngularMomentum_ += angularImpulse;
        rigidBody.synchronizeAngularVelocityFromMomentum();
    };

    for (Contact& contact : contacts_) {
        const RigidBody* rigidBodyA = body(contact.bodyA);
        const RigidBody* rigidBodyB = body(contact.bodyB);
        if (rigidBodyA == nullptr || rigidBodyB == nullptr) {
            continue;
        }
        const Vec3 relativeVelocity =
            rigidBodyB->velocityAtWorldPoint(contact.point) -
            rigidBodyA->velocityAtWorldPoint(contact.point);
        const Real closingVelocity = dot(relativeVelocity, contact.normal);
        const Real restitution =
            std::max(
                rigidBodyA->material_.restitution,
                rigidBodyB->material_.restitution
            );
        contact.restitutionVelocity =
            closingVelocity < -settings_.restitutionVelocityThreshold
                ? -restitution * closingVelocity
                : 0.0F;
    }

    for (std::uint32_t iteration = 0;
         iteration < settings_.velocityIterations;
         ++iteration) {
        for (Contact& contact : contacts_) {
            RigidBody* rigidBodyA = body(contact.bodyA);
            RigidBody* rigidBodyB = body(contact.bodyB);
            if (rigidBodyA == nullptr || rigidBodyB == nullptr) {
                continue;
            }
            const Vec3 rA = contact.point - rigidBodyA->transform_.position;
            const Vec3 rB = contact.point - rigidBodyB->transform_.position;
            Vec3 relativeVelocity =
                rigidBodyB->velocityAtWorldPoint(contact.point) -
                rigidBodyA->velocityAtWorldPoint(contact.point);
            const Real normalVelocity = dot(relativeVelocity, contact.normal);
            const Real normalMass =
                effectiveMass(*rigidBodyA, rA, contact.normal) +
                effectiveMass(*rigidBodyB, rB, contact.normal);
            if (normalMass <= kEpsilon) {
                continue;
            }

            const Real penetrationVelocity = std::min(
                settings_.maximumDepenetrationVelocity,
                settings_.penetrationVelocityFactor *
                    std::max(0.0F, contact.penetration - settings_.contactSlop) /
                    deltaSeconds
            );
            const Real targetVelocity =
                std::max(contact.restitutionVelocity, penetrationVelocity);
            const Real normalImpulseDelta =
                (targetVelocity - normalVelocity) / normalMass;
            const Real oldNormalImpulse = contact.accumulatedNormalImpulse;
            contact.accumulatedNormalImpulse =
                std::max(0.0F, oldNormalImpulse + normalImpulseDelta);
            const Real appliedNormalImpulse =
                contact.accumulatedNormalImpulse - oldNormalImpulse;
            const Vec3 normalImpulse = contact.normal * appliedNormalImpulse;
            applyImpulse(*rigidBodyA, -normalImpulse, rA);
            applyImpulse(*rigidBodyB, normalImpulse, rB);

            relativeVelocity =
                rigidBodyB->velocityAtWorldPoint(contact.point) -
                rigidBodyA->velocityAtWorldPoint(contact.point);
            const Vec3 tangentVelocity =
                relativeVelocity - contact.normal * dot(relativeVelocity, contact.normal);
            const Real tangentSpeed = length(tangentVelocity);
            if (tangentSpeed > 1.0e-5F) {
                const Vec3 tangent = tangentVelocity / tangentSpeed;
                const Real tangentMass =
                    effectiveMass(*rigidBodyA, rA, tangent) +
                    effectiveMass(*rigidBodyB, rB, tangent);
                if (tangentMass > kEpsilon) {
                    const Real tangentImpulseDelta =
                        -dot(relativeVelocity, tangent) / tangentMass;
                    const Vec3 candidateImpulse =
                        contact.accumulatedTangentImpulse +
                        tangent * tangentImpulseDelta;
                    const Real staticLimit =
                        combinedStaticFriction(
                            rigidBodyA->material_,
                            rigidBodyB->material_
                        ) * contact.accumulatedNormalImpulse;

                    Vec3 newTangentImpulse = candidateImpulse;
                    if (lengthSquared(candidateImpulse) >
                        staticLimit * staticLimit) {
                        const Real dynamicLimit =
                            combinedDynamicFriction(
                                rigidBodyA->material_,
                                rigidBodyB->material_
                            ) * contact.accumulatedNormalImpulse;
                        newTangentImpulse =
                            normalized(candidateImpulse, tangent) * dynamicLimit;
                    }
                    const Vec3 tangentImpulse =
                        newTangentImpulse - contact.accumulatedTangentImpulse;
                    contact.accumulatedTangentImpulse = newTangentImpulse;
                    applyImpulse(*rigidBodyA, -tangentImpulse, rA);
                    applyImpulse(*rigidBodyB, tangentImpulse, rB);
                }
            }

            // Rolling resistance is a bounded angular impulse at an active
            // contact. It is proportional to normal impulse and contact radius,
            // not an arbitrary global angular damping term.
            const Vec3 relativeAngularVelocity =
                rigidBodyB->angularVelocity_ - rigidBodyA->angularVelocity_;
            const Vec3 rollingVelocity =
                relativeAngularVelocity -
                contact.normal * dot(relativeAngularVelocity, contact.normal);
            const Real rollingSpeed = length(rollingVelocity);
            if (rollingSpeed <= 1.0e-5F) {
                continue;
            }
            const Vec3 rollingAxis = rollingVelocity / rollingSpeed;
            const Vec3 inverseAngularResponse =
                rigidBodyA->multiplyWorldInverseInertia(rollingAxis) +
                rigidBodyB->multiplyWorldInverseInertia(rollingAxis);
            const Real rollingMass = dot(rollingAxis, inverseAngularResponse);
            if (rollingMass <= kEpsilon) {
                continue;
            }

            Real contactRadius = std::numeric_limits<Real>::max();
            if (rigidBodyA->collider_.type == ColliderType::Box) {
                const Vec3 half = rigidBodyA->collider_.box.halfExtents;
                contactRadius = std::min({half.x, half.y, half.z});
            }
            if (rigidBodyB->collider_.type == ColliderType::Box) {
                const Vec3 half = rigidBodyB->collider_.box.halfExtents;
                contactRadius = std::min(
                    contactRadius,
                    std::min({half.x, half.y, half.z})
                );
            }
            if (!std::isfinite(contactRadius)) {
                continue;
            }

            const Real rollingImpulseDelta =
                -dot(relativeAngularVelocity, rollingAxis) / rollingMass;
            const Vec3 candidateRollingImpulse =
                contact.accumulatedRollingImpulse +
                rollingAxis * rollingImpulseDelta;
            const Real rollingFriction = std::sqrt(
                rigidBodyA->material_.rollingFriction *
                rigidBodyB->material_.rollingFriction
            );
            const Real rollingLimit =
                rollingFriction * contact.accumulatedNormalImpulse * contactRadius;
            const Vec3 newRollingImpulse =
                clampLength(candidateRollingImpulse, rollingLimit);
            const Vec3 appliedRollingImpulse =
                newRollingImpulse - contact.accumulatedRollingImpulse;
            contact.accumulatedRollingImpulse = newRollingImpulse;
            applyAngularImpulse(*rigidBodyA, -appliedRollingImpulse);
            applyAngularImpulse(*rigidBodyB, appliedRollingImpulse);
        }
    }

    // Split positional correction avoids adding kinetic energy. Multi-point
    // manifolds share one correction so a four-corner resting face does not
    // receive four times the intended movement.
    for (Contact& contact : contacts_) {
        RigidBody* rigidBodyA = body(contact.bodyA);
        RigidBody* rigidBodyB = body(contact.bodyB);
        if (rigidBodyA == nullptr || rigidBodyB == nullptr) {
            continue;
        }
        const Real inverseMassA =
            rigidBodyA->motionType_ == MotionType::Dynamic &&
                    !rigidBodyA->sleeping_
                ? rigidBodyA->inverseMass_
                : 0.0F;
        const Real inverseMassB =
            rigidBodyB->motionType_ == MotionType::Dynamic &&
                    !rigidBodyB->sleeping_
                ? rigidBodyB->inverseMass_
                : 0.0F;
        const Real inverseMassSum = inverseMassA + inverseMassB;
        if (inverseMassSum <= kEpsilon) {
            continue;
        }

        const Real penetration =
            std::max(0.0F, contact.penetration - settings_.contactSlop);
        const Real correctionMagnitude =
            settings_.positionCorrectionFraction * penetration /
            (inverseMassSum * static_cast<Real>(contact.manifoldPointCount));
        const Vec3 correction = contact.normal * correctionMagnitude;
        rigidBodyA->transform_.position -= correction * inverseMassA;
        rigidBodyB->transform_.position += correction * inverseMassB;
    }

    debugContacts_.reserve(contacts_.size());
    for (const Contact& contact : contacts_) {
        debugContacts_.push_back({
            contact.bodyA,
            contact.bodyB,
            contact.point,
            contact.normal,
            contact.penetration,
            contact.accumulatedNormalImpulse,
        });
    }
}

void World::updateSleeping(Real deltaSeconds) {
    const Real linearThresholdSquared =
        settings_.sleepLinearSpeed * settings_.sleepLinearSpeed;
    const Real angularThresholdSquared =
        settings_.sleepAngularSpeed * settings_.sleepAngularSpeed;
    const Vec3 gravity = environment_.gravity();
    const Real gravityMagnitudeSquared = lengthSquared(gravity);
    const bool hasGravity =
        gravityMagnitudeSquared > kEpsilon * kEpsilon;
    const Vec3 supportUp = hasGravity
        ? -gravity / std::sqrt(gravityMagnitudeSquared)
        : Vec3{0.0F, 1.0F, 0.0F};
    const Vec3 tangentSeed =
        std::abs(supportUp.y) < 0.9F
            ? Vec3{0.0F, 1.0F, 0.0F}
            : Vec3{1.0F, 0.0F, 0.0F};
    const Vec3 supportTangentX =
        normalized(cross(tangentSeed, supportUp));
    const Vec3 supportTangentY =
        normalized(cross(supportUp, supportTangentX));

    const auto hasStableSupport = [&](const RigidBody& rigidBody) {
        // With no preferred load direction, the ordinary speed/contact sleep
        // test remains appropriate. Under gravity, sleeping is a static
        // stability decision: the projected CoM must be inside an area, not
        // balanced on a zero-area mathematical corner or edge.
        if (!hasGravity) {
            return true;
        }
        std::vector<SupportPoint2> supportPoints;
        supportPoints.reserve(8U);
        for (const Contact& contact : contacts_) {
            Vec3 supportNormal{};
            if (contact.bodyB == rigidBody.id_) {
                supportNormal = contact.normal;
            } else if (contact.bodyA == rigidBody.id_) {
                supportNormal = -contact.normal;
            } else {
                continue;
            }
            if (dot(supportNormal, supportUp) <= 1.0e-3F) {
                continue;
            }
            const Vec3 relativePoint =
                contact.point - rigidBody.transform_.position;
            supportPoints.push_back({
                dot(relativePoint, supportTangentX),
                dot(relativePoint, supportTangentY),
            });
        }
        return supportPolygonContainsOrigin(
            std::move(supportPoints),
            settings_.sleepSupportMargin
        );
    };

    for (RigidBody& rigidBody : bodies_) {
        if (!rigidBody.alive_ || rigidBody.motionType_ != MotionType::Dynamic) {
            continue;
        }
        if (!rigidBody.allowSleep_) {
            rigidBody.sleeping_ = false;
            rigidBody.quietTime_ = 0.0F;
            continue;
        }
        bool hasStoredRotorMomentum = false;
        for (const RigidBody::RotorSlot& rotor : rigidBody.rotors_) {
            if (rotor.alive &&
                (std::abs(
                     rotor.state.absoluteAxialAngularMomentum
                 ) > kEpsilon ||
                 std::abs(
                     rotor.state.relativeAngularVelocity
                 ) > kEpsilon ||
                 std::abs(rotor.torqueAccumulator) > kEpsilon ||
                 std::abs(
                     rotor.state.motorTorqueCommand
                 ) > kEpsilon)) {
                hasStoredRotorMomentum = true;
                break;
            }
        }
        if (hasStoredRotorMomentum) {
            rigidBody.sleeping_ = false;
            rigidBody.quietTime_ = 0.0F;
            continue;
        }
        if (!rigidBody.touchedThisStep_) {
            if (rigidBody.sleeping_) {
                rigidBody.sleeping_ = false;
            }
            rigidBody.quietTime_ = 0.0F;
            continue;
        }

        const bool belowSpeedThresholds =
            lengthSquared(rigidBody.linearVelocity_) <=
                linearThresholdSquared &&
            lengthSquared(rigidBody.angularVelocity_) <=
                angularThresholdSquared;
        if (belowSpeedThresholds && hasStableSupport(rigidBody)) {
            rigidBody.quietTime_ += deltaSeconds;
            if (rigidBody.quietTime_ >= settings_.sleepDelaySeconds) {
                rigidBody.sleeping_ = true;
                rigidBody.linearVelocity_ = {};
                rigidBody.angularVelocity_ = {};
                rigidBody.worldAngularMomentum_ = {};
            }
        } else {
            rigidBody.sleeping_ = false;
            rigidBody.quietTime_ = 0.0F;
        }
    }
}

void World::clearExternalForces() noexcept {
    for (RigidBody& rigidBody : bodies_) {
        rigidBody.clearAccumulators();
    }
}

void World::prepareTimedForces(Real fixedDeltaSeconds) noexcept {
    const double fixedDelta =
        static_cast<double>(fixedDeltaSeconds);
    for (TimedForceSlot& slot : timedForces_) {
        slot.tickScale = 0.0F;
        if (!slot.alive) {
            continue;
        }

        RigidBody* selectedBody = body(slot.force.body);
        if (selectedBody == nullptr ||
            selectedBody->motionType_ != MotionType::Dynamic ||
            !std::isfinite(slot.remainingSeconds) ||
            slot.remainingSeconds <= 0.0) {
            slot.alive = false;
            slot.remainingSeconds = 0.0;
            slot.force.remainingSeconds = 0.0F;
            continue;
        }
        if (!slot.force.enabled) {
            continue;
        }

        selectedBody->wake();
        if (fixedDelta > 0.0 && std::isfinite(fixedDelta)) {
            slot.tickScale = static_cast<Real>(
                std::clamp(
                    slot.remainingSeconds / fixedDelta,
                    0.0,
                    1.0
                )
            );
        }
    }
}

void World::expireTimedForces(Real fixedDeltaSeconds) noexcept {
    const double fixedDelta =
        static_cast<double>(fixedDeltaSeconds);
    if (!std::isfinite(fixedDelta) || fixedDelta <= 0.0) {
        return;
    }

    for (TimedForceSlot& slot : timedForces_) {
        if (!slot.alive || !slot.force.enabled ||
            slot.tickScale <= 0.0F) {
            continue;
        }
        slot.remainingSeconds =
            std::max(0.0, slot.remainingSeconds - fixedDelta);
        slot.force.remainingSeconds =
            static_cast<Real>(slot.remainingSeconds);
        slot.tickScale = 0.0F;
        if (slot.remainingSeconds <= 1.0e-12) {
            slot.alive = false;
            slot.remainingSeconds = 0.0;
            slot.force.remainingSeconds = 0.0F;
        }
    }
}

} // namespace uaview::physics
