#include "buff_prediction_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

namespace {

using Geometry = BuffPredictionGeometry;
using Vec3 = Geometry::Vec3;

constexpr double kTolerance = 1e-9;

void require(bool condition) {
    if (!condition) std::abort();
}

double distance(const Vec3& a, const Vec3& b) {
    return std::sqrt(
        (a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y) +
        (a.z - b.z) * (a.z - b.z));
}

Geometry::Vec2 projectNormalized(const Vec3& point) {
    return {point.x / point.z, point.y / point.z};
}

double distance(const Geometry::Vec2& a, const Geometry::Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

int main() {
    double physical_phase = 0.0;
    require(Geometry::physicalPhaseForImagePhase(0.40, 0.02, physical_phase));
    require(std::abs(physical_phase - 0.40) < kTolerance);
    require(Geometry::physicalPhaseForImagePhase(0.40, -0.02, physical_phase));
    require(std::abs(physical_phase + 0.40) < kTolerance);
    require(!Geometry::physicalPhaseForImagePhase(0.40, 0.0, physical_phase));

    // A tilted Buff plane: uniform image translation is not a valid substitute
    // for rotating all physical points around the R-center in this plane.
    const Vec3 plane_normal = Geometry::normalized({0.31, -0.24, 0.92});
    const Vec3 plane_point{0.18, -0.12, 4.35};
    const Vec3 tangent_u = Geometry::normalized({plane_normal.z, 0.0, -plane_normal.x});
    const Vec3 tangent_v = Geometry::cross(plane_normal, tangent_u);

    const Vec3 r_center = plane_point + tangent_u * 0.20 - tangent_v * 0.15;
    const Vec3 fan_center = r_center + tangent_u * 0.58 + tangent_v * 0.24;
    const double phase = 0.72;

    Vec3 recovered_r_center;
    Vec3 recovered_fan_center;
    require(Geometry::rayPlaneIntersection(
        {projectNormalized(r_center).x, projectNormalized(r_center).y, 1.0},
        plane_point, plane_normal, recovered_r_center));
    require(Geometry::rayPlaneIntersection(
        {projectNormalized(fan_center).x, projectNormalized(fan_center).y, 1.0},
        plane_point, plane_normal, recovered_fan_center));
    require(distance(recovered_r_center, r_center) < kTolerance);
    require(distance(recovered_fan_center, fan_center) < kTolerance);

    Vec3 predicted_offset;
    require(Geometry::rotateAroundAxis(
        recovered_fan_center - recovered_r_center, plane_normal, phase,
        predicted_offset));
    const Vec3 predicted_center = recovered_r_center + predicted_offset;

    Vec3 expected_offset;
    require(Geometry::rotateAroundAxis(
        fan_center - r_center, plane_normal, phase, expected_offset));
    const Vec3 expected_center = r_center + expected_offset;
    require(distance(predicted_center, expected_center) < kTolerance);

    const Geometry::Vec2 current_fan_pixel = projectNormalized(fan_center);
    const Geometry::Vec2 predicted_fan_pixel = projectNormalized(expected_center);
    const Geometry::Vec2 pixel_offset{
        predicted_fan_pixel.x - current_fan_pixel.x,
        predicted_fan_pixel.y - current_fan_pixel.y};

    const std::array<Vec3, 4> current_corners{{
        fan_center + tangent_u * 0.12,
        fan_center - tangent_u * 0.12,
        fan_center + tangent_v * 0.10,
        fan_center - tangent_v * 0.10}};

    double largest_pixel_error = 0.0;
    for (const Vec3& corner : current_corners) {
        Vec3 rotated_corner_offset;
        require(Geometry::rotateAroundAxis(
            corner - r_center, plane_normal, phase, rotated_corner_offset));
        const Geometry::Vec2 true_pixel = projectNormalized(
            r_center + rotated_corner_offset);
        const Geometry::Vec2 old_shift_pixel{
            projectNormalized(corner).x + pixel_offset.x,
            projectNormalized(corner).y + pixel_offset.y};
        largest_pixel_error = std::max(
            largest_pixel_error, distance(true_pixel, old_shift_pixel));
    }
    require(largest_pixel_error > 1e-3);
}
