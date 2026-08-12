#pragma once

#include <cmath>
#include <limits>

class BuffPredictionGeometry {
public:
    struct Vec2 {
        double x = 0.0;
        double y = 0.0;
    };

    struct Vec3 {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Vec3 operator+(const Vec3& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }

        Vec3 operator-(const Vec3& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        Vec3 operator*(double scale) const {
            return {x * scale, y * scale, z * scale};
        }
    };

    static double dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    static Vec3 normalized(const Vec3& value) {
        const double length = std::sqrt(dot(value, value));
        if (!std::isfinite(length) || length < kEpsilon) return {};
        return value * (1.0 / length);
    }

    static bool rayPlaneIntersection(const Vec3& ray_direction,
                                     const Vec3& plane_point,
                                     const Vec3& plane_normal,
                                     Vec3& intersection) {
        const double denominator = dot(plane_normal, ray_direction);
        if (!std::isfinite(denominator) || std::abs(denominator) < kEpsilon) {
            return false;
        }

        const double scale = dot(plane_normal, plane_point) / denominator;
        if (!std::isfinite(scale) || scale <= 0.0) return false;

        intersection = ray_direction * scale;
        return std::isfinite(intersection.x) && std::isfinite(intersection.y) &&
            std::isfinite(intersection.z);
    }

    static bool rotateAroundAxis(const Vec3& value, const Vec3& axis,
                                 double angle, Vec3& rotated) {
        const Vec3 unit_axis = normalized(axis);
        if (dot(unit_axis, unit_axis) < kEpsilon || !std::isfinite(angle)) {
            return false;
        }

        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        rotated = value * cosine + cross(unit_axis, value) * sine +
            unit_axis * (dot(unit_axis, value) * (1.0 - cosine));
        return std::isfinite(rotated.x) && std::isfinite(rotated.y) &&
            std::isfinite(rotated.z);
    }

    static bool physicalPhaseForImagePhase(double image_phase,
                                           double positive_image_phase,
                                           double& physical_phase) {
        if (!std::isfinite(image_phase) ||
            !std::isfinite(positive_image_phase) ||
            std::abs(positive_image_phase) < kEpsilon) {
            return false;
        }
        physical_phase = positive_image_phase > 0.0
            ? image_phase : -image_phase;
        return true;
    }

private:
    static constexpr double kEpsilon = 1e-9;
};
