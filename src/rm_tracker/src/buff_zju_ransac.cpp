#include "buff_zju_ransac.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

#include <Eigen/SVD>

namespace buff_zju {
namespace {

constexpr double kMinThreshold = 1e-4;
constexpr double kMinSingularValue = 1e-10;
constexpr double kMaxConditionNumber = 1e10;
constexpr int kMaxRansacIterations = 1000;
constexpr std::size_t kMaxSamples = 300;
constexpr double kMaxGapSeconds = 30.0;

// 非有限或为负的值回落为 fallback，保证 uniform_real_distribution 前提
// (min <= max 且均为有限值) 成立
double finite_nonnegative_or(const double value, const double fallback) {
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

double finite_positive_or(const double value, const double fallback) {
    return std::isfinite(value) && value >= kMinThreshold ? value : fallback;
}

}  // namespace

RansacSineFitter::RansacSineFitter(
    const int max_iterations,
    const double threshold,
    const double min_omega,
    const double max_omega,
    const double min_amplitude,
    const double max_amplitude,
    const unsigned int seed,
    const double max_gap_seconds
):
    max_iterations_(max_iterations > 0
        ? std::clamp(max_iterations, 1, kMaxRansacIterations)
        : 200),
    threshold_(finite_positive_or(threshold, 0.5)),
    min_omega_(finite_positive_or(min_omega, 1.884)),
    max_omega_(std::max(min_omega_, finite_positive_or(max_omega, 2.0))),
    min_amplitude_(finite_nonnegative_or(min_amplitude, 0.780)),
    max_amplitude_(std::max(min_amplitude_, finite_nonnegative_or(max_amplitude, 1.045))),
    max_gap_seconds_(std::clamp(
        finite_positive_or(max_gap_seconds, 5.0), 1e-4, kMaxGapSeconds)),
    generator_(seed) {}

void RansacSineFitter::reset() {
    fit_data_.clear();
    best_result_ = Result{};
    t0_ = 0.0;
    last_timestamp_ = 0.0;
    has_t0_ = false;
    has_last_timestamp_ = false;
}

void RansacSineFitter::add_data(const double timestamp, const double velocity) {
    if (!std::isfinite(timestamp) || !std::isfinite(velocity)) {
        return;
    }
    if (has_last_timestamp_ && timestamp <= last_timestamp_) {
        return;
    }

    if (!has_t0_) {
        t0_ = timestamp;
        has_t0_ = true;
    }

    if (has_last_timestamp_ && timestamp - last_timestamp_ > max_gap_seconds_) {
        fit_data_.clear();
        best_result_ = Result{};
        t0_ = timestamp;
    }

    fit_data_.emplace_back(timestamp - t0_, velocity);
    last_timestamp_ = timestamp;
    has_last_timestamp_ = true;
    while (fit_data_.size() > kMaxSamples) {
        fit_data_.pop_front();
    }
}

void RansacSineFitter::fit() {
    // Re-fit against the current sliding window. A previous result must not
    // keep the predictor ready after the motion regime has changed.
    best_result_ = Result{};
    if (fit_data_.size() < 3) {
        return;
    }

    std::uniform_real_distribution<double> omega_distribution(min_omega_, max_omega_);
    std::vector<std::size_t> indices(fit_data_.size());
    std::iota(indices.begin(), indices.end(), 0);

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        // Three draws without replacement suffice; shuffling the entire
        // 300-sample window on every iteration does unnecessary work.
        std::array<std::pair<double, double>, 3> sample;
        for (int index = 0; index < 3; ++index) {
            const auto slot = static_cast<std::size_t>(index);
            std::uniform_int_distribution<std::size_t> pick(slot, indices.size() - 1);
            std::swap(indices[slot], indices[pick(generator_)]);
            sample[slot] = fit_data_[indices[slot]];
        }

        const double omega = omega_distribution(generator_);
        Eigen::Vector3d params;
        if (!fit_partial_model(sample, omega, params)) {
            continue;
        }

        const double sin_coefficient = params(0);
        const double cos_coefficient = params(1);
        const double amplitude = std::hypot(sin_coefficient, cos_coefficient);
        const double phi = std::atan2(cos_coefficient, sin_coefficient);
        const double offset = params(2);
        if (!std::isfinite(amplitude) ||
            amplitude < min_amplitude_ || amplitude > max_amplitude_) {
            continue;
        }

        const int inliers = evaluate_inliers(amplitude, omega, phi, offset);
        if (inliers > best_result_.inliers) {
            best_result_.A = amplitude;
            best_result_.omega = omega;
            best_result_.phi = phi;
            best_result_.C = offset;
            best_result_.inliers = inliers;
        }
    }

}

bool RansacSineFitter::fit_partial_model(
    const std::array<std::pair<double, double>, 3>& sample,
    const double omega,
    Eigen::Vector3d& params
) const {
    if (sample.size() != 3 || !std::isfinite(omega)) {
        return false;
    }

    Eigen::Matrix3d design;
    Eigen::Vector3d values;
    for (std::size_t index = 0; index < sample.size(); ++index) {
        const double timestamp = sample[index].first;
        design(static_cast<Eigen::Index>(index), 0) = std::sin(omega * timestamp);
        design(static_cast<Eigen::Index>(index), 1) = std::cos(omega * timestamp);
        design(static_cast<Eigen::Index>(index), 2) = 1.0;
        values(static_cast<Eigen::Index>(index)) = sample[index].second;
    }

    try {
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            design, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Vector3d singular_values = svd.singularValues();
        if (singular_values.size() != 3 || !singular_values.allFinite()) {
            return false;
        }
        const double largest = singular_values(0);
        const double smallest = singular_values(2);
        if (largest <= kMinSingularValue ||
            smallest <= kMinSingularValue ||
            largest / smallest > kMaxConditionNumber) {
            return false;
        }
        params = svd.solve(values);
        return params.allFinite();
    } catch (...) {
        return false;
    }
}

int RansacSineFitter::evaluate_inliers(
    const double amplitude,
    const double omega,
    const double phi,
    const double offset
) const {
    int count = 0;
    for (const auto& point : fit_data_) {
        const double prediction =
            amplitude * std::sin(omega * point.first + phi) + offset;
        if (std::abs(point.second - prediction) < threshold_) {
            ++count;
        }
    }
    return count;
}

}  // namespace buff_zju
