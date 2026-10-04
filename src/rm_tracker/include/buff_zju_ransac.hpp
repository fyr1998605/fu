#pragma once

#include <cstddef>
#include <deque>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace buff_zju {

// Fits the large-buff angular velocity model:
//   v(t) = A * sin(omega * t + phi) + C
class RansacSineFitter {
public:
    struct Result {
        double A = 0.0;
        double omega = 0.0;
        double phi = 0.0;
        double C = 0.0;
        int inliers = 0;
    };

    RansacSineFitter(
        int max_iterations,
        double threshold,
        double min_omega,
        double max_omega,
        double min_amplitude,
        double max_amplitude,
        unsigned int seed = 0x5EEDu,
        double max_gap_seconds = 5.0
    );

    void add_data(double timestamp, double velocity);
    void fit();
    void reset();

    double relative_time(double timestamp) const { return timestamp - t0_; }
    const Result& best_result() const { return best_result_; }
    std::size_t sample_count() const { return fit_data_.size(); }

private:
    bool fit_partial_model(
        const std::vector<std::pair<double, double>>& sample,
        double omega,
        Eigen::Vector3d& params
    ) const;

    int evaluate_inliers(double amplitude, double omega, double phi, double offset) const;

    int max_iterations_;
    double threshold_;
    double min_omega_;
    double max_omega_;
    double min_amplitude_;
    double max_amplitude_;
    double max_gap_seconds_;
    std::mt19937 generator_;
    std::deque<std::pair<double, double>> fit_data_;
    Result best_result_;
    double t0_ = 0.0;
    double last_timestamp_ = 0.0;
    bool has_t0_ = false;
    bool has_last_timestamp_ = false;
};

}  // namespace buff_zju
