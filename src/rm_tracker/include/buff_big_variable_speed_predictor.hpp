#pragma once

namespace buff_zju {

class BigBuffVariableSpeedPredictor {
public:
    static constexpr int kWarmupSamples = 10;
    bool update(double timestamp_seconds,
                double raw_angle,
                double horizon_seconds,
                double& predicted_angle);
    void reset();

    // Return only the future angular lead. The caller supplies the current
    // bright-blade angle when constructing the output point, so aliases in
    // the predictor's accumulated phase cannot change blade identity.
    double prediction_lead(double horizon_seconds) const;

    bool ready() const { return accepted_samples_ >= kWarmupSamples; }
    double angular_speed() const { return angular_speed_; }
    double angular_acceleration() const { return angular_acceleration_; }

private:
    static double normalize_angle(double angle);
    bool initialize(double timestamp_seconds, double raw_angle);

    double last_timestamp_ = 0.0;
    double last_raw_angle_ = 0.0;
    double phase_ = 0.0;
    double angular_speed_ = 0.0;
    double angular_acceleration_ = 0.0;
    int accepted_samples_ = 0;
    bool initialized_ = false;
};

}  // namespace buff_zju
