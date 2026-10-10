#!/usr/bin/env python3
# Reproduction harness for "after a hit the rune switches target blade and the
# predicted point reverses".
#
# Models the real geometry: FIVE blades at fixed 2*pi/5 spacing, rotating at
# constant omega. The detector always reports the *currently lit* (active)
# blade; "打中后切换靶子" = the lit index moves to another blade.
#
# Ports: Fu_angleprocess  (src/rm_tracker/include/buff_angle_processor.hpp)
#        SmallBuffPhaseTracker (src/rm_tracker/include/blade_tracker.hpp)
import math
import numpy as np
import cv2

TWO_PI = 2.0 * math.pi
PI = math.pi
BLADE = TWO_PI / 5.0          # 72 deg
DEG = 180.0 / math.pi


def fold180(a):
    return math.degrees(math.remainder(a, TWO_PI))


# ----------------------------------------------------------------------------
# Port of Fu_angleprocess (buff_angle_processor.hpp) - the legacy fallback path
# ----------------------------------------------------------------------------
class Fu_angleprocess:
    def __init__(self):
        self.angle_ = 0.0
        self.loop_count = 0
        self.angle_include = 0.0
        self.angle_polar_back = 0.0
        self.angle_polar_now = 0.0
        self.angle_date = []
        self.angle_time = []
        self.anticlockwise = True
        self.anticlockwise_flag = False
        self.loop_add_flag = False
        self.first_angle = True
        self.trun_count = []
        self.dir_events = []

    def turn(self, angle):
        if len(self.trun_count) < 21:
            self.trun_count.append(angle)
        if len(self.trun_count) == 21 and not self.anticlockwise_flag:
            filt = [self.trun_count[i] for i in range(1, len(self.trun_count))
                    if abs(self.trun_count[i] - self.trun_count[i - 1]) < PI * 0.028]
            pos = neg = 0
            for i in range(1, len(filt)):
                d = filt[i] - filt[i - 1]
                if abs(d) >= 300 or abs(d) == 0:   # radians -> dead legacy guard
                    continue
                pos += d > 0
                neg += d < 0
            if pos > neg:
                self.anticlockwise, self.anticlockwise_flag = True, True
                self.dir_events.append((len(self.angle_time), "正转/clockwise"))
            elif neg > pos:
                self.anticlockwise, self.anticlockwise_flag = False, True
                self.dir_events.append((len(self.angle_time), "反转/counterclockwise"))

    def angle2kf(self, angle):
        self.angle_ = angle
        if not self.anticlockwise and self.anticlockwise_flag:
            self.angle_ -= 6.283185
        if not self.first_angle:
            self.angle_include = abs(self.angle_polar_now - self.angle_polar_back)
        if self.anticlockwise and self.anticlockwise_flag:
            if 0.0 <= self.angle_ <= 0.5235986:
                if self.first_angle:
                    self.loop_count -= 1
                    self.angle_ += 6.283185 * (self.loop_count + 1)
                    self.loop_add_flag = True
                else:
                    self.angle_ += 6.283185 * (self.loop_count + 1)
                    self.loop_add_flag = True
            elif 6.195918 <= self.angle_ <= 6.283185:
                self.angle_ += 6.283185 * self.loop_count
                self.loop_add_flag = False
            elif 0.5235986 < self.angle_ < 6.195918:
                if self.loop_add_flag:
                    self.loop_count += 1
                    self.loop_add_flag = False
                self.angle_ += 6.283185 * self.loop_count
                if (1.047197 <= self.angle_include <= 5.232986 and
                        self.angle_polar_back > self.angle_polar_now):
                    self.loop_count += 1
                    self.angle_ += 6.283185 * self.loop_count
        if not self.anticlockwise and self.anticlockwise_flag:
            if -0.5235986 <= self.angle_ <= 0.0:
                if self.first_angle:
                    self.loop_count -= 1
                    self.angle_ -= 6.283185 * (self.loop_count + 1)
                    self.loop_add_flag = True
                else:
                    self.angle_ -= 6.283185 * (self.loop_count + 1)
                    self.loop_add_flag = True
            elif -6.283185 <= self.angle_ <= -6.195918:
                self.angle_ -= 6.283185 * self.loop_count
                self.loop_add_flag = False
            elif -6.195918 < self.angle_ < -0.5235986:
                if self.loop_add_flag:
                    self.loop_count += 1
                    self.loop_add_flag = False
                self.angle_ -= 6.283185 * self.loop_count
                if (1.047197 <= self.angle_include <= 5.232986 and
                        self.angle_polar_back < self.angle_polar_now):
                    self.loop_count += 1
                    self.angle_ -= 6.283185 * self.loop_count
        self.angle_polar_back = angle
        self.first_angle = False
        return self.angle_

    def angle_diff_fun(self, delta_t, fps):
        if (delta_t <= 0 or fps <= 0 or len(self.angle_date) != len(self.angle_time)
                or len(self.angle_time) < 5 or len(self.angle_time) < fps * delta_t):
            return -50.0
        n = len(self.angle_time)
        for offset in range(max(2, int(fps * delta_t * 0.5)), n - 2):
            ti = n - 1 - offset
            if ti < 2 or ti + 2 >= n:
                continue
            elapsed = self.angle_time[-1] - self.angle_time[ti]
            if elapsed < 0 or elapsed > delta_t * 1000.0 + 80.0:
                return -50.0
            if elapsed >= delta_t * 1000.0:
                diff = self.angle_date[-1] - (
                    self.angle_date[ti] + self.angle_date[ti - 1] +
                    self.angle_date[ti + 1] + self.angle_date[ti - 2] +
                    self.angle_date[ti + 2]) / 5.0
                if (abs(diff) > 1.169 or (self.anticlockwise and diff < 0.0) or
                        (not self.anticlockwise and diff > 0.0)) and self.anticlockwise_flag:
                    for c in range(-6, 7):
                        corr = diff - 2.0 * c * PI / 5.0
                        if self.anticlockwise and corr < 2.0 * PI / 6.55 and corr > -1e-9:
                            return corr
                        if (not self.anticlockwise and corr > -2.0 * PI / 6.55
                                and corr < 1e-9):
                            return corr
                    return -50.0
                return diff
        return -50.0

    def feed(self, angle, time_ms, delta_t, fps):
        self.angle_polar_now = angle
        self.turn(angle)
        acc = self.angle2kf(angle)
        self.angle_date.append(acc)
        self.angle_time.append(time_ms)
        return self.angle_diff_fun(delta_t, fps)


# ----------------------------------------------------------------------------
# Port of SmallBuffPhaseTracker (blade_tracker.hpp) - the primary Kalman path
# ----------------------------------------------------------------------------
class SmallBuffPhaseTracker:
    kBladePeriod = TWO_PI / 5.0
    kConvergeDelay = 5
    kAnchorConfirmFrames = 2
    # reanchor=True mirrors the blade_tracker.hpp fix (position anchored to the
    # blade the detector reports); False reproduces the old behaviour.
    reanchor = True
    kMaxRollResidual = 0.55
    kMinSmallBuffSpeed = 0.052
    kMaxSmallBuffSpeed = 1.571
    kPhaseProcessNoise = 0.005
    kPhaseMeasurementNoise = 0.004

    def __init__(self):
        self.initialized = False
        self.locked_direction = 0
        self.saved_direction = 0
        self.direction_votes = 0
        self.accepted = 0
        self.rejected = 0
        self.last_t = 0.0
        self.last_phase = 0.0
        self.radius = 0.0
        self.center = (0.0, 0.0)
        self.blade_anchor = 0.0
        self.blade_anchor_steps = 0
        self.pending_anchor_steps = 0
        self.pending_anchor_count = 0
        self.trace = []

    def init_kalman(self, phase):
        self.kf = cv2.KalmanFilter(2, 1, 0, cv2.CV_64F)
        self.kf.transitionMatrix = np.array([[1.0, 0.0], [0.0, 1.0]], np.float64)
        self.kf.measurementMatrix = np.array([[1.0, 0.0]], np.float64)
        self.kf.statePost = np.array([[phase], [0.0]], np.float64)
        self.kf.errorCovPost = np.array([[0.10, 0.0], [0.0, 0.50]], np.float64)
        self.kf.measurementNoiseCov = np.eye(1, dtype=np.float64) * self.kPhaseMeasurementNoise

    def update(self, r_center, blade_center, t, predict_s):
        offset = (blade_center[0] - r_center[0], blade_center[1] - r_center[1])
        meas_r = math.hypot(*offset)
        raw = math.atan2(offset[1], offset[0])
        if not self.initialized:
            self.initialized = True
            self.last_t, self.last_phase = t, raw
            self.radius, self.center = meas_r, r_center
            self.init_kalman(raw)
            return None
        dt = t - self.last_t
        if dt <= 1e-4 or dt > 0.5:
            return None
        self.kf.transitionMatrix = np.array([[1.0, dt], [0.0, 1.0]], np.float64)
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = self.kPhaseProcessNoise
        self.kf.processNoiseCov = np.array(
            [[0.25 * dt4 * q, 0.5 * dt3 * q], [0.5 * dt3 * q, dt2 * q]], np.float64)
        pred = self.kf.predict()
        pred_phase = float(pred[0, 0])
        obs_phase = pred_phase + math.remainder(raw - pred_phase, self.kBladePeriod)
        innov = obs_phase - pred_phase
        if abs(innov) > self.kMaxRollResidual:
            self.kf.statePost = self.kf.statePre.copy()
            self.kf.errorCovPost = self.kf.errorCovPre.copy()
            self.last_t = t
            if self.rejected > 3:
                self.reset()
            self.rejected += 1
            return None
        phase_delta = obs_phase - self.last_phase
        if abs(phase_delta) > 0.002:
            self.direction_votes += 1 if phase_delta > 0 else -1
            self.direction_votes = max(-20, min(20, self.direction_votes))
        direction = (1 if self.direction_votes >= self.kConvergeDelay
                     else -1 if self.direction_votes <= -self.kConvergeDelay else 0)
        self.kf.correct(np.array([[obs_phase]], np.float64))
        if direction != 0 and self.locked_direction == 0:
            self.locked_direction = direction
            self.kf.statePost[1, 0] = direction * self.kMinSmallBuffSpeed
            self.kf.errorCovPost[1, 1] = 0.05
        if self.locked_direction != 0:
            omega = float(self.kf.statePost[1, 0])
            if omega * self.locked_direction <= 0.0:
                omega = self.locked_direction * self.kMinSmallBuffSpeed
            else:
                omega = self.locked_direction * max(
                    self.kMinSmallBuffSpeed, min(self.kMaxSmallBuffSpeed, abs(omega)))
            self.kf.statePost[1, 0] = omega
        self.radius = 0.9 * self.radius + 0.1 * meas_r
        self.center = (self.center[0] * 0.8 + r_center[0] * 0.2,
                       self.center[1] * 0.8 + r_center[1] * 0.2)
        self.last_t, self.last_phase = t, obs_phase
        self.rejected, self.accepted = 0, self.accepted + 1
        self.trace.append((t, raw, obs_phase, float(self.kf.statePost[1, 0]),
                           self.locked_direction))
        if self.locked_direction == 0:
            return None
        if self.accepted < self.kConvergeDelay and self.saved_direction == 0:
            return None
        if self.reanchor:
            filtered = float(self.kf.statePost[0, 0])
            meas_delta = math.remainder(raw - filtered, TWO_PI)
            implied = round(meas_delta / self.kBladePeriod)
            if implied == self.blade_anchor_steps:
                self.pending_anchor_count = 0
            elif implied == self.pending_anchor_steps:
                self.pending_anchor_count += 1
                if self.pending_anchor_count >= self.kAnchorConfirmFrames:
                    self.blade_anchor_steps = implied
                    self.pending_anchor_count = 0
            else:
                self.pending_anchor_steps = implied
                self.pending_anchor_count = 1
        self.blade_anchor = self.kBladePeriod * self.blade_anchor_steps
        phase = float(self.kf.statePost[0, 0]) + self.blade_anchor
        omega = float(self.kf.statePost[1, 0])
        future = phase + omega * predict_s
        return (self.center[0] + self.radius * math.cos(future),
                self.center[1] + self.radius * math.sin(future))

    def reset(self):
        if self.locked_direction != 0:
            self.saved_direction = self.locked_direction
        self.initialized = False
        self.locked_direction = 0
        self.direction_votes = self.saved_direction * self.kConvergeDelay
        self.accepted = self.rejected = 0


# ----------------------------------------------------------------------------
# Wheel: 5 blades, lit blade switches after each "hit"
# ----------------------------------------------------------------------------
def run(omega_deg=-60.0, fps=50.0, delta_t=0.10, hit_frames=(250, 500, 750),
        switch_step=+1, n_frames=1000, reanchor=True):
    omega = math.radians(omega_deg)
    center = (650.0, 502.0)
    radius = 160.0
    ap = Fu_angleprocess()
    tr = SmallBuffPhaseTracker()
    tr.reanchor = reanchor
    lit = 0
    print(f"\n=== 5-blade rune, omega={omega_deg:+.0f} deg/s, hits at frames "
          f"{hit_frames}, switched blade index step {switch_step:+d}, "
          f"re-anchor={'ON (fixed)' if reanchor else 'OFF (before fix)'} ===")
    print("frame  lit  polar(deg)  loop  diff(deg)   kf_omega  point_target_err  "
          "point_vs_lit(deg)  anchor(deg)")
    for f in range(n_frames):
        if f in hit_frames:
            lit = (lit + switch_step) % 5
        t = f / fps
        base = omega * t
        # detector reports the lit blade's centre
        theta = base + lit * BLADE
        blade = (center[0] + radius * math.cos(theta),
                 center[1] + radius * math.sin(theta))
        raw = math.atan2(blade[1] - center[1], blade[0] - center[0])
        diff = ap.feed(raw, t * 1000.0, delta_t, fps)
        pred = tr.update(center, blade, t, delta_t)
        if f % 50 == 0 or (f - 1) in hit_frames or f in hit_frames:
            lit_future = math.atan2(
                math.sin(raw + omega * delta_t), math.cos(raw + omega * delta_t))
            if pred is None:
                err = float("nan")
            else:
                pt = math.atan2(pred[1] - center[1], pred[0] - center[0])
                err = fold180(pt - lit_future)
            d = "  -50(inv)" if diff == -50.0 else f"{math.degrees(diff):+8.2f}"
            om = tr.trace[-1][3] * DEG if tr.trace else 0.0
            print(f"{f:5d} {lit:4d} {math.degrees(raw):10.2f} {ap.loop_count:5d} "
                  f"{d:>10} {om:9.1f} {err:17.1f} {err:17.1f}"
                  f" {tr.blade_anchor * DEG:+10.1f}")
    return tr


if __name__ == "__main__":
    run(switch_step=+1, reanchor=False)
    run(switch_step=+1, reanchor=True)
    run(switch_step=-1, reanchor=False)
    run(switch_step=-1, reanchor=True)
