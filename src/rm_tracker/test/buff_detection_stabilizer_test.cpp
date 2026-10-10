#include "buff_detection_stabilizer.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace {

void require(bool condition) {
    if (!condition) std::abort();
}

void expectPoint(const BuffPoint& point, float x, float y) {
    require(std::abs(point.x - x) < 1e-5f);
    require(std::abs(point.y - y) < 1e-5f);
}

BuffTemporalCandidate candidate(float r_x, float r_y, float fan_x, float fan_y,
                                float confidence) {
    return {{r_x, r_y}, {fan_x, fan_y}, confidence};
}

}  // namespace

int main() {
    const std::vector<BuffTemporalCandidate> candidates{
        candidate(10.0f, 10.0f, 102.0f, 99.0f, 0.55f),
        candidate(300.0f, 300.0f, 400.0f, 400.0f, 0.95f)};

    require(BuffDetectionStabilizer::selectCandidate(
                candidates, {100.0f, 100.0f}, true, 40.0f) == 0);
    require(BuffDetectionStabilizer::selectCandidate(
                candidates, {0.0f, 0.0f}, true, 40.0f) == -1);
    require(std::abs(BuffDetectionStabilizer::associationGateForRadius(100.0f) -
                     120.0f) < 1e-5f);
    // The locked gate must still tolerate a realistic one-frame gimbal motion
    // in pixel space, while remaining narrower than the adjacent blade chord.
    require(std::abs(BuffDetectionStabilizer::lockedAssociationGateForRadius(100.0f) -
                     75.0f) < 1e-5f);

    // Once the first target is locked, a higher-confidence second target must
    // not steal the track. A short loss keeps the lock; expiry allows a fresh
    // confidence-based acquisition.
    BuffTargetLock target_lock(0.20);
    target_lock.lock({100.0f, 100.0f}, 0.0);
    require(!target_lock.canAcceptGenericJump(0.0));
    require(target_lock.select(candidates, 40.0f, 0.0) == 0);
    target_lock.accept({104.0f, 100.0f}, 0.04);
    require(target_lock.noteMiss(0.08));
    require(target_lock.select(
                {candidate(10.0f, 10.0f, 108.0f, 100.0f, 0.2f),
                 candidate(300.0f, 300.0f, 400.0f, 400.0f, 0.99f)},
                40.0f, 0.08) == 0);
    require(target_lock.canAcceptGenericJump(0.29));
    require(!target_lock.noteMiss(0.29));
    require(target_lock.select(candidates, 40.0f, 0.29) == 1);
    target_lock.reset();
    require(target_lock.canAcceptGenericJump(0.30));

    // A second jump is valid only when it confirms the same pending candidate.
    BuffDetectionStabilizer jump_confirmation;
    const BuffTemporalCandidate false_jump_x =
        candidate(20.0f, 20.0f, 200.0f, 200.0f, 0.9f);
    const BuffTemporalCandidate false_jump_y =
        candidate(40.0f, 40.0f, 300.0f, 300.0f, 0.9f);
    const BuffTemporalCandidate confirmed_y =
        candidate(42.0f, 39.0f, 304.0f, 297.0f, 0.9f);
    require(!jump_confirmation.confirmPersistentJump(false_jump_x, 45.0f));
    require(!jump_confirmation.confirmPersistentJump(false_jump_y, 45.0f));
    require(!jump_confirmation.confirmPersistentJump(confirmed_y, 45.0f));
    require(jump_confirmation.confirmPersistentJump(confirmed_y, 45.0f));

    // A missed detection clears pending confirmation rather than accepting a stale jump.
    jump_confirmation.reset();
    require(!jump_confirmation.confirmPersistentJump(confirmed_y, 45.0f));

    // After a missed detection, reacquisition also needs two matching frames.
    BuffDetectionStabilizer reacquisition;
    reacquisition.beginReacquisition();
    require(!reacquisition.confirmReacquisition(confirmed_y, 45.0f));
    require(!reacquisition.confirmReacquisition(confirmed_y, 45.0f));
    require(reacquisition.confirmReacquisition(confirmed_y, 45.0f));

    BuffDetectionStabilizer stabilizer(0.5f);
    const std::array<BuffPoint, 4> first_corners{{
        {95.0f, 90.0f}, {90.0f, 95.0f}, {95.0f, 100.0f}, {100.0f, 95.0f}}};
    const BuffStabilizedGeometry first = stabilizer.update(
        candidate(10.0f, 10.0f, 100.0f, 95.0f, 0.9f), first_corners);
    expectPoint(first.r_center, 10.0f, 10.0f);
    expectPoint(first.fan_center, 100.0f, 95.0f);

    const std::array<BuffPoint, 4> second_corners{{
        {107.0f, 90.0f}, {102.0f, 95.0f}, {107.0f, 100.0f}, {112.0f, 95.0f}}};
    const BuffStabilizedGeometry second = stabilizer.update(
        candidate(14.0f, 10.0f, 110.0f, 95.0f, 0.9f), second_corners);
    expectPoint(second.r_center, 12.0f, 10.0f);
    expectPoint(second.fan_center, 105.0f, 95.0f);
    expectPoint(second.corners[0], 98.5f, 90.0f);
    expectPoint(second.corners[1], 93.5f, 95.0f);
    expectPoint(second.corners[2], 98.5f, 100.0f);
    expectPoint(second.corners[3], 103.5f, 95.0f);

    stabilizer.reset();
    const BuffStabilizedGeometry reset = stabilizer.update(
        candidate(14.0f, 10.0f, 110.0f, 95.0f, 0.9f), second_corners);
    expectPoint(reset.r_center, 14.0f, 10.0f);
    expectPoint(reset.fan_center, 110.0f, 95.0f);
}
