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
    expectPoint(second.corners[0], 101.0f, 90.0f);
    expectPoint(second.corners[1], 96.0f, 95.0f);
    expectPoint(second.corners[2], 101.0f, 100.0f);
    expectPoint(second.corners[3], 106.0f, 95.0f);

    const BuffStabilizedGeometry reset = stabilizer.update(
        candidate(14.0f, 10.0f, 110.0f, 95.0f, 0.9f), second_corners, true);
    expectPoint(reset.r_center, 14.0f, 10.0f);
    expectPoint(reset.fan_center, 110.0f, 95.0f);
}
