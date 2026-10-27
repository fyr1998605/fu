#include "buff_display_frame_queue.hpp"

#include <cassert>

int main() {
    BuffDisplayFrameQueue queue(2);

    cv::Mat first(1, 1, CV_8UC1, cv::Scalar(11));
    cv::Mat second(1, 1, CV_8UC1, cv::Scalar(22));

    assert(queue.push(first, 1));
    assert(queue.push(second, 2));

    cv::Mat output;
    std::uint64_t sequence = 0;
    assert(queue.tryPop(output, sequence));
    assert(sequence == 1);
    assert(output.at<unsigned char>(0, 0) == 11);
    assert(queue.tryPop(output, sequence));
    assert(sequence == 2);
    assert(output.at<unsigned char>(0, 0) == 22);
    assert(!queue.tryPop(output, sequence));
    return 0;
}
