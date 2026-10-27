#include "RmTracker.hpp"
#include <boost/make_shared.hpp>
#include <mutex>
#include <cstdlib>

int main(int argc, char** argv) {
    ros::init(argc, argv, "armor_input_guard_test");
    ros::NodeHandle nh;
    Tracker<500> tracker(nh, argv[1]);
    std::mutex mutex;
    std::vector<int> codes;
    auto sub = nh.subscribe<geometry_msgs::Vector3>("/auto_angle", 10,
        [&](const geometry_msgs::Vector3ConstPtr& message) {
            std::lock_guard<std::mutex> lock(mutex);
            codes.push_back(static_cast<int>(message->z));
        });
    ros::AsyncSpinner spinner(1);
    spinner.start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (sub.getNumPublishers() == 0 &&
           std::chrono::steady_clock::now() < deadline) ros::WallDuration(.01).sleep();
    auto expect_code = [&](int expected, const char* reason) {
        ros::WallDuration(.04).sleep();
        std::lock_guard<std::mutex> lock(mutex);
        if (codes.empty() || codes.back() != expected) {
            std::cerr << reason << '\n'; std::exit(1);
        }
    };
    auto aim = [&]() {
        tracker.Pub_Aangle(true, false, cv::Point2d(.1, .2));
        expect_code(3, "valid aim angle must send 3, not the observed-only state");
    };
    aim();
    tracker.Track();
    expect_code(0, "no serial observation must clear a previous aiming command");

    // Freeze source time so wall-time checks cannot be bypassed by /clock.
    ros::Time::setNow(ros::Time(100));
    auto serial = boost::make_shared<rm_msgs::RmSerial>();
    serial->header.stamp = ros::Time(100);
    serial->ShootFlag = 'a'; serial->Camp = 'r'; serial->Qw = 1;
    tracker.SetSerial(serial);
    auto armors = boost::make_shared<rm_msgs::ArmorArray>();
    armors->header.stamp = ros::Time(100);
    tracker.doArmors(armors);
    aim();
    ros::WallDuration(.22).sleep();
    tracker.SetSerial(serial); // Duplicate source stamps do not renew freshness.
    tracker.doArmors(armors);
    tracker.Track();
    expect_code(0, "paused clock/duplicate data must not keep old inputs alive");

    ros::Time::setNow(ros::Time(100.01));
    serial->header.stamp = ros::Time(100.01);
    tracker.SetSerial(serial);
    aim();
    tracker.Track();
    expect_code(0, "expired armor arrival must be rejected even with fresh serial data");

    serial->header.stamp = ros::Time(100.011);
    ros::Time::setNow(serial->header.stamp);
    serial->Pitch = std::numeric_limits<double>::quiet_NaN();
    tracker.SetSerial(serial);
    aim();
    tracker.Track();
    expect_code(0, "invalid pose must not be replaced by the previous valid one");

    serial->Pitch = 0;
    serial->header.stamp = ros::Time(100.012);
    ros::Time::setNow(serial->header.stamp);
    tracker.SetSerial(serial);
    armors = boost::make_shared<rm_msgs::ArmorArray>();
    armors->header.stamp = ros::Time(100.013);
    tracker.doArmors(armors);
    aim();
    tracker.Track();
    expect_code(0, "future armor data must not be treated as a valid source frame");
    Visual::VisualTF transforms(nh);
    for (int query = 0; query < 2; ++query) {
        const auto start = std::chrono::steady_clock::now();
        bool failed = false;
        try {
            if (query == 0) transforms.GetChildInFrameData("missing_map", "missing_child");
            else transforms.GetChildInFrameData("missing_map", "missing_child", 1);
        } catch (const tf2::TransformException&) { failed = true; }
        if (!failed || std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count() > .2) {
            std::cerr << "missing TF was replaced by zeros or blocked the loop\n";
            return 1;
        }
    }
    std::cout << "Armor control states, absent/invalid/stale inputs and paused clock passed\n";
}
