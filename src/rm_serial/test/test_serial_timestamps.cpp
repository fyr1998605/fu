#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>
#include <memory>
#include <cstdio>
#include <array>
#include <boost/make_shared.hpp>

// Exercise the deployed receive/publish code, including the real Asio reader.
#define main serial_node_main
#include "../src/serial.cpp"
#undef main

class SerialTimestamps : public ::testing::Test {
protected:
    int master = -1;
    int slave = -1;
    std::unique_ptr<RmSerial> serial;

    void SetUp() override {
        char name[128];
        ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
        SPName = name;
        ros::NodeHandle nh;
        serial.reset(new RmSerial(nh, ""));
    }
    void TearDown() override {
        serial.reset();
        if (master >= 0) close(master);
        if (slave >= 0) close(slave);
    }
    std::vector<unsigned char> packet(unsigned char yaw_low = 7) {
        std::vector<unsigned char> result(36, 0);
        result[0] = 0xff;
        result[9] = yaw_low;
        result[10] = 0x66;
        result[20] = 0x77;
        result[30] = 0x88;
        result[34] = 23;
        result[35] = 0xfe;
        return result;
    }
    void send(const std::vector<unsigned char>& bytes) {
        ASSERT_EQ(write(master, bytes.data(), bytes.size()),
                  static_cast<ssize_t>(bytes.size()));
    }
};

TEST_F(SerialTimestamps, Exact36BytesPublishImmediatelyWithOneStamp) {
    const ros::Time before = ros::Time::now();
    send(packet());
    serial->ReceiveSerial();
    EXPECT_GE(serial->sample_stamp, before);
    EXPECT_LE(serial->sample_stamp, ros::Time::now());
    EXPECT_EQ(serial->Sp.receive_stamp, serial->PubSerialData.header.stamp);
    EXPECT_EQ(serial->ImuData.header.stamp, serial->PubSerialData.header.stamp);
    EXPECT_DOUBLE_EQ(serial->PubSerialData.Yaw, 7.0 / 100000.0);
}

TEST_F(SerialTimestamps, PartialPacketDoesNotRepublishPreviousPose) {
    send(packet(7));
    serial->ReceiveSerial();
    const ros::Time first_stamp = serial->sample_stamp;
    const auto next = packet(9);
    send({next.begin(), next.begin() + 11});
    serial->ReceiveSerial();
    EXPECT_TRUE(serial->Sp.Rm_Serial_Data.empty());
    EXPECT_EQ(serial->sample_stamp, first_stamp);
    EXPECT_DOUBLE_EQ(serial->PubSerialData.Yaw, 7.0 / 100000.0);
    send({next.begin() + 11, next.end()});
    const ros::Time final_read = ros::Time::now();
    serial->ReceiveSerial();
    EXPECT_GE(serial->sample_stamp, final_read);
    EXPECT_DOUBLE_EQ(serial->PubSerialData.Yaw, 9.0 / 100000.0);
}

TEST_F(SerialTimestamps, NoiseAndConsecutivePacketsRetainRemainder) {
    auto bytes = std::vector<unsigned char>(5, 0x42);
    const auto first = packet(8);
    const auto second = packet(9);
    bytes.insert(bytes.end(), first.begin(), first.end());
    bytes.insert(bytes.end(), second.begin(), second.end());
    send(bytes);
    serial->ReceiveSerial(); // 5 noise bytes + 31 packet bytes
    EXPECT_TRUE(serial->sample_stamp.isZero());
    serial->ReceiveSerial(); // finish first, retain 31 bytes of second
    EXPECT_DOUBLE_EQ(serial->PubSerialData.Yaw, 8.0 / 100000.0);
    serial->ReceiveSerial(); // finish second without discarding it
    EXPECT_DOUBLE_EQ(serial->PubSerialData.Yaw, 9.0 / 100000.0);
}

TEST_F(SerialTimestamps, ReceiptDelayIsAppliedOnceToBothMessages) {
    serial->receive_delay_sec = 0.003;
    send(packet());
    serial->ReceiveSerial();
    EXPECT_EQ(serial->sample_stamp,
              serial->Sp.receive_stamp - ros::Duration(0.003));
    EXPECT_EQ(serial->PubSerialData.header.stamp, serial->sample_stamp);
    EXPECT_EQ(serial->ImuData.header.stamp, serial->sample_stamp);
}

TEST_F(SerialTimestamps, InvalidDelayConfigurationIsRejected) {
    for (const char* value : {"-0.01", ".nan", "wrong_type"}) {
        char filename[] = "/tmp/buff-serial-delay-XXXXXX";
        const int fd = mkstemp(filename);
        ASSERT_GE(fd, 0);
        const std::string yaml = std::string("Time:\n  receive_delay_sec: ") + value + "\n";
        ASSERT_EQ(write(fd, yaml.data(), yaml.size()), static_cast<ssize_t>(yaml.size()));
        close(fd);
        ros::NodeHandle nh;
        EXPECT_THROW(RmSerial invalid(nh, filename), std::runtime_error);
        std::remove(filename);
    }
}

TEST_F(SerialTimestamps, CallbackStoresLatestCommandForTheIoOwner) {
    auto command = boost::make_shared<geometry_msgs::Vector3>();
    command->x = 0.125; command->y = 0.25; command->z = 1;
    serial->ReceiveYolov8(command);
    pollfd output{master, POLLIN, 0};
    EXPECT_EQ(poll(&output, 1, 0), 0); // The ROS callback must not write the port.

    command = boost::make_shared<geometry_msgs::Vector3>();
    command->x = 0.5; command->y = -0.25; command->z = 3;
    serial->ReceiveYolov8(command);
    const auto started = std::chrono::steady_clock::now();
    serial->Progress(); // No telemetry: bounded receive must still allow sending.
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(), .1);
    ASSERT_EQ(poll(&output, 1, 100), 1);
    std::array<unsigned char, 8> bytes{};
    ASSERT_EQ(read(master, bytes.data(), bytes.size()), 8);
    const std::array<unsigned char, 8> expected{{0xff, 0x01, 0xf4, 0x80, 0xfa, 0x01, 0x03, 0xfe}};
    EXPECT_EQ(bytes, expected);
    EXPECT_TRUE(serial->sample_stamp.isZero()); // Silence did not synthesize a pose.
    serial->Progress();
    EXPECT_EQ(poll(&output, 1, 0), 0); // The consumed command is not resent.
}

TEST_F(SerialTimestamps, SilentReceiveReturnsWithinItsBound) {
    const auto started = std::chrono::steady_clock::now();
    serial->ReceiveSerial();
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(), .1);
    EXPECT_TRUE(serial->sample_stamp.isZero());
}

TEST_F(SerialTimestamps, AllControlStatesReachTheWireUnchanged) {
    for (int code : {0, 1, 3}) {
        auto command = boost::make_shared<geometry_msgs::Vector3>();
        command->z = code;
        serial->ReceiveYolov8(command);
        serial->Progress();
        pollfd output{master, POLLIN, 0};
        ASSERT_EQ(poll(&output, 1, 100), 1);
        std::array<unsigned char, 8> bytes{};
        ASSERT_EQ(read(master, bytes.data(), bytes.size()), 8);
        EXPECT_EQ(bytes[6], code);
        EXPECT_TRUE(serial->sample_stamp.isZero());
    }
}

TEST_F(SerialTimestamps, DisconnectIsReportedToTheIoOwner) {
    close(master);
    master = -1;
    EXPECT_THROW(serial->ReceiveSerial(), std::runtime_error);
    EXPECT_TRUE(ros::ok()); // The owner handles shutdown/restart outside the reader.
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_timestamp_tests");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
