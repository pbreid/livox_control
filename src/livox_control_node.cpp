#include <ros/ros.h>
#include <std_msgs/String.h>
#include "livox_sdk.h"

class LivoxControl {
private:
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    bool lidar_connected_ = false;

public:
    LivoxControl() {
        command_sub_ = nh_.subscribe("livox_command", 1, &LivoxControl::commandCallback, this);

        if (!LivoxSdkInit()) {
            ROS_ERROR("Livox SDK init failed.");
            ros::shutdown();
            return;
        }

        LivoxLidarCallback cb;
        cb.OnDeviceBroadcast = std::bind(&LivoxControl::OnDeviceBroadcast, this, std::placeholders::_1);
        LivoxLidarSetCallback(cb);

        if (LivoxLidarStartScan() != kStatusSuccess) {
            ROS_ERROR("Start scan failed.");
            ros::shutdown();
            return;
        }

        ROS_INFO("Searching for Livox LiDAR...");
    }

    void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
        if (info == nullptr) {
            return;
        }
        ROS_INFO("Livox LiDAR found. Connecting...");
        livox_status result = LivoxLidarConnect(info->broadcast_code);
        if (result == kStatusSuccess) {
            ROS_INFO_STREAM("Connected to Livox LiDAR: " << info->broadcast_code);
            lidar_connected_ = true;
        } else {
            ROS_ERROR("Failed to connect to Livox LiDAR");
        }
    }

    void SetLidarMode(uint8_t mode) {
        if (!lidar_connected_) {
            ROS_WARN("LiDAR not connected. Cannot set mode.");
            return;
        }
        LidarSetMode(nullptr, kLivoxLidarMode(mode));
    }

    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "sleep") {
            SetLidarMode(kLivoxLidarSleep);
            ROS_INFO("LiDAR set to sleep mode.");
        } else if (msg->data == "wake") {
            SetLidarMode(kLivoxLidarNormal);
            ROS_INFO("LiDAR set to normal mode.");
        } else {
            ROS_WARN_STREAM("Unknown command: " << msg->data);
        }
    }

    ~LivoxControl() {
        LivoxLidarStopScan();
        LivoxSdkUninit();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_control_node");
    LivoxControl livox_control;
    ros::spin();
    return 0;
}
