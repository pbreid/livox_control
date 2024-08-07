#include <ros/ros.h>
#include <std_msgs/String.h>
#include "livox_def.h"
#include "livox_sdk.h"

class LivoxControl {
private:
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    bool lidar_connected_ = false;
    uint8_t lidar_handle_ = 0;

public:
    LivoxControl() {
        command_sub_ = nh_.subscribe("livox_command", 1, &LivoxControl::commandCallback, this);

        if (!Init()) {
            ROS_ERROR("Livox SDK init failed.");
            ros::shutdown();
            return;
        }

        SetBroadcastCallback(LivoxControl::OnDeviceBroadcast);

        if (Start() != kStatusSuccess) {
            ROS_ERROR("Start scan failed.");
            ros::shutdown();
            return;
        }

        ROS_INFO("Searching for Livox LiDAR...");
    }

    static void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
        if (info == nullptr) {
            return;
        }
        ROS_INFO("Livox LiDAR found. Connecting...");
        uint8_t handle = 0;
        livox_status status = AddLidarToConnect(info->broadcast_code, &handle);
        if (status == kStatusSuccess) {
            ROS_INFO_STREAM("Connected to Livox LiDAR: " << info->broadcast_code);
            // You might want to store this handle for later use
            // This is just a static function, so you can't directly access class members here
        } else {
            ROS_ERROR("Failed to connect to Livox LiDAR");
        }
    }

    void SetLidarMode(LidarMode mode) {
        if (!lidar_connected_) {
            ROS_WARN("LiDAR not connected. Cannot set mode.");
            return;
        }
        livox_status status = LidarSetMode(lidar_handle_, mode, nullptr, nullptr);
        if (status == kStatusSuccess) {
            ROS_INFO("LiDAR mode set successfully.");
        } else {
            ROS_ERROR("Failed to set LiDAR mode.");
        }
    }

    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "sleep") {
            SetLidarMode(kLidarModePowerSaving);
            ROS_INFO("LiDAR set to power saving mode.");
        } else if (msg->data == "wake") {
            SetLidarMode(kLidarModeNormal);
            ROS_INFO("LiDAR set to normal mode.");
        } else {
            ROS_WARN_STREAM("Unknown command: " << msg->data);
        }
    }

    ~LivoxControl() {
        Uninit();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_control_node");
    LivoxControl livox_control;
    ros::spin();
    return 0;
}

