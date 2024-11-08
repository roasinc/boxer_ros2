// Copyright (c) 2023, ROAS Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class BoxerBridge : public rclcpp::Node
{
public:
  BoxerBridge(const std::string& node) : Node(node)
  {
    this->declare_parameter("api_version", rclcpp::ParameterValue(""));
    this->declare_parameter("serial_no", rclcpp::ParameterValue(""));
    this->declare_parameter("relay_odom", rclcpp::ParameterValue(false));
    this->declare_parameter("relay_scan", rclcpp::ParameterValue(false));

    this->get_parameter("api_version", api_version_);
    this->get_parameter("serial_no", serial_no_);
    this->get_parameter("relay_odom", relay_odom_);
    this->get_parameter("relay_scan", relay_scan_);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/cpr_platform_api/" + api_version_ + "/" + serial_no_ + "/platform/odom", rclcpp::SensorDataQoS(),
        [=](const nav_msgs::msg::Odometry::SharedPtr msg) { OdometryCallback(msg); });

    if (relay_odom_)
    {
      pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
      rp_odom_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(pub_odom_);
    }

    if (relay_scan_)
    {
      pub_front_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/front/scan", 5);
      pub_rear_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/rear/scan", 5);

      rp_front_scan_ =
          std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(pub_front_scan_);
      rp_rear_scan_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(pub_rear_scan_);

      sub_front_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/cpr_platform_api/" + api_version_ + "/" + serial_no_ + "/laser/module0/scan", rclcpp::SensorDataQoS(),
          [=](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            if (rp_front_scan_->trylock())
            {
              rp_front_scan_->msg_ = *msg;
              rp_front_scan_->unlockAndPublish();
            }
          });
      sub_rear_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/cpr_platform_api/" + api_version_ + "/" + serial_no_ + "/laser/module1/scan", rclcpp::SensorDataQoS(),
          [=](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            if (rp_rear_scan_->trylock())
            {
              rp_rear_scan_->msg_ = *msg;
              rp_rear_scan_->unlockAndPublish();
            }
          });
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  virtual ~BoxerBridge() = default;

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header = msg->header;
    t.child_frame_id = msg->child_frame_id;
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(t);

    if (relay_odom_)
    {
      if (rp_odom_->trylock())
      {
        rp_odom_->msg_ = *msg;
        rp_odom_->unlockAndPublish();
      }
    }
  }

private:
  /// ROS2 interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_front_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_rear_scan_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rp_odom_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> rp_front_scan_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> rp_rear_scan_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_front_scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_rear_scan_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string api_version_, serial_no_;
  bool relay_odom_, relay_scan_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoxerBridge>("boxer_bridge");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}