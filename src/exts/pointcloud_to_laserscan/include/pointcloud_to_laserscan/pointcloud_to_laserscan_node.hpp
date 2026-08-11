/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#ifndef POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
#define POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pointcloud_to_laserscan/visibility_control.h"

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

/**
* Class to process incoming pointclouds into laserscans.
* Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
*/
class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
  POINTCLOUD_TO_LASERSCAN_PUBLIC
  explicit PointCloudToLaserScanNode(const rclcpp::NodeOptions & options);

  ~PointCloudToLaserScanNode() override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

  void subscriptionListenerThreadLoop();

  // 新增：处理单个激光扫描转换的辅助函数
  void processLaserScan(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> publisher,
    const std::string& target_frame,
    double min_height, double max_height,
    double angle_min, double angle_max, double angle_increment,
    double scan_time, double range_min, double range_max,
    bool use_inf, double inf_epsilon);

  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
  
  // 修改：支持两个激光扫描发布器
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub1_;  // 第一个激光扫描发布器
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub2_;  // 第二个激光扫描发布器
  
  std::unique_ptr<MessageFilter> message_filter_;

  std::thread subscription_listener_thread_;
  std::atomic_bool alive_{true};

  // ROS Parameters - 主要参数（用于第一个扫描）
  int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
  bool use_inf_;
  double inf_epsilon_;

  // 新增：第二个激光扫描的参数
  bool enable_second_scan_;           // 是否启用第二个扫描转换
  std::string target_frame2_;         // 第二个扫描的目标坐标系
  double min_height2_, max_height2_;  // 第二个扫描的高度过滤范围
  double angle_min2_, angle_max2_, angle_increment2_;  // 第二个扫描的角度参数
  double scan_time2_, range_min2_, range_max2_;       // 第二个扫描的时间距离参数
  bool use_inf2_;                                      // 第二个扫描是否使用无穷大
  double inf_epsilon2_;                               // 第二个扫描的无穷大容差值
  
  // 新增：话题名称配置
  std::string scan_out_topic1_;  // 第一个扫描输出话题
  std::string scan_out_topic2_;  // 第二个扫描输出话题
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_