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

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  // 主要参数声明（第一个扫描）
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  scan_out_topic1_ = this->declare_parameter("scan_out_topic", "scan");  // 第一个扫描输出话题

  // 新增：第二个扫描的参数声明
  enable_second_scan_ = this->declare_parameter("enable_second_scan", false);  // 默认关闭第二个扫描
  target_frame2_ = this->declare_parameter("target_frame2", target_frame_);    // 默认与第一个相同
  min_height2_ = this->declare_parameter("min_height2", min_height_);
  max_height2_ = this->declare_parameter("max_height2", max_height_);
  angle_min2_ = this->declare_parameter("angle_min2", angle_min_);
  angle_max2_ = this->declare_parameter("angle_max2", angle_max_);
  angle_increment2_ = this->declare_parameter("angle_increment2", angle_increment_);
  scan_time2_ = this->declare_parameter("scan_time2", scan_time_);
  range_min2_ = this->declare_parameter("range_min2", range_min_);
  range_max2_ = this->declare_parameter("range_max2", range_max_);
  use_inf2_ = this->declare_parameter("use_inf2", use_inf_);
  inf_epsilon2_ = this->declare_parameter("inf_epsilon2", inf_epsilon_);
  scan_out_topic2_ = this->declare_parameter("scan_out_topic2", "scan2");  // 第二个扫描输出话题

  // 创建第一个激光扫描发布器
  pub1_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    scan_out_topic1_, rclcpp::QoS(10).reliable());

  // 如果启用第二个扫描，创建第二个发布器
  if (enable_second_scan_) {
    pub2_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      scan_out_topic2_, rclcpp::QoS(10).reliable());
    RCLCPP_INFO(this->get_logger(), "Second laser scan enabled, publishing to topic: %s", 
                scan_out_topic2_.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "First laser scan publishing to topic: %s", scan_out_topic1_.c_str());

  using std::placeholders::_1;
  // 如果指定了点云目标坐标系，需要通过变换可用性进行过滤
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // 否则设置直接订阅
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    // 修改：检查两个发布器的订阅状态
    int subscription_count = 0;
    if (pub1_) {
      subscription_count += pub1_->get_subscription_count() + pub1_->get_intra_process_subscription_count();
    }
    if (enable_second_scan_ && pub2_) {
      subscription_count += pub2_->get_subscription_count() + pub2_->get_intra_process_subscription_count();
    }
    
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got subscribers to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // 处理第一个激光扫描
  processLaserScan(cloud_msg, pub1_, target_frame_, 
                   min_height_, max_height_, 
                   angle_min_, angle_max_, angle_increment_,
                   scan_time_, range_min_, range_max_,
                   use_inf_, inf_epsilon_);

  // 如果启用第二个扫描，处理第二个激光扫描
  if (enable_second_scan_ && pub2_) {
    processLaserScan(cloud_msg, pub2_, target_frame2_,
                     min_height2_, max_height2_,
                     angle_min2_, angle_max2_, angle_increment2_,
                     scan_time2_, range_min2_, range_max2_,
                     use_inf2_, inf_epsilon2_);
  }
}

// 新增：处理单个激光扫描转换的辅助函数
void PointCloudToLaserScanNode::processLaserScan(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> publisher,
  const std::string& target_frame,
  double min_height, double max_height,
  double angle_min, double angle_max, double angle_increment,
  double scan_time, double range_min, double range_max,
  bool use_inf, double inf_epsilon)
{
  // 构建激光扫描输出
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame.empty()) {
    scan_msg->header.frame_id = target_frame;
  }

  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = angle_increment;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time;
  scan_msg->range_min = range_min;
  scan_msg->range_max = range_max;

  // 确定要创建的射线数量
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // 确定没有障碍物数据的激光扫描射线将评估为无穷大还是最大范围
  if (use_inf) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon);
  }

  // 如果需要，变换点云坐标系
  if (!target_frame.empty() && scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // 迭代处理点云数据
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    // 高度过滤
    if (*iter_z > max_height || *iter_z < min_height) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height, max_height);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    // 距离过滤
    if (range < range_min) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    // 角度过滤
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // 如果新距离更小，覆盖激光扫描射线上的距离
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  
  // 发布激光扫描数据
  publisher->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)