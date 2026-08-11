/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
#include <asio.hpp>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>
#include <array>
#include <cmath>

namespace mid360_driver {

    struct Point {
        double timestamp;
        float x, y, z;
        float intensity;
    };

    struct ImuMsg {
        double timestamp;
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct IpAddressHasher {
        std::size_t operator()(const asio::ip::address &addr) const noexcept;
    };

    class Transform {
    private:
        std::array<std::array<float, 4>, 4> transform_matrix;
        bool enabled;
        float min_distance;
        float max_distance;
        float min_distance_sq;  // 预计算的距离平方值
        float max_distance_sq;  // 预计算的距离平方值
        bool distance_filter_enabled;
        float min_x, max_x, min_y, max_y, min_z, max_z;
        bool enable_x_filter, enable_y_filter, enable_z_filter;

    public:
        Transform() : enabled(false), min_distance(-1.0f), max_distance(-1.0f),
                      min_distance_sq(-1.0f), max_distance_sq(-1.0f), distance_filter_enabled(false),
                      min_x(-1.0f), max_x(-1.0f), min_y(-1.0f), max_y(-1.0f), min_z(-1.0f), max_z(-1.0f),
                      enable_x_filter(false), enable_y_filter(false), enable_z_filter(false) {
            // 初始化单位矩阵
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    transform_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }

        void setTransform(float roll_deg, float pitch_deg, float yaw_deg, float tx, float ty, float tz, bool enable) {
            enabled = enable;
            if (!enabled) {
                // 重置为单位矩阵
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        transform_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
                    }
                }
                return;
            }

            // 将角度从度转换为弧度
            float roll = roll_deg * M_PI / 180.0f;
            float pitch = pitch_deg * M_PI / 180.0f;
            float yaw = yaw_deg * M_PI / 180.0f;

            // 计算旋转矩阵（ZYX顺序：yaw -> pitch -> roll）
            float cr = cos(roll);
            float sr = sin(roll);
            float cp = cos(pitch);
            float sp = sin(pitch);
            float cy = cos(yaw);
            float sy = sin(yaw);

            // 旋转矩阵 R = Rz(yaw) * Ry(pitch) * Rx(roll)
            transform_matrix[0][0] = cy * cp;
            transform_matrix[0][1] = cy * sp * sr - sy * cr;
            transform_matrix[0][2] = cy * sp * cr + sy * sr;
            transform_matrix[0][3] = tx;

            transform_matrix[1][0] = sy * cp;
            transform_matrix[1][1] = sy * sp * sr + cy * cr;
            transform_matrix[1][2] = sy * sp * cr - cy * sr;
            transform_matrix[1][3] = ty;

            transform_matrix[2][0] = -sp;
            transform_matrix[2][1] = cp * sr;
            transform_matrix[2][2] = cp * cr;
            transform_matrix[2][3] = tz;

            transform_matrix[3][0] = 0.0f;
            transform_matrix[3][1] = 0.0f;
            transform_matrix[3][2] = 0.0f;
            transform_matrix[3][3] = 1.0f;
        }

        void setDistanceFilter(float min_dist, float max_dist) {
            min_distance = min_dist;
            max_distance = max_dist;
            min_distance_sq = (min_dist >= 0.0f) ? (min_dist * min_dist) : -1.0f;
            max_distance_sq = (max_dist >= 0.0f) ? (max_dist * max_dist) : -1.0f;
            distance_filter_enabled = (min_dist >= 0.0f) || (max_dist >= 0.0f);
        }

        void setAxisFilter(bool enable_x, float min_x_, float max_x_,
                          bool enable_y, float min_y_, float max_y_,
                          bool enable_z, float min_z_, float max_z_) {
            enable_x_filter = enable_x; min_x = min_x_; max_x = max_x_;
            enable_y_filter = enable_y; min_y = min_y_; max_y = max_y_;
            enable_z_filter = enable_z; min_z = min_z_; max_z = max_z_;
        }

        bool filterPointByDistance(float x, float y, float z) const {
            if (!distance_filter_enabled && !enable_x_filter && !enable_y_filter && !enable_z_filter) {
                return true; // 不过滤
            }

            // 距离筛选 - 使用距离平方比较，避免 sqrt 计算
            if (distance_filter_enabled) {
                float distance_sq = x*x + y*y + z*z;
                if (min_distance_sq >= 0.0f && distance_sq < min_distance_sq) {
                    return false; // 过滤掉
                }
                if (max_distance_sq >= 0.0f && distance_sq > max_distance_sq) {
                    return false; // 过滤掉
                }
            }

            // 轴向筛选
            if (enable_x_filter && (x < min_x || x > max_x)) return false;
            if (enable_y_filter && (y < min_y || y > max_y)) return false;
            if (enable_z_filter && (z < min_z || z > max_z)) return false;

            return true; // 保留
        }

        void transformPoint(float &x, float &y, float &z) const {
            if (!enabled) return;

            float x_new = transform_matrix[0][0] * x + transform_matrix[0][1] * y + transform_matrix[0][2] * z + transform_matrix[0][3];
            float y_new = transform_matrix[1][0] * x + transform_matrix[1][1] * y + transform_matrix[1][2] * z + transform_matrix[1][3];
            float z_new = transform_matrix[2][0] * x + transform_matrix[2][1] * y + transform_matrix[2][2] * z + transform_matrix[2][3];

            x = x_new;
            y = y_new;
            z = z_new;
        }

        void transformVector(float &vx, float &vy, float &vz) const {
            if (!enabled) return;

            // 只应用旋转，不应用平移
            float vx_new = transform_matrix[0][0] * vx + transform_matrix[0][1] * vy + transform_matrix[0][2] * vz;
            float vy_new = transform_matrix[1][0] * vx + transform_matrix[1][1] * vy + transform_matrix[1][2] * vz;
            float vz_new = transform_matrix[2][0] * vx + transform_matrix[2][1] * vy + transform_matrix[2][2] * vz;

            vx = vx_new;
            vy = vy_new;
            vz = vz_new;
        }

        bool isEnabled() const { return enabled; }
        bool isDistanceFilterEnabled() const { return distance_filter_enabled; }
        float getMinDistance() const { return min_distance; }
        float getMaxDistance() const { return max_distance; }
    };

    class Mid360Driver {
    private:
        std::atomic<bool> is_running = true;
        asio::ip::address host_ip;
        asio::ip::udp::socket receive_pointcloud_socket;
        asio::ip::udp::socket receive_imu_socket;
        std::vector<Point> points;
        std::unordered_map<asio::ip::address, double, IpAddressHasher> delta_time_map;
        std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> on_receive_pointcloud;
        std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> on_receive_imu;
        Transform transform;

    public:
        Mid360Driver(asio::io_context &io_context,
                     const asio::ip::address &host_ip,
                     const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                     const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu,
                     const Transform &transform = Transform());

        ~Mid360Driver();

        void stop();

        void setTransform(const Transform &new_transform);

        asio::awaitable<void> receive_pointcloud();

        asio::awaitable<void> receive_imu();
    };

}// namespace mid360_driver