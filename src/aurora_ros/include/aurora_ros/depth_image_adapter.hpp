#pragma once

#include <Eigen/Core>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aurora::ros {

struct DepthImageAdapterOptions {
  std::size_t max_points{100000U};
  std::size_t pixel_stride{1U};
  double min_depth{0.1};
  double max_depth{30.0};
  double camera_info_time_tolerance{0.5};
};

// A depth image is converted to rays in the image frame. TF application and
// map mutation remain owned by the ROS node so this adapter has no map state.
struct DepthImagePointCloud {
  bool valid{false};
  double stamp{0.0};
  std::string frame_id;
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> endpoints;
  std::size_t sampled_pixel_count{0U};
  std::size_t invalid_depth_count{0U};
  std::string detail;
};

class DepthImageAdapter {
public:
  explicit DepthImageAdapter(DepthImageAdapterOptions options = {});

  const DepthImageAdapterOptions &options() const noexcept { return options_; }

  DepthImagePointCloud convert(const sensor_msgs::msg::Image &image,
                               const sensor_msgs::msg::CameraInfo &camera_info) const;

private:
  static bool validRosTime(const builtin_interfaces::msg::Time &time);
  static double timeToSeconds(const builtin_interfaces::msg::Time &time);
  static std::size_t sampledDimension(std::size_t dimension, std::size_t stride);
  static std::size_t sampledPixelCount(std::size_t width, std::size_t height,
                                       std::size_t stride);
  static std::size_t minimumStrideForLimit(std::size_t width, std::size_t height,
                                           std::size_t initial_stride,
                                           std::size_t max_points);
  static bool hasValidProjection(const sensor_msgs::msg::CameraInfo &camera_info,
                                 double *fx, double *fy, double *cx, double *cy);

  DepthImageAdapterOptions options_;
};

}  // namespace aurora::ros
