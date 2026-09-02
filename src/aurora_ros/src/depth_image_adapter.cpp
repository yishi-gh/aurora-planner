#include "aurora_ros/depth_image_adapter.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace aurora::ros {
namespace {

constexpr double kEpsilon = 1e-12;

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool isZeroTime(const builtin_interfaces::msg::Time &time) {
  return time.sec == 0 && time.nanosec == 0U;
}

std::uint16_t readUint16(const std::uint8_t *data, bool big_endian) {
  if (big_endian) {
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
  }
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[1]) << 8U) | data[0]);
}

std::uint32_t readUint32(const std::uint8_t *data, bool big_endian) {
  if (big_endian) {
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
           (static_cast<std::uint32_t>(data[1]) << 16U) |
           (static_cast<std::uint32_t>(data[2]) << 8U) | data[3];
  }
  return (static_cast<std::uint32_t>(data[3]) << 24U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[1]) << 8U) | data[0];
}

bool safeImageSize(std::size_t height, std::size_t step, std::size_t data_size,
                   std::size_t row_bytes) {
  if (height == 0U || row_bytes > step) {
    return false;
  }
  const std::size_t last_row = height - 1U;
  if (last_row != 0U && step > (std::numeric_limits<std::size_t>::max() / last_row)) {
    return false;
  }
  const std::size_t last_row_offset = last_row * step;
  return last_row_offset <= data_size && row_bytes <= data_size - last_row_offset;
}

}  // namespace

DepthImageAdapter::DepthImageAdapter(DepthImageAdapterOptions options) : options_(options) {
  if (options_.max_points == 0U || options_.pixel_stride == 0U ||
      !finitePositive(options_.min_depth) || !finitePositive(options_.max_depth) ||
      options_.max_depth < options_.min_depth ||
      !std::isfinite(options_.camera_info_time_tolerance) ||
      options_.camera_info_time_tolerance < 0.0) {
    throw std::invalid_argument("invalid depth image adapter options");
  }
}

bool DepthImageAdapter::validRosTime(const builtin_interfaces::msg::Time &time) {
  return time.sec >= 0 && time.nanosec < 1000000000U;
}

double DepthImageAdapter::timeToSeconds(const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec);
}

std::size_t DepthImageAdapter::sampledDimension(std::size_t dimension,
                                                std::size_t stride) {
  return dimension == 0U ? 0U : (dimension - 1U) / stride + 1U;
}

std::size_t DepthImageAdapter::sampledPixelCount(std::size_t width, std::size_t height,
                                                std::size_t stride) {
  const std::size_t sampled_width = sampledDimension(width, stride);
  const std::size_t sampled_height = sampledDimension(height, stride);
  if (sampled_width != 0U &&
      sampled_height > std::numeric_limits<std::size_t>::max() / sampled_width) {
    return std::numeric_limits<std::size_t>::max();
  }
  return sampled_width * sampled_height;
}

std::size_t DepthImageAdapter::minimumStrideForLimit(std::size_t width, std::size_t height,
                                                      std::size_t initial_stride,
                                                      std::size_t max_points) {
  const std::size_t upper_bound = std::max(width, height);
  if (upper_bound == 0U || sampledPixelCount(width, height, initial_stride) <= max_points) {
    return initial_stride;
  }

  std::size_t lower = initial_stride;
  std::size_t upper = upper_bound;
  while (lower < upper) {
    const std::size_t middle = lower + (upper - lower) / 2U;
    if (sampledPixelCount(width, height, middle) <= max_points) {
      upper = middle;
    } else {
      lower = middle + 1U;
    }
  }
  return lower;
}

bool DepthImageAdapter::hasValidProjection(const sensor_msgs::msg::CameraInfo &camera_info,
                                           double *fx, double *fy, double *cx,
                                           double *cy) {
  // Prefer P because it is the rectified pinhole projection used by the
  // standard *_rect_raw depth topics. Fall back to K for distortion-free data.
  if (finitePositive(camera_info.p[0]) && finitePositive(camera_info.p[5]) &&
      std::isfinite(camera_info.p[2]) && std::isfinite(camera_info.p[6])) {
    *fx = camera_info.p[0];
    *fy = camera_info.p[5];
    *cx = camera_info.p[2];
    *cy = camera_info.p[6];
    return true;
  }
  if (finitePositive(camera_info.k[0]) && finitePositive(camera_info.k[4]) &&
      std::isfinite(camera_info.k[2]) && std::isfinite(camera_info.k[5])) {
    *fx = camera_info.k[0];
    *fy = camera_info.k[4];
    *cx = camera_info.k[2];
    *cy = camera_info.k[5];
    for (const double distortion : camera_info.d) {
      if (std::abs(distortion) > kEpsilon) {
        return false;
      }
    }
    return true;
  }
  return false;
}

DepthImagePointCloud DepthImageAdapter::convert(
    const sensor_msgs::msg::Image &image,
    const sensor_msgs::msg::CameraInfo &camera_info) const {
  DepthImagePointCloud result;
  result.frame_id = image.header.frame_id;

  if (!validRosTime(image.header.stamp)) {
    result.detail = "depth image timestamp is invalid";
    return result;
  }
  result.stamp = timeToSeconds(image.header.stamp);
  if (image.header.frame_id.empty() ||
      camera_info.header.frame_id != image.header.frame_id) {
    result.detail = "depth image and camera info frame ids must match and be non-empty";
    return result;
  }
  if (!validRosTime(camera_info.header.stamp)) {
    result.detail = "camera info timestamp is invalid";
    return result;
  }
  if (!isZeroTime(camera_info.header.stamp) &&
      std::abs(timeToSeconds(camera_info.header.stamp) - result.stamp) >
          options_.camera_info_time_tolerance) {
    result.detail = "camera info is outside the configured time tolerance";
    return result;
  }
  if (image.width == 0U || image.height == 0U ||
      (camera_info.width != 0U && camera_info.width != image.width) ||
      (camera_info.height != 0U && camera_info.height != image.height)) {
    result.detail = "depth image and camera info dimensions do not match";
    return result;
  }

  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  if (!hasValidProjection(camera_info, &fx, &fy, &cx, &cy)) {
    result.detail = "camera info does not provide a valid rectified pinhole projection";
    return result;
  }

  std::size_t bytes_per_pixel = 0U;
  const bool is_uint16 = image.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  const bool is_float32 = image.encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  if (is_uint16) {
    bytes_per_pixel = sizeof(std::uint16_t);
  } else if (is_float32) {
    bytes_per_pixel = sizeof(float);
  } else {
    result.detail = "unsupported depth encoding; expected 16UC1 or 32FC1";
    return result;
  }
  if (image.step < image.width * bytes_per_pixel ||
      !safeImageSize(image.height, image.step, image.data.size(),
                     image.width * bytes_per_pixel)) {
    result.detail = "depth image step or data size is invalid";
    return result;
  }

  const std::size_t stride = minimumStrideForLimit(
      image.width, image.height, options_.pixel_stride, options_.max_points);
  result.sampled_pixel_count = sampledPixelCount(image.width, image.height, stride);
  result.endpoints.reserve(std::min(options_.max_points, result.sampled_pixel_count));

  for (std::size_t v = 0U; v < image.height; v += stride) {
    const std::size_t row_offset = v * image.step;
    for (std::size_t u = 0U; u < image.width; u += stride) {
      const std::uint8_t *pixel = image.data.data() + row_offset + u * bytes_per_pixel;
      double depth = std::numeric_limits<double>::quiet_NaN();
      if (is_uint16) {
        depth = 0.001 * static_cast<double>(readUint16(pixel, image.is_bigendian));
      } else {
        const std::uint32_t bits = readUint32(pixel, image.is_bigendian);
        float value = 0.0F;
        std::memcpy(&value, &bits, sizeof(value));
        depth = static_cast<double>(value);
      }
      if (!std::isfinite(depth) || depth < options_.min_depth ||
          depth > options_.max_depth) {
        ++result.invalid_depth_count;
        continue;
      }
      const Eigen::Vector3d endpoint(
          (static_cast<double>(u) - cx) * depth / fx,
          (static_cast<double>(v) - cy) * depth / fy,
          depth);
      if (!endpoint.allFinite()) {
        ++result.invalid_depth_count;
        continue;
      }
      result.endpoints.push_back(endpoint);
    }
  }

  result.valid = true;
  result.detail = result.endpoints.empty() ? "depth image contained no valid samples" : "ok";
  return result;
}

}  // namespace aurora::ros
