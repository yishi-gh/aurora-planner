#include "aurora_ros/depth_image_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace {

using aurora::ros::DepthImageAdapter;
using aurora::ros::DepthImageAdapterOptions;

builtin_interfaces::msg::Time makeTime(double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - std::floor(seconds)) * 1e9));
  return time;
}

sensor_msgs::msg::CameraInfo makeCameraInfo(std::uint32_t width = 2U,
                                            std::uint32_t height = 2U) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.stamp = makeTime(10.0);
  camera_info.header.frame_id = "camera_optical_frame";
  camera_info.width = width;
  camera_info.height = height;
  camera_info.k[0] = 2.0;
  camera_info.k[2] = 0.5;
  camera_info.k[4] = 4.0;
  camera_info.k[5] = 0.5;
  camera_info.p[0] = 2.0;
  camera_info.p[2] = 0.5;
  camera_info.p[5] = 4.0;
  camera_info.p[6] = 0.5;
  return camera_info;
}

sensor_msgs::msg::Image makeImage(const std::string &encoding, std::uint32_t width = 2U,
                                  std::uint32_t height = 2U) {
  sensor_msgs::msg::Image image;
  image.header.stamp = makeTime(10.0);
  image.header.frame_id = "camera_optical_frame";
  image.width = width;
  image.height = height;
  image.encoding = encoding;
  image.is_bigendian = false;
  return image;
}

void appendUint16(sensor_msgs::msg::Image *image, std::uint16_t value, bool big_endian) {
  if (big_endian) {
    image->data.push_back(static_cast<std::uint8_t>(value >> 8U));
    image->data.push_back(static_cast<std::uint8_t>(value & 0xffU));
  } else {
    image->data.push_back(static_cast<std::uint8_t>(value & 0xffU));
    image->data.push_back(static_cast<std::uint8_t>(value >> 8U));
  }
}

void appendFloat32(sensor_msgs::msg::Image *image, float value, bool big_endian) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  if (big_endian) {
    image->data.push_back(static_cast<std::uint8_t>(bits >> 24U));
    image->data.push_back(static_cast<std::uint8_t>(bits >> 16U));
    image->data.push_back(static_cast<std::uint8_t>(bits >> 8U));
    image->data.push_back(static_cast<std::uint8_t>(bits));
  } else {
    image->data.push_back(static_cast<std::uint8_t>(bits));
    image->data.push_back(static_cast<std::uint8_t>(bits >> 8U));
    image->data.push_back(static_cast<std::uint8_t>(bits >> 16U));
    image->data.push_back(static_cast<std::uint8_t>(bits >> 24U));
  }
}

TEST(DepthImageAdapter, ConvertsPinholeUint16MillimetresAndSkipsInvalidPixels) {
  DepthImageAdapter adapter;
  auto image = makeImage("16UC1");
  image.step = 6U;
  appendUint16(&image, 1000U, false);
  appendUint16(&image, 2000U, false);
  image.data.push_back(0U);
  image.data.push_back(0U);
  appendUint16(&image, 0U, false);
  appendUint16(&image, 1000U, false);
  image.data.push_back(0U);
  image.data.push_back(0U);

  const auto result = adapter.convert(image, makeCameraInfo());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.endpoints.size(), 3U);
  EXPECT_EQ(result.sampled_pixel_count, 4U);
  EXPECT_EQ(result.invalid_depth_count, 1U);
  EXPECT_NEAR(result.endpoints[0].x(), -0.25, 1e-12);
  EXPECT_NEAR(result.endpoints[0].y(), -0.125, 1e-12);
  EXPECT_DOUBLE_EQ(result.endpoints[0].z(), 1.0);
  EXPECT_NEAR(result.endpoints[1].x(), 0.5, 1e-12);
  EXPECT_NEAR(result.endpoints[1].y(), -0.25, 1e-12);
}

TEST(DepthImageAdapter, ConvertsBigEndianFloatAndUsesZeroStampedCalibration) {
  DepthImageAdapter adapter;
  auto image = makeImage("32FC1", 1U, 1U);
  image.is_bigendian = true;
  image.step = 4U;
  appendFloat32(&image, 2.5F, true);
  auto camera_info = makeCameraInfo(1U, 1U);
  camera_info.header.stamp = builtin_interfaces::msg::Time{};

  const auto result = adapter.convert(image, camera_info);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.endpoints.size(), 1U);
  EXPECT_DOUBLE_EQ(result.endpoints.front().z(), 2.5);
  EXPECT_NEAR(result.endpoints.front().x(), -0.625, 1e-12);
  EXPECT_NEAR(result.endpoints.front().y(), -0.3125, 1e-12);
}

TEST(DepthImageAdapter, EnforcesDeterministicPixelLimitWithStride) {
  DepthImageAdapter adapter(DepthImageAdapterOptions{3U, 1U, 0.1, 30.0, 0.5});
  auto image = makeImage("16UC1", 4U, 4U);
  image.step = 8U;
  for (std::size_t index = 0U; index < 16U; ++index) {
    appendUint16(&image, 1000U, false);
  }
  auto camera_info = makeCameraInfo(4U, 4U);

  const auto result = adapter.convert(image, camera_info);

  ASSERT_TRUE(result.valid);
  EXPECT_LE(result.sampled_pixel_count, 3U);
  EXPECT_EQ(result.endpoints.size(), result.sampled_pixel_count);
}

TEST(DepthImageAdapter, RejectsMalformedEncodingLayoutFramesAndCalibration) {
  DepthImageAdapter adapter;
  auto image = makeImage("mono8");
  image.step = 2U;
  image.data.resize(4U);
  EXPECT_FALSE(adapter.convert(image, makeCameraInfo()).valid);

  auto malformed = makeImage("16UC1");
  malformed.step = 2U;
  malformed.data.resize(2U);
  EXPECT_FALSE(adapter.convert(malformed, makeCameraInfo()).valid);

  auto wrong_frame = makeCameraInfo();
  wrong_frame.header.frame_id = "other_frame";
  EXPECT_FALSE(adapter.convert(makeImage("16UC1"), wrong_frame).valid);

  auto distorted = makeCameraInfo();
  distorted.p.fill(0.0);
  distorted.d = {0.1};
  EXPECT_FALSE(adapter.convert(makeImage("16UC1"), distorted).valid);
}

TEST(DepthImageAdapter, RejectsStaleCameraInfoAndInvalidOptions) {
  DepthImageAdapter adapter;
  auto stale = makeCameraInfo();
  stale.header.stamp = makeTime(12.0);
  EXPECT_FALSE(adapter.convert(makeImage("16UC1"), stale).valid);

  EXPECT_THROW(DepthImageAdapter(DepthImageAdapterOptions{0U, 1U, 0.1, 1.0, 0.5}),
               std::invalid_argument);
  EXPECT_THROW(DepthImageAdapter(DepthImageAdapterOptions{10U, 1U, 2.0, 1.0, 0.5}),
               std::invalid_argument);
}

}  // namespace
