#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dual_nero_camera_relay
{
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

class ColorJpegRelay : public rclcpp::Node
{
public:
  ColorJpegRelay()
  : Node("color_jpeg_relay")
  {
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 78);
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      RCLCPP_WARN(get_logger(), "jpeg_quality=%d is outside 1..100; clamping",
        jpeg_quality_);
      jpeg_quality_ = std::clamp(jpeg_quality_, 1, 100);
    }

    add_relay("scene", "scene_input", "scene_output");
    add_relay("left_wrist", "left_wrist_input", "left_wrist_output");
    add_relay("right_wrist", "right_wrist_input", "right_wrist_output");
  }

private:
  void add_relay(
    const std::string & name,
    const std::string & input_parameter,
    const std::string & output_parameter)
  {
    const auto input = declare_parameter<std::string>(input_parameter, "");
    const auto output = declare_parameter<std::string>(output_parameter, "");
    if (input.empty() || output.empty()) {
      RCLCPP_WARN(get_logger(), "%s relay disabled: input/output is empty",
        name.c_str());
      return;
    }

    auto publisher = create_publisher<CompressedImage>(
      output, rclcpp::SensorDataQoS().keep_last(5));
    auto subscription = create_subscription<Image>(
      input, rclcpp::SensorDataQoS().keep_last(10),
      [this, name, publisher](Image::ConstSharedPtr image) {
        publish_jpeg(name, publisher, *image);
      });
    subscriptions_.push_back(subscription);
    RCLCPP_INFO(get_logger(), "%s relay: %s -> %s (JPEG q=%d)",
      name.c_str(), input.c_str(), output.c_str(), jpeg_quality_);
  }

  void publish_jpeg(
    const std::string & name,
    const rclcpp::Publisher<CompressedImage>::SharedPtr & publisher,
    const Image & image)
  {
    cv::Mat bgr;
    std::string encoding = image.encoding;
    std::transform(encoding.begin(), encoding.end(), encoding.begin(),
      [](unsigned char value) {return static_cast<char>(std::tolower(value));});

    auto raw = const_cast<unsigned char *>(image.data.data());
    if (encoding == "rgb8") {
      cv::Mat rgb(image.height, image.width, CV_8UC3, raw, image.step);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    } else if (encoding == "bgr8") {
      bgr = cv::Mat(image.height, image.width, CV_8UC3, raw, image.step);
    } else if (encoding == "rgba8") {
      cv::Mat rgba(image.height, image.width, CV_8UC4, raw, image.step);
      cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    } else if (encoding == "bgra8") {
      cv::Mat bgra(image.height, image.width, CV_8UC4, raw, image.step);
      cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    } else if (
      encoding == "yuv422_yuy2" || encoding == "yuyv" ||
      encoding == "yuv422")
    {
      cv::Mat yuyv(image.height, image.width, CV_8UC2, raw, image.step);
      cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
    } else if (encoding == "mono8" || encoding == "8uc1") {
      cv::Mat mono(image.height, image.width, CV_8UC1, raw, image.step);
      cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "%s relay does not support encoding %s", name.c_str(),
        image.encoding.c_str());
      return;
    }

    std::vector<unsigned char> jpeg;
    const std::vector<int> parameters{
      cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", bgr, jpeg, parameters)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "%s relay failed to encode JPEG", name.c_str());
      return;
    }

    auto output = std::make_unique<CompressedImage>();
    output->header = image.header;
    output->format = "jpeg";
    output->data = std::move(jpeg);
    publisher->publish(std::move(output));
  }

  int jpeg_quality_{78};
  std::vector<rclcpp::Subscription<Image>::SharedPtr> subscriptions_;
};
}  // namespace dual_nero_camera_relay

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dual_nero_camera_relay::ColorJpegRelay>());
  rclcpp::shutdown();
  return 0;
}
