#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <memory>


class EPDCore : public rclcpp::Node
{
  public:
    EPDCore()
    : Node("epd_core")
    {
      cv::namedWindow("image_viewer", cv::WINDOW_AUTOSIZE);
      int x_position = 0;
      int y_position = 0;
      cv::moveWindow("image_viewer", x_position, y_position);
      cv::waitKey(1);

      size_t depth_ = rmw_qos_profile_default.depth;
      rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
      rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;

      auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(history_policy_, depth_));
      qos.reliability(reliability_policy_);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "epd_core/image", 10, std::bind(&EPDCore::image_callback, this, std::placeholders::_1));
    
      // rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
      // this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &EPDCore::add);
      
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
    }

  private:
    int encoding2mat_type(const std::string & encoding) const
    {
      if (encoding == "mono8") {
        return CV_8UC1;
      } else if (encoding == "bgr8") {
        return CV_8UC3;
      } else if (encoding == "mono16") {
        return CV_16SC1;
      } else if (encoding == "rgba8") {
        return CV_8UC4;
      } else if (encoding == "bgra8") {
        return CV_8UC4;
      } else if (encoding == "32FC1") {
        return CV_32FC1;
      } else if (encoding == "rgb8") {
        return CV_8UC3;
      } else {
        throw std::runtime_error("Unsupported encoding type");
      }
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Image Received");
      if (msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Input image empty. Discarding.");
        return;
      }
      // // Convert ROS Image message to cv::Mat for processing.
      // std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg, "bgr8");
      // cv::Mat img = imgptr->image;
      cv::Mat frame(
        msg->height, msg->width, this->encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);

      if (msg->encoding == "rgb8") {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      }

      cv::Mat cvframe = frame;

      cv::imshow("image_viewer", cvframe);
      cv::waitKey(1);
    }
    // void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    //       std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    // {
    //   response->sum = request->a + request->b;
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
    //             request->a, request->b);
    //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    // }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EPDCore>());
  rclcpp::shutdown();
  return 0;
}