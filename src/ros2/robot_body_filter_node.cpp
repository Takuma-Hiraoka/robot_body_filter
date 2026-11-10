// SPDX-License-Identifier: BSD-3-Clause
#include <deque>
#include <robot_body_filter/ros2/robot_body_filter.hpp>

class RobotBodyFilterNode : public rclcpp::Node
{
public:
  RobotBodyFilterNode() : Node("robot_body_filter")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/cloud_in");
    const auto output = declare_parameter<std::string>("output_topic", "/cloud_out");
    queue_size_ = declare_parameter<int>("max_queue_size", 5);
    if (queue_size_ < 1) throw std::runtime_error("max_queue_size must be positive");
    if (!filter_.configure("", get_name(), get_node_logging_interface(), get_node_parameters_interface()))
      throw std::runtime_error("Cannot configure robot_body_filter");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::QoS(queue_size_).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input, rclcpp::SensorDataQoS().keep_last(queue_size_),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          pending_.push_back(std::move(cloud));
          if (pending_.size() > queue_size_) {
            pending_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Dropping old cloud while waiting for collision-link TF");
          }
          process();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { process(); });
  }
private:
  void process()
  {
    while (!pending_.empty()) {
      sensor_msgs::msg::PointCloud2 output;
      if (!filter_.update(*pending_.front(), output)) break;
      publisher_->publish(output);
      pending_.pop_front();
    }
  }
  robot_body_filter::RobotBodyFilterPointCloud2 filter_;
  size_t queue_size_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RobotBodyFilterNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_body_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
