// ROS1/ROS2 serial GNSS publisher. Parsing and byte capture are shared with NTRIP.
#include "rtk_stream.hpp"

#if defined(USE_ROS2)
class RtkNode : public rclcpp::Node {
public:
  RtkNode() : rclcpp::Node("handsfree_rtk") {
    port_ = declare_parameter<std::string>("port", "/dev/HFRobotRTK");
    baud_ = declare_parameter<int>("baudrate", 115200);
    frame_ = declare_parameter<std::string>("frame_id", "gps");
    heading_frame_ = declare_parameter<std::string>("heading_frame_id", "map");
    log_dir_ = declare_parameter<std::string>("raw_log_dir", "");
    publisher_ = std::make_unique<rtk::RosPublisher>(*this, frame_, heading_frame_);
    capture_ = std::make_unique<rtk::SerialCapture>(port_, baud_, log_dir_,
      [this](const std::string& line) { publisher_->process(line); });
  }
  bool run() {
    capture_->start();
    while (rclcpp::ok() && !capture_->fatal()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    capture_->stop();
    std::cerr << "raw ROS lines dropped locally: " << capture_->dropped_ros_lines() << std::endl;
    return !capture_->fatal();
  }
private:
  std::string port_, frame_, heading_frame_, log_dir_;
  int baud_ = 115200;
  std::unique_ptr<rtk::RosPublisher> publisher_;
  std::unique_ptr<rtk::SerialCapture> capture_;
};
#else
class RtkNode {
public:
  RtkNode() : private_node_("~") {
    private_node_.param<std::string>("port", port_, "/dev/HFRobotRTK");
    private_node_.param("baudrate", baud_, 115200);
    private_node_.param<std::string>("frame_id", frame_, "gps");
    private_node_.param<std::string>("heading_frame_id", heading_frame_, "map");
    private_node_.param<std::string>("raw_log_dir", log_dir_, "");
    publisher_ = std::make_unique<rtk::RosPublisher>(node_, frame_, heading_frame_);
    capture_ = std::make_unique<rtk::SerialCapture>(port_, baud_, log_dir_,
      [this](const std::string& line) { publisher_->process(line); });
  }
  bool run() {
    capture_->start();
    while (ros::ok() && !capture_->fatal()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    capture_->stop();
    std::cerr << "raw ROS lines dropped locally: " << capture_->dropped_ros_lines() << std::endl;
    return !capture_->fatal();
  }
private:
  ros::NodeHandle node_, private_node_;
  std::string port_, frame_, heading_frame_, log_dir_;
  int baud_ = 115200;
  std::unique_ptr<rtk::RosPublisher> publisher_;
  std::unique_ptr<rtk::SerialCapture> capture_;
};
#endif

int main(int argc, char** argv) {
#if defined(USE_ROS2)
  rclcpp::init(argc, argv);
#else
  ros::init(argc, argv, "handsfree_rtk");
#endif
  int result = 0;
  try {
#if defined(USE_ROS2)
    auto node = std::make_shared<RtkNode>();
    result = node->run() ? 0 : 1;
#else
    RtkNode node;
    result = node.run() ? 0 : 1;
#endif
  } catch (const std::exception& e) {
    std::cerr << "FATAL: " << e.what() << std::endl;
    result = 1;
  }
#if defined(USE_ROS2)
  rclcpp::shutdown();
#endif
  return result;
}
