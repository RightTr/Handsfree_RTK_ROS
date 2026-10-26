// ROS1/ROS2 GNSS publisher with an independent NTRIP correction thread.
#include "rtk_stream.hpp"

namespace {
std::string base64(const std::string& input) {
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  for (std::size_t i = 0; i < input.size(); i += 3) {
    const uint32_t value = (static_cast<unsigned char>(input[i]) << 16) |
      (i + 1 < input.size() ? static_cast<unsigned char>(input[i + 1]) << 8 : 0) |
      (i + 2 < input.size() ? static_cast<unsigned char>(input[i + 2]) : 0);
    result += alphabet[(value >> 18) & 63];
    result += alphabet[(value >> 12) & 63];
    result += i + 1 < input.size() ? alphabet[(value >> 6) & 63] : '=';
    result += i + 2 < input.size() ? alphabet[value & 63] : '=';
  }
  return result;
}
}

#if defined(USE_ROS2)
class RtkNtripNode : public rclcpp::Node {
#else
class RtkNtripNode {
#endif
public:
#if defined(USE_ROS2)
  RtkNtripNode() : rclcpp::Node("handsfree_rtk_ntrip") {
    port_ = declare_parameter<std::string>("port", "/dev/HFRobotRTK");
    baud_ = declare_parameter<int>("baudrate", 115200);
    frame_ = declare_parameter<std::string>("frame_id", "gps");
    heading_frame_ = declare_parameter<std::string>("heading_frame_id", "map");
    log_dir_ = declare_parameter<std::string>("raw_log_dir", "");
    server_ = declare_parameter<std::string>("ntrip_server", "120.253.239.161");
    server_port_ = declare_parameter<int>("ntrip_port", 8002);
    username_ = declare_parameter<std::string>("ntrip_username", "ctea952");
    password_ = declare_parameter<std::string>("ntrip_password", "cm286070");
    mountpoint_ = declare_parameter<std::string>("ntrip_mountpoint", "RTCM33_GRCE");
    gga_period_ = declare_parameter<double>("gga_period", 3.0);
    publisher_ = std::make_unique<rtk::RosPublisher>(*this, frame_, heading_frame_);
#else
  RtkNtripNode() : private_node_("~") {
    private_node_.param<std::string>("port", port_, "/dev/HFRobotRTK");
    private_node_.param("baudrate", baud_, 115200);
    private_node_.param<std::string>("frame_id", frame_, "gps");
    private_node_.param<std::string>("heading_frame_id", heading_frame_, "map");
    private_node_.param<std::string>("raw_log_dir", log_dir_, "");
    private_node_.param<std::string>("ntrip_server", server_, "120.253.239.161");
    private_node_.param("ntrip_port", server_port_, 8002);
    private_node_.param<std::string>("ntrip_username", username_, "ctea952");
    private_node_.param<std::string>("ntrip_password", password_, "cm286070");
    private_node_.param<std::string>("ntrip_mountpoint", mountpoint_, "RTCM33_GRCE");
    private_node_.param("gga_period", gga_period_, 3.0);
    publisher_ = std::make_unique<rtk::RosPublisher>(node_, frame_, heading_frame_);
#endif
    publisher_->set_valid_gga_callback([this](const std::string& gga) {
      std::lock_guard<std::mutex> lock(gga_mu_);
      latest_gga_ = gga;
    });
    capture_ = std::make_unique<rtk::SerialCapture>(port_, baud_, log_dir_,
      [this](const std::string& line) { publisher_->process(line); });
  }

  bool run() {
    capture_->start();
    network_thread_ = std::thread([this] { network_loop(); });
    while (ok() && !capture_->fatal()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stop_ = true;
    capture_->stop();
    if (network_thread_.joinable()) network_thread_.join();
    std::cerr << "raw ROS lines dropped locally: " << capture_->dropped_ros_lines() << std::endl;
    return !capture_->fatal();
  }

private:
  bool ok() const {
#if defined(USE_ROS2)
    return rclcpp::ok() && !stop_;
#else
    return ros::ok() && !stop_;
#endif
  }

  void network_loop() {
    using namespace std::chrono_literals;
    while (ok() && !capture_->fatal()) {
      try {
        boost::asio::io_service io;
        boost::asio::ip::tcp::resolver resolver(io);
        boost::asio::ip::tcp::socket socket(io);
        const auto endpoints = resolver.resolve(server_, std::to_string(server_port_));
        boost::asio::connect(socket, endpoints);
        const std::string request = "GET /" + mountpoint_ + " HTTP/1.0\r\n"
          "User-Agent: NTRIP handsfree_rtk\r\nAuthorization: Basic " + base64(username_ + ":" + password_) + "\r\n\r\n";
        boost::asio::write(socket, boost::asio::buffer(request));
        boost::asio::streambuf response;
        boost::asio::read_until(socket, response, "\r\n\r\n");
        std::istream in(&response);
        std::string status;
        std::getline(in, status);
        if (status.find("200") == std::string::npos) throw std::runtime_error("NTRIP rejected: " + status);
        // read_until may also have received the first RTCM bytes.
        std::string header_line;
        while (std::getline(in, header_line) && header_line != "\r") {}
        while (response.size()) {
          char chunk[4096];
          const auto n = response.sgetn(chunk, std::min<std::size_t>(response.size(), sizeof(chunk)));
          if (n) boost::asio::write(capture_->serial(), boost::asio::buffer(chunk, n));
        }
        socket.non_blocking(true);
        auto next_gga = std::chrono::steady_clock::now();
        while (ok() && !capture_->fatal() && capture_->serial().is_open()) {
          if (std::chrono::steady_clock::now() >= next_gga) {
            std::string gga;
            { std::lock_guard<std::mutex> lock(gga_mu_); gga = latest_gga_; }
            if (!gga.empty()) boost::asio::write(socket, boost::asio::buffer(gga + "\r\n"));
            next_gga = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(gga_period_ * 1000));
          }
          char chunk[4096]; boost::system::error_code ec;
          const auto n = socket.read_some(boost::asio::buffer(chunk), ec);
          if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) { std::this_thread::sleep_for(10ms); continue; }
          if (ec) throw boost::system::system_error(ec);
          if (n) boost::asio::write(capture_->serial(), boost::asio::buffer(chunk, n));
        }
      } catch (const std::exception& e) {
        if (ok() && !capture_->fatal()) {
          std::cerr << "NTRIP: " << e.what() << "; retrying" << std::endl;
          std::this_thread::sleep_for(1s);
        }
      }
    }
  }

#if !defined(USE_ROS2)
  ros::NodeHandle node_, private_node_;
#endif
  std::string port_, frame_, heading_frame_, log_dir_;
  std::string server_, username_, password_, mountpoint_;
  int baud_ = 115200, server_port_ = 8002;
  double gga_period_ = 3.0;
  std::unique_ptr<rtk::RosPublisher> publisher_;
  std::unique_ptr<rtk::SerialCapture> capture_;
  std::mutex gga_mu_;
  std::string latest_gga_;
  std::thread network_thread_;
  std::atomic<bool> stop_{false};
};

int main(int argc, char** argv) {
#if defined(USE_ROS2)
  rclcpp::init(argc, argv);
#else
  ros::init(argc, argv, "handsfree_rtk_ntrip");
#endif
  int result = 0;
  try {
#if defined(USE_ROS2)
    auto node = std::make_shared<RtkNtripNode>();
    result = node->run() ? 0 : 1;
#else
    RtkNtripNode node;
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
