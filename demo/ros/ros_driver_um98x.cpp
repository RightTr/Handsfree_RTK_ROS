// ROS1/ROS2 compatible C++ GNSS serial node.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <ctime>
#include <vector>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#if defined(_WIN32)
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#if defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
using MsgString = std_msgs::msg::String;
using MsgFloat64 = std_msgs::msg::Float64;
using MsgHeading = nav_msgs::msg::Odometry;
using MsgNavSatFix = sensor_msgs::msg::NavSatFix;
using MsgNavSatStatus = sensor_msgs::msg::NavSatStatus;
using RosNodeBase = rclcpp::Node;
#else
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
using MsgString = std_msgs::String;
using MsgFloat64 = std_msgs::Float64;
using MsgHeading = nav_msgs::Odometry;
using MsgNavSatFix = sensor_msgs::NavSatFix;
using MsgNavSatStatus = sensor_msgs::NavSatStatus;
#endif

namespace asio = boost::asio;
using boost::asio::ip::tcp;

namespace {

struct UtcDate {
  int year = 1970;
  int month = 1;
  int day = 1;
  bool valid = false;
};

struct UtcTimeOfDay {
  int hour = 0;
  int minute = 0;
  int second = 0;
  uint32_t nanosecond = 0;
  bool valid = false;
};

static std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

static bool checksum_ok(const std::string& sentence) {
  auto star = sentence.find('*');
  if (star == std::string::npos || star + 2 >= sentence.size()) {
    return true;
  }
  unsigned char sum = 0;
  for (std::size_t i = 1; i < star; ++i) {
    sum ^= static_cast<unsigned char>(sentence[i]);
  }
  char buf[3];
  std::snprintf(buf, sizeof(buf), "%02X", sum);
  std::string got = sentence.substr(star + 1, 2);
  std::transform(got.begin(), got.end(), got.begin(), ::toupper);
  return got == buf;
}

static double to_double(const std::string& s, double fallback) {
  try {
    return s.empty() ? fallback : std::stod(s);
  } catch (...) {
    return fallback;
  }
}

static int to_int(const std::string& s, int fallback) {
  try {
    return s.empty() ? fallback : std::stoi(s);
  } catch (...) {
    return fallback;
  }
}

static int64_t days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + static_cast<int64_t>(doe) - 719468LL;
}

static UtcDate current_utc_date() {
  std::time_t now = std::time(nullptr);
  std::tm tm_utc;
#if defined(_WIN32)
  gmtime_s(&tm_utc, &now);
#else
  gmtime_r(&now, &tm_utc);
#endif
  UtcDate date;
  date.year = tm_utc.tm_year + 1900;
  date.month = tm_utc.tm_mon + 1;
  date.day = tm_utc.tm_mday;
  date.valid = true;
  return date;
}

static UtcTimeOfDay parse_nmea_utc_time(const std::string& hhmmss) {
  UtcTimeOfDay out;
  try {
    if (hhmmss.size() < 6) {
      return out;
    }
    out.hour = std::stoi(hhmmss.substr(0, 2));
    out.minute = std::stoi(hhmmss.substr(2, 2));
    const double seconds = std::stod(hhmmss.substr(4));
    out.second = static_cast<int>(seconds);
    out.nanosecond = static_cast<uint32_t>((seconds - out.second) * 1000000000.0);
    out.valid = out.hour >= 0 && out.hour < 24 &&
                out.minute >= 0 && out.minute < 60 &&
                out.second >= 0 && out.second < 60;
  } catch (...) {
    out.valid = false;
  }
  return out;
}

static UtcDate parse_nmea_date(const std::string& ddmmyy) {
  UtcDate out;
  try {
    if (ddmmyy.size() != 6) {
      return out;
    }
    out.day = std::stoi(ddmmyy.substr(0, 2));
    out.month = std::stoi(ddmmyy.substr(2, 2));
    int yy = std::stoi(ddmmyy.substr(4, 2));
    out.year = yy >= 80 ? 1900 + yy : 2000 + yy;
    out.valid = out.month >= 1 && out.month <= 12 && out.day >= 1 && out.day <= 31;
  } catch (...) {
    out.valid = false;
  }
  return out;
}

static int64_t utc_epoch_seconds(const UtcDate& date, const UtcTimeOfDay& time) {
  return days_from_civil(date.year, static_cast<unsigned>(date.month), static_cast<unsigned>(date.day)) * 86400LL +
         time.hour * 3600 + time.minute * 60 + time.second;
}

static double ddmm_to_deg(const std::string& value, const std::string& dir, bool is_lat) {
  try {
    if (value.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t deg_width = is_lat ? 2 : 3;
    if (value.size() <= deg_width) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double deg = std::stod(value.substr(0, deg_width));
    double minutes = std::stod(value.substr(deg_width));
    double dec = deg + minutes / 60.0;
    if (!dir.empty() && (dir[0] == 'S' || dir[0] == 'W')) {
      dec = -dec;
    }
    return dec;
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

struct GgaData {
  bool valid = false;
  UtcTimeOfDay utc_time;
  int fix_quality = 0;
  int num_sats = 0;
  double hdop = 99.9;
  double lat = std::numeric_limits<double>::quiet_NaN();
  double lon = std::numeric_limits<double>::quiet_NaN();
  double alt = 0.0;
};

struct RmcData {
  bool valid = false;
  bool status_valid = false;
  UtcDate utc_date;
  double speed_knots = 0.0;
  double speed_mps = 0.0;
  double cog_deg = 0.0;
};

struct ThsData {
  bool valid = false;
  double heading_deg = 0.0;
};

static bool parse_gga(const std::string& line, GgaData& out) {
  if (!(line.rfind("$GNGGA", 0) == 0 || line.rfind("$GPGGA", 0) == 0)) {
    return false;
  }
  if (!checksum_ok(line)) {
    return false;
  }
  auto fields = split_csv(line);
  if (fields.size() < 10) {
    return false;
  }
  out.utc_time = parse_nmea_utc_time(fields[1]);
  out.fix_quality = to_int(fields[6], 0);
  out.num_sats = to_int(fields[7], 0);
  out.hdop = to_double(fields[8], 99.9);
  out.alt = to_double(fields[9], 0.0);
  out.lat = ddmm_to_deg(fields[2], fields[3], true);
  out.lon = ddmm_to_deg(fields[4], fields[5], false);
  out.valid = std::isfinite(out.lat) && std::isfinite(out.lon);
  return true;
}

static bool parse_rmc(const std::string& line, RmcData& out) {
  if (!(line.rfind("$GNRMC", 0) == 0 || line.rfind("$GPRMC", 0) == 0)) {
    return false;
  }
  if (!checksum_ok(line)) {
    return false;
  }
  auto fields = split_csv(line);
  if (fields.size() < 12) {
    return false;
  }
  char status = fields[2].empty() ? 'V' : fields[2][0];
  out.status_valid = (status == 'A');
  out.utc_date = parse_nmea_date(fields[9]);
  out.speed_knots = to_double(fields[7], 0.0);
  out.speed_mps = out.speed_knots * 0.514444;
  out.cog_deg = to_double(fields[8], 0.0);
  out.valid = true;
  return true;
}

static bool parse_ths(const std::string& line, ThsData& out) {
  if (!(line.rfind("$GNTHS", 0) == 0 || line.rfind("$GPTHS", 0) == 0)) {
    return false;
  }
  if (!checksum_ok(line)) {
    return false;
  }
  auto fields = split_csv(line);
  if (fields.size() < 3) {
    return false;
  }
  out.heading_deg = to_double(fields[1], 0.0);
  out.valid = !fields[2].empty() && fields[2][0] == 'A';
  return true;
}

}  // namespace

class RosNmeaNode
#if defined(USE_ROS2)
  : public RosNodeBase
#endif
{
public:
#if defined(USE_ROS2)
  RosNmeaNode()
  : RosNodeBase("handsfree_rtk"), serial_(io_) {
    this->declare_parameter<std::string>("port", "/dev/HFRobotRTK");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<double>("timeout", 0.1);
    this->declare_parameter<std::string>("frame_id", "gps");
    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    timeout_sec_ = this->get_parameter("timeout").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    setup_publishers();
  }
#else
  RosNmeaNode()
  : nh_(), pnh_("~"), serial_(io_) {
    pnh_.param<std::string>("port", port_, "/dev/HFRobotRTK");
    pnh_.param("baudrate", baudrate_, 115200);
    pnh_.param("timeout", timeout_sec_, 0.1);
    pnh_.param<std::string>("frame_id", frame_id_, "gps");
    setup_publishers();
  }
#endif

  void run() {
    while (ros_ok()) {
      if (!open_serial()) {
        sleep_ms(1000);
        continue;
      }

      std::cout << "Opened serial: " << port_ << " @ " << baudrate_ << std::endl;
      while (ros_ok()) {
        std::string line;
        if (!read_line(line)) {
          if (!ros_ok()) {
            break;
          }
          continue;
        }
        process_line(line);
      }

      close_serial();
      if (ros_ok()) {
        sleep_ms(1000);
      }
    }
    close_serial();
  }

private:
  bool ros_ok() const {
#if defined(USE_ROS2)
    return rclcpp::ok();
#else
    return ros::ok();
#endif
  }

  void sleep_ms(int ms) const {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  void setup_publishers() {
#if defined(USE_ROS2)
    raw_pub_ = this->create_publisher<MsgString>("handsfree/rtk/raw", 10);
    gnss_pub_ = this->create_publisher<MsgNavSatFix>("handsfree/rtk/gnss", 10);
    speed_pub_ = this->create_publisher<MsgFloat64>("handsfree/rtk/speed", 10);
    cog_pub_ = this->create_publisher<MsgFloat64>("handsfree/rtk/cog", 10);
    heading_pub_ = this->create_publisher<MsgHeading>("handsfree/rtk/heading", 10);
#else
    raw_pub_ = nh_.advertise<MsgString>("handsfree/rtk/raw", 10);
    gnss_pub_ = nh_.advertise<MsgNavSatFix>("handsfree/rtk/gnss", 10);
    speed_pub_ = nh_.advertise<MsgFloat64>("handsfree/rtk/speed", 10);
    cog_pub_ = nh_.advertise<MsgFloat64>("handsfree/rtk/cog", 10);
    heading_pub_ = nh_.advertise<MsgHeading>("handsfree/rtk/heading", 10);
#endif
  }

  bool open_serial() {
    try {
      if (serial_.is_open()) {
        return true;
      }
      serial_.open(port_);
      serial_.set_option(asio::serial_port_base::baud_rate(baudrate_));
      serial_.set_option(asio::serial_port_base::character_size(8));
      serial_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
      serial_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
      serial_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
      make_nonblocking();
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Failed to open serial: " << e.what() << std::endl;
      close_serial();
      return false;
    }
  }

  void make_nonblocking() {
#if !defined(_WIN32)
    int fd = serial_.native_handle();
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
#endif
  }

  void close_serial() {
    try {
      if (serial_.is_open()) {
        serial_.close();
      }
    } catch (...) {
    }
  }

  bool read_line(std::string& out) {
    out.clear();
    auto start = std::chrono::steady_clock::now();
    char c = 0;
    while (ros_ok()) {
      boost::system::error_code ec;
      std::size_t n = serial_.read_some(asio::buffer(&c, 1), ec);
      if (ec) {
        if (ec == asio::error::would_block || ec == asio::error::try_again) {
          sleep_ms(2);
        } else {
          return false;
        }
      } else if (n == 1) {
        if (c == '\r' || c == '\n') {
          if (!out.empty()) {
            return true;
          }
        } else {
          out.push_back(c);
        }
      }

      if (timeout_sec_ > 0.0) {
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec_) {
          return false;
        }
      }
    }
    return false;
  }

  void publish_raw(const std::string& line) {
    MsgString msg;
    msg.data = line;
#if defined(USE_ROS2)
    raw_pub_->publish(msg);
#else
    raw_pub_.publish(msg);
#endif
  }

  void publish_float(const std::string& topic, double value) {
    (void)topic;
    MsgFloat64 msg;
    msg.data = value;
#if defined(USE_ROS2)
    if (topic == "speed") {
      speed_pub_->publish(msg);
    } else if (topic == "cog") {
      cog_pub_->publish(msg);
    }
#else
    if (topic == "speed") {
      speed_pub_.publish(msg);
    } else if (topic == "cog") {
      cog_pub_.publish(msg);
    }
#endif
  }

  void publish_heading(double heading_deg, const UtcTimeOfDay& utc_time) {
    if (!utc_time.valid) {
      return;
    }
    MsgHeading msg;
    const UtcDate stamp_date = latest_utc_date_.valid ? latest_utc_date_ : current_utc_date();
    const int64_t stamp_sec = utc_epoch_seconds(stamp_date, utc_time);
#if defined(USE_ROS2)
    msg.header.stamp.sec = static_cast<int32_t>(stamp_sec);
    msg.header.stamp.nanosec = utc_time.nanosecond;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = "heading";
    const double heading_rad = heading_deg * M_PI / 180.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(heading_rad * 0.5);
    msg.pose.pose.orientation.w = std::cos(heading_rad * 0.5);
    heading_pub_->publish(msg);
#else
    msg.header.stamp = ros::Time(static_cast<uint32_t>(stamp_sec), utc_time.nanosecond);
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = "heading";
    const double heading_rad = heading_deg * M_PI / 180.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(heading_rad * 0.5);
    msg.pose.pose.orientation.w = std::cos(heading_rad * 0.5);
    heading_pub_.publish(msg);
#endif
  }

  void publish_navsat(double lat, double lon, double alt, int fix_q, const UtcTimeOfDay& utc_time) {
    MsgNavSatFix msg;
    UtcDate stamp_date = latest_utc_date_.valid ? latest_utc_date_ : current_utc_date();
    const int64_t stamp_sec = utc_time.valid ? utc_epoch_seconds(stamp_date, utc_time) : 0;
#if defined(USE_ROS2)
    if (utc_time.valid) {
      msg.header.stamp.sec = static_cast<int32_t>(stamp_sec);
      msg.header.stamp.nanosec = utc_time.nanosecond;
    } else {
      auto now = this->now();
      msg.header.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
      msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    }
#else
    if (utc_time.valid) {
      msg.header.stamp = ros::Time(static_cast<uint32_t>(stamp_sec), utc_time.nanosecond);
    } else {
      msg.header.stamp = ros::Time::now();
    }
#endif
    msg.header.frame_id = frame_id_;
    msg.status.status = nav_status_from_fix(fix_q);
    msg.status.service = MsgNavSatStatus::SERVICE_GPS;
    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = alt;
    const double variance = covariance_from_fix(fix_q);
    msg.position_covariance = {
      variance, 0.0, 0.0,
      0.0, variance, 0.0,
      0.0, 0.0, variance
    };
    msg.position_covariance_type = MsgNavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#if defined(USE_ROS2)
    gnss_pub_->publish(msg);
#else
    gnss_pub_.publish(msg);
#endif
  }

  int nav_status_from_fix(int fix_q) const {
    switch (fix_q) {
      case 1: return MsgNavSatStatus::STATUS_FIX;
      case 2: return MsgNavSatStatus::STATUS_SBAS_FIX;
      case 4: return MsgNavSatStatus::STATUS_GBAS_FIX;
      case 5: return MsgNavSatStatus::STATUS_GBAS_FIX;
      case 6: return MsgNavSatStatus::STATUS_FIX;
      case 7: return MsgNavSatStatus::STATUS_GBAS_FIX;
      case 8: return MsgNavSatStatus::STATUS_FIX;
      default: return MsgNavSatStatus::STATUS_NO_FIX;
    }
  }

  double covariance_from_fix(int fix_q) const {
    switch (fix_q) {
      case 1: return 1.0;
      case 2: return 0.5;
      case 4: return 0.01;
      case 5: return 0.1;
      case 6: return 5.0;
      case 7: return 0.01;
      default: return 10000.0;
    }
  }

  void process_line(const std::string& line) {
    publish_raw(line);

    GgaData gga;
    if (parse_gga(line, gga) && gga.valid) {
      if (gga.utc_time.valid) {
        heading_utc_time_ = gga.utc_time;
        heading_utc_time_valid_ = true;
      }
      publish_navsat(gga.lat, gga.lon, gga.alt, gga.fix_quality, gga.utc_time);
      return;
    }

    RmcData rmc;
    if (parse_rmc(line, rmc) && rmc.valid) {
      if (rmc.utc_date.valid) {
        latest_utc_date_ = rmc.utc_date;
      }
      if (rmc.status_valid) {
        publish_float("speed", rmc.speed_mps);
        publish_float("cog", rmc.cog_deg);
      }
      return;
    }

    ThsData ths;
    if (parse_ths(line, ths) && ths.valid) {
      if (!heading_utc_time_valid_) {
        return;
      }
      publish_heading(ths.heading_deg, heading_utc_time_);
      heading_utc_time_valid_ = false;
    }
  }

private:
#if defined(USE_ROS2)
  std::shared_ptr<rclcpp::Publisher<MsgString>> raw_pub_;
  std::shared_ptr<rclcpp::Publisher<MsgNavSatFix>> gnss_pub_;
  std::shared_ptr<rclcpp::Publisher<MsgFloat64>> speed_pub_;
  std::shared_ptr<rclcpp::Publisher<MsgFloat64>> cog_pub_;
  std::shared_ptr<rclcpp::Publisher<MsgHeading>> heading_pub_;
#else
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher raw_pub_;
  ros::Publisher gnss_pub_;
  ros::Publisher speed_pub_;
  ros::Publisher cog_pub_;
  ros::Publisher heading_pub_;
#endif

  asio::io_service io_;
  asio::serial_port serial_;

  std::string port_;
  int baudrate_ = 115200;
  double timeout_sec_ = 0.1;
  std::string frame_id_ = "gps";
  UtcDate latest_utc_date_;
  UtcTimeOfDay heading_utc_time_;
  bool heading_utc_time_valid_ = false;
};

int main(int argc, char** argv) {
#if defined(USE_ROS2)
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosNmeaNode>();
  node->run();
  rclcpp::shutdown();
#else
  ros::init(argc, argv, "handsfree_rtk");
  RosNmeaNode node;
  node.run();
#endif
  return 0;
}
