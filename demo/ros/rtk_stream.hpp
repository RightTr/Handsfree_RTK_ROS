#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <stdexcept>
#if !defined(_WIN32)
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#endif

#include <boost/asio.hpp>

#if defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
using RtkString = std_msgs::msg::String;
using RtkBool = std_msgs::msg::Bool;
using RtkFix = sensor_msgs::msg::NavSatFix;
using RtkStatus = sensor_msgs::msg::NavSatStatus;
using RtkHeading = nav_msgs::msg::Odometry;
#else
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
using RtkString = std_msgs::String;
using RtkBool = std_msgs::Bool;
using RtkFix = sensor_msgs::NavSatFix;
using RtkStatus = sensor_msgs::NavSatStatus;
using RtkHeading = nav_msgs::Odometry;
#endif

namespace rtk {

inline std::vector<std::string> fields(const std::string& s) {
  std::vector<std::string> out;
  std::size_t begin = 0;
  for (std::size_t i = 0; i <= s.size(); ++i) {
    if (i == s.size() || s[i] == ',') {
      out.emplace_back(s.substr(begin, i - begin));
      begin = i + 1;
    }
  }
  return out;
}

inline bool nmea_checksum_ok(const std::string& line) {
  const auto star = line.find('*');
  if (line.empty() || line[0] != '$' || star == std::string::npos || star + 2 >= line.size()) return false;
  unsigned int checksum = 0;
  for (std::size_t i = 1; i < star; ++i) checksum ^= static_cast<unsigned char>(line[i]);
  unsigned int received = 0;
  try { received = std::stoul(line.substr(star + 1, 2), nullptr, 16); }
  catch (...) { return false; }
  return checksum == received;
}

inline bool number(const std::string& text, double& value) {
  try {
    std::size_t used = 0;
    value = std::stod(text, &used);
    return used == text.size() && std::isfinite(value);
  } catch (...) { return false; }
}

inline bool integer(const std::string& text, int64_t& value) {
  try {
    std::size_t used = 0;
    value = std::stoll(text, &used);
    return used == text.size();
  } catch (...) { return false; }
}

inline int64_t days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  return era * 146097LL + static_cast<int64_t>(yoe) * 365 + yoe / 4 - yoe / 100 + doy - 719468;
}

inline bool valid_civil_date(int y, int m, int d) {
  if (m < 1 || m > 12 || d < 1) return false;
  const bool leap = y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
  const int days[] = {31, leap ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  return d <= days[m - 1];
}

inline bool time_of_day(const std::string& s, int64_t& whole, uint32_t& nanosecond) {
  if (s.size() < 6) return false;
  int64_t h, m;
  double seconds;
  if (!integer(s.substr(0, 2), h) || !integer(s.substr(2, 2), m) || !number(s.substr(4), seconds) ||
      h > 23 || m > 59 || seconds < 0 || seconds >= 60) return false;
  const auto sec = static_cast<int64_t>(seconds);
  whole = h * 3600 + m * 60 + sec;
  nanosecond = static_cast<uint32_t>(std::llround((seconds - sec) * 1e9));
  if (nanosecond == 1000000000U) { ++whole; nanosecond = 0; }
  return true;
}

inline bool coordinate(const std::string& s, const std::string& hemisphere, bool latitude, double& out) {
  const std::size_t width = latitude ? 2 : 3;
  double deg, minutes;
  if (s.size() <= width || !number(s.substr(0, width), deg) || !number(s.substr(width), minutes) ||
      minutes < 0 || minutes >= 60 || deg > (latitude ? 90 : 180)) return false;
  if (hemisphere != (latitude ? "N" : "E") && hemisphere != (latitude ? "S" : "W")) return false;
  out = deg + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") out = -out;
  return true;
}

class RosPublisher {
public:
#if defined(USE_ROS2)
  RosPublisher(rclcpp::Node& node, std::string fix_frame, std::string heading_frame)
      : fix_frame_(std::move(fix_frame)), heading_frame_(std::move(heading_frame)) {
    raw2_ = node.create_publisher<RtkString>("handsfree/rtk/raw", rclcpp::QoS(1000).reliable());
    fix2_ = node.create_publisher<RtkFix>("handsfree/rtk/gnss", 100);
    heading2_ = node.create_publisher<RtkHeading>("handsfree/rtk/heading", 100);
    valid2_ = node.create_publisher<RtkBool>("handsfree/rtk/heading_valid", 100);
  }
#else
  RosPublisher(ros::NodeHandle& node, std::string fix_frame, std::string heading_frame)
      : fix_frame_(std::move(fix_frame)), heading_frame_(std::move(heading_frame)) {
    raw1_ = node.advertise<RtkString>("handsfree/rtk/raw", 1000);
    fix1_ = node.advertise<RtkFix>("handsfree/rtk/gnss", 100);
    heading1_ = node.advertise<RtkHeading>("handsfree/rtk/heading", 100);
    valid1_ = node.advertise<RtkBool>("handsfree/rtk/heading_valid", 100);
  }
#endif

  void set_valid_gga_callback(std::function<void(const std::string&)> cb) { on_valid_gga_ = std::move(cb); }

  void process(const std::string& line) {
    if (line.rfind("#UNIHEADINGA,", 0) == 0) process_uniheading(line);
    else if (nmea_checksum_ok(line)) {
      if (line.rfind("$GNGGA,", 0) == 0 || line.rfind("$GPGGA,", 0) == 0) process_gga(line);
      else if (line.rfind("$GNRMC,", 0) == 0 || line.rfind("$GPRMC,", 0) == 0) process_rmc(line);
      else if (line.rfind("$GNZDA,", 0) == 0 || line.rfind("$GPZDA,", 0) == 0) process_zda(line);
    } else if (line.rfind("$GNRMC,", 0) == 0 || line.rfind("$GPRMC,", 0) == 0) rmc_streak_ = 0;
    RtkString raw;
    raw.data = line;
#if defined(USE_ROS2)
    raw2_->publish(raw);
#else
    raw1_.publish(raw);
#endif
  }

private:
  void set_stamp(RtkFix& msg, int64_t sec, uint32_t ns) {
#if defined(USE_ROS2)
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = ns;
#else
    msg.header.stamp = ros::Time(static_cast<uint32_t>(sec), ns);
#endif
  }
  void set_stamp(RtkHeading& msg, int64_t sec, uint32_t ns) {
#if defined(USE_ROS2)
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = ns;
#else
    msg.header.stamp = ros::Time(static_cast<uint32_t>(sec), ns);
#endif
  }

  void process_rmc(const std::string& line) {
    const auto f = fields(line);
    if (f.size() < 10 || f[9].size() != 6 || f[2] != "A") { rmc_streak_ = 0; return; }
    int64_t d, m, y, tod; uint32_t ns;
    if (!integer(f[9].substr(0, 2), d) || !integer(f[9].substr(2, 2), m) ||
        !integer(f[9].substr(4, 2), y) || !time_of_day(f[1], tod, ns) ||
        !valid_civil_date(y >= 80 ? 1900 + y : 2000 + y, m, d)) { rmc_streak_ = 0; return; }
    const int64_t utc_sec = days_from_civil(y >= 80 ? 1900 + y : 2000 + y, m, d) * 86400 + tod;
    const int64_t stamp_ns = utc_sec * 1000000000LL + ns;
    if (rmc_streak_ > 0 && stamp_ns > last_rmc_ns_ && stamp_ns - last_rmc_ns_ <= 10000000000LL)
      rmc_streak_ = std::min(rmc_streak_ + 1, 3);
    else rmc_streak_ = 1;
    last_rmc_ns_ = stamp_ns;
    if (rmc_streak_ == 3) reference_utc_ = utc_sec;
  }

  void process_zda(const std::string& line) {
    const auto f = fields(line);
    if (f.size() < 5) return;
    int64_t d, m, y, tod; uint32_t ns;
    if (!integer(f[2], d) || !integer(f[3], m) || !integer(f[4], y) ||
        !time_of_day(f[1], tod, ns) || !valid_civil_date(y, m, d)) return;
    reference_utc_ = days_from_civil(y, m, d) * 86400 + tod;
  }

  int64_t closest_utc_day(int64_t tod) const {
    const int64_t day = reference_utc_ / 86400;
    int64_t best = day * 86400 + tod;
    for (int offset : {-1, 1}) {
      const int64_t candidate = (day + offset) * 86400 + tod;
      if (std::llabs(candidate - reference_utc_) < std::llabs(best - reference_utc_)) best = candidate;
    }
    return best;
  }

  void process_gga(const std::string& line) {
    const auto f = fields(line);
    int64_t tod, quality = 0; uint32_t ns;
    if (f.size() < 10 || !time_of_day(f[1], tod, ns) || !integer(f[6], quality)) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double lat = nan, lon = nan, msl = nan, geoid = nan;
    const bool coords = coordinate(f[2], f[3], true, lat) && coordinate(f[4], f[5], false, lon);
    const bool good = quality > 0 && coords;
    if (!good) { lat = nan; lon = nan; }
    if (reference_utc_ == 0) {
      // Keep feeding NTRIP, but do not publish an undated GNSS fix.
      if (good && on_valid_gga_) on_valid_gga_(line);
      return;
    }
    if (f.size() > 11) { number(f[9], msl); number(f[11], geoid); }
    RtkFix msg;
    const int64_t utc_sec = closest_utc_day(tod);
    if (utc_sec > reference_utc_) reference_utc_ = utc_sec;
    set_stamp(msg, utc_sec, ns);
    msg.header.frame_id = fix_frame_;
    msg.status.status = !good ? RtkStatus::STATUS_NO_FIX :
      (quality == 4 || quality == 5 ? RtkStatus::STATUS_GBAS_FIX :
       quality == 2 ? RtkStatus::STATUS_SBAS_FIX : RtkStatus::STATUS_FIX);
    msg.status.service = RtkStatus::SERVICE_GPS;
    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = good ? msl + geoid : nan;
    const double variance = quality == 4 ? 0.01 : quality == 5 ? 0.1 : quality == 2 ? 0.5 : good ? 1.0 : 10000.0;
    msg.position_covariance = {variance, 0, 0, 0, variance, 0, 0, 0, variance};
    msg.position_covariance_type = RtkFix::COVARIANCE_TYPE_APPROXIMATED;
#if defined(USE_ROS2)
    fix2_->publish(msg);
#else
    fix1_.publish(msg);
#endif
    if (good && on_valid_gga_) on_valid_gga_(line);
  }

  void process_uniheading(const std::string& line) {
    const auto semi = line.find(';');
    const auto star = line.find('*');
    if (semi == std::string::npos || star == std::string::npos || star <= semi || star + 8 >= line.size()) return;
    uint32_t crc = 0;
    for (std::size_t i = 1; i < star; ++i) {
      crc ^= static_cast<unsigned char>(line[i]);
      for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0U);
    }
    uint32_t received = 0;
    try {
      if (line.substr(star + 1, 8).find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) return;
      received = static_cast<uint32_t>(std::stoul(line.substr(star + 1, 8), nullptr, 16));
    } catch (...) { return; }
    if (crc != received) return;
    const auto h = fields(line.substr(0, semi));
    const auto b = fields(line.substr(semi + 1, star - semi - 1));
    if (h.size() < 9 || b.size() < 4) return;
    int64_t week, tow_ms, leap;
    if (!integer(h[4], week) || !integer(h[5], tow_ms) || !integer(h[8], leap) ||
        week < 0 || tow_ms < 0 || tow_ms >= 604800000 || leap < 0 || leap > 60) return;
    const int64_t utc_sec = 315964800LL + week * 604800LL + tow_ms / 1000 - leap;
    const uint32_t ns = static_cast<uint32_t>(tow_ms % 1000) * 1000000U;
    reference_utc_ = utc_sec;
    double angle;
    const bool valid = b[0] == "SOL_COMPUTED" && number(b[3], angle) && angle >= 0 && angle <= 360;
    RtkBool status;
    status.data = valid;
#if defined(USE_ROS2)
    valid2_->publish(status);
#else
    valid1_.publish(status);
#endif
    if (!valid) return;
    RtkHeading msg;
    set_stamp(msg, utc_sec, ns);
    msg.header.frame_id = heading_frame_;
    msg.child_frame_id = "heading";
    const double yaw = (90.0 - angle) * 3.14159265358979323846 / 180.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2);
    msg.pose.pose.orientation.w = std::cos(yaw / 2);
#if defined(USE_ROS2)
    heading2_->publish(msg);
#else
    heading1_.publish(msg);
#endif
  }

  std::string fix_frame_, heading_frame_;
  int64_t reference_utc_ = 0;
  int64_t last_rmc_ns_ = 0;
  int rmc_streak_ = 0;
  std::function<void(const std::string&)> on_valid_gga_;
#if defined(USE_ROS2)
  rclcpp::Publisher<RtkString>::SharedPtr raw2_;
  rclcpp::Publisher<RtkFix>::SharedPtr fix2_;
  rclcpp::Publisher<RtkHeading>::SharedPtr heading2_;
  rclcpp::Publisher<RtkBool>::SharedPtr valid2_;
#else
  ros::Publisher raw1_, fix1_, heading1_, valid1_;
#endif
};

class SerialCapture {
public:
  SerialCapture(std::string port, int baud, std::string log_dir, std::function<void(const std::string&)> on_line)
      : port_name_(std::move(port)), baud_(baud), log_dir_(std::move(log_dir)), on_line_(std::move(on_line)), serial_(io_) {}

  ~SerialCapture() { stop(); }
  boost::asio::serial_port& serial() { return serial_; }
  bool running() const { return !stop_ && !fatal_; }
  bool fatal() const { return fatal_; }
  const std::string& file_path() const { return file_path_; }
  uint64_t dropped_ros_lines() const { return dropped_lines_; }

  void start() {
    namespace fs = std::filesystem;
    if (log_dir_.empty()) {
      const char* root = std::getenv("ROS_HOME");
      if (!root) root = std::getenv("HOME");
      if (!root) throw std::runtime_error("ROS_HOME or HOME is required for raw log path");
      log_dir_ = fs::path(root) / (std::getenv("ROS_HOME") ? "handsfree_rtk/raw" : ".ros/handsfree_rtk/raw");
    }
    fs::create_directories(log_dir_);
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    char name[80];
    std::strftime(name, sizeof(name), "raw_%Y%m%dT%H%M%SZ", &utc);
    file_path_ = (fs::path(log_dir_) / (std::string(name) + "_" + std::to_string(
#if defined(_WIN32)
      ++file_number_
#else
      ::getpid()
#endif
    ) + "_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".txt")).string();
    file_ = std::fopen(file_path_.c_str(), "wbx");
    if (!file_) throw std::runtime_error("cannot open raw file: " + file_path_);
    writer_ = std::thread([this] { write_loop(); });
    parser_ = std::thread([this] { parse_loop(); });
    reader_ = std::thread([this] { read_loop(); });
  }

  void stop() {
    if (joined_.exchange(true)) return;
    stop_ = true;
    file_cv_.notify_all(); line_cv_.notify_all();
    if (serial_.is_open()) {
      boost::system::error_code ec;
      serial_.cancel(ec);
      serial_.close(ec);
    }
    if (reader_.joinable()) reader_.join();
    if (parser_.joinable()) parser_.join();
    if (writer_.joinable()) writer_.join();
    if (file_) { std::fclose(file_); file_ = nullptr; }
  }

private:
  void read_loop() {
    using namespace std::chrono_literals;
    while (running()) {
      if (!serial_.is_open()) {
        boost::system::error_code ec;
        serial_.open(port_name_, ec);
        if (!ec) {
          serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_), ec);
          serial_.set_option(boost::asio::serial_port_base::character_size(8), ec);
          serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
          serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), ec);
          serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
#if !defined(_WIN32)
          const int fd = serial_.native_handle();
          const int flags = ::fcntl(fd, F_GETFL, 0);
          if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
            ec = boost::asio::error::operation_not_supported;
#endif
        }
        if (ec) { if (serial_.is_open()) serial_.close(); std::this_thread::sleep_for(1s); continue; }
        std::cout << "RTK serial open: " << port_name_ << " @ " << baud_ << "; raw file: " << file_path_ << std::endl;
      }
      char buf[4096];
#if !defined(_WIN32)
      const ssize_t n = ::read(serial_.native_handle(), buf, sizeof(buf));
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        std::this_thread::sleep_for(1ms); continue;
      }
      if (n < 0) {
        if (serial_.is_open()) { boost::system::error_code ec; serial_.close(ec); }
        pending_.clear(); std::this_thread::sleep_for(100ms); continue;
      }
#else
      boost::system::error_code ec;
      const auto n = serial_.read_some(boost::asio::buffer(buf), ec);
      if (ec) { pending_.clear(); std::this_thread::sleep_for(100ms); continue; }
#endif
      if (!n) {
        // A closed peer can report EOF continuously (not EAGAIN).
        if (serial_.is_open()) { boost::system::error_code ec; serial_.close(ec); }
        pending_.clear(); std::this_thread::sleep_for(100ms); continue;
      }
      {
        std::lock_guard<std::mutex> lock(file_mu_);
        if (file_bytes_ + n > 64 * 1024 * 1024) { fail("raw file queue overflow"); break; }
        file_queue_.emplace_back(buf, n); file_bytes_ += n;
      }
      file_cv_.notify_one();
      for (std::size_t i = 0; i < n; ++i) {
        const char c = buf[i];
        if (c == '\r' || c == '\n') {
          if (line_overflow_) { pending_.clear(); line_overflow_ = false; continue; }
          if (!pending_.empty()) {
            std::lock_guard<std::mutex> lock(line_mu_);
            if (line_bytes_ + pending_.size() <= 64 * 1024 * 1024) {
              line_bytes_ += pending_.size(); line_queue_.push_back(std::move(pending_)); line_cv_.notify_one();
            } else { ++dropped_lines_; pending_.clear(); }
          }
        } else if (pending_.size() < 8192) pending_.push_back(c);
        else line_overflow_ = true;
      }
    }
    file_cv_.notify_all(); line_cv_.notify_all();
  }

  void write_loop() {
    auto next_sync = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (true) {
      std::string chunk;
      {
        std::unique_lock<std::mutex> lock(file_mu_);
        file_cv_.wait_for(lock, std::chrono::seconds(1), [this] { return stop_ || !file_queue_.empty(); });
        if (file_queue_.empty()) {
          if (stop_ || fatal_) break;
          if (!sync_file()) break;
          next_sync = std::chrono::steady_clock::now() + std::chrono::seconds(1);
          continue;
        }
        chunk = std::move(file_queue_.front()); file_queue_.pop_front(); file_bytes_ -= chunk.size();
      }
      if (std::fwrite(chunk.data(), 1, chunk.size(), file_) != chunk.size()) { fail("raw file write failed"); break; }
      if (std::chrono::steady_clock::now() >= next_sync) {
        if (!sync_file()) break;
        next_sync = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      }
    }
    sync_file();
  }

  bool sync_file() {
    if (std::fflush(file_) != 0) { fail("raw file flush failed"); return false; }
#if !defined(_WIN32)
    if (::fsync(::fileno(file_)) != 0) { fail("raw file sync failed"); return false; }
#endif
    return true;
  }

  void parse_loop() {
    while (true) {
      std::string line;
      {
        std::unique_lock<std::mutex> lock(line_mu_);
        line_cv_.wait(lock, [this] { return stop_ || !line_queue_.empty(); });
        if (line_queue_.empty()) { if (stop_) break; else continue; }
        line = std::move(line_queue_.front()); line_queue_.pop_front(); line_bytes_ -= line.size();
      }
      try { on_line_(line); }
      catch (const std::exception& e) {
        ++dropped_lines_;
        std::cerr << "ROS line publish failed: " << e.what() << std::endl;
      }
    }
  }

  void fail(const char* reason) {
    std::cerr << "FATAL: " << reason << "; raw file: " << file_path_ << std::endl;
    fatal_ = true; stop_ = true; file_cv_.notify_all(); line_cv_.notify_all();
  }

  std::string port_name_, log_dir_, file_path_, pending_;
  bool line_overflow_ = false;
  int baud_;
  std::function<void(const std::string&)> on_line_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::FILE* file_ = nullptr;
  std::atomic<bool> stop_{false}, fatal_{false}, joined_{false};
  std::atomic<uint64_t> dropped_lines_{0};
  std::mutex file_mu_, line_mu_;
  std::condition_variable file_cv_, line_cv_;
  std::deque<std::string> file_queue_, line_queue_;
  std::size_t file_bytes_ = 0, line_bytes_ = 0;
  std::thread reader_, writer_, parser_;
  inline static std::atomic<uint64_t> file_number_{0};
};

}  // namespace rtk
