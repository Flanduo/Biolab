#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

class JKBmsStatusNode : public rclcpp::Node {
public:
  JKBmsStatusNode() : Node("jk_bms_status_node") {
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<int>("slave_addr", 1);
    declare_parameter<int>("poll_ms", 500);
    declare_parameter<int>("response_timeout_ms", 150);
    declare_parameter<int>("retry_count", 3);
    declare_parameter<bool>("verbose_frame_log", false);
    declare_parameter<std::string>("parity", "none");  // none/even/odd
    declare_parameter<int>("stop_bits", 1);            // 1/2
    declare_parameter<int>("data_bits", 8);            // 7/8
    declare_parameter<bool>("diag_hex_log", false);    // print TX/RX hex
    declare_parameter<bool>("probe_mode", false);      // read one register block only
    declare_parameter<int>("probe_reg", 0x1290);
    declare_parameter<int>("probe_count", 2);
    declare_parameter<int>("soc_register", 0x12A7);
    declare_parameter<int>("soc_alt_block_start", 0x12A6);
    declare_parameter<int>("soc_alt_block_count", 2);
    declare_parameter<int>("soc_alt_word_index", 1);
    declare_parameter<int>("soc_alt_byte_index", 0);  // 0: low byte, 1: high byte
    declare_parameter<bool>("soc_estimate_from_voltage", true);
    declare_parameter<double>("soc_pack_empty_v", 18.0);
    declare_parameter<double>("soc_pack_full_v", 25.2);
    declare_parameter<bool>("publish_legacy_topics", false);

    get_parameter("serial_port", serial_port_);
    get_parameter("baudrate", baudrate_);
    get_parameter("slave_addr", slave_addr_);
    get_parameter("poll_ms", poll_ms_);
    get_parameter("response_timeout_ms", response_timeout_ms_);
    get_parameter("retry_count", retry_count_);
    get_parameter("verbose_frame_log", verbose_frame_log_);
    get_parameter("parity", parity_);
    get_parameter("stop_bits", stop_bits_);
    get_parameter("data_bits", data_bits_);
    get_parameter("diag_hex_log", diag_hex_log_);
    get_parameter("probe_mode", probe_mode_);
    get_parameter("probe_reg", probe_reg_);
    get_parameter("probe_count", probe_count_);
    get_parameter("soc_register", soc_register_);
    get_parameter("soc_alt_block_start", soc_alt_block_start_);
    get_parameter("soc_alt_block_count", soc_alt_block_count_);
    get_parameter("soc_alt_word_index", soc_alt_word_index_);
    get_parameter("soc_alt_byte_index", soc_alt_byte_index_);
    get_parameter("soc_estimate_from_voltage", soc_estimate_from_voltage_);
    get_parameter("soc_pack_empty_v", soc_pack_empty_v_);
    get_parameter("soc_pack_full_v", soc_pack_full_v_);
    get_parameter("publish_legacy_topics", publish_legacy_topics_);

    if (poll_ms_ < 50) {
      poll_ms_ = 50;
    }
    if (response_timeout_ms_ < 20) {
      response_timeout_ms_ = 20;
    }
    retry_count_ = std::max(1, retry_count_);

    pub_full_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("bms/full_state", 10);
    if (publish_legacy_topics_) {
      pub_all_data_ = create_publisher<std_msgs::msg::Float64MultiArray>("bms/all_data", 10);
      pub_pack_voltage_v_ = create_publisher<std_msgs::msg::Float64>("bms/pack_voltage_v", 10);
      pub_pack_current_a_ = create_publisher<std_msgs::msg::Float64>("bms/pack_current_a", 10);
      pub_soc_ = create_publisher<std_msgs::msg::UInt8>("bms/soc", 10);
      pub_alarm_flags_ = create_publisher<std_msgs::msg::UInt32>("bms/alarm_flags", 10);
      pub_online_ = create_publisher<std_msgs::msg::Bool>("bms/online", 10);
    }

    if (!openAndConfigSerial()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize serial port %s", serial_port_.c_str());
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_ms_),
      std::bind(&JKBmsStatusNode::pollOnce, this));

    RCLCPP_INFO(
      get_logger(), "JK BMS node started: port=%s addr=%d poll=%dms",
      serial_port_.c_str(), slave_addr_, poll_ms_);
  }

  ~JKBmsStatusNode() override {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  struct BmsSnapshot {
    std::vector<uint16_t> cell_mv;
    uint32_t pack_voltage_mv = 0;
    int32_t pack_current_ma = 0;
    uint32_t pack_power_mw = 0;
    int16_t temp1_deci_c = 0;
    int16_t temp2_deci_c = 0;
    uint32_t alarm_flags = 0;
    uint8_t soc = 0;
  };

  std::string serial_port_;
  int baudrate_ = 115200;
  int slave_addr_ = 1;
  int poll_ms_ = 500;
  int response_timeout_ms_ = 150;
  int retry_count_ = 3;
  bool verbose_frame_log_ = false;
  std::string parity_ = "none";
  int stop_bits_ = 1;
  int data_bits_ = 8;
  bool diag_hex_log_ = false;
  bool probe_mode_ = false;
  int probe_reg_ = 0x1290;
  int probe_count_ = 2;
  int soc_register_ = 0x12A7;
  int soc_alt_block_start_ = 0x12A6;
  int soc_alt_block_count_ = 2;
  int soc_alt_word_index_ = 1;
  int soc_alt_byte_index_ = 0;
  bool soc_estimate_from_voltage_ = true;
  double soc_pack_empty_v_ = 18.0;
  double soc_pack_full_v_ = 25.2;
  bool publish_legacy_topics_ = false;

  int serial_fd_ = -1;
  int consecutive_failures_ = 0;
  uint8_t last_soc_ = 0;
  BmsSnapshot last_snapshot_;
  bool has_snapshot_ = false;
  std::mutex serial_mtx_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_full_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_all_data_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pack_voltage_v_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pack_current_a_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_soc_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_alarm_flags_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_online_;

  static std::string toHex(const std::vector<uint8_t> & data) {
    std::ostringstream oss;
    oss << std::hex;
    for (size_t i = 0; i < data.size(); ++i) {
      if (i != 0) {
        oss << " ";
      }
      const int v = static_cast<int>(data[i]);
      if (v < 16) {
        oss << "0";
      }
      oss << v;
    }
    return oss.str();
  }

  static uint16_t crc16Modbus(const uint8_t * data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int j = 0; j < 8; ++j) {
        if (crc & 0x0001) {
          crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
        } else {
          crc = static_cast<uint16_t>(crc >> 1);
        }
      }
    }
    return crc;
  }

  static uint16_t beU16(const uint8_t hi, const uint8_t lo) {
    return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }

  static uint32_t regsToU32(const uint16_t high_reg, const uint16_t low_reg) {
    return (static_cast<uint32_t>(high_reg) << 16) | static_cast<uint32_t>(low_reg);
  }

  static int32_t regsToI32(const uint16_t high_reg, const uint16_t low_reg) {
    return static_cast<int32_t>(regsToU32(high_reg, low_reg));
  }

  uint8_t estimateSocFromVoltageMv(const uint32_t pack_mv) const {
    if (soc_pack_full_v_ <= soc_pack_empty_v_) {
      return last_soc_;
    }
    const double pack_v = static_cast<double>(pack_mv) / 1000.0;
    double ratio = (pack_v - soc_pack_empty_v_) / (soc_pack_full_v_ - soc_pack_empty_v_);
    if (ratio < 0.0) {
      ratio = 0.0;
    } else if (ratio > 1.0) {
      ratio = 1.0;
    }
    return static_cast<uint8_t>(ratio * 100.0 + 0.5);
  }

  speed_t toSpeedT(const int baud) {
    switch (baud) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
      default:
        return B115200;
    }
  }

  bool openAndConfigSerial() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", serial_port_.c_str(), strerror(errno));
      return false;
    }

    termios cfg {};
    if (tcgetattr(serial_fd_, &cfg) < 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      return false;
    }

    cfmakeraw(&cfg);
    const speed_t spd = toSpeedT(baudrate_);
    cfsetispeed(&cfg, spd);
    cfsetospeed(&cfg, spd);

    cfg.c_cflag &= ~CSIZE;
    if (data_bits_ == 7) {
      cfg.c_cflag |= CS7;
    } else {
      cfg.c_cflag |= CS8;
      data_bits_ = 8;
    }

    cfg.c_cflag &= ~(PARENB | PARODD);
    if (parity_ == "even") {
      cfg.c_cflag |= PARENB;
    } else if (parity_ == "odd") {
      cfg.c_cflag |= PARENB;
      cfg.c_cflag |= PARODD;
    } else {
      parity_ = "none";
    }

    cfg.c_cflag &= ~CSTOPB;
    if (stop_bits_ == 2) {
      cfg.c_cflag |= CSTOPB;
    } else {
      stop_bits_ = 1;
    }
    cfg.c_cflag &= ~CRTSCTS;
    cfg.c_cflag |= CLOCAL | CREAD;
    cfg.c_cc[VTIME] = 0;
    cfg.c_cc[VMIN] = 0;

    tcflush(serial_fd_, TCIOFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &cfg) < 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Serial configured: baud=%d data_bits=%d parity=%s stop_bits=%d",
      baudrate_, data_bits_, parity_.c_str(), stop_bits_);
    return true;
  }

  bool writeAll(const std::vector<uint8_t> & frame, int timeout_ms) {
    size_t sent = 0;
    while (sent < frame.size()) {
      pollfd pfd {};
      pfd.fd = serial_fd_;
      pfd.events = POLLOUT;
      const int rv = poll(&pfd, 1, timeout_ms);
      if (rv <= 0) {
        return false;
      }
      const ssize_t n = write(serial_fd_, frame.data() + sent, frame.size() - sent);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

  ssize_t readExact(uint8_t * out, size_t need, int timeout_ms) {
    size_t got = 0;
    int elapsed = 0;
    constexpr int kStepMs = 20;
    while (got < need && elapsed < timeout_ms) {
      pollfd pfd {};
      pfd.fd = serial_fd_;
      pfd.events = POLLIN;
      const int wait_ms = std::min(kStepMs, timeout_ms - elapsed);
      const int rv = poll(&pfd, 1, wait_ms);
      elapsed += wait_ms;
      if (rv < 0) {
        if (errno == EINTR) {
          continue;
        }
        return -1;
      }
      if (rv == 0) {
        continue;
      }
      const ssize_t n = read(serial_fd_, out + got, need - got);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        return -1;
      }
      if (n == 0) {
        continue;
      }
      got += static_cast<size_t>(n);
    }
    return static_cast<ssize_t>(got);
  }

  bool readHoldingRegisters(uint16_t start_reg, uint16_t reg_count, std::vector<uint16_t> & out_regs) {
    out_regs.clear();
    if (serial_fd_ < 0) {
      return false;
    }

    std::vector<uint8_t> req = {
      static_cast<uint8_t>(slave_addr_ & 0xFF), 0x03,
      static_cast<uint8_t>((start_reg >> 8) & 0xFF), static_cast<uint8_t>(start_reg & 0xFF),
      static_cast<uint8_t>((reg_count >> 8) & 0xFF), static_cast<uint8_t>(reg_count & 0xFF)};
    const uint16_t crc_req = crc16Modbus(req.data(), req.size());
    req.push_back(static_cast<uint8_t>(crc_req & 0xFF));
    req.push_back(static_cast<uint8_t>((crc_req >> 8) & 0xFF));
    if (diag_hex_log_) {
      RCLCPP_INFO(get_logger(), "[TX] %s", toHex(req).c_str());
    }

    tcflush(serial_fd_, TCIFLUSH);
    if (!writeAll(req, response_timeout_ms_)) {
      return false;
    }

    uint8_t header[3] = {0};
    const ssize_t h = readExact(header, sizeof(header), response_timeout_ms_);
    if (h != static_cast<ssize_t>(sizeof(header))) {
      if (diag_hex_log_) {
        RCLCPP_WARN(get_logger(), "[RX] timeout while reading header");
      }
      return false;
    }

    if (header[0] != static_cast<uint8_t>(slave_addr_ & 0xFF)) {
      if (diag_hex_log_) {
        RCLCPP_WARN(get_logger(), "[RX] addr mismatch: got=0x%02X expect=0x%02X", header[0], slave_addr_);
      }
      return false;
    }

    if (header[1] == static_cast<uint8_t>(0x03 | 0x80)) {
      uint8_t tail[2] = {0};
      if (readExact(tail, sizeof(tail), response_timeout_ms_) != static_cast<ssize_t>(sizeof(tail))) {
        return false;
      }
      if (diag_hex_log_) {
        std::vector<uint8_t> ex = {header[0], header[1], header[2], tail[0], tail[1]};
        RCLCPP_WARN(get_logger(), "[RX-EXCEPTION] %s", toHex(ex).c_str());
      }
      // SOC register may be unsupported on some JK firmwares; avoid log spam.
      if (start_reg != static_cast<uint16_t>(soc_register_ & 0xFFFF)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Modbus exception: code=0x%02X reg=0x%04X",
          header[2], start_reg);
      }
      return false;
    }

    if (header[1] != 0x03) {
      return false;
    }

    const uint8_t byte_count = header[2];
    if (byte_count != static_cast<uint8_t>(reg_count * 2)) {
      return false;
    }

    std::vector<uint8_t> body(static_cast<size_t>(byte_count) + 2U, 0);
    const ssize_t b = readExact(body.data(), body.size(), response_timeout_ms_);
    if (b != static_cast<ssize_t>(body.size())) {
      if (diag_hex_log_) {
        RCLCPP_WARN(get_logger(), "[RX] timeout while reading payload");
      }
      return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(3 + body.size());
    frame.push_back(header[0]);
    frame.push_back(header[1]);
    frame.push_back(header[2]);
    frame.insert(frame.end(), body.begin(), body.end());
    if (diag_hex_log_) {
      RCLCPP_INFO(get_logger(), "[RX] %s", toHex(frame).c_str());
    }

    const uint16_t crc_calc = crc16Modbus(frame.data(), frame.size() - 2U);
    const uint16_t crc_recv = beU16(frame[frame.size() - 1], frame[frame.size() - 2]);
    if (crc_calc != crc_recv) {
      return false;
    }

    if (verbose_frame_log_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Read reg=0x%04X count=%u OK", start_reg, reg_count);
    }

    out_regs.reserve(reg_count);
    for (size_t i = 0; i < reg_count; ++i) {
      const size_t idx = 3U + i * 2U;
      out_regs.push_back(beU16(frame[idx], frame[idx + 1]));
    }
    return true;
  }

  bool readHoldingRegistersRetry(
    uint16_t start_reg, uint16_t reg_count, std::vector<uint16_t> & out_regs)
  {
    std::lock_guard<std::mutex> lock(serial_mtx_);
    for (int i = 0; i < retry_count_; ++i) {
      if (readHoldingRegisters(start_reg, reg_count, out_regs)) {
        return true;
      }
    }
    return false;
  }

  bool readBmsSnapshot(BmsSnapshot & snap) {
    std::vector<uint16_t> cells;
    if (!readHoldingRegistersRetry(0x1200, 32, cells)) {
      return false;
    }
    snap.cell_mv = cells;

    std::vector<uint16_t> regs_fast;
    if (!readHoldingRegistersRetry(0x1290, 6, regs_fast)) {
      return false;
    }
    if (regs_fast.size() < 6) {
      return false;
    }

    snap.pack_voltage_mv = regsToU32(regs_fast[0], regs_fast[1]);
    snap.pack_power_mw = regsToU32(regs_fast[2], regs_fast[3]);
    snap.pack_current_ma = regsToI32(regs_fast[4], regs_fast[5]);

    // Read temperatures and alarm flags by exact addresses from protocol doc.
    std::vector<uint16_t> reg_t1;
    if (!readHoldingRegistersRetry(0x129C, 1, reg_t1) || reg_t1.empty()) {
      return false;
    }
    snap.temp1_deci_c = static_cast<int16_t>(reg_t1[0]);

    std::vector<uint16_t> reg_t2;
    if (!readHoldingRegistersRetry(0x129E, 1, reg_t2) || reg_t2.empty()) {
      return false;
    }
    snap.temp2_deci_c = static_cast<int16_t>(reg_t2[0]);

    std::vector<uint16_t> reg_alarm;
    if (!readHoldingRegistersRetry(0x12A0, 2, reg_alarm) || reg_alarm.size() < 2) {
      return false;
    }
    snap.alarm_flags = regsToU32(reg_alarm[0], reg_alarm[1]);

    // User requirement: do not read SOC register, always estimate from pack voltage.
    if (soc_estimate_from_voltage_) {
      snap.soc = estimateSocFromVoltageMv(snap.pack_voltage_mv);
      last_soc_ = snap.soc;
    } else {
      snap.soc = last_soc_;
    }
    return true;
  }

  void publishOnline(bool online) {
    if (publish_legacy_topics_ && pub_online_) {
      std_msgs::msg::Bool msg;
      msg.data = online;
      pub_online_->publish(msg);
    }
  }

  void publishSnapshot(const BmsSnapshot & snap, bool online) {
    // Unified topic format:
    // [online(0/1), pack_voltage_v, pack_current_a, soc_percent, pack_power_w,
    //  temp1_c, temp2_c, alarm_flags_u32, cell_count, cell1_mv ...]
    std_msgs::msg::Float64MultiArray full_msg;
    full_msg.data.reserve(9 + snap.cell_mv.size());
    full_msg.data.push_back(online ? 1.0 : 0.0);
    full_msg.data.push_back(static_cast<double>(snap.pack_voltage_mv) / 1000.0);
    full_msg.data.push_back(static_cast<double>(snap.pack_current_ma) / 1000.0);
    full_msg.data.push_back(static_cast<double>(snap.soc));
    full_msg.data.push_back(static_cast<double>(snap.pack_power_mw) / 1000.0);
    full_msg.data.push_back(static_cast<double>(snap.temp1_deci_c) / 10.0);
    full_msg.data.push_back(static_cast<double>(snap.temp2_deci_c) / 10.0);
    full_msg.data.push_back(static_cast<double>(snap.alarm_flags));
    full_msg.data.push_back(static_cast<double>(snap.cell_mv.size()));
    for (const auto cell : snap.cell_mv) {
      full_msg.data.push_back(static_cast<double>(cell));
    }
    pub_full_state_->publish(full_msg);

    if (!publish_legacy_topics_) {
      return;
    }

    std_msgs::msg::Float64 v_msg;
    v_msg.data = static_cast<double>(snap.pack_voltage_mv) / 1000.0;
    pub_pack_voltage_v_->publish(v_msg);

    std_msgs::msg::Float64 i_msg;
    i_msg.data = static_cast<double>(snap.pack_current_ma) / 1000.0;
    pub_pack_current_a_->publish(i_msg);

    std_msgs::msg::UInt8 soc_msg;
    soc_msg.data = snap.soc;
    pub_soc_->publish(soc_msg);

    std_msgs::msg::UInt32 alarm_msg;
    alarm_msg.data = snap.alarm_flags;
    pub_alarm_flags_->publish(alarm_msg);

    std_msgs::msg::Float64MultiArray all_msg;
    all_msg.data.reserve(1 + 1 + 1 + 1 + 1 + 1 + snap.cell_mv.size());
    all_msg.data.push_back(static_cast<double>(snap.pack_voltage_mv));   // mV
    all_msg.data.push_back(static_cast<double>(snap.pack_current_ma));   // mA
    all_msg.data.push_back(static_cast<double>(snap.pack_power_mw));     // mW
    all_msg.data.push_back(static_cast<double>(snap.temp1_deci_c) / 10.0);
    all_msg.data.push_back(static_cast<double>(snap.temp2_deci_c) / 10.0);
    all_msg.data.push_back(static_cast<double>(snap.soc));               // %
    for (const auto cell : snap.cell_mv) {
      all_msg.data.push_back(static_cast<double>(cell));                 // mV
    }
    pub_all_data_->publish(all_msg);
  }

  void pollOnce() {
    if (probe_mode_) {
      std::vector<uint16_t> out;
      const bool ok = readHoldingRegistersRetry(
        static_cast<uint16_t>(probe_reg_ & 0xFFFF),
        static_cast<uint16_t>(std::max(1, probe_count_)),
        out);
      publishOnline(ok);
      if (ok) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Probe OK reg=0x%04X count=%d", probe_reg_ & 0xFFFF, std::max(1, probe_count_));
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Probe failed reg=0x%04X count=%d", probe_reg_ & 0xFFFF, std::max(1, probe_count_));
      }
      return;
    }

    BmsSnapshot snap;
    if (!readBmsSnapshot(snap)) {
      ++consecutive_failures_;
      publishOnline(false);
      if (has_snapshot_) {
        publishSnapshot(last_snapshot_, false);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "BMS poll failed, consecutive=%d", consecutive_failures_);
      return;
    }

    if (consecutive_failures_ >= 5) {
      RCLCPP_INFO(get_logger(), "BMS communication recovered");
    }
    consecutive_failures_ = 0;
    last_snapshot_ = snap;
    has_snapshot_ = true;
    publishOnline(true);
    publishSnapshot(snap, true);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JKBmsStatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
