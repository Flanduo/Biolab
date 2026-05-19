#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <atomic>
#include <poll.h>
#include <errno.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class MotorSerialIntegratedNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_;
    std::mutex serial_mutex_;
    std::atomic<bool> is_running_;

    const std::string SERIAL_DEV_ = "/dev/ttyACM2";
    const uint8_t SLAVE_ADDR_ = 0x01;
    const speed_t BAUDRATE_ = B57600;
    const int COMM_TIMEOUT_MS_ = 500;
    const int16_t MAX_SPEED_ = 6000;
    const int16_t MIN_SPEED_ = 1;
    const uint16_t ABSOLUTE_POSITION_LOW32_REG_ = 0x0B4D;
    const double COUNTS_PER_REV_ = 131072.0;
    const int WORD_ORDER_ = 1;
    const int POSITION_READ_TIMEOUT_MS_ = 100;

    uint16_t crc16_modbus(const uint8_t* data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
            }
        }
        return crc;
    }

    uint16_t u16_from_be(const uint8_t hi, const uint8_t lo) {
        return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
    }

    int32_t decode_int32_from_regs(const uint16_t reg0, const uint16_t reg1, const int word_order) {
        const uint16_t low_word = (word_order == 1) ? reg0 : reg1;
        const uint16_t high_word = (word_order == 1) ? reg1 : reg0;
        const uint32_t raw = (static_cast<uint32_t>(high_word) << 16) | static_cast<uint32_t>(low_word);
        return static_cast<int32_t>(raw);
    }

    bool write_all(const uint8_t* buf, size_t len, int timeout_ms = 500) {
        size_t sent = 0;
        while (sent < len) {
            struct pollfd pfd;
            pfd.fd = serial_fd_;
            pfd.events = POLLOUT;
            int rv = poll(&pfd, 1, timeout_ms);
            if (rv <= 0) {
                RCLCPP_ERROR(this->get_logger(), "串口写等待超时或出错: %s", rv == 0 ? "timeout" : strerror(errno));
                return false;
            }
            ssize_t w = write(serial_fd_, buf + sent, len - sent);
            if (w < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "串口写入失败: %s", strerror(errno));
                return false;
            }
            sent += static_cast<size_t>(w);
        }
        return true;
    }

    ssize_t read_exact(uint8_t* buf, size_t len, int timeout_ms) {
        size_t received = 0;
        int elapsed = 0;
        const int step_ms = 20;
        while (received < len && elapsed < timeout_ms) {
            struct pollfd pfd;
            pfd.fd = serial_fd_;
            pfd.events = POLLIN;
            int wait = std::min(step_ms, timeout_ms - elapsed);
            int rv = poll(&pfd, 1, wait);
            elapsed += wait;
            if (rv < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "串口 poll 出错: %s", strerror(errno));
                return -1;
            } else if (rv == 0) {
                continue;
            } else {
                ssize_t r = read(serial_fd_, buf + received, len - received);
                if (r < 0) {
                    if (errno == EINTR) continue;
                    RCLCPP_ERROR(this->get_logger(), "串口读取失败: %s", strerror(errno));
                    return -1;
                } else if (r == 0) {
                    continue;
                } else {
                    received += static_cast<size_t>(r);
                }
            }
        }
        return static_cast<ssize_t>(received);
    }

    bool set_serial_config() {
        struct termios cfg;
        if (tcgetattr(serial_fd_, &cfg) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr 失败: %s", strerror(errno));
            return false;
        }

        cfmakeraw(&cfg);
        if (cfsetispeed(&cfg, BAUDRATE_) < 0 || cfsetospeed(&cfg, BAUDRATE_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置波特率失败");
            return false;
        }

        cfg.c_cflag &= ~CSIZE;
        cfg.c_cflag |= CS8;
        cfg.c_cflag &= ~PARENB;
        cfg.c_cflag |= CSTOPB;
        cfg.c_cflag &= ~CRTSCTS;
        cfg.c_cflag |= CLOCAL | CREAD;

        cfg.c_cc[VTIME] = 0;
        cfg.c_cc[VMIN] = 0;

        tcflush(serial_fd_, TCIOFLUSH);
        if (tcsetattr(serial_fd_, TCSANOW, &cfg) < 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr 失败: %s", strerror(errno));
            return false;
        }
        return true;
    }

    bool write_reg_locked(uint16_t reg_addr, int16_t data) {
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x06;
        frame[2] = (reg_addr >> 8) & 0xFF;
        frame[3] = reg_addr & 0xFF;
        frame[4] = (data >> 8) & 0xFF;
        frame[5] = data & 0xFF;

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, sizeof(frame), COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "寄存器写入请求发送失败（地址0x%X）", reg_addr);
            return false;
        }

        uint8_t resp[8];
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got != static_cast<ssize_t>(sizeof(resp))) {
            RCLCPP_ERROR(this->get_logger(), "寄存器写入响应超时或长度不符（期望8，实际%zd）", got);
            return false;
        }
        if (resp[0] != SLAVE_ADDR_ || resp[1] != 0x06) {
            if (resp[1] == static_cast<uint8_t>(0x86)) {
                RCLCPP_ERROR(this->get_logger(), "写寄存器异常响应，错误码0x%X", (int)resp[2]);
            } else {
                RCLCPP_ERROR(this->get_logger(), "寄存器写入响应格式错误");
            }
            return false;
        }
        return true;
    }

    bool write_reg(uint16_t reg_addr, int16_t data) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        return write_reg_locked(reg_addr, data);
    }

    bool read_holding_registers_locked(uint16_t start_reg, uint16_t reg_count, std::vector<uint16_t>& out_regs, int timeout_ms) {
        out_regs.clear();
        uint8_t req[8] = {0};
        req[0] = SLAVE_ADDR_;
        req[1] = 0x03;
        req[2] = static_cast<uint8_t>(start_reg >> 8);
        req[3] = static_cast<uint8_t>(start_reg & 0xFF);
        req[4] = static_cast<uint8_t>(reg_count >> 8);
        req[5] = static_cast<uint8_t>(reg_count & 0xFF);

        const uint16_t crc = crc16_modbus(req, 6);
        req[6] = static_cast<uint8_t>(crc & 0xFF);
        req[7] = static_cast<uint8_t>(crc >> 8);

        if (!write_all(req, sizeof(req), timeout_ms)) {
            return false;
        }

        uint8_t header[3] = {0};
        const ssize_t header_len = read_exact(header, sizeof(header), timeout_ms);
        if (header_len != static_cast<ssize_t>(sizeof(header))) {
            return false;
        }

        if (header[0] != SLAVE_ADDR_) {
            return false;
        }

        if (header[1] & 0x80) {
            uint8_t tail[2] = {0};
            const ssize_t tail_len = read_exact(tail, sizeof(tail), timeout_ms);
            if (tail_len != static_cast<ssize_t>(sizeof(tail))) {
                return false;
            }
            const uint8_t exc_frame[3] = {header[0], header[1], header[2]};
            const uint16_t crc_calc = crc16_modbus(exc_frame, sizeof(exc_frame));
            const uint16_t crc_recv = u16_from_be(tail[1], tail[0]);
            if (crc_calc != crc_recv) {
                return false;
            }
            return false;
        }

        if (header[1] != 0x03) {
            return false;
        }

        const uint8_t byte_count = header[2];
        if (byte_count != reg_count * 2) {
            return false;
        }

        std::vector<uint8_t> body(static_cast<size_t>(byte_count) + 2U, 0);
        const ssize_t body_len = read_exact(body.data(), body.size(), timeout_ms);
        if (body_len != static_cast<ssize_t>(body.size())) {
            return false;
        }

        std::vector<uint8_t> frame;
        frame.reserve(3U + body.size());
        frame.push_back(header[0]);
        frame.push_back(header[1]);
        frame.push_back(header[2]);
        frame.insert(frame.end(), body.begin(), body.end());

        const uint16_t crc_calc = crc16_modbus(frame.data(), frame.size() - 2U);
        const uint16_t crc_recv = u16_from_be(frame[frame.size() - 1U], frame[frame.size() - 2U]);
        if (crc_calc != crc_recv) {
            return false;
        }

        out_regs.reserve(reg_count);
        for (size_t i = 0; i < reg_count; ++i) {
            const size_t idx = 3U + i * 2U;
            out_regs.push_back(u16_from_be(frame[idx], frame[idx + 1U]));
        }
        return true;
    }

    bool check_error(uint16_t& err_code) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x03;
        frame[2] = 0x0B;
        frame[3] = 0x22;
        frame[4] = 0x00;
        frame[5] = 0x01;

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, 8, COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "故障码请求发送失败");
            return false;
        }

        uint8_t resp[7] = {0};
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got <= 0) {
            RCLCPP_ERROR(this->get_logger(), "故障码响应超时或出错（len=%zd）", got);
            return false;
        }
        if (resp[1] == 0x03) {
            if (got < 7) {
                RCLCPP_ERROR(this->get_logger(), "故障码响应长度不足（%zd）", got);
                return false;
            }
            err_code = (resp[3] << 8) | resp[4];
            return true;
        } else if (resp[1] == 0x83) {
            RCLCPP_ERROR(this->get_logger(), "读取故障码被拒绝，异常码0x%X", resp[2]);
            return false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知故障码响应函数码: 0x%X", resp[1]);
            return false;
        }
    }

    bool read_position(int32_t& position) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        uint8_t frame[8] = {0};
        frame[0] = SLAVE_ADDR_;
        frame[1] = 0x03;
        frame[2] = (2823 >> 8) & 0xFF;
        frame[3] = 2823 & 0xFF;
        frame[4] = 0x00;
        frame[5] = 0x02;

        uint16_t crc = crc16_modbus(frame, 6);
        frame[6] = crc & 0xFF;
        frame[7] = (crc >> 8) & 0xFF;

        if (!write_all(frame, 8, COMM_TIMEOUT_MS_)) {
            RCLCPP_ERROR(this->get_logger(), "位置读取请求发送失败");
            return false;
        }

        uint8_t resp[9] = {0};
        ssize_t got = read_exact(resp, sizeof(resp), COMM_TIMEOUT_MS_);
        if (got <= 0) {
            RCLCPP_ERROR(this->get_logger(), "位置读取响应超时或出错（len=%zd）", got);
            return false;
        }
        if (resp[1] == 0x03) {
            if (got < 9) {
                RCLCPP_ERROR(this->get_logger(), "位置响应长度不足（%zd）", got);
                return false;
            }
            int16_t high_word = (resp[3] << 8) | resp[4];
            int16_t low_word = (resp[5] << 8) | resp[6];
            position = ((int32_t)high_word << 16) | (uint16_t)low_word;
            return true;
        } else if (resp[1] == 0x83) {
            RCLCPP_ERROR(this->get_logger(), "位置读取异常响应，错误码0x%X", resp[2]);
            return false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知位置响应函数码: 0x%X", resp[1]);
            return false;
        }
    }

    void emergency_stop_locked() {
        uint16_t addr = 771;
        write_reg_locked(addr, 0);
        RCLCPP_WARN(this->get_logger(), "电机紧急停止（已发送断开使能）");
        is_running_ = false;
    }

    void emergency_stop() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_fd_ >= 0) {
            emergency_stop_locked();
        } else {
            is_running_ = false;
        }
    }

    bool init_motor_params() {
        RCLCPP_INFO(this->get_logger(), "初始化电机参数...");
        if (!write_reg(770, 1)) return false;
        if (!write_reg(771, 0)) return false;
        if (!write_reg(512, 0)) return false;
        if (!write_reg(1538, 0)) return false;
        if (!write_reg(3085, 1)) {
            RCLCPP_WARN(this->get_logger(), "保存参数到 EEPROM 失败（非致命）");
        }
        RCLCPP_INFO(this->get_logger(), "电机参数初始化完成");
        return true;
    }

    void cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!is_running_) return;
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "收到无效控制指令长度");
            return;
        }

        int8_t direction = static_cast<int8_t>(msg->data[0]);
        int16_t speed = static_cast<int16_t>(msg->data[1]);

        if (direction == 0) {
            if (!write_reg(771, 0)) {
                RCLCPP_ERROR(this->get_logger(), "发送停止命令失败");
            } else {
                RCLCPP_INFO(this->get_logger(), "收到停止命令，已断开使能");
            }
            return;
        }

        speed = std::max<int16_t>(MIN_SPEED_, std::min<int16_t>(MAX_SPEED_, speed));

        if (direction == 1) {
            if (!write_reg(1539, speed)) {
                RCLCPP_ERROR(this->get_logger(), "设置正转速度失败");
                return;
            }
        } else if (direction == -1) {
            if (!write_reg(1539, static_cast<int16_t>(-speed))) {
                RCLCPP_ERROR(this->get_logger(), "设置反转速度失败");
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "无效方向指令：%d（仅支持1/-1/0）", direction);
            return;
        }

        if (!write_reg(771, 1)) {
            RCLCPP_ERROR(this->get_logger(), "使能电机失败");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "电机已使能，方向=%d 速度=%d", direction, abs(speed));
        }
    }

    void position_timer_callback() {
        if (!is_running_) {
            return;
        }

        std::vector<uint16_t> regs;
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            const bool ok = read_holding_registers_locked(
                ABSOLUTE_POSITION_LOW32_REG_,
                4,
                regs,
                POSITION_READ_TIMEOUT_MS_);
            if (!ok) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "位置读取失败");
                return;
            }
        }

        if (regs.size() != 4) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "位置寄存器数量异常: %zu", regs.size());
            return;
        }

        const int32_t low32 = decode_int32_from_regs(regs[0], regs[1], WORD_ORDER_);
        const int32_t high32 = decode_int32_from_regs(regs[2], regs[3], WORD_ORDER_);
        const int64_t absolute_position = (static_cast<int64_t>(high32) << 32) | static_cast<uint32_t>(low32);

        const double turns_raw = static_cast<double>(absolute_position) / COUNTS_PER_REV_;
        const double turns = std::round(turns_raw * 100.0) / 100.0;

        std_msgs::msg::Float64 msg;
        msg.data = turns;
        position_pub_->publish(msg);
    }

public:
    MotorSerialIntegratedNode() :
        Node("motor_serial_integrated_node"),
        serial_fd_(-1),
        is_running_(false) {

        serial_fd_ = open(SERIAL_DEV_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "串口打开失败：%s (%s)", SERIAL_DEV_.c_str(), strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "串口打开：%s", SERIAL_DEV_.c_str());

        if (!set_serial_config()) {
            RCLCPP_FATAL(this->get_logger(), "串口配置失败");
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        uint16_t err_code = 0;
        if (check_error(err_code)) {
            if (err_code != 0) {
                RCLCPP_WARN(this->get_logger(), "检测到驱动器故障码: 0x%X，尝试复位...", err_code);
                if (!write_reg(3329, 1)) {
                    RCLCPP_FATAL(this->get_logger(), "故障复位失败，请人工检查驱动器");
                    close(serial_fd_);
                    serial_fd_ = -1;
                    return;
                }
                usleep(200 * 1000);
                RCLCPP_INFO(this->get_logger(), "故障复位发送完毕");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "读取故障码失败（继续尝试初始化）");
        }

        if (!init_motor_params()) {
            RCLCPP_FATAL(this->get_logger(), "电机参数初始化失败，退出");
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        cmd_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/lift_control_cmd", 10,
            std::bind(&MotorSerialIntegratedNode::cmd_callback, this, std::placeholders::_1)
        );

        position_pub_ = this->create_publisher<std_msgs::msg::Float64>("/aimotor/position_state", rclcpp::QoS(10));
        timer_ = this->create_wall_timer(20ms, std::bind(&MotorSerialIntegratedNode::position_timer_callback, this));

        is_running_ = true;
        RCLCPP_INFO(this->get_logger(), "串口读写整合节点启动成功");
    }

    ~MotorSerialIntegratedNode() {
        emergency_stop();
        if (serial_fd_ >= 0) {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            write_reg_locked(771, 0);
            ::close(serial_fd_);
            serial_fd_ = -1;
            RCLCPP_INFO(this->get_logger(), "串口已关闭（析构）");
        }
    }

    void run() {
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "串口未就绪，退出节点");
            rclcpp::shutdown();
            return;
        }
        rclcpp::spin(shared_from_this());
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSerialIntegratedNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
