/******************************************************************************
 * @file    zlac8015d_rpm_node_ros2.cpp
 * @brief   ZLAC8015D RPM reader - svtrobo
 * @author  Adapted for ROS2
 * @date    2026/3/27
 *****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <cstring>
#include <cmath>
#include <stdexcept>
#include <map>

struct RpmState
{
    double left  = 0.0;
    double right = 0.0;
    int zero_cnt = 0;
};

class ZLAC8015DRpmNode : public rclcpp::Node {
public:
    ZLAC8015DRpmNode() : Node("zlac8015d_rpm_node") {
        // 声明并获取参数
        this->declare_parameter<std::string>("can_interface", "can3");
        this->declare_parameter<double>("poll_hz", 50.0);
        this->declare_parameter<int>("response_window_ms", 8);
        this->get_parameter("can_interface", can_iface_);
        this->get_parameter("poll_hz", poll_hz_);
        this->get_parameter("response_window_ms", response_window_ms_);

        if (poll_hz_ <= 0.0) {
            poll_hz_ = 50.0;
        }
        if (response_window_ms_ < 1) {
            response_window_ms_ = 1;
        }

        // 初始化驱动器TPDO ID（需先于CAN过滤器配置）
        initDrivers();
        // 初始化CAN套接字
        openCanSocket();
        // 初始化发布者
        initPublishers();
    }

    ~ZLAC8015DRpmNode() {
        if (can_sock_ >= 0)
            close(can_sock_);
    }

    void spin() {
        rclcpp::Rate rate(poll_hz_);

        while (rclcpp::ok())
        {
            requestActualSpeed(front_sdo_tx_id_);
            requestActualSpeed(rear_sdo_tx_id_);
            readFramesWithinWindow(response_window_ms_);
            publishAll();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    // 零速确认配置
    static constexpr int ZERO_CONFIRM_COUNT = 3;
    static constexpr double ZERO_THRESHOLD_RPM = 1.0;

    // ROS发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_front_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_front_right_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rear_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rear_right_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_all_rpm_;

    // CAN相关
    std::string can_iface_;
    int can_sock_ = -1;
    double poll_hz_ = 50.0;
    int response_window_ms_ = 8;

    // 驱动器状态
    std::map<uint32_t, RpmState> driver_states_;
    uint32_t front_sdo_tx_id_;
    uint32_t rear_sdo_tx_id_;
    uint32_t front_sdo_rx_id_;
    uint32_t rear_sdo_rx_id_;

    void initDrivers() {
        front_sdo_tx_id_ = 0x600 + 1; // 前驱动器node_id=1, 主站->从站
        rear_sdo_tx_id_  = 0x600 + 2; // 后驱动器node_id=2, 主站->从站
        front_sdo_rx_id_ = 0x580 + 1; // 前驱动器node_id=1, 从站->主站
        rear_sdo_rx_id_  = 0x580 + 2; // 后驱动器node_id=2, 从站->主站

        driver_states_[front_sdo_rx_id_] = RpmState();
        driver_states_[rear_sdo_rx_id_]  = RpmState();
    }

    void initPublishers() {
        pub_front_left_  = this->create_publisher<std_msgs::msg::Float64>("front_left_rpm", 10);
        pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("front_right_rpm", 10);
        pub_rear_left_   = this->create_publisher<std_msgs::msg::Float64>("rear_left_rpm", 10);
        pub_rear_right_  = this->create_publisher<std_msgs::msg::Float64>("rear_right_rpm", 10);
        pub_all_rpm_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("all_wheel_rpm", 10);
    }

    void openCanSocket() {
        // 创建CAN原始套接字
        can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sock_ < 0)
            throw std::runtime_error("Failed to create CAN socket: " + std::string(strerror(errno)));

        // 获取CAN接口索引
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ);
        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_sock_);
            throw std::runtime_error("Failed to get CAN interface index: " + std::string(strerror(errno)));
        }

        // 绑定CAN套接字
        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(can_sock_);
            throw std::runtime_error("Failed to bind CAN socket: " + std::string(strerror(errno)));
        }

        // 设置非阻塞读取，避免在无CAN帧时卡住主循环
        int flags = fcntl(can_sock_, F_GETFL, 0);
        if (flags < 0 || fcntl(can_sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
            close(can_sock_);
            throw std::runtime_error("Failed to set non-blocking CAN socket: " + std::string(strerror(errno)));
        }

        // 设置CAN过滤器，仅接收前后驱动器的SDO响应帧
        struct can_filter filters[2];
        filters[0].can_id   = front_sdo_rx_id_;
        filters[0].can_mask = CAN_SFF_MASK;
        filters[1].can_id   = rear_sdo_rx_id_;
        filters[1].can_mask = CAN_SFF_MASK;

        if (setsockopt(can_sock_, SOL_CAN_RAW, CAN_RAW_FILTER,
                       &filters, sizeof(filters)) < 0) {
            close(can_sock_);
            throw std::runtime_error("Failed to set CAN filter: " + std::string(strerror(errno)));
        }
    }

    void requestActualSpeed(uint32_t sdo_tx_id) {
        struct can_frame req {};
        req.can_id = sdo_tx_id;
        req.can_dlc = 8;
        req.data[0] = 0x40; // SDO expedited read request
        req.data[1] = 0x6C; // index low byte: 0x606C
        req.data[2] = 0x60; // index high byte
        req.data[3] = 0x03; // sub-index
        req.data[4] = 0x00;
        req.data[5] = 0x00;
        req.data[6] = 0x00;
        req.data[7] = 0x00;

        int nbytes = write(can_sock_, &req, sizeof(req));
        if (nbytes != static_cast<int>(sizeof(req))) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Failed to send SDO speed request on can_id=0x%03X", sdo_tx_id);
        }
    }

    void readCanFrame() {
        struct can_frame frame {};
        // 非阻塞读取CAN帧
        int nbytes = read(can_sock_, &frame, sizeof(frame));
        if (nbytes <= 0)
            return;

        // 查找对应驱动器状态
        auto it = driver_states_.find(frame.can_id);
        if (it == driver_states_.end())
            return;

        // 仅处理8字节数据帧
        if (frame.can_dlc != 8)
            return;

        const uint8_t* d = frame.data;

        // 解析0x606C:03的SDO读取响应（cmd=0x43,4字节数据）
        if (d[0] == 0x43 && d[1] == 0x6C && d[2] == 0x60 && d[3] == 0x03)
        {
            // 原始值为0.1rpm单位
            int16_t left_raw  = static_cast<int16_t>(d[4] | (d[5] << 8));
            int16_t right_raw = static_cast<int16_t>(d[6] | (d[7] << 8));

            // 现场方向定义与驱动器反馈符号相反：前进应为正，后退应为负
            double left_rpm  = -left_raw  / 10.0;
            double right_rpm = -right_raw / 10.0;

            RpmState& st = it->second;

            // 零速防抖处理
            if (std::abs(left_rpm) < ZERO_THRESHOLD_RPM &&
                std::abs(right_rpm) < ZERO_THRESHOLD_RPM)
            {
                st.zero_cnt++;
                if (st.zero_cnt >= ZERO_CONFIRM_COUNT)
                {
                    st.left  = 0.0;
                    st.right = 0.0;
                }
            }
            else
            {
                st.zero_cnt = 0;
                st.left  = left_rpm;
                st.right = right_rpm;
            }
        }
    }

    void readFramesWithinWindow(int window_ms) {
        struct pollfd pfd {};
        pfd.fd = can_sock_;
        pfd.events = POLLIN;

        int remaining_ms = window_ms;
        while (remaining_ms > 0) {
            int ret = poll(&pfd, 1, remaining_ms);
            if (ret <= 0) {
                break;
            }
            readCanFrame();
            // 收到一帧后尽快继续读，尽量在同一周期拿到两路响应
            remaining_ms = 1;
        }
    }

    void publishAll() {
        const RpmState& front = driver_states_[front_sdo_rx_id_];
        const RpmState& rear  = driver_states_[rear_sdo_rx_id_];

        // 发布前驱动器转速
        publishPair(pub_front_left_, pub_front_right_, front);
        // 发布后驱动器转速
        publishPair(pub_rear_left_, pub_rear_right_, rear);

        // 聚合话题顺序: [front_left, front_right, rear_left, rear_right]
        std_msgs::msg::Float64MultiArray all_msg;
        all_msg.data = {front.left, front.right, rear.left, rear.right};
        pub_all_rpm_->publish(all_msg);
    }

    static void publishPair(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& left_pub,
                            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& right_pub,
                            const RpmState& st)
    {
        std_msgs::msg::Float64 msg;

        msg.data = st.left;
        left_pub->publish(msg);

        msg.data = st.right;
        right_pub->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ZLAC8015DRpmNode>();
        node->spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("zlac8015d_rpm_node"), "%s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
