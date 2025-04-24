#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <thread>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode()
        : Node("serial_reader_node")
    {
        // 声明发布者
        mag_pub_  = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_CBoard", 10);
        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        // 读取参数：串口设备号、波特率
        this->declare_parameter<std::string>("port", "/dev/serial_twistctl");
        this->declare_parameter<int>("baud", 115200);
        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        RCLCPP_INFO(this->get_logger(), "Serial port param: %s, baud=%d", port_.c_str(), baud_);

        // 打开并配置串口
        if (!openSerialPort(port_, baud_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            throw std::runtime_error("Failed to open serial port");
        }
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully!");

        // 启动读串口线程
        run_read_thread_ = true;
        read_thread_ = std::thread(&SerialReaderNode::readSerialLoop, this);
    }

    ~SerialReaderNode()
    {
        run_read_thread_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    bool openSerialPort(const std::string &port, int baud)
    {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            perror("openSerialPort");
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            perror("tcgetattr");
            return false;
        }

        // 设置波特率
        speed_t baud_rate;
        switch (baud) {
        case 115200: baud_rate = B115200; break;
        case 921600: baud_rate = B921600; break;
        case 460800: baud_rate = B460800; break;
        case 57600:  baud_rate = B57600;  break;
        default:
            baud_rate = B115200;
            RCLCPP_WARN(this->get_logger(), "Unsupported baud rate. Using 115200 by default.");
        }
        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);

        // 8N1
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= (CLOCAL | CREAD);

        // 原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 读取配置：最少 0 字节，超时 1 秒
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            return false;
        }
        return true;
    }

    void readSerialLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Start reading serial data loop.");
        std::string buffer;
        buffer.reserve(256);

        while (rclcpp::ok() && run_read_thread_) {
            char c;
            int n = ::read(fd_, &c, 1);
            if (n > 0) {
                if (c == '\n' || c == '\r') {
                    if (!buffer.empty()) {
                        parseAndPublish(buffer);
                        buffer.clear();
                    }
                } else {
                    buffer.push_back(c);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void parseAndPublish(const std::string &line)
    {
        // 期望 16 个浮点数
        std::vector<double> vals;
        vals.reserve(16);
        std::stringstream ss(line);
        for (int i = 0; i < 16; i++) {
            std::string token;
            if (!std::getline(ss, token, ',')) {
                return;  // 数据不完整则丢弃
            }
            try {
                vals.push_back(std::stod(token));
            } catch (...) {
                return;  // 解析失败则丢弃
            }
        }

        if (vals.size() < 16) {
            return;
        }

        auto now = this->now();

        // 发布磁力计
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp    = now;
        mag_msg.header.frame_id = "base_link";
        mag_msg.magnetic_field.x = vals[13];
        mag_msg.magnetic_field.y = vals[14];
        mag_msg.magnetic_field.z = vals[15];
        mag_pub_->publish(mag_msg);

        // 发布里程计 Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        // 位姿
        odom.pose.pose.position.x    = vals[0];
        odom.pose.pose.position.y    = vals[1];
        odom.pose.pose.position.z    = vals[2];
        odom.pose.pose.orientation.x = vals[3];
        odom.pose.pose.orientation.y = vals[4];
        odom.pose.pose.orientation.z = vals[5];
        odom.pose.pose.orientation.w = vals[6];
        // 速度
        odom.twist.twist.linear.x  = vals[7];
        odom.twist.twist.linear.y  = vals[8];
        odom.twist.twist.linear.z   = vals[9];
        odom.twist.twist.angular.x = vals[10];
        odom.twist.twist.angular.y = vals[11];
        odom.twist.twist.angular.z = vals[12];
        odom_pub_->publish(odom);

        // 发布 IMU
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp            = now;
        imu_msg.header.frame_id         = "base_link";
        imu_msg.orientation             = odom.pose.pose.orientation;
        imu_msg.angular_velocity.x      = vals[10];
        imu_msg.angular_velocity.y      = vals[11];
        imu_msg.angular_velocity.z      = vals[12];
        // 下位机暂未输出线加速度，置 0
        imu_msg.linear_acceleration.x   = 0.0;
        imu_msg.linear_acceleration.y   = 0.0;
        imu_msg.linear_acceleration.z   = 0.0;
        imu_pub_->publish(imu_msg);
    }

    // 串口成员
    std::string port_;
    int baud_;
    int fd_{-1};
    bool run_read_thread_{false};
    std::thread read_thread_;

    // ROS 发布者
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
