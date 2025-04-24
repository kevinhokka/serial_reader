#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
// POSIX 串口头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode()
        : Node("serial_reader_node")
    {
        /***** 1. 声明发布者 *****/
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);

        // 读取参数：串口设备号、波特率
        this->declare_parameter<std::string>("port", "/dev/serial_twistctl");
        this->declare_parameter<int>("baud", 115200);
        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        RCLCPP_INFO(this->get_logger(), "Serial port param: %s, baud=%d", port_.c_str(), baud_);

        /***** 2. 打开并配置串口 *****/
        if (!openSerialPort(port_, baud_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            throw std::runtime_error("Failed to open serial port");
        }
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully!");

        /***** 3. 创建读取线程 (或定时器) 以持续读取串口数据 *****/
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
    /**
     * @brief 打开并配置串口 (阻塞模式简单示例)
     */
    bool openSerialPort(const std::string &port, int baud)
    {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            perror("openSerialPort");
            return false;
        }

        // 获取并修改串口属性
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            perror("tcgetattr");
            return false;
        }

        // 设置输入输出波特率
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

        // 8N1 设置
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8位数据
        tty.c_cflag &= ~PARENB;                      // 无校验
        tty.c_cflag &= ~CSTOPB;                      // 1位停止位
        tty.c_cflag |= (CLOCAL | CREAD);             // 本地连接, 允许读取

        // 原始模式 (non-canonical)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 最短读取字节数 1, 超时 1秒
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10; // 一单位=0.1s, 10->1秒

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            return false;
        }
        return true;
    }

    /**
     * @brief 循环读取串口数据，每行解析后发布 ROS 消息
     */
    void readSerialLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Start reading serial data loop.");
        std::string buffer;
        buffer.reserve(256);
        while (rclcpp::ok() && run_read_thread_) {
            // 一次读一个字节，直到读到换行符或超时
            char c;
            int n = ::read(fd_, &c, 1);
            if (n > 0) {
                if (c == '\n' || c == '\r') {
                    // 行结束，解析
                    if (!buffer.empty()) {
                        parseAndPublish(buffer);
                        buffer.clear();
                    }
                } else {
                    buffer.push_back(c);
                }
            } else {
                // 暂无数据可读，sleep一会儿
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    /**
     * @brief 核心：解析单行（包含 16 个逗号分隔的浮点数），并发布磁力计消息
     */
    void parseAndPublish(const std::string &line)
    {
        // 预期有 16 个浮点数
        // x, y, z, qx, qy, qz, qw, 0.0, real_vc, 0.0, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z
        std::vector<double> vals;
        vals.reserve(16);

        {
            std::stringstream ss(line);
            for (int i = 0; i < 16; i++) {
                std::string token;
                if (!std::getline(ss, token, ',')) {
                    // 如果数据行不完整，直接忽略
                    return;
                }
                try {
                    double d = std::stod(token);
                    vals.push_back(d);
                } catch (...) {
                    // 如果转换失败，直接忽略
                    return;
                }
            }
        }

        if (vals.size() < 16) {
            // 数据不足，直接忽略
            return;
        }

        // 提取磁力计数据
        double mag_x = vals[13];
        double mag_y = vals[14];
        double mag_z = vals[15];

        // 发布磁力计消息
        auto now = this->now();
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = now;
        mag_msg.header.frame_id = "base_link"; // 或其他
        mag_msg.magnetic_field.x = mag_x;
        mag_msg.magnetic_field.y = mag_y;
        mag_msg.magnetic_field.z = mag_z;

        mag_pub_->publish(mag_msg);
    }

private:
    // 串口相关
    std::string port_;
    int baud_;
    int fd_{-1};          // 串口文件描述符
    std::thread read_thread_; 
    bool run_read_thread_{false};

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}