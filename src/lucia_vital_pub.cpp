#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/wait.h>
#include <iostream>
#include <thread>
#include <chrono>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B2000000 // 2000000bps
#define BUFSIZE 16

class VitalPublisher : public rclcpp::Node{
public:
    VitalPublisher() : Node("vital_publisher"), is_calibrating(false){
        publisher_vital = this->create_publisher<std_msgs::msg::Int32MultiArray>("vital_data", 10);
        publisher_pressure = this->create_publisher<std_msgs::msg::Int32MultiArray>("pressure_data", 10);

        serial_fd = open_serial_port();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VitalPublisher::process_sensor_data, this));

        send_calibration_request();
    }

    ~VitalPublisher(){
        if(serial_fd >= 0){
            close(serial_fd);
        }
    }

private:
    int open_serial_port(){
        int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
        if(fd < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return -1;
        }

        struct termios tty;
        if(tcgetattr(fd, &tty) != 0){
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            close(fd);
            return -1;
        }

        cfsetospeed(&tty, BAUDRATE);
        cfsetispeed(&tty, BAUDRATE);
        cfmakeraw(&tty); // RAWモードに設定
        tty.c_cflag |= (CLOCAL | CREAD);

        if(tcsetattr(fd, TCSANOW, &tty) != 0){
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            close(fd);
            return -1;
        }

        return fd;
    }

    void send_calibration_request(){
        if(serial_fd < 0) return;

        std::vector<uint8_t> cmd = {0xAA, 0xC1, 0x0A, 0x00, 0x23, 0x55};
        write(serial_fd, cmd.data(), cmd.size());
        RCLCPP_INFO(this->get_logger(), "Sent calibration request");
        is_calibrating = true;
    }

    void check_calibration_status(){
        unsigned char rxData[BUFSIZE] = {};
        int len = read(serial_fd, rxData, BUFSIZE);
        if(len <= 0) return;

        std::vector<int> vec(rxData, rxData + len);
        if(vec.size() < 6) return;

        int error_code = vec[5];
		switch (error_code){
		case 0:
			RCLCPP_ERROR(this->get_logger(), "[0x00]There is no data to reply");
			is_calibrating = true;
			break;
		case 1:
			RCLCPP_ERROR(this->get_logger(), "[0x01]Invalid Command");
			is_calibrating = true;
			break;
		case 32:
			RCLCPP_INFO(this->get_logger(), "[0x20]Calibrating vital sensors");
			for(int count = 0; count < 600; count++){
				if(count % 10 == 0){
					RCLCPP_INFO(this->get_logger(), "Calibrating...");
				}
			}
			is_calibrating = false;
			break;
		case 33:
			RCLCPP_INFO(this->get_logger(), "[0x21]Vital sensor updating");
			for(int count = 0; count < 600; count++){
				if(count % 10 == 0){
					RCLCPP_INFO(this->get_logger(), "Updating...");
				}
			}
			is_calibrating = false;
			break;
		case 34:
			RCLCPP_ERROR(this->get_logger(), "[0x22]Vital sensor update failed");
			is_calibrating = true;
			break;
		case 35:
			RCLCPP_ERROR(this->get_logger(), "[0x23]Vital sensor not connected");
			is_calibrating = true;
			break;
		case 36:
			RCLCPP_INFO(this->get_logger(), "[0x24]Resetting data");
			for(int count = 0; count < 600; count++){
				if(count % 10 == 0){
					RCLCPP_INFO(this->get_logger(), "Resetting...");
				}
			}
			is_calibrating = false;
			break;
		case 37:
			RCLCPP_ERROR(this->get_logger(), "[0x25]No input signal");
			is_calibrating = true;
			break;
		case 255:
			RCLCPP_ERROR(this->get_logger(), "[0xFF]Unknown error");
			is_calibrating = true;
			break;

		default:
			break;
		}
		// std::this_thread::sleep_for(std::chrono::seconds(10));
    }

	// prosess_sensor_data()をループで回す
    void process_sensor_data(){
        if(is_calibrating){
            check_calibration_status();
            return;
        }
		std::this_thread::sleep_for(std::chrono::seconds(10));	// 10秒待機
        request_sensor_data();
        read_sensor_data();
    }

    void request_sensor_data(){
        if(serial_fd < 0) return;

        std::vector<std::vector<uint8_t>> request = {
            {0xAA, 0xC1, 0x0A, 0x00, 0x20, 0x55},
            {0xAA, 0xC1, 0x0B, 0x00, 0x20, 0x55},
            {0xAA, 0xC1, 0x0C, 0x00, 0x20, 0x55},
            {0xAA, 0xC1, 0x0C, 0x00, 0x21, 0x55},
            {0xAA, 0xC1, 0x0B, 0x00, 0x21, 0x55},
            {0xAA, 0xC1, 0x0A, 0x00, 0x21, 0x55},
        };

        for(auto &cmd : request){
            write(serial_fd, cmd.data(), cmd.size());
            usleep(100000);
        }
    }

    void read_sensor_data(){
        unsigned char rxData[BUFSIZE] = {};
        int len = read(serial_fd, rxData, BUFSIZE);
        if(len <= 0) return;

        std::vector<int> vec(rxData, rxData + len);
        if(vec.size() < 12) return;

        int board_id = vec[2];

        if(board_id == 10 || board_id == 11 || board_id == 12){
            double spo2 = (double)(vec[6] * 256 + vec[5]) * 0.1;
            int pulse = vec[4];
            int blood_pressure_max = vec[7];
            int blood_pressure_min = vec[8];

            if(vec[1] == 197){
                auto message = std_msgs::msg::Int32MultiArray();
                message.data = {pulse, static_cast<int>(spo2), blood_pressure_max, blood_pressure_min};
                publisher_vital->publish(message);
            }
        } else if(vec[1] == 200){
            auto message = std_msgs::msg::Int32MultiArray();
            message.data.assign(vec.begin() + 2, vec.end());
            publisher_pressure->publish(message);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_vital;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_pressure;
    rclcpp::TimerBase::SharedPtr timer;
    int serial_fd;
    bool is_calibrating;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VitalPublisher>());
    rclcpp::shutdown();
    return 0;
}
