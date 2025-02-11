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

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B2000000	// 2000000bps
#define BUFSIZE 16

class VitalPublisher : public rclcpp::Node{
public:
	VitalPublisher() : Node("vital_publisher"){
		publisher_vital = this->create_publisher<std_msgs::msg::Int32MultiArray>("vital_data", 10);
		publisher_pressure = this->create_publisher<std_msgs::msg::Int32MultiArray>("pressure_data", 10);

		serial_fd = open_serial_port();

		timer = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&VitalPublisher::request_sensor_data, this));
	}

	~VitalPublisher(){
		if(serial_fd >= 0){
			close(serial_fd);
		}
	}

private:
	int open_serial_port(){
		int fd = open(SERIAL_PORT, O_RDWR);
		if(fd < 0){
			RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
			return -1;
		}

		struct termios tty;
		if(tcgetattr(fd, &tty) != 0){
			RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
			return -1;
		}

		cfsetospeed(&tty, BAUDRATE);
		cfsetispeed(&tty, BAUDRATE);
		tty.c_cflag = BAUDRATE | CS8 | CREAD;	//tty.c_cflag = BAUDRATE | CS8 | CREAD | CLOCAL;
		tcsetattr(fd, TCSANOW, &tty);

		return fd;
	}

	void request_sensor_data(){
		if(serial_fd < 0) return;

		std::vector<std::vector<uint8_t>> request = {
			/*バイタルセンサ値*/
			{0xAA, 0xC1, 0x0A, 0x00, 0x20, 0x55},	//ID: 0x0A
			{0xAA, 0xC1, 0x0B, 0x00, 0x20, 0x55},	//ID: 0x0B
			{0xAA, 0xC1, 0x0C, 0x00, 0x20, 0x55},	//ID: 0x0C
			/*圧力センサ値*/
			{0xAA, 0xC1, 0x0C, 0x00, 0x21, 0x55},	//ID: 0x0C
			{0xAA, 0xC1, 0x0B, 0x00, 0x21, 0x55},	//ID: 0x0B
			{0xAA, 0xC1, 0x0A, 0x00, 0x21, 0x55},	//ID: 0x0A
		};

		for(auto &cmd : request){
			write(serial_fd, cmd.data(), cmd.size());
			usleep(0.1*1000000);
			read_sensor_data();
		}
	}

	// 修正の必要あり
	void read_sensor_data(){
        unsigned char rxData[BUFSIZE] = {};
        int len = read(serial_fd, rxData, BUFSIZE);
        RCLCPP_INFO(this->get_logger(), "Read %d bytes", len);

        if(len <= 0){
            RCLCPP_ERROR(this->get_logger(), "No data received");
            return;
        }

        std::vector<int> vec(rxData, rxData + len);

		for(int i = 0; i < len;){
			// パケットの先頭(0xAA)を検出
			if(rxData[i] == 0xAA){
				vec.clear();
				vec.push_back(rxData[i]);
				i++;
				// パケットの終端(0x55)を検出・格納
				while(i < len && rxData[i] != 0x55){
					vec.push_back(rxData[i]);
					i++;
				}
				// vec[]に格納
				if(i < len){
					vec.push_back(rxData[i]);
				}
			}
		}

        if (vec.size() < 12) {
            RCLCPP_WARN(this->get_logger(), "Invalid packet received");
            return;
        }

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
                RCLCPP_INFO(this->get_logger(), "Vital [ID: %d] Pulse: %d, spo2: %.lf, BP max/min: %d/%d",
                            board_id, pulse, spo2, blood_pressure_max, blood_pressure_min);
            }
        }
        else if(vec[1] == 200){
            auto message = std_msgs::msg::Int32MultiArray();
            message.data.assign(vec.begin() + 2, vec.end());
            publisher_pressure->publish(message);
            RCLCPP_INFO(this->get_logger(), "Pressure Data [ID=%d]: %d, %d, %d, %d, %d, %d, %d, %d",
                        vec[2], vec[4], vec[5], vec[6], vec[7], vec[8], vec[9], vec[10], vec[11]);
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Unknown board ID: %d", board_id);
        }
    }

	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_vital;
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_pressure;
	rclcpp::TimerBase::SharedPtr timer;
	int serial_fd;
};

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VitalPublisher>());
	rclcpp::shutdown();
	return 0;
}
