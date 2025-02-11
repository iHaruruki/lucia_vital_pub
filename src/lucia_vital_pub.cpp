#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
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
		publisher_vital = this->create_publisher<std_msgs::msg::Int32>("vital_data", 10);
		publisher_pressure = this->create_publisher<std_msgs::msg::Int32>("pressure_data", 10);

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
			//read_sensor_data();
		}
	}

	// 修正の必要あり
	void read_sensor_data(){
		uint8_t rxData[BUFSIZE] = {};
		int n = read(serial_fd, rxData, BUFSIZE);
		if(n > 0){
			int sensor_value = rxData[2];
			auto message = std_msgs::msg::Int32();
			message.data = sensor_value;

			if(rxData[3] == 0x0A){
				publisher_vital->publish(message);
			}
			else if(rxData[3] == 0x0B){
				publisher_pressure->publish(message);
			}
		}
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_vital;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_pressure;
	rclcpp::TimerBase::SharedPtr timer;
	int serial_fd;
};

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VitalPublisher>());
	rclcpp::shutdown();
	return 0;
}
