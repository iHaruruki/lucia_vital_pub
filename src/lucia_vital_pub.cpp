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
using namespace std;
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 2000000
#define BUFSIZE 16

int main(int argc, char *argv[]){

	unsigned char txData[BUFSIZE]={};
	unsigned char txData_r[BUFSIZE]={};
	int fd;

	struct termios oldtio, newtio;

	fd = open(SERIAL_PORT, O_RDWR);

	if(fd < 0){
			printf("open error");
	}

	cout << "fd:" << fd << endl;

	ioctl(fd, TCGETS, &oldtio);
	newtio.c_cflag = BAUDRATE | CS8 | CREAD;
	tcsetattr(fd, TCSANOW, &newtio);
	ioctl(fd, TCSETS, &newtio);

	while(1){

    /*バイタル測定*/
		// バイタル値を要求(ID:0x0A)
		txData[0] = 0xAA;
		txData[1] = 0xC1;
		txData[2] = 0x0A;
		txData[3] = 0x00;
		txData[4] = 0x20;
		txData[5] = 0x55;
		write(fd, txData, BUFSIZE);
		usleep(0.1*1000000);

		// バイタル値を要求(ID:0x0B)
		txData[0] = 0xAA;
		txData[1] = 0xC1;
		txData[2] = 0x0B;
		txData[3] = 0x00;
		txData[4] = 0x20;
		txData[5] = 0x55;
		write(fd, txData, BUFSIZE);
		usleep(0.1*1000000);

		// バイタル値を要求(ID:0x0C)
		txData[0] = 0xAA;
		txData[1] = 0xC1;
		txData[2] = 0x0C;
		txData[3] = 0x00;
		txData[4] = 0x20;
		txData[5] = 0x55;
		write(fd, txData, BUFSIZE);
		usleep(0.1*1000000);


    /*圧力測定*/
        // 圧力値を要求(ID:0x0C)
        txData[0] = 0xAA;
        txData[1] = 0xC1;
        txData[2] = 0x0C;
        txData[3] = 0x00;
        txData[4] = 0x21;
        txData[5] = 0x55;

        write(fd, txData, BUFSIZE);
        usleep(0.1*1000000);

        // 圧力値を要求(ID:0x0B)
        txData[0] = 0xAA;
        txData[1] = 0xC1;
        txData[2] = 0x0B;
        txData[3] = 0x00;
        txData[4] = 0x21;
        txData[5] = 0x55;

        write(fd, txData, BUFSIZE);
        usleep(0.1*1000000);

        // 圧力値を要求(ID:0x0A)
        txData[0] = 0xAA;
        txData[1] = 0xC1;
        txData[2] = 0x0A;
        txData[3] = 0x00;
        txData[4] = 0x21;
        txData[5] = 0x55;

        write(fd, txData, BUFSIZE);
        usleep(0.1*1000000);

	}
	ioctl(fd, TCSETS, &oldtio);
	close(fd);
}
