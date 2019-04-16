/**
 *serial_test
 *测试的时候应用程序在后台运行./serial_test &
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h>//终端控制定义
#include <errno.h>
#include <ctype.h>

#include <cutils/log.h>
#define TAG "serial_tag"

#define DEVICE "/dev/ttyS3"

static int serial_fd = 0;

//打开串口并初始化设置
int init_serial(char *dev_path)
{
    serial_fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        ALOGE("open error");
        return -1;
    }

    //串口主要设置结构体termios <termios.h>
    struct termios options;

    /**1. tcgetattr函数用于获取与终端相关的参数.
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    tcgetattr(serial_fd, &options);
    /**2. 修改所获得的参数*/
    options.c_cflag |= (CLOCAL | CREAD);	//设置控制模式状态，本地连接，接收使能
    options.c_cflag &= ~CSIZE;				//字符长度，设置数据位之前一定要屏掉这个位
    options.c_cflag &= ~CRTSCTS;			//无硬件流控
    options.c_cflag |= CS8;					//8位数据长度
    options.c_cflag &= ~CSTOPB;				//1位停止位
    options.c_iflag |= IGNPAR;				//无奇偶检验位
    options.c_oflag = 0;					//输出模式
    options.c_lflag = 0;					//不激活终端模式
    cfsetospeed(&options, B115200);			//设置波特率
    ///cfsetospeed(&options, B9600);			//设置波特率

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(serial_fd, TCIFLUSH);			//溢出数据可以接收，但不读
    tcsetattr(serial_fd, TCSANOW, &options);

    return 0;
}

/**
 *串口发送数据
 *@fd:串口描述符
 *@data:待发送数据
 *@datalen:数据长度
 */
int uart_send(int fd, char *data, int datalen)
{
    int len = 0;
    len = write(fd, data, datalen);	//实际写入的长度
    if (len == datalen) {
		ALOGE("uart send:%s\n", data);
        return len;
    } else {
        tcflush(fd, TCOFLUSH);		//TCOFLUSH刷新写入的数据但不传送
        return -1;
    }

    return 0;
}

/**
 *串口接收数据
 *要求启动后，在pc端发送ascii文件
 */
int uart_recv(int fd, char *data, int datalen)
{
    int len = 0;
    int ret = 0;
    fd_set fs_read;
    struct timeval tv_timeout;
	int i = 0;

#if 1
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);
    tv_timeout.tv_sec  = (10*20/115200+2);
    tv_timeout.tv_usec = 0;

    ret = select(fd+1, &fs_read, NULL, NULL, &tv_timeout);
    ALOGE("%s --- ret = %d\n", TAG, ret);
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1

    if (FD_ISSET(fd, &fs_read)) {
		usleep(50000);
        len = read(fd, data, datalen);
        ALOGE("%s ---------- len = %d\n", __func__, len);

		for (i=0; i<len; i++)
			ALOGE("%s ---------- data[%d] = 0x%x\n", __func__, i, data[i]);

        return len;
    } else {
        ALOGE("select error 11");
        return -1;
    }
    return 0;
#else
	len = read(fd, data, datalen);
	ALOGE("%s ---------- len = %d\n", __func__, len);
	return len;
#endif
}

unsigned char StrToHex(char *pbSrc)
{
	unsigned char pbDest = 0;
	unsigned char h1 = 0, h2 = 0;
	unsigned char s1 = 0, s2 = 0;

	ALOGE("StrToHex -------------------------- pbSrc=%s\n", pbSrc);
	if (strlen(pbSrc) == 3) {
		h1 = 0x30;
		h2 = *(pbSrc+2) & 0xff;
	} else if (strlen(pbSrc) == 4) {
		h1 = pbSrc[2];
		h2 = pbSrc[3];
	}

	s1 = toupper(h1) - 0x30;
	if (s1 > 9)
		s1 -= 7;

	s2 = toupper(h2) - 0x30;
	if (s2 > 9)
		s2 -= 7;

	ALOGE("StrToHex -------------------------- s1=%x\n", s1);
	ALOGE("StrToHex -------------------------- s2=%x\n", s2);

	pbDest = s1*16 + s2;

	return pbDest;
}

int main(int argc, char **argv)
{
	int ret = 0;

	if (argc < 3)
		return -1;

	ret = init_serial(argv[1]);
	if (ret < 0) {
		ALOGE("Init uart failed\n");
		return -1;
	}

	while (1) {
		if (0 == strcmp(argv[2], "read")) {
			char recv_buf[1024] = {0};

			uart_recv(serial_fd, recv_buf, 1024);
			ALOGE("uart receive %s\n", recv_buf);
		} else if (0 == strcmp(argv[2], "write")) {
			if (argc == 3) {
				char *send_buf = "hello world\n";
				uart_send(serial_fd, send_buf, strlen(send_buf));
			} else if (argc >= 4) {
				char send_buf[32] = {0};

				int i = 0;
				for (i=3; i<argc; i++)
				{
					strcat(send_buf, argv[i]);
					strcat(send_buf, " ");
				}

				send_buf[strlen(send_buf)] = '\n';
				uart_send(serial_fd, send_buf, strlen(send_buf));
				break;
			}
		} else if (0 == strcmp(argv[2], "test")) {
			ALOGE("uart ------------------------------------ test start argc=%d\n", argc);
			unsigned char send_buf[32] = {0};
			unsigned char hex_dest = 0;
			int count = 0;

			for (count=0; count<argc-3; count++) {
				hex_dest = StrToHex(argv[count+3]);

				ALOGE("uart ------------------------------------ test start argv[count+3]=%s hex_dest=%x\n", argv[count+3], hex_dest);

				send_buf[count] = hex_dest;
			}

			/*
			send_buf[0] = 0xAA;
			send_buf[1] = 0xAA;
			send_buf[2] = 0xAA;
			send_buf[3] = 0x96;
			send_buf[4] = 0x69;
			send_buf[5] = 0x00;
			send_buf[6] = 0x04;
			send_buf[7] = 0x61;
			send_buf[8] = 0xFF;
			send_buf[9] = 0x90;
			send_buf[10] = 0x0A;
			*/
			send_buf[count] = '\n';
			uart_send(serial_fd, send_buf, strlen(send_buf)-1);
			ALOGE("uart ------------------------------------ test end\n");
			break;
		}
		sleep(1);
	}

	ALOGE("Close uart device.");
	close(serial_fd);

	return 0;
}

