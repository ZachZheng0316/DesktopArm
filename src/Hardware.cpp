#include "Hardware.h"
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "RobotisDef.h"

static int fd_led_ph15 = 0;
static int fd_led_ph16 = 0;
static char value_led[] ={'0','1'};

//打开端口成功，返回1；
//打开端口失败，返回0。
int Hardware_open_port()
{
	fd_led_ph15 = open(THE_DEVICE15, O_RDWR);
	if(fd_led_ph15 < 0){
		printf("%s: %d: open fd_f15 failed !\n ", __FILE__, __LINE__);
		return 0;
	}

	fd_led_ph16 = open(THE_DEVICE16, O_RDWR);
	if(fd_led_ph16 < 0){
		printf("%s: %d: open fd_f16 failed !\n ", __FILE__, __LINE__);
		return 0;
	}

	return 1;
}

//打开泵成功，返回1
//打开泵失败，返回0
int Hardware_open_beng()
{
	int ret;

	//打开气泵
	ret = write(fd_led_ph15, &value_led[0], sizeof(value_led[0]));
	if(ret < 0) {
		printf("%s: %d: send fd_ph15 fail\n", __FILE__, __LINE__);
		return 0;
	}

	//关闭电磁阀
	ret = write(fd_led_ph16, &value_led[1], sizeof(value_led[1]));
	if(ret < 0) {
		printf("%s: %d: send fd_ph16 fail\n", __FILE__, __LINE__);
		return 0;
	}

	return 1;
}

//关闭泵成功，返回1
//关闭泵失败，返回0
int Hardware_close_beng()
{
	int ret;

	//关闭气泵
	ret = write(fd_led_ph15, &value_led[1], sizeof(value_led[1]));
	if(ret < 0) {
		printf("%s: %d: send fd_ph15 fail\n", __FILE__, __LINE__);
		return 0;
	}

	//打开电磁阀
	ret = write(fd_led_ph16, &value_led[0], sizeof(value_led[0]));
	if(ret < 0) {
		printf("%s: %d: send fd_ph16 fail\n", __FILE__, __LINE__);
		return 0;
	}

	return 1;
}

//往指定的gpio口写入数据指定的数据
//如果写入成功，返回1
//如果写入失败，返回0
/*********************************************
函数意义：
	往指定的gpio口写入指定的数据
参数意义：
	num：16：代表gpio口16
		 15：代表gpio口15
	value:字符'0'：代表关闭gpio口
		  字符'1'：代表开启gpio口
返回值：
	如果写入成功，返回1
	如果写入失败，返回0
*********************************************/
int Hardware_write_gpio(int num, UINT8_T value)
{
	int ret;

	if(15 == num)
		ret = write(fd_led_ph15, &value, 1);
	else if(16 == num)
		ret = write(fd_led_ph16, &value, 1);
	else
		return 0;
	return 1;
}

int Hardware_close_port()
{
	close(fd_led_ph15);
	close(fd_led_ph16);

	return 0;
}
