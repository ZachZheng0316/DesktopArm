#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include "Dmath.h"
#include "AX12A.h"
#include "ProtocolV3.h"
#include "LampsControl.h"
#include "RobotisDef.h"

using namespace ROBOTIS;

#define SERVO_NUM   (10)   //Servo个数
#define FIVEARMID   (0x41) //机械臂ID
#define LEDID       (0xC8) //LEDID

char SERVODEVICE[] = "/dev/ttyUSB0";//舵机串口号
int SERVOBAUD = 1000000;            //舵机串口波特率
char LEDDEVICE[] = "/dev/ttyUSB1";  //led串口号
int LEDBAUD = 115200;               //led串口波特率
char PCDEVICE[] = "/dev/ttyS5";     //pc的串口
int PCBAUD = 9600;                  //pc的波特率
int CURLARM[5]; //存储左臂最新姿态
int CURRARM[5]; //存储右臂最新姿态
int CURKEYNUM;  //指示当前顺序键序
int CURLNUM;    //存储左臂最新位置
int CURRNUM;    //存储右臂最新位置
int LKeyPos[10][5]; //存储左边键值
int RKeyPos[10][5]; //存储右边键值
char LKEYPOSPATH[] = "LKeyPos.txt"; //左边琴键的姿态
char RKEYPOSPATH[] = "RKeyPos.txt"; //右边琴键的姿态
char SONGNUM = 0x00;   //存储曲目信息
bool SONGSTOP = false; //存储曲目终止信息
bool ISSTART = true;   //
ProtocolV3 *pProtoV3 = NULL; //协议V3
LampsControl *pLed = NULL;   //LED控制类
AX12A *pAX12 = NULL;         //AX12舵机控制类
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;  // 互斥锁
pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER; // 互斥锁

//主要函数
bool initial_sys();
int set_default_pos();
int set_led_mode(char _flag);
int get_user_input();
bool set_led_light(int _ledNum);
bool get_led_statue(int _ledNum);
int play_music(UINT8_T _songNum);
int get_hands_num(int hans[]);
void send_rt_hands();
void clear_sys();

//细节函数
bool read_key_pos();
int set_enable_relax(int _flag);
int wait_arm_stop_exten(char _flag, int _exten);
int hit_and_hit(int num, bool _isHit);
int from_A_to_B_order(int num1, int num2);
int from_A_to_B_order1(int num1, int num2, int lastChoose);
int set_arm_bytes(int _byteNum, char dir, int addr, int *value);

//多线程
void* conmmunicateThread(void* arg); //通信线程
void* actionProcesThread(void* arg); //运动流程线程

int main(void)
{
    int ret1, ret2;
    pthread_t t_commu, t_action;

    if(!initial_sys()) {
        printf("%s: %d: initial_sys is failed\n", __FILE__, __LINE__);
        clear_sys();
        getchar();
        return 0;
    }

    //创建线程
    ret1 = pthread_create(&t_commu, NULL, conmmunicateThread, NULL);
    ret2 = pthread_create(&t_action, NULL, actionProcesThread, NULL);
    if((0 != ret1) || (0 != ret2)) {
        printf("%s: %d: Create thread failed ret1 or ret2\n", __FILE__, __LINE__);
        UINT8_T sdValue[2];
        sdValue[0] = FIVEARMID;
        sdValue[1] = OTHER_ERROR;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return 0;
    }

    //等待线程结束
    pthread_join(t_commu,NULL);
    pthread_join(t_action, NULL);
    pthread_mutex_destroy(&mutex);

    clear_sys();
    return 1;
}

/********************
函数意义：
    初始化系统
    1、打开与一体机的通信端口
    2、打开灯具串口
    3、打开机械臂串口
    4、舵机Slope初始化
    5、设置舵机舵机Eanble
    6、读取琴键位置信息
参数意义：
    无参数
返回值：
    执行成功，返回true
    执行失败，返回false;
*********************/
bool initial_sys()
{
    int i, result1, result2, result;
    UINT8_T sdValue[2];

    sdValue[0] = FIVEARMID; //机械臂ID

    //初始化机械臂程序
    printf("\n\n%s: %d: initial system\n\n", __FILE__, __LINE__);

    //打开与一体机连接的串口
    pProtoV3 = new ProtocolV3(PCDEVICE, PCBAUD);
    if(!pProtoV3->isSuccessInit()) {
        printf("%s: %d: ProtocolV3 open failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: open serial success\n", __FILE__, __LINE__);

    //初始化LED Port
    pLed = new LampsControl(LEDDEVICE, LEDBAUD);
    if(!pLed->SuccessOpenDevice()) {
        printf("%s: %d: failed to open LedPort\n", __FILE__, __LINE__);
        sdValue[1] = LEDPORT_ERROR;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    else
        printf("%s: %d: success to open LampsControl\n", __FILE__, __LINE__);

    //初始化机械臂
    pAX12 = new AX12A(SERVODEVICE, SERVOBAUD, SERVO_NUM);
    if(!pAX12->SuccessOpenDevice()) {
        printf("%s: %d: failed to open AX12A\n", __FILE__, __LINE__);
        sdValue[1] = (UINT8_T)SERVOPORT_ERROR;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    else
        printf("%s: %d: success to open AX12A\n", __FILE__, __LINE__);

    //初始化舵机的Slope
    for(i = 0; i < SERVO_NUM; i++) {
        result1 = pAX12->set_one_servo_bytes(1, i+1, CW_Slope, 64);
        result2 = pAX12->set_one_servo_bytes(1, i+1, CCW_Slope, 64);
        if(result1 || result2) {
            result = result1 > 0 ?result1 : result2;
            printf("%s: %d: abnormal(%d) set one servo byte failed\n", __FILE__, __LINE__, result);
            sdValue[1] = (UINT8_T)result;
            pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
            return false;
        }
    }
    result1 = pAX12->set_one_servo_bytes(1, 2, CW_Slope, 32);
    result2 = pAX12->set_one_servo_bytes(1, 2, CCW_Slope, 32);
    if(result1 || result2) {
        result = result1 > 0 ?result1 : result2;
        printf("%s: %d: abnormal(%d) set one servo byte failed\n", __FILE__, __LINE__, result);
        sdValue[1] = (UINT8_T)result;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    result1 = pAX12->set_one_servo_bytes(1, 7, CW_Slope, 32);
    result2 = pAX12->set_one_servo_bytes(1, 7, CCW_Slope, 32);
    if(result1 || result2) {
        result = result1 > 0 ?result1 : result2;
        printf("%s: %d: abnormal(%d) set one servo byte failed\n", __FILE__, __LINE__, result);
        sdValue[1] = (UINT8_T)result;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    result1 = pAX12->set_one_servo_bytes(1, 5, CW_Slope, 64);
    result2 = pAX12->set_one_servo_bytes(1, 5, CCW_Slope, 64);
    if(result1 || result2) {
        result = result1 > 0 ?result1 : result2;
        printf("%s: %d: abnormal(%d) set one servo byte failed\n", __FILE__, __LINE__, result);
        sdValue[1] = (UINT8_T)result;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    result1 = pAX12->set_one_servo_bytes(1, 10, CW_Slope, 64);
    result2 = pAX12->set_one_servo_bytes(1, 10, CCW_Slope, 64);
    if(result1 || result2) {
        result = result1 > 0 ?result1 : result2;
        printf("%s: %d: abnormal(%d) set one servo byte failed\n", __FILE__, __LINE__, result);
        sdValue[1] = (UINT8_T)result;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }

    //读取琴键位置文件
    if(!read_key_pos()) {
        printf("%s: %d: read key pos failed\n", __FILE__, __LINE__);
        sdValue[1] = FILE_NO;
        pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
        return false;
    }
    else
        printf("%s: %d: read key pos succes\n", __FILE__, __LINE__);

    return true;
}

/**********************
函数意义：
    设置机械臂到初始姿态。
    步骤：1、上刚度；2、运动到目标位置
参数意义：
    无参数
返回值：
    当执行成功时，返回0。
    当执行失败时，返回错误代码。
***********************/
int set_default_pos()
{
    int result, result1, result2;
    int speK = 100;

    //上紧刚度
    result = set_enable_relax(1);
    if(result > 0)
        return result;

    //设置速度
    for(int i = 1; i <= SERVO_NUM; i++) {
        result = pAX12->set_one_servo_bytes(2, i, Moving_Speed, speK);
        if(result > 0)
            return result;
    }

    //设置目标位置
    result1 = pAX12->set_one_servo_bytes(2, 5, Goal_Position, RKeyPos[0][4]);
    result2 = pAX12->set_one_servo_bytes(2, 10, Goal_Position, LKeyPos[0][4]);
    if(result1 || result2) {
       result = result1 > 0 ? result1 : result2;
       printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
       return result;
   }
   delay_ms(1000.0);
   for(int i = 0; i < 4; i++) {
       result1 = pAX12->set_one_servo_bytes(2, i+1, Goal_Position, RKeyPos[0][i]);
       result2 = pAX12->set_one_servo_bytes(2, i+6, Goal_Position, LKeyPos[0][i]);
       if(result1 || result2) {
          result = result1 > 0 ? result1 : result2;
          printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
          return result;
      }
      delay_ms(20.0);
   }

   //等待运动停止
   result1 = wait_arm_stop_exten('L', 20);
   result2 = wait_arm_stop_exten('R', 20);
   if(result1 || result2) {
       result = result1 > 0 ? result1 : result2;
       printf("%s: %d: wait arm stop exten failed\n", __FILE__, __LINE__);
       return result;
   }
   delay_ms(500);

   //更新当前姿态
   for(int i = 0; i < 5; i++) {
       CURLARM[i] = LKeyPos[0][i];
       CURRARM[i] = RKeyPos[0][i];
   }

   //设置当前位置
   CURKEYNUM = 5;
   CURLNUM = 5;
   CURRNUM = 14;

   return 0;
}

/***************************
函数意义：
    设置LED的模式
参数意义：
    _flag：1：表示菜单模式
           2：表示演奏模式
返回值：
    如果执行成功，返回0;
    如果执行失败，返回返回错误代码;
****************************/
int set_led_mode(char _flag)
{
    UINT8_T value[8];
    int _abnormal;

    //设置控制数据
    value[0] = 0x01; //模式选择
    value[1] = 0x01; //LED控制权
    value[2] = 0x05; //每个键LED数目
    value[3] = 0x32; //演奏键超时
    value[4] = 0x01; //对键值反馈的清零
    value[5] = 0x00; //目标简直1-8
    value[6] = 0x00; //目标简直9-16
    value[7] = 0x00; //目标简直17-27

    if(1 == _flag)
        value[0] = 0x01; //设置菜单模式
    else if(2 == _flag)
        value[0] = 0x02; //设置演奏模式
    else
        return OTHER_ERROR;

    pthread_mutex_lock(&mutex);
    _abnormal = pLed->set_Lamps_bytes(LEDID, ADDR_MODE, value, 8);
    pthread_mutex_unlock(&mutex);

    return _abnormal;
}

/**************************
函数意义：
    获取用户的选择信息.
    假设没有任何输入，执行detectNum次检测就退出函数。
    如果检测到输入，则返回选择结果。
参数意义：
    无参数
返回值：
    如果执行成功：
        0x00:表示没有输入
        0x01:表示返回信息
        0x02:表示向左移动
        0x03:表示向右移动
        0x04:表示确认信息
    如果执行失败，返回错误代码(负数表示)
***************************/
int get_user_input()
{
    UINT8_T moveIncre1, moveIncre2, leftPut, rightPut;
    int result, diff, rang = 17;
    bool isFirst = true;
    int detectNum = 100;

    while(detectNum >= 0) {
        //获取LED地址为0x46的值
        result = pLed->get_Lamps_bytes(LEDID, ADDR_MOVEADDVALUE, 4);
        if(result) {
            printf("%s: %d: get lamps bytes failed\n", __FILE__, __LINE__);
            return -abs(result);
        }
        if(isFirst) {
            moveIncre1 = pLed->read_Lamps_StatusPacket(0);
            moveIncre2 = moveIncre1;
            isFirst = false;
        }
        else {
            moveIncre2 = pLed->read_Lamps_StatusPacket(0);
            diff = moveIncre2 - moveIncre1;

            leftPut = pLed->read_Lamps_StatusPacket(2);
            rightPut = pLed->read_Lamps_StatusPacket(3);

            if((-rang <= diff) && (diff < 0-2)) {
                //当增量只有<=2时，不算移动
                return 0x02; //左移动
            }
            else if((diff <= 255-2) && (diff >= 255 - rang))
                return 0x02; //左移动
            else if((diff <= rang) && (diff > 0+2))
                return 0x03; //右移动
            else if((diff <= rang - 255) && (diff >= 2-255))
                return 0x03; //右移动
            else if(leftPut)
                return 0x01;
            else if(rightPut)
                return 0x04;
            else{}
        }
        detectNum -= 1;
    }
    return 0x00;
}

/******************************
函数意义：
    设置琴键高亮。
    由于LED排灯顺序与琴键的顺序相反
    所以，要把琴键的序号转换为LED排灯
    的序号。
参数意义：
    _ledNum:琴键序号。需要转化为排灯
    的序号：_ledNum=19-_ledNum
返回值：
    如果执行成功，返回true；
    如果执行失败，返回false;
******************************/
bool set_led_light(int _ledNum)
{
    UINT8_T value[8];
    int _abnormal;

    _ledNum = 19 - _ledNum;

    //设置控制数据
    value[0] = 0x02; //模式选择:演奏模式
    value[1] = 0x01; //LED控制权
    value[2] = 0x05; //每个键LED数目
    value[3] = 0x32; //演奏键超时
    value[4] = 0x01; //对键值反馈的清零
    value[5] = 0x00; //目标简直1-8
    value[6] = 0x00; //目标简直9-16
    value[7] = 0x00; //目标简直17-24
    if((1 <= _ledNum)&&(_ledNum <= 8))
        value[5] = (UINT8_T)pow(2, _ledNum-1);
    else if((9 <= _ledNum) && (_ledNum <= 16))
        value[6] = (UINT8_T)pow(2, _ledNum-9);
    else if((17<= _ledNum) && (_ledNum <= 18))
        value[7] = (UINT8_T)pow(2, _ledNum-17);
    else{
        printf("%s: %d: _ledNum(%d) is out of range\n", __FILE__, __LINE__, _ledNum);
        return false;
    }

    _abnormal = pLed->set_Lamps_bytes(LEDID, ADDR_MODE, value, 8);
    if(_abnormal) {
        printf("%s: %d: set lamps bytes failed\n", __FILE__, __LINE__);
        return false;
    }

    return true;
}

/*********************************
函数意义：
    获取指定琴键对应的led灯的状态信息。
    由于，LED排灯顺序与琴键顺序是相反的
    所以，要把琴键顺序转化为排灯顺序。
    _ledNum = 19 - _ledNum;
参数意义：
    _ledNum:琴键的序号。
返回值：
    如果用户击打成功，返回true;
    如果用户击打失败，返回false;
**********************************/
bool get_led_statue(int _ledNum)
{
    UINT8_T value1, value2, value3;
    UINT8_T value;
    int result;

    _ledNum = 19 - _ledNum;

    //获取LED地址为0x43的值
    pthread_mutex_lock(&mutex);
    result = pLed->get_Lamps_bytes(LEDID, ADDR_PCKEYVALUE1, 3);
    if(result) {
        printf("%s: %d: get lamps bytes failed\n", __FILE__, __LINE__);
        return false;
    }
    value1 = pLed->read_Lamps_StatusPacket(0);
    value2 = pLed->read_Lamps_StatusPacket(1);
    value3 = pLed->read_Lamps_StatusPacket(2);
    pthread_mutex_unlock(&mutex);
    printf("%s: %d: get_led_statue _ledNum(%d) value1(%x) value2(%x) value3(%x)\n", __FILE__, __LINE__, _ledNum, value1, value2, value3);

    if((1 <= _ledNum)&&(_ledNum <= 8)) {
        value = (UINT8_T)pow(2, _ledNum-1);
        if(value == (value1 & value))
            return true;
    }
    else if((9 <= _ledNum) && (_ledNum <= 16)) {
        value = (UINT8_T)pow(2, _ledNum-9);
        if(value == (value2 & value))
            return true;
    }
    else if((17<= _ledNum) && (_ledNum <= 18)) {
        value = (UINT8_T)pow(2, _ledNum-17);
        if(value == (value3 & value))
            return true;
    }
    else{
        printf("%s: %d: _ledNum is out of range\n", __FILE__, __LINE__);
        return false;
    }

    return false;
}

/******************************
函数意义：
    执行曲目
参数意义：
    musicNum:曲目编号
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
******************************/
int play_music(UINT8_T _songNum)
{
    FILE *fpR = NULL;
    bool _songStop = false, firstKey = true, notEnd = true;
    UINT8_T sdValue[4];
    char path[20] = {0, };
    int start, nextKey, end, result1, result2, result;
    double delay1, nextdelay, delay2, tmRate = 1.5;
    long delay;

    // 机械臂上紧刚度
    result = set_enable_relax(1);
    if(result) {
        printf("%s: %d: set_enable_relax failed\n", __FILE__, __LINE__);
        return result;
    }
    delay_ms(1000.0);

    // 打开曲目文件
    sdValue[0] = FIVEARMID;
    sprintf(path, "songs/%d.txt", (int)_songNum);
    fpR = fopen(path, "r");
    if(!fpR) {
        printf("%s: %d: open %s failed\n", __FILE__, __LINE__, path);
        return FILE_NO;
    }

    //读取琴键的第一个位置
    if(EOF == fscanf(fpR, "%d %lf\n", &start, &delay1)) {
        printf("%s: %d: read file %s failed\n", __FILE__, __LINE__, path);
        return FILE_NO;
    }

    while(notEnd) {
        // 读取第二个琴键的位置
        if(!feof(fpR)) {
            if(EOF == fscanf(fpR, "%d %lf\n", &nextKey, &nextdelay)) {
                printf("%s: %d: read fiel %s faield\n", __FILE__, __LINE__, path);
                return FILE_NO;
            }
        }
        else {
            // 表示曲目结束
            notEnd = false;
        }

        // 机械臂从当前位置运动到start的位置
        if(firstKey) {
            // 发送start和nextKey位置消息给主机pc
            sdValue[1] = 0;
            sdValue[2] = start;
            printf("%s: %d: send pre and next key pos(%x, %x)\n", __FILE__, __LINE__, sdValue[1], sdValue[2]);
            pthread_mutex_lock(&mutex);
            pProtoV3->sendMessage(0x40, 0x86, 2, sdValue);
            pthread_mutex_unlock(&mutex);

            // 点亮琴键start
            pthread_mutex_lock(&mutex);
            set_led_light(start);
            pthread_mutex_unlock(&mutex);
            delay_us(1000 * 1000);

            printf("%s: %d: first start(%d) lastChoose(1)\n", __FILE__, __LINE__, start);
            //getchar();
            //getchar();

            // 运动到指定的琴键
            if(start <= 9) {
                result1 = set_arm_bytes(2, 'L', Goal_Position, LKeyPos[start]);
                result2 =  wait_arm_stop_exten('L', 15);
                if(result1 || result2) {
                    result = result1 > 0 ? result1 : result2;
                    printf("%s: %d: set arm and wait arm stop failed\n", __FILE__, __LINE__);
                    return result;
                }
            }
            else{
                result1 = set_arm_bytes(2, 'R', Goal_Position, RKeyPos[start - 9]);
                result2 = wait_arm_stop_exten('R', 15);
                if(result1 || result2) {
                    result = result1 > 0 ? result1 : result2;
                    printf("%s: %d: set arm and wait arm stop failed\n", __FILE__, __LINE__);
                    return result;
                }
            }
            CURKEYNUM = start;
            firstKey = false;
            //send_rt_hands(); // 发送hands的实时数据

            // 获取用户是否覆盖start位置
            // 根据用户覆盖信息选择是否敲击start的琴键
            sdValue[1] = (UINT8_T)start;
            bool _statue = get_led_statue(start);
            printf("%s: %d: get led statue ledNum(%d) statue(%d)\n", __FILE__, __LINE__, start, (int)_statue);
            pthread_mutex_lock(&mutex);
            result = hit_and_hit(start, _statue);
            pthread_mutex_unlock(&mutex);
            if(result) {
                printf("%s: %d: hit and hit failed\n", __FILE__, __LINE__);
                return result;
            }

            // 给主机PC发送敲击的结果
            if(_statue) {
                // 用户击中琴键
                sdValue[2] = 0x01;
                pthread_mutex_lock(&mutex);  //加锁
                pProtoV3->sendMessage(0x40, 0x83, 2, sdValue);
                pthread_mutex_unlock(&mutex);//解锁
            }
            else{
                // 用户没有击中琴键
                sdValue[2] = 0x00;
                pthread_mutex_lock(&mutex);  //加锁
                pProtoV3->sendMessage(0x40, 0x83, 2, sdValue);
                pthread_mutex_unlock(&mutex);//解锁
            }
        }
        else {
            // 判断end是不是等待符号
            if(0 == end) {
        	    delay = (long)(delay2 * tmRate * 80.0 / 0.25);
                delay_us(delay*1000);
            }
            else {
                //设置琴键高亮
                pthread_mutex_lock(&mutex);
                set_led_light(end);
                pthread_mutex_unlock(&mutex);

                //移动到目标位置
                result = from_A_to_B_order1(start, end, 1);
                if(result) {
                    printf("%s: %d: from A to B failed\n", __FILE__, __LINE__);
                    return result;
                }

                // 发送下一个要敲击的位置
                sdValue[1] = start;
                sdValue[2] = end;
                printf("%s: %d: send pre-key and next-key pos(%x, %x)\n", __FILE__, __LINE__, sdValue[1], sdValue[2]);
                pthread_mutex_lock(&mutex);
                pProtoV3->sendMessage(0x40, 0x86, 2, sdValue);
                pthread_mutex_unlock(&mutex);

                //延时delay1
                delay = (long)(delay1 * tmRate * 80.0 / 0.25);
                delay_us(delay*1000);

                // 获取用户是否覆盖的信息
                // 根据用户是否覆盖的信息决定是否打击琴键
                sdValue[1] = (UINT8_T)end;
                bool _statue = get_led_statue(end);
                printf("%s: %d: get led statue ledNum(%d) statue(%d)\n", __FILE__, __LINE__, end, (int)_statue);
                pthread_mutex_lock(&mutex);
                result = hit_and_hit(end, _statue);
                pthread_mutex_unlock(&mutex);//解锁
                if(result) {
                    printf("%s: %d: hit and hit failed\n", __FILE__, __LINE__);
                    return result;
                }

                // 给主机PC发送敲击结果
                if(_statue) {
                    //击中琴键
                    sdValue[2] = 0x01;
                    pthread_mutex_lock(&mutex);  //加锁
                    pProtoV3->sendMessage(0x40, 0x83, 2, sdValue);
                    pthread_mutex_unlock(&mutex);//解锁
                }
                else{
                    //没有击中琴键
                    sdValue[2] = 0x00;
                    pthread_mutex_lock(&mutex);  //加锁
                    pProtoV3->sendMessage(0x40, 0x83, 2, sdValue);
                    pthread_mutex_unlock(&mutex);//解锁
                }

                // 更新start, 更新delay1
                start = end;
                delay1 = delay2;
                CURKEYNUM = end;
            }
        }

        //判断是否受到曲目终止消息
        pthread_mutex_lock(&mutex);  //加锁
        _songStop = SONGSTOP;
        pthread_mutex_unlock(&mutex);//解锁
        //如果是曲目终止，则停止并推出演奏
        if(_songStop)
            break;

        if(notEnd) {
            // 更新end = nextKey; delay2 = nextDelay
            end = nextKey;
            delay2 = nextdelay;
        }
    }

    // 回到初始位置
    result = from_A_to_B_order1(CURKEYNUM, 14, 0);
    if(result) {
        printf("%s: %d: from A to from B faile\n", __FILE__, __LINE__);
        return result;
    }
    CURKEYNUM = 14; CURRNUM = 14;
    result = from_A_to_B_order1(CURKEYNUM, 5, 0);
    if(result) {
        printf("%s: %d: from A to B failed\n", __FILE__, __LINE__);
        return result;
    }
    CURKEYNUM = 5; CURLNUM = 5;
    set_enable_relax(0);
    delay_ms(1000.0);

    // 设置结束信号
    pthread_mutex_lock(&mutex);
    SONGNUM = 0x00;
    SONGSTOP = false;
    pthread_mutex_unlock(&mutex);
    delay_us(500*1000); //等待实时显示hands线程结束

    // 发送演奏结束的消息
    if(!notEnd) {
        //曲目非正常结束
        sdValue[1] = 0x01;
        pthread_mutex_lock(&mutex);  //加锁
        pProtoV3->sendMessage(0x03, 0x85, 1, sdValue);
        pthread_mutex_unlock(&mutex);//解锁
    }
    else {
        //曲目正常结束
        sdValue[1] = (UINT8_T)_songNum;
        pthread_mutex_lock(&mutex);  //加锁
        pProtoV3->sendMessage(0x30, 0x82, 1, sdValue);
        pthread_mutex_unlock(&mutex);//解锁
    }

    fclose(fpR);
    return 0;
}

/********************************
函数意义：
    读取琴盘的原始键值，判断手掌的个数
参数意义：
    hands[]:存储手掌的位置
返回值：
    -1：表示获取失败
    0：表示没有手掌。没有手掌数据存储在hands[]中
    1：表示有一只手。手掌的坐标存储在hands[0]
    2：表示有两只手。手掌的坐标存储在hands[0]和hands[1]
    >=3：表示有很多手。没有手掌数据存储在hands[]中
********************************/
int get_hands_num(int hands[])
{
    UINT8_T value1, value2, value3;
    UINT8_T value, mask;
    int result, i;
    bool isHands[18];
    int s1, s2;
    int handsNum = 0;

    hands[0] = 0; hands[1]=0;

    // 获取LED地址为ADDR_RTKEYVALUE1的3个值
    pthread_mutex_lock(&mutex);
    result = pLed->get_Lamps_bytes(LEDID, ADDR_RTKEYVALUE1, 3);
    if(result) {
        printf("%s: %d: get lamps bytes failed\n", __FILE__, __LINE__);
        return -1;
    }
    value1 = pLed->read_Lamps_StatusPacket(0);
    value2 = pLed->read_Lamps_StatusPacket(1);
    value3 = pLed->read_Lamps_StatusPacket(2);
    pthread_mutex_unlock(&mutex);
    //printf("%s: %d: value1(%x) value2(%x) value3(%x)\n", __FILE__, __LINE__, value1, value2, value3);

    // 把十六进制实时键值转化为数据队列
    for(i = 0; i < 8; i++) {
        mask = (UINT8_T)(pow(2, i));
        isHands[i] = (bool)(value1 & mask);
    }
    for(i = 8+0; i < 8 + 8; i++){
        mask = (UINT8_T)(pow(2, i - 8));
        isHands[i] = (bool)(value2 & mask);
    }
    for(i=16+0; i < 16+2; i++) {
        mask = (UINT8_T)(pow(2, i - 16));
        isHands[i] = (bool)(value3 & mask);
    }


    // 识别手的个数和每只手的位置
    i = 0;
    while(i < 18) {
        // 找到用户的hands开始覆盖的位置
        while((i < 18) && (!isHands[i]))
            i++;
        if(i < 18)
            s1 = i; //记录用户手覆盖的起始位置
        else
            break;

        // 找到用户的hands结束覆盖的位置
        while((i < 18) && isHands[i])
            i++;
        if(i < 18)
            s2 = i+1; //记录用户手覆盖的终止位置
        else
            s2 = 18 + 1;

        // 判别用户手势
        if((s2-s1 <= 4) && (s2-s1 >= 0)) {
            handsNum += 1;  //记录手势的个数
            if(handsNum <= 2)
                hands[handsNum-1] = 19 - (s2+s1)/2;   //记录手势的位置
        }
    }

    return handsNum;
}

/********************************
函数意义：
    发送键盘的被覆盖的实时数据
参数意义：
    无参数
返回值：
    无返回值
*********************************/
void send_rt_hands()
{
    int hands[2] = {0, 0}, handsNum;
    UINT8_T sdValue[3];

    //设置sdValue的值
    sdValue[0] = FIVEARMID;

    // 获取键盘的实时数据
    handsNum = get_hands_num(hands);

    //发送实时数据
    sdValue[1] = (UINT8_T)handsNum;
    sdValue[2] = (UINT8_T)hands[0];
    sdValue[3] = (UINT8_T)hands[1];
    if(handsNum != 0) {
        printf("%s: %d: set rt hands data: handsNum(%d) hands(%d, %d)\n", __FILE__, __LINE__, handsNum, hands[0], hands[1]);
        pthread_mutex_lock(&mutex);
        pProtoV3->sendMessage(0x40, 0x87, 3, sdValue);
        pthread_mutex_unlock(&mutex);
    }
}

/******************
函数意义：
    读取琴键位置
参数意义：
    无参数
返回值：
    如果读取成功，返回true；
    如果读取失败，返回false;
*******************/
bool read_key_pos()
{
    FILE *fpL = NULL, *fpR = NULL;
    int valueL[5], valueR[5];
    int i, j = 0;

    fpL = fopen(LKEYPOSPATH, "r");
    fpR = fopen(RKEYPOSPATH, "r");
    if((NULL == fpL) || (NULL == fpR)) {
        printf("%s: %d: read file %s and %s failed\n", __FILE__, __LINE__, LKEYPOSPATH, RKEYPOSPATH);
        return false;
    }

    //读取坐标
    while(!feof(fpL)) {
        if(EOF == fscanf(fpL, "%d %d %d %d %d\n", &valueL[0], &valueL[1], &valueL[2], &valueL[3], &valueL[4])) {
            printf("%s: %d: read key pos read %s failed\n", __FILE__, __LINE__, LKEYPOSPATH);
            return false;
        }
        //else
        //    printf("%d %d %d %d %d\n", valueL[0], valueL[1], valueL[2], valueL[3], valueL[4]);
        if(EOF == fscanf(fpR, "%d %d %d %d %d\n", &valueR[0], &valueR[1], &valueR[2], &valueR[3], &valueR[4])) {
            printf("%s: %d: read key pos read %s failed\n", __FILE__, __LINE__, RKEYPOSPATH);
            return false;
        }
        //else
        //    printf("%d %d %d %d %d\n", valueR[0], valueR[1], valueR[2], valueR[3], valueR[4]);
        for (i = 0; i < 5; i++) {
            LKeyPos[j][i] = valueL[i];
            RKeyPos[j][i] = valueR[i];
        }
        j += 1;
    }
    fclose(fpL);
    fclose(fpR);

    return true;
}

/****************
函数意义：
    给机械臂上刚度或者松刚度
参数意义：
    _flag：0：松掉刚度
    _flag：1：上紧刚度
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码；
*****************/
int set_enable_relax(int _flag)
{
    int result;

    for(int i = 1; i <= SERVO_NUM; i++) {
        result = pAX12->set_one_servo_bytes(1, i, Torque_Enable, _flag);
        if(result > 0)
            return result;
    }

    return 0;
}

/*******************************************
函数意义：
    当机械臂在运动的时候，这个函数可以等待机械臂
    运动到目标位置
参数意义：
    _flag：'L':表示左臂；'R'：表示右臂
    _exten：目标位置的容错范围
返回值：
    当执行成功，返回0：
    当执行失败时，返回错误代码(为正数)
*******************************************/
int wait_arm_stop_exten(char _flag, int _exten)
{
    int i, base = 1, diff, result;
    int goal[5], pre[5], flag = 0;

    //设置方向
    if('L' == _flag)
        base = 6;
    else if('R' == _flag)
        base = 1;
    else
        return OTHER_ERROR;

    //等到目标舵机运动停止
    for(int i = base; i < base + 5; i++) {
        result = pAX12->wait_for_one_servo_exten(i, _exten);
        if(result < 0)
            return abs(result);
    }

    return 0;
}

/*******************************
函数意义：
    控制机械臂敲击琴键
参数意义：
    num:琴键的编号
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码；
********************************/
int hit_and_hit(int num, bool _isHit)
{
    //int diffK[12] = {3, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 44};
    //int diffK[10] = {3, 8, 18, 28, 36, 40, 44, 47, 50, 55};
    int diffK[10] = {3, 8, 18, 28, 36, 40, 44, 47, 50, 55};
    int posK, i, result;

    //_isHit = true;

    if(num <= 9) {
        result = pAX12->set_one_servo_bytes(2, 10, Moving_Speed, 559);
        if(result) {
            printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
            return result;
        }
        for(i = 0; i < 10; i++) {
            posK = LKeyPos[num][4] - diffK[i];
            if(_isHit) {
                result = pAX12->set_one_servo_bytes(2, 10, Goal_Position, posK);
                if(result) {
                    printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
                    return result;
                }
            }
            delay_us(5.0 * 1000);
        }

        result = pAX12->set_one_servo_bytes(2, 10, Moving_Speed, 759);
        if(result) {
            printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
            return result;
        }
        for(i = 6; i >= 0; i--) {
            posK = LKeyPos[num][4] - diffK[i];
            if(_isHit) {
                result = pAX12->set_one_servo_bytes(2, 10, Goal_Position, posK);
                if(result) {
                    printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
                    return result;
                }
            }
            delay_us(5.0 * 1000);
        }

    }
    if(num >= 10) {
        result = pAX12->set_one_servo_bytes(2, 5, Moving_Speed, 559);
        if(result) {
            printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
            return result;
        }
        for(i = 0; i < 10; i++) {
            posK = RKeyPos[num-9][4] + diffK[i];
            if(_isHit) {
                result = pAX12->set_one_servo_bytes(2, 5, Goal_Position, posK);
                if(result) {
                    printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
                    return result;
                }
            }
            delay_us(5.0 * 1000);
        }

        result = pAX12->set_one_servo_bytes(2, 5, Moving_Speed, 759);
        if(result) {
            printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
            return result;
        }
        for(i = 6; i >= 0; i--) {
            posK = RKeyPos[num-9][4] + diffK[i];
            if(_isHit) {
                result = pAX12->set_one_servo_bytes(2, 5, Goal_Position, posK);
                if(result) {
                    printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
                    return result;
                }
            }
            delay_us(5.0 * 1000);
        }
    }

    return 0;
}

/*******************************************
函数意义：
    控制机械臂从num1平滑的运动到num2
参数意义：
    num1：起始位置
    num2：终止位置
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
********************************************/
int from_A_to_B_order(int num1, int num2)
{
    int base = 1, mid, start[5], end[5], middle[5], posK;
    int i, j, path[12][5], diffK[5], speK[5], pathNum = 6;
    int result;
    double unit[5];

    if((num1 <= 9) && (num2 <= 9)) {
        //完全在左边运动
        mid = (int)(num1 + num2)/2;
        base = 6;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = LKeyPos[num1][i];
            middle[i] = LKeyPos[mid][i];
            end[i] = LKeyPos[num2][i];
            CURLARM[i] = end[i];
        }
        //更新当前位置
        CURLNUM = num2;
    }
    else if((num1 <= 9) && (num2 >= 10)) {
        //从左运动到右运动
        mid = (int)(CURRNUM + num2)/2;
        base = 1;
        for(i = 0; i < SERVO_NUM/2 ; i++) {
            start[i] = CURRARM[i];
            middle[i] = RKeyPos[mid - 9][i];
            end[i] = RKeyPos[num2 - 9][i];
            CURRARM[i] = end[i];
        }
        //更新当前位置
        CURRNUM = num2;
    }
    else if((num1 >= 10) && (num2 <= 9)) {
        //从右运动到左
        mid = (int)(CURLNUM + num2)/2;
        base = 6;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = CURLARM[i];
            middle[i] = LKeyPos[mid][i];
            end[i] = LKeyPos[num2][i];
            CURLARM[i] = end[i];
        }
        //更新当前位置
        CURLNUM = num2;
    }
    else if((num1 >= 10) && (num2 >= 10)) {
        //完全在右边运动
        mid = (int)(num1+num2)/2;
        base = 1;
        for(i = 0; i < SERVO_NUM/2; i++) {
            start[i] = RKeyPos[num1 - 9][i];
            middle[i] = RKeyPos[num1 - 9][i];
            end[i] = RKeyPos[num2 - 9][i];
            CURRARM[i] = end[i];
        }
        //更新当前位置
        CURRNUM = num2;
    }
    else
        return OTHER_ERROR;

    //计算刻度之差和速度
    //float speBase = 50.0;
    //float k = (1020.0 - speBase)/(1023.0 - 0.0);
    for(i = 0; i < SERVO_NUM/2; i++) {
        diffK[i] = end[i] - start[i];
        //speK[i] = (int)((float)abs(diffK[i]) * k + speBase);
        speK[i] = (int)(((double)abs(diffK[i]) + 2.0) * 3.5);
        if(speK[i] >= 1020)
            speK[i] = 1020;
    }

    //计算path1
    for(i = 0; i < SERVO_NUM/2; i++)
    	diffK[i] = middle[i] - start[i];
    for(i = 0; i < SERVO_NUM/2; i++)
        unit[i] = (double)diffK[i] / pathNum;
    for(i = 1; i <= pathNum; i++) {
        for(j = 0; j < 5; j++) {
            path[i - 1][j] = unit[j] * i;
        }
    }

    //计算path2
    for(i = 0; i < SERVO_NUM/2; i++)
    	diffK[i] = end[i] - middle[i];
    for(i = 0; i < SERVO_NUM/2; i++)
    	unit[i] = (double)diffK[i] / pathNum;
    for(i = 1; i <= pathNum; i++) {
    	for(j = 0; j < 5; j++)
    		path[i + pathNum - 1][j] = unit[j] * i + path[pathNum-1][j];
    }

    /*
    //计算path
    for(i = 1; i <= pathNum; i++) {
        for(j = 0; j < 5; j++) {
            path[i - 1][j] = unit[j] * i;
        }
    }
    */

    //从A运动到B
    //设置速度
    for(i = 0; i < SERVO_NUM/2; i++) {
        result = pAX12->set_one_servo_bytes(2, i+base, Moving_Speed, speK[i]);
        if(result) {
            printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
            return result;
        }
    }
    //运动到目标位置
    for(i = 0; i < pathNum; i++) {
        for(j = 0; j < 5; j++) {
            posK = path[i][j] + start[j];
            if(3 == (j + base)) posK += 20;
            result = pAX12->set_one_servo_bytes(2, j+base, Goal_Position, posK);
            if(result) {
                printf("%s: %d: set one servo word failed\n", __FILE__, __LINE__);
                return result;
            }
            delay_us(2.5 * 1000);
        }
        //delay_ms(7.5);
    }
    for(j = 0; j < 5; j++) {
        result = pAX12->set_one_servo_bytes(2, j + base, Goal_Position, end[j]);
        if(result) {
            printf("%s: %d: set one servo word\n", __FILE__, __LINE__);
            return result;
        }
    }
    //delay_ms(15.0);

    //设置提示灯亮
    //set_led_light(num2);
    /*
    if(1 == base) {
        //等待右臂运动停止
        wait_arm_stop_exten('R', 15);
    }
    else{
        //等待左臂运动停止
        wait_arm_stop_exten('L', 15);
    }
    */

    return 0;
}

/***********************************************
函数意义：
    控制机械臂从num1平滑的运动到num2。
    平滑的过程为：如果num1=1, num2=4
    机械臂要经过1、2、3、4
参数意义：
    num1：起始位置
    num2: 终止位
    lastChoose:控制最后位置
返回值:
    如果执行成功，返回0；
    如果执行失败，返回错误代码
***********************************************/
int from_A_to_B_order1(int num1, int num2, int lastChoose)
{
    int base = 1, start, end, increDirect = 1;
    double speUnit = 0.666, posUnit = 0.29;
    int srcPosK[5], endPosK[5];
    double diffK[5];
    int speK[5];
    double delayTm = 0.0;

    printf("%s: %d: start(%d) end(%d) lastChoose(%d)\n", __FILE__, __LINE__, num1, num2, lastChoose);
    //getchar();
    //getchar();

    // lastChoose的范围
    if(lastChoose != 0 && lastChoose != 1)
        return OTHER_ERROR;

    //判断运动方向
    if((num1 <= 9) && (num2 <= 9)) {
        // 完全在左边运动
        base = 6; //舵机编号基数
        start = num1;
        end = num2;
        CURLNUM = num2;
        if(num1 < num2) {
            //1:表示递增；-1：表示递减；0:敲相同位置
            increDirect = 1;
        }
        else if(num1 > num2) {
            increDirect = -1;
        }
        else {
            increDirect = 0;
        }
    }
    else if((num1 <= 9) && (num2 >= 10)) {
        // 从左向右运动
        base = 1;
        start = CURRNUM;
        end = num2;
        CURRNUM = num2;
        if(start < end)
            increDirect = 1;
        else if(start > end)
            increDirect = -1;
        else
            increDirect = 0;
    }
    else if((num1 >= 10) && (num2 <= 9)) {
        //从右运动到左
        base = 6;
        start = CURLNUM;
        end = num2;
        CURLNUM = num2;
        if(start < end)
            increDirect = 1;
        else if(start > end)
            increDirect = -1;
        else
            increDirect = 0;
    }
    else if((num1 >= 10) && (num2 >= 10)) {
        //从右运动到右
        base = 1;
        start = num1;
        end = num2;
        CURRNUM = end;
        if(num1 < num2) {
            increDirect = 1;
        }
        else if(num1 > num2) {
            increDirect = -1;
        }
        else{
            increDirect = 0;
        }
    }
    else {
        return OTHER_ERROR;
    }

    // 进行运动控制
    //左边的递增运动
    if((1 == increDirect) && (end <= 9)) {
        // 左边的递增运动
        for(int i = start; i < end; i += 1) {
            for(int j = 0; j < SERVO_NUM/2; j++) {
                diffK[j] = (double)abs(LKeyPos[i+1][j] - LKeyPos[i][j]);
                speK[j] = (int)((diffK[j] + 2.0) * 3.5);
                if(speK[j] > 1020)  speK[j] = 1020;
                delayTm = (diffK[j]*posUnit)/(speK[j]*speUnit);
                //设置速度
                pAX12->set_one_servo_bytes(2, j+base, Moving_Speed, speK[j]);
                //设置目标位置
                pAX12->set_one_servo_bytes(2, j+base, Goal_Position, LKeyPos[i+1][j]);
            }
            delay_us(delayTm*1000000);
        }
    }
    else if((1 == increDirect) && (end >= 10)) {
        // 右边的递增运动
        for(int i = start; i < end; i += 1) {
            for(int j = 0; j < SERVO_NUM/2; j++) {
                diffK[j] = (double)abs(RKeyPos[i+1 - 9][j] - RKeyPos[i-9][j]);
                speK[j] = (int)((diffK[j] +2.0) * 3.5);
                if(speK[j] > 1020)  speK[j] = 1020;
                delayTm = (diffK[j]*posUnit) / (speK[j]*speUnit);
                //设置速度
                pAX12->set_one_servo_bytes(2, j+base, Moving_Speed, speK[j]);
                //设置目标位置
                pAX12->set_one_servo_bytes(2, j+base, Goal_Position, RKeyPos[i+1-9][j]);
            }
            delay_us(delayTm*1000000);
        }
    }
    else if((-1 == increDirect) && (end <= 9)) {
        // 左边的递减运动
        for(int i = start; i > end; i--) {
            for(int j = 0; j < SERVO_NUM/2; j++) {
                diffK[j] = (double)abs(LKeyPos[i-1][j] - LKeyPos[i][j]);
                speK[j] = (int)((diffK[j] + 2.0) * 3.5);
                if(speK[j] > 1020)  speK[j] = 1020;
                delayTm = (diffK[j]*posUnit)/(speK[j]*speUnit);
                //设置速度
                pAX12->set_one_servo_bytes(2, j+base, Moving_Speed, speK[j]);
                //设置目标位置
                pAX12->set_one_servo_bytes(2, j+base, Goal_Position, LKeyPos[i-1][j]);
            }
            delay_us(delayTm*1000000);
        }
    }
    else if((-1 == increDirect) && (end >= 10)) {
        // 右边的递减运动
        for(int i = start; i > end; i--) {
            for(int j = 0; j < SERVO_NUM/2; j++) {
                diffK[j] = (double)abs(RKeyPos[i-1 - 9][j] - RKeyPos[i-9][j]);
                speK[j] = (int)((diffK[j] +2.0) * 3.5);
                if(speK[j] > 1020)  speK[j] = 1020;
                delayTm = (diffK[j]*posUnit) / (speK[j]*speUnit);
                //设置速度
                pAX12->set_one_servo_bytes(2, j+base, Moving_Speed, speK[j]);
                //设置目标位置
                pAX12->set_one_servo_bytes(2, j+base, Goal_Position, RKeyPos[i-1-9][j]);
            }
            delay_us(delayTm*1000000);
        }
    }
    else {
        delayTm = posUnit/(speUnit*3.5);
        delay_us((delayTm + 0.1) * 1000000);
    }

    //设置最后的目标位置
    for(int j = 0; j < SERVO_NUM/2; j++) {
        if(end<=9) {
            //end在左半部分
            pAX12->set_one_servo_bytes(2, j+base, Goal_Position, LKeyPos[end*lastChoose][j]);
            CURLARM[j] = LKeyPos[end*lastChoose][j];
        }
        else {
            pAX12->set_one_servo_bytes(2, j+base, Goal_Position, RKeyPos[(end-9)*lastChoose][j]);
            CURRARM[j] = RKeyPos[(end-9)*lastChoose][j];
        }
    }

    return 0;
}

/**************************************************
函数意义：
    设置手臂的字节信息
参数意义：
    _byteNum：1：表示单字节；2：表示双字节
    dir:'L':表示左臂；'R':表示右臂
    address:地址
    value:要写入的值
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
**************************************************/
int set_arm_bytes(int _byteNum, char dir, int addr, int *value)
{
    int i, base = 1, result;

    //如果是左臂
    if('L' == dir)
        base = 6;
    else if('R' == dir) //如果是右臂
        base = 1;
    else
        return OTHER_ERROR;

    for(i = 0; i < SERVO_NUM/2; i++) {
        result = pAX12->set_one_servo_bytes(_byteNum, i+base, addr, value[i]);
        if(result) {
            printf("%s: %d: set arm byte address:%d failed...\n", __FILE__, __LINE__, addr);
            return result;
        }
    }

    return 0;
}

//通信线程
void* conmmunicateThread(void* arg)
{
    UINT8_T sdValue[2];
    char song;

    sdValue[0] = FIVEARMID;

    printf("%s: %d: Enter conmmunicateThread\n", __FILE__, __LINE__);

    while(true) {
        //读取数据
        if(pProtoV3->receiveMessage(0)) {
            printf("\n");

            //如果是指令
            if(0x01 == pProtoV3->getInst(4)) {
                printf("%s: %d: shake hands\n", __FILE__, __LINE__);
                pthread_mutex_lock(&mutex);  //加锁
                pProtoV3->sendMessage(0x01, 0, 0, sdValue);//设置PING的反馈信息
                pthread_mutex_unlock(&mutex);//解锁
            }
            else if((0x03==pProtoV3->getInst(4))&&
                    (0x80==pProtoV3->getInst(6))&&
                    (0X08==pProtoV3->getInst(7))) {
                    //获取应用程序类型
                    printf("%s: %d: application is InterPlayMusic\n", __FILE__, __LINE__);
                    sdValue[1] = 0x08;
                    pthread_mutex_lock(&mutex);  //加锁
                    pProtoV3->sendMessage(0x03, 0x80, 1, sdValue);
                    pthread_mutex_unlock(&mutex);//解锁
            }
            else if((0x03==pProtoV3->getInst(4)&&
                    (0x81==pProtoV3->getInst(6)&&
                    (0x01==pProtoV3->getInst(7))))) {
                    //获取开始指令
                    printf("\n%s: %d: application is start...\n", __FILE__, __LINE__);
                    sdValue[1] = 0x01;
                    pthread_mutex_lock(&mutex);  //加锁
                    pProtoV3->sendMessage(0x03, 0x81, 1, sdValue);
                    ISSTART = true;
                    pthread_mutex_unlock(&mutex);//解锁
            }
            else if((0x03==pProtoV3->getInst(4)&&
                    (0x82==pProtoV3->getInst(6)))) {
                    //获取曲目信息
                    printf("%s: %d: Be to play default songs %d\n", __FILE__, __LINE__, (int)pProtoV3->getInst(7));
                    sdValue[1] = pProtoV3->getInst(7);
                    pthread_mutex_lock(&mutex);  //加锁
                    SONGNUM = (int)pProtoV3->getInst(7);
                    SONGSTOP = false;
                    pProtoV3->sendMessage(0x03, 0x82, 1, sdValue);
                    pthread_mutex_unlock(&mutex);//解锁
            }
            else if((0x03==pProtoV3->getInst(4))&&
                    (0x85==pProtoV3->getInst(6))&&
                    (0x01==pProtoV3->getInst(7))) {
                    //获取曲目终止消息
                    sdValue[1] = 0x01;
                    printf("%s: %d: Be to stop songs\n", __FILE__, __LINE__);
                    pthread_mutex_lock(&mutex);  //加锁
                    //pProtoV3->sendMessage(0x03, 0x85, 1, sdValue);
                    if(SONGNUM)
                        SONGSTOP = true;
                    else
                        printf("%s: %d: err: no songNum\n", __FILE__, __LINE__);
                    pthread_mutex_unlock(&mutex);//解锁
            }
            else{
                printf("%s: %d: other message\n", __FILE__, __LINE__);
            }
        }

        // 实时发送用户的hands信息
        // 判断是否进入演奏模式
        pthread_mutex_lock(&mutex);
        song = SONGNUM;
        pthread_mutex_unlock(&mutex);
        if(song) {
            send_rt_hands(); // 发送hands的实时信息
            delay_us(50*1000);
        }
    }
}

//运动流程线程
//pthread_mutex_lock(&mutex);  //加锁
//pthread_mutex_unlock(&mutex);//解锁
void* actionProcesThread(void* arg)
{
    bool _isStart = false;
    int result, result1, result2;
    UINT8_T sdValue[2], _songNum;
    double timeUp = 2000.0; //2000ms

    sdValue[0] = FIVEARMID;

    //前期准备信号和动作
    while(true) {
        pthread_mutex_lock(&mutex);  //加锁
        _isStart = ISSTART;
        pthread_mutex_unlock(&mutex);//解锁
        if(_isStart) {
            printf("\n%s: %d: Enter actionProcesThread\n", __FILE__, __LINE__);

            //动作进入初始化模式
            result1 = set_default_pos();
            delay_ms(1000.0);
            result2 = set_enable_relax(0);
            result = result1 > 0 ? result1 : result2;
            if(result) {
                sprintf("%s: %d: set default pos failed or relax failed\n", __FILE__, __LINE__);
                sdValue[1] = (UINT8_T)result;
                pthread_mutex_lock(&mutex);  //加锁
                pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
                pthread_mutex_unlock(&mutex);//解锁
                continue;
            }
            else
                printf("%s: %d: set default pos succes\n", __FILE__, __LINE__);

            break;
        }
        else{
            delay_ms(10.0);
            printf("\r%s: %d: action waitting to start...", __FILE__, __LINE__);
        }
    }

    delay_start(); //设置起始时间
    while(true) {
        // LED进入菜单模式
        result = set_led_mode(1);
        if(result) {
            sprintf("%s: %d: set led mode failed\n", __FILE__, __LINE__);
            sdValue[1] = (UINT8_T)result;
            pthread_mutex_lock(&mutex);  //加锁
            pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
            pthread_mutex_unlock(&mutex);//解锁
            continue;
        }
        else
            printf("%s: %d: set sed mode 1 succes\n", __FILE__, __LINE__);

        // 判断用户输入
        printf("%s: %d: waitting for user inputing...\n", __FILE__, __LINE__);
        while(true) {
            //获取用户输入
            pthread_mutex_lock(&mutex);
            result = get_user_input(); //获取用户的输入信息
            pthread_mutex_unlock(&mutex);
            printf("\r%s: %d: result(%d) waiting for inputing...", __FILE__, __LINE__, result);
            fflush(stdout);

            //检测一体机是否发送了歌曲信息
            pthread_mutex_lock(&mutex);  //加锁
            _songNum = SONGNUM;
            SONGSTOP = false;
            pthread_mutex_unlock(&mutex);//解锁
            if(_songNum) {//判断是否有演奏曲目
                printf("\n%s: %d: start to into play mode, the song(%d)\n", __FILE__, __LINE__, (int)_songNum);
                set_led_mode(2); //进入演奏模式
                break;
            }

            sdValue[1] = (UINT8_T)abs(result);
            if(result < 0) {
                printf("%s: %d: get user input failed\n", __FILE__, __LINE__);
                pthread_mutex_lock(&mutex);  //加锁
                pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
                pthread_mutex_unlock(&mutex);//解锁
                continue;
            }
            else if(result > 0) { //发送用户选择
                set_led_mode(1);  //设置led菜单模式
                if((sdValue[1] == 0x01) || (sdValue[1] == 0x04)) {
                    double tmdiff;
                    tmdiff = delay_end();
                    if(tmdiff > timeUp) {
                        delay_start();  //重置起始时间
                        printf("\n%s: %d: valid result(%d)...\n", __FILE__, __LINE__, result);
                        pthread_mutex_lock(&mutex);  //加锁
                        pProtoV3->sendMessage(0x40, 0x84, 1, sdValue);
                        pthread_mutex_unlock(&mutex);  //解锁
                    }
                }
                else {
                    delay_start();
                    printf("\n%s: %d: valid result(%d)...\n", __FILE__, __LINE__, result);
                    pthread_mutex_lock(&mutex);    //加锁
                    pProtoV3->sendMessage(0x40, 0x84, 1, sdValue);
                    pthread_mutex_unlock(&mutex);  //解锁
                }
            }
            else{
            }
        }

        // 上刚度，进入准备演奏的姿态
        set_default_pos();
        delay_ms(500.0);

        // 进入演奏模式
        result = play_music(_songNum);
        if(result) {
            //如果演奏失败，发送失败消息
            sdValue[1] = (UINT8_T)result;
            pthread_mutex_lock(&mutex);  //加锁
            pProtoV3->sendMessage(0x40, 0x04, 1, sdValue);
            pthread_mutex_unlock(&mutex);//解锁
        }
    }
}

/*****************
函数意义：
    清理系统
参数意义：
    无参数
返回值：
    无返回值
*****************/
void clear_sys()
{
    delete pProtoV3;
    delete pLed;
    delete pAX12;
}
