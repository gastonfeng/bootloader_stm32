#include <lib/firmata_extend/mFirmata.h>
#include <lib/firmata_extend/socketFirmata.h>
#include <lib/firmata_extend/firmata_client.h>

mFirmata fm;        // Firmata类
firmata_client fclient; // Firmata客户端接口封装类
socketFirmata sk;       //socket网络接口封装

typedef struct {    //数据示例
    u8 p1;
    short p2;
    float p3;
} data_t;


data_t data;

void begin() {
    fclient.begin(&fm, &sk);
    int res = firmata_client::get_var_bool(0, (u8 *) &data.p1); //读取单字节变量
    if (res < 0) {
        logger.error("get_var_bool error");
    }
    res = firmata_client::get_var_int(10, (short *) &data.p2);//读取整数,两字节
    if (res < 0) {
        logger.error("get_var_int error");
    }
    res = firmata_client::get_var_float(20, (float *) &data.p3);    //读浮点数
    if (res < 0) {
        logger.error("get_var_float error");
    }

    res = firmata_client::set_var_bool(0, data.p1); //写单字节变量
    if (res < 0) {
        logger.error("set_var_bool error");
    }
    res = firmata_client::set_var_float(20, data.p3);   //写两字节整数
    if (res < 0) {
        logger.error("set_var_float error");
    }
    res = firmata_client::set_var_int(10, data.p2);     //写浮点数
    if (res < 0) {
        logger.error("set_var_int error");
    }
}