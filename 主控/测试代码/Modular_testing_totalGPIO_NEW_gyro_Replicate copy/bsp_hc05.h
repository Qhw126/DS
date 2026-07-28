/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#ifndef __BSP_HC05_H__
#define __BSP_HC05_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <string.h>

// 蓝牙接收缓冲区最大长度
#define BLERX_LEN_MAX       256

// 蓝牙连接状态定义
#define DISCONNECT           0
#define CONNECT              1

// 蓝牙STATE引脚电平读取宏（HC05_PORT=GPIOB, HC05_STATE_PIN=PIN_23, 由SysConfig生成）
// DL_GPIO_readPins 返回引脚掩码值（非0即为高电平），用 != 0 判断
#define BLUETOOTH_LINK       (DL_GPIO_readPins(HC05_PORT, HC05_STATE_PIN) != 0)

// 是否开启串口调试输出    1开启  0关闭
#define BLE_DEBUG            1

// 外部变量声明
extern unsigned char Bluetooth_ConnectFlag;  // 蓝牙连接状态 0=未连接 1=已连接
extern unsigned char BLERX_BUFF[BLERX_LEN_MAX]; // 蓝牙接收缓冲区
extern unsigned char BLERX_FLAG;             // 接收完成标志
extern unsigned char BLERX_LEN;              // 接收数据长度

// 函数声明
void Bluetooth_Init(void);                     // 蓝牙初始化
void Bluetooth_Mode(void);                     // 检测蓝牙连接状态
unsigned char Get_Bluetooth_ConnectFlag(void); // 获取蓝牙连接状态
void BLE_Send_Bit(unsigned char ch);           // 发送单个字节
void BLE_send_String(unsigned char *str);      // 发送字符串
void Clear_BLERX_BUFF(void);                   // 清除接收缓冲区
void Receive_Bluetooth_Data(void);             // 处理接收到的蓝牙数据
void Send_Bluetooth_Data(char *dat);           // 向蓝牙发送数据（已连接时）

#endif /* __BSP_HC05_H__ */
