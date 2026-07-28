/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#include "bsp_hc05.h"
#include <stdio.h>

// 全局变量定义
unsigned char Bluetooth_ConnectFlag = 0;           // 蓝牙连接状态 0=未连接 1=已连接
unsigned char BLERX_BUFF[BLERX_LEN_MAX];           // 蓝牙接收缓冲区
unsigned char BLERX_FLAG = 0;                      // 接收完成标志
unsigned char BLERX_LEN = 0;                       // 接收数据长度

/******************************************************************
 * 函 数 名 称：BLE_Send_Bit
 * 函 数 说 明：向蓝牙发送单个字节
 * 函 数 形 参：ch=要发送的字节
 * 函 数 返 回：无
 * 备       注：使用SysConfig配置的HC_INST（UART1）
******************************************************************/
void BLE_Send_Bit(unsigned char ch)
{
    // 等待串口空闲
    while( DL_UART_isBusy(HC_INST) == true );
    // 发送单个字节
    DL_UART_Main_transmitData(HC_INST, ch);
}

/******************************************************************
 * 函 数 名 称：BLE_send_String
 * 函 数 说 明：向蓝牙发送字符串
 * 函 数 形 参：str=要发送的字符串
 * 函 数 返 回：无
******************************************************************/
void BLE_send_String(unsigned char *str)
{
    while( str && *str )
    {
        BLE_Send_Bit(*str++);
    }
}

/******************************************************************
 * 函 数 名 称：Clear_BLERX_BUFF
 * 函 数 说 明：清除蓝牙接收缓冲区
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void Clear_BLERX_BUFF(void)
{
    BLERX_LEN = 0;
    BLERX_FLAG = 0;
    memset(BLERX_BUFF, 0, BLERX_LEN_MAX);
}

/******************************************************************
 * 函 数 名 称：Bluetooth_Init
 * 函 数 说 明：蓝牙初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：默认波特率为115200（由SysConfig配置）
******************************************************************/
void Bluetooth_Init(void)
{
    // UART 已在 SYSCFG_DL_init() → SYSCFG_DL_HC_init() 中完成初始化和使能
    // 此处只需清除中断标志并确保 NVIC 使能

    // 清除串口中断标志
    NVIC_ClearPendingIRQ(HC_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(HC_INST_INT_IRQN);

    // 注意：不在初始化时发送调试信息，避免 DL_UART_isBusy() 卡死
    // 如需测试蓝牙发送功能，可在主循环中通过蓝牙连接后调用 BLE_send_String()
}

/******************************************************************
 * 函 数 名 称：Get_Bluetooth_ConnectFlag
 * 函 数 说 明：获取手机连接状态
 * 函 数 形 参：无
 * 函 数 返 回：返回1=已连接  返回0=未连接
 * 备       注：使用该函数前，必须先调用 Bluetooth_Mode 函数
******************************************************************/
unsigned char Get_Bluetooth_ConnectFlag(void)
{
    return Bluetooth_ConnectFlag;
}

/******************************************************************
 * 函 数 名 称：Bluetooth_Mode
 * 函 数 说 明：判断蓝牙模块的连接状态
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：未连接时STATE低电平  连接成功时STATE高电平
******************************************************************/
void Bluetooth_Mode(void)
{
    static char flag = 0;

    // 如果没有手机连接
    if( DISCONNECT == BLUETOOTH_LINK )
    {
        Bluetooth_ConnectFlag = 0;
        if( flag == 1 )
        {
            flag = 0;
        }
        return;
    }

    // 如果手机已经连接
    if( CONNECT == BLUETOOTH_LINK )
    {
        Bluetooth_ConnectFlag = 1;
        if( flag == 0 )
        {
            flag = 1;
        }
    }
}

/******************************************************************
 * 函 数 名 称：Receive_Bluetooth_Data
 * 函 数 说 明：处理接收到的蓝牙数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：在主循环中调用，检测到BLERX_FLAG后处理数据
******************************************************************/
void Receive_Bluetooth_Data(void)
{
    if( BLERX_FLAG == 1 ) // 接收到蓝牙数据
    {
        #if BLE_DEBUG
            // 打印接收到的数据（可通过其他串口输出调试）
            // 注意：这里不能再用HC口回显，避免与蓝牙通信冲突
            // 如需调试，可使用Gyro串口或其他调试串口输出
        #endif

        // 这里可以添加自定义的数据处理逻辑
        // 例如：解析控制指令、设置参数等

        Clear_BLERX_BUFF(); // 清除接收缓存
    }
}

/******************************************************************
 * 函 数 名 称：Send_Bluetooth_Data
 * 函 数 说 明：向蓝牙模块发送数据（需已连接状态）
 * 函 数 形 参：dat=要发送的字符串
 * 函 数 返 回：无
 * 备       注：如果手机连接了蓝牙，就是向手机发送数据
******************************************************************/
void Send_Bluetooth_Data(char *dat)
{
    // 获取蓝牙状态
    Bluetooth_Mode();
    // 如果手机已经连接
    if( Bluetooth_ConnectFlag == 1 )
    {
        // 发送数据
        BLE_send_String((unsigned char*)dat);
    }
}

/******************************************************************
 * 函 数 名 称：HC_INST_IRQHandler
 * 函 数 说 明：HC蓝牙串口中断服务函数（UART1）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：SysConfig中HC实例对应的中断处理函数名
******************************************************************/
/* [已移到 empty.c] 中断服务函数不在这里定义 */
/*
void HC_INST_IRQHandler(void)
{
    switch( DL_UART_getPendingInterrupt(HC_INST) )
    {
        case DL_UART_IIDX_RX:
            if( BLERX_LEN < BLERX_LEN_MAX - 1 )
            {
                BLERX_BUFF[BLERX_LEN++] = DL_UART_Main_receiveData(HC_INST);
            }
            else
            {
                uint8_t temp = DL_UART_Main_receiveData(HC_INST);
                (void)temp;
            }
            BLERX_BUFF[BLERX_LEN] = '\0';
            BLERX_FLAG = 1;
            break;

        default:
            break;
    }
}
*/
