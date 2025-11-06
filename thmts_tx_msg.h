#ifndef THMTS_MSG_COM_H
#define THMTS_MSG_COM_H

// incldue necessary headers
#include <stdint.h>
#include "thmts_bb_config.h"

#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */

// 宏定义区域
// 物理层参数宏定义

#define TYPE_THMTS_TX_SLOT_INFO_T	0x91   // 对应发射消息
#define TYPE_THMTS_RX_SLOT_INFO_T   0x92   // 对应接收消息
#define TYPE_THMTS_RX_CIR_INTO_T    0x93   // 对应CIR消息
#define TYPE_THMTS_RANGING_INTO_T   0x94   // 对应测距消息
#define TYPE_THMTS_RANGE_AND_DATA_T 0x95   // 对应测距和数据包消息
#define TYPE_THMTS_SENSOR_IMU_DATA_T  0x97   // 对应IMU数据




//-----------------------------------------------------------------------
// 定义往串口发送msg的结构体
// 所有类型的msg都使用统一的这样的结构体，各种类型的数据都是送到msg_ptr中
#pragma pack(1)
typedef struct
{
    uint8_t  msg_type;
    uint8_t  msg_status;
    uint8_t  msg_id;
    uint16_t msg_length;

    void*    msg_ptr;
} thmts_tx_msg_info_t;
#pragma pack()

//-----------------------------------------------------------------------
// 不同类型的发射数据


// 定义TX info类型msg结构体，#define TYPE_THMTS_TX_SLOT_INFO_T	0x91
#pragma pack(1)
typedef struct
{
    uint32_t PHR;                     // 发射PHR bit
    uint16_t dev_id;                  // 本身设备号
    uint16_t slot_id;                 // 本身slot_id
    uint16_t tx_length;              //  发射数据包tx_info长度
    thmts_ranging_packet_t tx_data;   //  发射数据包
} thmts_tx_slot_info_t;
#pragma pack()


// 定义RX info类型msg结构体，//#define TYPE_THMTS_RX_SLOT_INFO_T   0x92
#pragma pack(1)
typedef struct
{
    uint32_t rx_info[10];              // 接收寄存器信息，从BB中读取
    uint64_t rmark;                   // 自己收到信息的接收时刻
    uint16_t dev_id;                  // 本身设备号
    uint16_t slot_id;                 // 本身slot_id
    uint16_t rx_length;               // 接收到的数据包rx_info长度

    uint8_t crc_res;                  // 接收CRC成功标志
    uint32_t rx_status;                // 接收到的信息低8bit代表接收状态 1: 接收超时  4: 接收成功  15: PHR出错
                                       // 次低字节代表AGC增益字
                                       //高16bit代表CIR累积次数

    uint32_t rx_cnt;                  // 总共接收到的次数
    uint32_t crc_ok_cnt;              // 接收成功且通过CRC验证的统计次数
	uint32_t rx_timeout_cnt;          // 接收超时的统计次数
	uint32_t rx_phr_error_cnt;        // 接收PHR出错的统计次数
	uint32_t crc_error_cnt;           // CRC出错的统计次数

	thmts_ranging_packet_t rx_data;    // 接收到的数据包
} thmts_rx_slot_info_t;
#pragma pack()


// 定义CIR类型msg结构体，  #define TYPE_THMTS_RX_CIR_INTO_T    0x93 
#pragma pack(1)
typedef struct
{
    uint8_t ch_id;             // CIR通道号
    uint32_t CIR[1022];         // 对应通道的CIR数据
} thmts_rx_CIR_info_t;
#pragma pack()


// 定义测距类型msg结构体，  #define TYPE_THMTS_RANGING_INTO_T    0x94
#pragma pack(1)
typedef struct
{
    int tof;             // 测距消息
} thmts_RANGING_info_t;
#pragma pack()



// 定义 range_and_data 类型msg结构体 #define TYPE_THMTS_RANGE_AND_DATA_T    0x95
#pragma pack(1)
typedef struct
{
	uint32_t frame_cnt;      //当前frame数据包的编号
	uint8_t tof_valid;       //当前frame的测距值是否有效的标志
	uint8_t data_valid;		 //当前frame的数据包数据data[512]是否有效的标志
	double tof_value;        //当前frame的测距值
	uint8_t data[512];       //当前frame的数据包数据，注意要按照uint16_t进行解析
} thmts_range_and_data_t;
#pragma pack()



// 定义IMU传感器数据结构体，#define TYPE_THMTS_SENSOR_IMU_DATA_T       0x97
#pragma pack(1)
typedef struct{
    double utime;
    float gyro[3];
    float acc[3];
    float attitude[3];      // roll, pitch, yaw
    float temperature;

    unsigned char type;     // 0x01  IMU
    unsigned char status;
    unsigned char msg_cnt;
} thmts_sensor_imu_data_t;
#pragma pack()




//#pragma pack(1)
//typedef struct
//{
//    uint8_t ch_id;             // CIR通道号
//    uint32_t CIR[20];         // 对应通道的CIR数据
//} thmts_rx_CIR_info_t;
//#pragma pack()


//-----------------------------------------------------------------------
// 全局开关声明
// 控制发射信息的开关
extern uint8_t SWITCH_THMTS_TX_SLOT_INFO_T;     // 控制TX info是否发射的开关,   1:上传TXinfo , 0:不上传TXinfo
extern uint8_t SWITCH_THMTS_RX_SLOT_INFO_T;     // 控制RX info是否发射的开关,   1:上传RXinfo , 0:不上传RXinfo
extern uint8_t SWITCH_THMTS_RX_CIR_CH0_INFO_T;  // 控制通道0 CIR 是否发射的开关, 1:上传通道0 CIR , 0:不上传通道0 CIR
extern uint8_t SWITCH_THMTS_RX_CIR_CH1_INFO_T;  // 控制通道1 CIR 是否发射的开关, 1:上传通道1 CIR , 0:不上传通道1 CIR
extern uint8_t SWITCH_THMTS_RX_CIR_CH2_INFO_T;  // 控制通道2 CIR 是否发射的开关, 1:上传通道2 CIR , 0:不上传通道2 CIR
extern uint8_t SWITCH_THMTS_RX_CIR_CH3_INFO_T;  // 控制通道3 CIR 是否发射的开关, 1:上传通道3 CIR , 0:不上传通道3 CIR
extern uint8_t SWITCH_THMTS_RANGING_INFO_T;     // 控制测距消息是否发射的开关,   1:上传测距结果     0:不上传测距结果
extern uint8_t SWITCH_THMTS_RANGE_AND_DATA_T;   // 控制数据包和测距值消息是否发射的开关,   1:上传结果     0:不上传结果
extern uint8_t SWITCH_THMTS_SENSOR_IMU_DATA_T;  // 控制IMU传感器数据是否发射的开关, 1:上传IMU数据, 0:不上传IMU数据




// 定义不同类型的msg
extern thmts_tx_slot_info_t TX_slot_info;         // TX info msg   0x91
extern thmts_rx_slot_info_t RX_slot_info;         // RX info msg   0x92
extern thmts_rx_CIR_info_t  RX_CIR_info_channel0; // 通道0 CIR msg  0x93
extern thmts_rx_CIR_info_t  RX_CIR_info_channel1; // 通道1 CIR msg  0x93
extern thmts_rx_CIR_info_t  RX_CIR_info_channel2; // 通道2 CIR msg  0x93
extern thmts_rx_CIR_info_t  RX_CIR_info_channel3; // 通道3 CIR msg  0x93
extern thmts_RANGING_info_t ranging_info;         // 测距      msg  0x94
extern thmts_range_and_data_t range_and_data_info;// 测距和数据包 msg 0x95
extern thmts_sensor_imu_data_t  Sensor_IMU_data;  // IMU传感器数据  0x97

//// 定义各通道的CIR存储结构体
//extern uint32_t channel0_CIR[1022];
//extern uint32_t channel1_CIR[1022];
//extern uint32_t channel2_CIR[1022];
//extern uint32_t channel3_CIR[1022];




//---------------------------------------------------------------------------
//定义全局函数
// 读取CIR数据，然后发送到对应的消息队列中
extern void read_and_send_CIR_msg(QueueHandle_t msgQueue);
extern void read_and_send_IMU_data(QueueHandle_t msgQueue);  // 读取IMU数据，然后发送到对应的消息队列中


#endif // THMTS_TX_MSG_H
