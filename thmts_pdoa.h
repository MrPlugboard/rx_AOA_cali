
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "thmts_bb_config.h"
#include "thmts_node_info.h"
#include <math.h>

#define SWITCH_OUTPUT_CIR 1 //是否在串口中输出CIR的数据

#define CIR_N 1022
#ifndef CIR_FOR_PDOA_LENGTH
#define CIR_FOR_PDOA_LENGTH 15 //计算PDOA所用的每通道CIR的采样点的总长度
#endif

#define SQRT3 1.7320508075688772
#define M_PI 3.14159265358979323846
#define COUNT_PER_PI 6434.0  // 采样点数每π弧度
#define PDOA_scale 35.7444   //  6434/pi * pi/180

extern int32_t AOA_index;                              // 串口播发的ID
extern uint32_t CIR_data0[3][CIR_FOR_PDOA_LENGTH];         // 通道0的CIR数据，第一个索引为三个子帧，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据
extern uint32_t CIR_data1[3][CIR_FOR_PDOA_LENGTH];         // 通道1的CIR数据，第一个索引为三个子帧，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据
extern uint32_t CIR_data2[3][CIR_FOR_PDOA_LENGTH];         // 通道2的CIR数据，第一个索引为三个子帧，采集CIR_FOR_PDOA_LENGTH个点，第一径点以及对应前后两个点的数据


typedef struct { double re, im; } c64;

// 定义了AoA校准参数表的数据格式
typedef struct {
    int aoaCaliType; // 目前固定为1，长度为1600Byte，只支持pdoa和方位角修正，不支持俯仰和更复杂的修正
    float pdoaOffset[6]; // 最多支持到4天线的offset
    float aoaCali[360]; // 方位角真值对应表，1°一个值
    float aoaOffset; // 方位角offset
    unsigned short rangeCali[58]; // 测距真值对应表，按第五章真值表
    unsigned char rsv[16]; // 预留字段，整个校准表1600Byte
} aoaCaliPara_t;

// 定义了AoA校准参数表的串口下发指令，超过1Byte的数据，用小段格式
typedef struct {
    unsigned char frameHead[2]; // 帧头 0xEB 0x90
    unsigned short msgLength; // 从该字段后到xorCheck之前的字节数
    aoaCaliPara_t aoaCali; // 校准表，1600Byte
    unsigned char revisionId; // 固定 0x01
    unsigned char msgType; // 固定 0x70 AoA校准表下发指令
    unsigned char msgCnt; // 消息顺序编号，自增1
    unsigned char xorCheck; // 包含msgLength到msgCnt所有异或
} aoaCaliMsg_t;

// 在Q3中，收到配置指令，会将参数和本地其他的参数在一起，写入格式如下的结构体中，并保存在flash地址 0x000F0000 中
typedef struct {
    signed short pdoa1;    // pdoa1 offset ，单位pi/6434
    signed short pdoa2;    // pdoa2 offset，单位pi/6434
    signed short pdoa3;    // 预留，在Q3中没有pdoa3
    unsigned char role;    // 设备角色（TAG / ANCHOR）
    unsigned char rsv;    // 预留字节
    float aoaOffset;    // AoA整体的offset // 只用来旋转到想要的AoA的0°方向
    float aoaCali[360];  // AoA在每一度上的原始测角
    char SN[16];    // 设备的SN编号
    unsigned short rangeCali[58];  // 如第五章测距校准点位上的原始测距
} uwb_config_t;

// 从 32bit 打包值中取出 I/Q（低16=I，高16=Q，均为有符号）
static inline c64 iq_from_u32(uint32_t x) {
    int16_t I = (int16_t)(x & 0xFFFF);
    int16_t Q = (int16_t)((x >> 16) & 0xFFFF);
    c64 z = { (double)I, (double)Q };
    return z;
}

/* 从任意通道数组 src 采一段窗口到 dst（长度由宏控制） */
void collect_cir_from(const uint32_t* src, uint32_t (*dst)[CIR_FOR_PDOA_LENGTH],int subfrm_idx,uint16_t firstPeakIdx);
/* 根据CIR数据计算PDOA*/
double cal_pdoa(const uint32_t CIR_a[3][CIR_FOR_PDOA_LENGTH], const uint32_t CIR_b[3][CIR_FOR_PDOA_LENGTH]);
/* 根据PDOA数据计算AOA*/
double aoa_compute_from_pdoa(double phi1_0, double phi2_0, double phi2_1, uwb_config_t *uwb_config);
