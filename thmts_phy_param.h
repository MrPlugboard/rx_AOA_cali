#ifndef THMTS_PHY_PARAM_H
#define THMTS_PHY_PARAM_H

// incldue necessary headers
#include <stdint.h>
#include "thmts_bb_config.h"

// 宏定义区域
// 物理层参数宏定义
#define BITRATE_1M_B500M                                ( 0x00 )
#define BITRATE_2M_B500M							    ( 0x01 )
#define BITRATE_4M_B500M                                ( 0x02 )
#define BITRATE_8M_B500M                                ( 0x03 )
#define BITRATE_1M_B1000M                               ( 0x04 )
#define BITRATE_2M_B1000M							    ( 0x05 )
#define BITRATE_4M_B1000M                               ( 0x06 )
#define BITRATE_8M_B1000M                               ( 0x07 )
#define BITRATE_850K_HRP31							    ( 0x08 )
#define BITRATE_850K_HRP127						        ( 0x09 )
#define BITRATE_6M8_HRP31							    ( 0x0A )
#define BITRATE_6M8_HRP127							    ( 0x0B )
#define BITRATE_1M_B500M_DIV2                     	    ( 0x0C )
#define BITRATE_2M_B500M_DIV2                           ( 0x0D )
#define BITRATE_4M_B500M_DIV2                     	    ( 0x0E )
#define BITRATE_8M_B500M_DIV2                           ( 0x0F )
#define BITRATE_1M_B1000M_DIV2                          ( 0x10 )
#define BITRATE_1M_B1000M_DIV4                          ( 0x11 )
#define BITRATE_1M_B500M_RADAR						    ( 0x12 )


// 全局物理层参数声明
extern THMTS_PhyParamConfig_t thmts_phycfg;

// 函数声明区域
// 设置通道配置声明
extern void Set_Channel_Config_By_Case(uint8_t case_idx);


#endif // THMT_PHY_PARAM_H
