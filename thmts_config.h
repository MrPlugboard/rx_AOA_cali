//  节点信息结构体

#ifndef THMTS_CONFIG_H
#define THMTS_CONFIG_H

// incldue necessary headers
#include <stdint.h>

//设置内外环模式

//#define LOOP_MODE  1       //设置为内环模式
#define LOOP_MODE  0     //设置为外环模式

//设置节点状态为主节点或者从节点
//#define MASTER_MODE         //设置为主节点
#define SLAVE_MODE         //设置为从节点

#define SlotPeriodInMS   50
#define SlotNunInFrame   80


#define AGC_MODE       // AGC模式
//#define FIX_GAIN_MODE  // 固定增益模式

#define AD_MODE           // rf配置到AD
//#define PAD_MODE        // rf配置到PAD
//#define AD_PAD_MODE     // rf配置到AD和PAD


#endif // THMTS_CONFIG_H
