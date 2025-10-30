/*
 * thmts_rf_config.c
 *
 *  Created on: 2025年4月21日
 *      Author: csj
 */

#include "thmts_rf_config.h"
#include "thmts_config.h"


// This is a configuration for the THMTSV1 clock PLL
// The configuration is used to set up the clock PLL for the THMTSV1
THMTS_RfRegConfig_t regconfig_clkpll[] = {
  {0x3A,0x300FFF,0xFFFFFF,20 },  //# R58, CLKPLL, 使用当前配置即可
  {0x1B,0x00001A,0xFFFFFF,20 },  //# R27, PLL Switch, bit<1>：CLK PLL使能，高有效；bit<3>：LO PLL时钟BUF使能，高有效；bit<4>：LO PLL使能，高有效
  {0x2F,0x00027F,0xFFFFFF,20 },  //# R47, CLKPLL, bit<13:7>：比较周期，保持当前配置即可；bit<6:0>：切换稳定时间（越长越好），使用当前配置即可；
  {0x35,0x1FFF40,0xFFFFFF,20 },  //# R53, CLKPLL, 使用当前配置即可
  {0x38,0x01805F,0xFFFFFF,20 },  //# R56, CLKPLL, 使用当前配置即可
  {0x37,0x80001F,0xFFFFFF,20 },  //# R55, CLKPLL, 所有寄存器用于带宽调整
  {0x36,0x20C520,0xFFFFFF,20 },  //# R54, CLKPLL, bit<21>：复位信号同步使能，高有效；其余寄存器使用当前配置即可
  {0x36,0x60C520,0xFFFFFF,20 },  //# R54, CLKPLL, bit<22>：分频器内部寄存器复位，高有效；其余寄存器使用当前配置即可
  {0x36,0x20C520,0xFFFFFF,20 },  //# R54, CLKPLL, bit<21>：复位信号同步使能，高有效；其余寄存器使用当前配置即可
  {0x36,0xA0C520,0xFFFFFF,20 },  //# R54, CLKPLL, bit<23>：分频器复位，低有效，bit<23:21>用于CLKPLL中的三分频模块；其余寄存器使用当前配置即可
  {0x33,0x7DFADF,0xFFFFFF,20 },  //# R51, CLKPLL, bit<23>；选择耦合方式，1-DC耦合，0-AC耦合；其余寄存器使用当前配置即可
  {0x2D,0xF14627,0xFFFFFF,200},  //# R45, CLKPLL, bit<14>：分频比对应15~62；bit<13>：分频比对应48~78；bit<12>：分频比对应63~128
  {0x2D,0xF14727,0xFFFFFF,200}   //# R45, CLKPLL, bit<8>：调频启动信号，配完分频值后，先拉高，再拉低，再拉高；bit<7:0>：设置分频值；其余参数使用当前配置即可
};

// This is a configuration for the THMTSV1 DAC
// The configuration is used to set up the DAC for the THMTSV1
THMTS_RfRegConfig_t regconfig_dac[] = {
  {0x40,0x833FBD,0xFFFFFF,20 },  //# R64, 配置DAC测试码，6bits对应一个采样点
  {0x41,0xBDC281,0xFFFFFF,20 },  //# R65, 配置DAC测试码，6bits对应一个采样点
  {0x42,0x154A39,0xFFFFFF,20 },  //# R66, 配置DAC测试码，6bits对应一个采样点
  {0x43,0xFF9A14,0xFFFFFF,20 },  //# R67, 配置DAC测试码，6bits对应一个采样点
  {0x44,0x14129C,0xFFFFFF,20 },  //# R68, 配置DAC测试码，6bits对应一个采样点
  {0x45,0xC3DFB3,0xFFFFFF,20 },  //# R69, 配置DAC测试码，6bits对应一个采样点
  {0x46,0x80D083,0xFFFFFF,20 },  //# R70, 配置DAC测试码，6bits对应一个采样点
  {0x47,0x424DBF,0xFFFFFF,20 },  //# R71, 配置DAC测试码，6bits对应一个采样点
  {0x48,0xEEC607,0xFFFFFF,20 },  //# R72, 配置DAC测试码，6bits对应一个采样点
  {0x49,0x04762C,0xFFFFFF,20 },  //# R73, 配置DAC测试码，6bits对应一个采样点
  {0x4A,0xEFFDA4,0xFFFFFF,20 },  //# R74, 配置DAC测试码，6bits对应一个采样点
  {0x4B,0x44308D,0xFFFFFF,20 },  //# R75, 配置DAC测试码，6bits对应一个采样点
  {0x4C,0x000000,0xFFFFFF,20 },  //# R76, 初始化寄存器，所有bit置为0
  {0x4C,0x100003,0xFFFFFF,20 },  //# R76, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd，低有效；<22:20>: 750MHz时钟选择信号,当前配置001
  {0x4D,0x000000,0xFFFFFF,20 },  //# R77, 初始化寄存器，所有bit置为0
  {0x4D,0x000004,0xFFFFFF,20 },  //# R77, bit<2>：对模拟复位作内部同步，高有效；其余寄存器使用当前配置即可
  {0x4D,0x000005,0xFFFFFF,20 },  //# R77, bit<0>：时钟分频模块触发器复位，高有效；其余寄存器使用当前配置即可
  {0x4D,0x000004,0xFFFFFF,20 },  //# R77, bit<2>：对模拟复位作内部同步，高有效；其余寄存器使用当前配置即可
  {0x4D,0x000006,0xFFFFFF,20 }   //# R77, bit<1>：时钟分频模块复位，低有效；其余寄存器使用当前配置即可；bit<2:0>用于dac内部的分频模块
};

// This is a configuration for the THMTSV1 LOPLL
// The configuration is used to set up the LOPLL for the THMTSV1
THMTS_RfRegConfig_t regconfig_lopll[] = {
  {0x29,0x300FFF,0xFFFFFF,20 },  //# R41, LOPLL, bit<11:0>：锁定检测配置，保持默认值即可
  {0x1B,0x00001A,0xFFFFFF,20 },  //# R27, PLL Switch, bit<1>：CLK PLL使能，高有效；bit<3>：LO PLL时钟BUF使能，高有效；bit<4>：LO PLL使能，高有效
  {0x1E,0x00027F,0xFFFFFF,20 },  //# R30, LOPLL, bit<13:7>：比较周期，保持当前配置即可；bit<6:0>：切换稳定时间（越长越好），使用当前配置即可；
  {0x1F,0x6E0588,0xFFFFFF,20 },  //# R31, LOPLL, 默认值即可
  {0x22,0x7DFADF,0xFFFFFF,20 },  //# R34, LOPLL, bit<23>：选择耦合方式，1：DC耦合；0：AC耦合；bit<23>：开启后增加VCO供电LDO的稳定性
  {0x25,0xA84020,0xFFFFFF,20 },  //# R37, LOPLL, bit<23>：向TX送时钟开启；bit<22>：加强向TX送时钟的驱动；bit<21>：向RX送时钟开启；bit<20>：加强向RX送时钟的驱动；bit<13>：打开测试口，外供本振或从外部测试本振，都需要使能该bit；bit<12>：开启测试口本振外灌模式
  {0x26,0x80001F,0xFFFFFF,20 },  //# R38, LOPLL, 24bits用于PLL带宽调节，00001F最宽，FFFFE0最窄。使用当前配置即可
  {0x24,0x1FFF40,0xFFFFFF,20 },  //# R36, LOPLL, 默认值即可
  {0x27,0x03845F,0xFFFFFF,20 },  //# R39, LOPLL, bit<11>：高频VCO使能，约5.9~8.4G；bit<10>：低频VCO使能，约5.6~8.3G；其余参数使用当前配置即可
  {0x1C,0xC21768,0xFFFFFF,200},  //# R28, LOPLL, bit<14>：分频比对应15~62；bit<13>：分频比对应48~78；bit<12>：分频比对应63~128
  {0x1C,0xC31768,0xFFFFFF,200}   //# R28, LOPLL, bit<8>：调频启动信号，配完分频值后，先拉高，再拉低，再拉高；bit<7:0>：设置分频值；其余参数使用当前配置即可
};




THMTS_RfRegConfig_t regconfig_adc0[] = {
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x600000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0xA00000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x60,0x012345,0xFFFFFF,20 }, //# ADC test code
  {0x61,0x6789AB,0xFFFFFF,20 }, //# ADC test code
  {0x62,0x00CDEF,0xFFFFFF,20 }, //# ADC test code
  {0x63,0x000000,0xFFFFFF,20 }, //# ADC reset and static settings
//  {0x63,0x000933,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x63,0x000973,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x64,0x000111,0xFFFFFF,20 }, //#
  {0x64,0x000112,0xFFFFFF,20 }, //#
  {0x64,0x000113,0xFFFFFF,20 }, //#
  {0x64,0x000114,0xFFFFFF,20 }, //#
  {0x64,0x000115,0xFFFFFF,20 }, //#
  {0x64,0x000116,0xFFFFFF,20 }, //#
  {0x64,0x000117,0xFFFFFF,20 }, //#
  {0x64,0x000118,0xFFFFFF,20 }, //#
  {0x64,0x000119,0xFFFFFF,20 }, //#
  {0x64,0x00011A,0xFFFFFF,20 }, //#
  {0x64,0x00011B,0xFFFFFF,20 }, //#
  {0x64,0x00011C,0xFFFFFF,20 }, //#
  {0x64,0x00011D,0xFFFFFF,20 }, //#
  {0x64,0x00011E,0xFFFFFF,20 }, //#
  {0x64,0x00011F,0xFFFFFF,20 }, //#
  {0x64,0x100110,0xFFFFFF,20 }, //# ADC I calib complete set high
  {0x65,0x000111,0xFFFFFF,20 }, //#
  {0x65,0x000112,0xFFFFFF,20 }, //#
  {0x65,0x000113,0xFFFFFF,20 }, //#
  {0x65,0x000114,0xFFFFFF,20 }, //#
  {0x65,0x000115,0xFFFFFF,20 }, //#
  {0x65,0x000116,0xFFFFFF,20 }, //#
  {0x65,0x000117,0xFFFFFF,20 }, //#
  {0x65,0x000118,0xFFFFFF,20 }, //#
  {0x65,0x000119,0xFFFFFF,20 }, //#
  {0x65,0x00011A,0xFFFFFF,20 }, //#
  {0x65,0x00011B,0xFFFFFF,20 }, //#
  {0x65,0x00011C,0xFFFFFF,20 }, //#
  {0x65,0x00011D,0xFFFFFF,20 }, //#
  {0x65,0x00011E,0xFFFFFF,20 }, //#
  {0x65,0x00011F,0xFFFFFF,20 }, //#
  {0x65,0x100110,0xFFFFFF,20 }  //# ADC Q calib complete set high
};

THMTS_RfRegConfig_t regconfig_adc1[] = {
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x600000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0xA00000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x68,0x012345,0xFFFFFF,20 }, //# ADC test code
  {0x69,0x6789AB,0xFFFFFF,20 }, //# ADC test code
  {0x6A,0x00CDEF,0xFFFFFF,20 }, //# ADC test code
  {0x6B,0x000000,0xFFFFFF,20 }, //# ADC reset and static settings
//  {0x6B,0x000933,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x6B,0x000973,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x6C,0x000111,0xFFFFFF,20 }, //#
  {0x6C,0x000112,0xFFFFFF,20 }, //#
  {0x6C,0x000113,0xFFFFFF,20 }, //#
  {0x6C,0x000114,0xFFFFFF,20 }, //#
  {0x6C,0x000115,0xFFFFFF,20 }, //#
  {0x6C,0x000116,0xFFFFFF,20 }, //#
  {0x6C,0x000117,0xFFFFFF,20 }, //#
  {0x6C,0x000118,0xFFFFFF,20 }, //#
  {0x6C,0x000119,0xFFFFFF,20 }, //#
  {0x6C,0x00011A,0xFFFFFF,20 }, //#
  {0x6C,0x00011B,0xFFFFFF,20 }, //#
  {0x6C,0x00011C,0xFFFFFF,20 }, //#
  {0x6C,0x00011D,0xFFFFFF,20 }, //#
  {0x6C,0x00011E,0xFFFFFF,20 }, //#
  {0x6C,0x00011F,0xFFFFFF,20 }, //#
  {0x6C,0x100110,0xFFFFFF,20 }, //# ADC I calib complete set high
  {0x6D,0x000111,0xFFFFFF,20 }, //#
  {0x6D,0x000112,0xFFFFFF,20 }, //#
  {0x6D,0x000113,0xFFFFFF,20 }, //#
  {0x6D,0x000114,0xFFFFFF,20 }, //#
  {0x6D,0x000115,0xFFFFFF,20 }, //#
  {0x6D,0x000116,0xFFFFFF,20 }, //#
  {0x6D,0x000117,0xFFFFFF,20 }, //#
  {0x6D,0x000118,0xFFFFFF,20 }, //#
  {0x6D,0x000119,0xFFFFFF,20 }, //#
  {0x6D,0x00011A,0xFFFFFF,20 }, //#
  {0x6D,0x00011B,0xFFFFFF,20 }, //#
  {0x6D,0x00011C,0xFFFFFF,20 }, //#
  {0x6D,0x00011D,0xFFFFFF,20 }, //#
  {0x6D,0x00011E,0xFFFFFF,20 }, //#
  {0x6D,0x00011F,0xFFFFFF,20 }, //#
  {0x6D,0x100110,0xFFFFFF,20 }  //# ADC Q calib complete set high
};

THMTS_RfRegConfig_t regconfig_adc2[] = {
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x600000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0xA00000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x70,0x012345,0xFFFFFF,20 }, //# ADC test code
  {0x71,0x6789AB,0xFFFFFF,20 }, //# ADC test code
  {0x72,0x00CDEF,0xFFFFFF,20 }, //# ADC test code
  {0x73,0x000000,0xFFFFFF,20 }, //# ADC reset and static settings
//  {0x73,0x000933,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x73,0x000973,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x74,0x000111,0xFFFFFF,20 }, //#
  {0x74,0x000112,0xFFFFFF,20 }, //#
  {0x74,0x000113,0xFFFFFF,20 }, //#
  {0x74,0x000114,0xFFFFFF,20 }, //#
  {0x74,0x000115,0xFFFFFF,20 }, //#
  {0x74,0x000116,0xFFFFFF,20 }, //#
  {0x74,0x000117,0xFFFFFF,20 }, //#
  {0x74,0x000118,0xFFFFFF,20 }, //#
  {0x74,0x000119,0xFFFFFF,20 }, //#
  {0x74,0x00011A,0xFFFFFF,20 }, //#
  {0x74,0x00011B,0xFFFFFF,20 }, //#
  {0x74,0x00011C,0xFFFFFF,20 }, //#
  {0x74,0x00011D,0xFFFFFF,20 }, //#
  {0x74,0x00011E,0xFFFFFF,20 }, //#
  {0x74,0x00011F,0xFFFFFF,20 }, //#
  {0x74,0x100110,0xFFFFFF,20 }, //# ADC I calib complete set high
  {0x75,0x000111,0xFFFFFF,20 }, //#
  {0x75,0x000112,0xFFFFFF,20 }, //#
  {0x75,0x000113,0xFFFFFF,20 }, //#
  {0x75,0x000114,0xFFFFFF,20 }, //#
  {0x75,0x000115,0xFFFFFF,20 }, //#
  {0x75,0x000116,0xFFFFFF,20 }, //#
  {0x75,0x000117,0xFFFFFF,20 }, //#
  {0x75,0x000118,0xFFFFFF,20 }, //#
  {0x75,0x000119,0xFFFFFF,20 }, //#
  {0x75,0x00011A,0xFFFFFF,20 }, //#
  {0x75,0x00011B,0xFFFFFF,20 }, //#
  {0x75,0x00011C,0xFFFFFF,20 }, //#
  {0x75,0x00011D,0xFFFFFF,20 }, //#
  {0x75,0x00011E,0xFFFFFF,20 }, //#
  {0x75,0x00011F,0xFFFFFF,20 }, //#
  {0x75,0x100110,0xFFFFFF,20 }  //# ADC Q calib complete set high
};

THMTS_RfRegConfig_t regconfig_adc3[] = {
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x600000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0x200000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x36,0xA00000,0xE00000,200}, // # R54, CLKPLL, bit<23:21> is used for resetting 2G clock div3 module
  {0x78,0x012345,0xFFFFFF,20 }, //# ADC test code
  {0x79,0x6789AB,0xFFFFFF,20 }, //# ADC test code
  {0x7A,0x00CDEF,0xFFFFFF,20 }, //# ADC test code
  {0x7B,0x000000,0xFFFFFF,20 }, //# ADC reset and static settings
//  {0x7B,0x000933,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x7B,0x000973,0xFFFFFF,20 }, //# ADC reset and static settings
  {0x7C,0x000111,0xFFFFFF,20 }, //#
  {0x7C,0x000112,0xFFFFFF,20 }, //#
  {0x7C,0x000113,0xFFFFFF,20 }, //#
  {0x7C,0x000114,0xFFFFFF,20 }, //#
  {0x7C,0x000115,0xFFFFFF,20 }, //#
  {0x7C,0x000116,0xFFFFFF,20 }, //#
  {0x7C,0x000117,0xFFFFFF,20 }, //#
  {0x7C,0x000118,0xFFFFFF,20 }, //#
  {0x7C,0x000119,0xFFFFFF,20 }, //#
  {0x7C,0x00011A,0xFFFFFF,20 }, //#
  {0x7C,0x00011B,0xFFFFFF,20 }, //#
  {0x7C,0x00011C,0xFFFFFF,20 }, //#
  {0x7C,0x00011D,0xFFFFFF,20 }, //#
  {0x7C,0x00011E,0xFFFFFF,20 }, //#
  {0x7C,0x00011F,0xFFFFFF,20 }, //#
  {0x7C,0x100110,0xFFFFFF,20 }, //# ADC I calib complete set high
  {0x7D,0x000111,0xFFFFFF,20 }, //#
  {0x7D,0x000112,0xFFFFFF,20 }, //#
  {0x7D,0x000113,0xFFFFFF,20 }, //#
  {0x7D,0x000114,0xFFFFFF,20 }, //#
  {0x7D,0x000115,0xFFFFFF,20 }, //#
  {0x7D,0x000116,0xFFFFFF,20 }, //#
  {0x7D,0x000117,0xFFFFFF,20 }, //#
  {0x7D,0x000118,0xFFFFFF,20 }, //#
  {0x7D,0x000119,0xFFFFFF,20 }, //#
  {0x7D,0x00011A,0xFFFFFF,20 }, //#
  {0x7D,0x00011B,0xFFFFFF,20 }, //#
  {0x7D,0x00011C,0xFFFFFF,20 }, //#
  {0x7D,0x00011D,0xFFFFFF,20 }, //#
  {0x7D,0x00011E,0xFFFFFF,20 }, //#
  {0x7D,0x00011F,0xFFFFFF,20 }, //#
  {0x7D,0x100110,0xFFFFFF,20 }  //# ADC Q calib complete set high
};

// This is a configuration for the THMTSV1 TX
// The configuration is used to set up the TX for the THMTSV1
THMTS_RfRegConfig_t regconfig_tx[] = {
  {0x14,0x000000,0x000200,20 },  //# R20, bit<9>：TX总关断，1关断，0使能；
  {0x25,0x400000,0x400000,20 }   //# R37, bit<23>：向TX送时钟开启；bit<22>：加强向TX送时钟的驱动；bit<21>：向RX送时钟开启；bit<20>：加强向RX送时钟的驱动；
};

// This is a configuration for the THMTSV1 RX
// The configuration is used to set up the RX for the THMTSV1
THMTS_RfRegConfig_t regconfig_rx0[] = {
  {0x00,0x000000,0x000300,20},  // # config channel 0 (0,1,2,3)
  {0x00,0x000000,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001fe1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000002,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001dc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000004,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001ba1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000006,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001981,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000008,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001761,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00000a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001541,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00000c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001321,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00000e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001101,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000010,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000ee1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000012,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000cc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000014,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000aa1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000016,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000881,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000018,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000661,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00001a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000441,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00001c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000221,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00001e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000020,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000089,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000022,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000109,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000024,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000209,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000026,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000409,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000028,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000809,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00002a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00002c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00002e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000030,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000032,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000051,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000034,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000091,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000036,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000111,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x000038,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000211,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003e80,0xFFFFFF,20},  //
  {0x00,0x00003a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000411,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x00003c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000811,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x00003e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x000040,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x000042,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003900,0xFFFFFF,20},  //
  {0x00,0x000044,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000061,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003d00,0xFFFFFF,20},  //
  {0x00,0x000046,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a1,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003d00,0xFFFFFF,20},  //
  {0x00,0x000048,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000121,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003d00,0xFFFFFF,20},  //
  {0x00,0x00004a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000221,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003c00,0xFFFFFF,20},  //
  {0x00,0x00004c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000421,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003b80,0xFFFFFF,20},  //
  {0x00,0x00004e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000821,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003b80,0xFFFFFF,20},  //
  {0x00,0x000050,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001021,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003b80,0xFFFFFF,20},  //
  {0x00,0x000052,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002021,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x000054,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004021,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x00,0x000056,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000062,0xFFFFFF,20},  //
  {0x09,0x004080,0xFFFFFF,20},  //
  {0x0A,0x003e00,0xFFFFFF,20},  //
  {0x00,0x000058,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a2,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003d00,0xFFFFFF,20},  //
  {0x00,0x00005a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000122,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003c00,0xFFFFFF,20},  //
  {0x00,0x00005c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000222,0xFFFFFF,20},  //
  {0x09,0x004080,0xFFFFFF,20},  //
  {0x0A,0x003b00,0xFFFFFF,20},  //
  {0x00,0x00005e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000422,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x003a00,0xFFFFFF,20},  //
  {0x01,0x000000,0x000040,20},  // # set bit<6> to 0 to turn on RX
  {0x02,0x0088FF,0xFFFFFF,20},  // # default value, 88 for 4GHz LO
  {0x03,0x3F3FE2,0xFFFFFF,20},  // #
  {0x04,0x100000,0xFFFFFF,20},  // # default value
  {0x05,0x10F005,0xFFFFFF,20},  //
  {0x06,0x01888C,0xFFFFFF,20}  //
};


THMTS_RfRegConfig_t regconfig_rx1[] = {
  {0x00,0x000100,0x000300,20},  // # config channel 1 (0,1,2,3)
  {0x00,0x000000,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001fe1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000002,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001dc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000004,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001ba1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000006,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001981,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000008,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001761,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00000a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001541,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00000c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001321,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00000e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001101,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000010,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000ee1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000012,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000cc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000014,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000aa1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000016,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000881,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x000018,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000661,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00001a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000441,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00001c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000221,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x00001e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000020,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000089,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000022,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000109,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000024,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000209,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000026,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000409,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000028,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000809,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x00002a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x00002c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x006000,0xFFFFFF,20},  //
  {0x00,0x00002e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004009,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x006000,0xFFFFFF,20},  //
  {0x00,0x000030,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001049,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x006000,0xFFFFFF,20},  //
  {0x00,0x000032,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000051,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000034,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000091,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005400,0xFFFFFF,20},  //
  {0x00,0x000036,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000111,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005400,0xFFFFFF,20},  //
  {0x00,0x000038,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000211,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005400,0xFFFFFF,20},  //
  {0x00,0x00003a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000411,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005400,0xFFFFFF,20},  //
  {0x00,0x00003c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000811,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x00003e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000040,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005800,0xFFFFFF,20},  //
  {0x00,0x000042,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004011,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005c00,0xFFFFFF,20},  //
  {0x00,0x000044,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000061,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000046,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a1,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004e00,0xFFFFFF,20},  //
  {0x00,0x000048,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000121,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005000,0xFFFFFF,20},  //
  {0x00,0x00004a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000221,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x005000,0xFFFFFF,20},  //
  {0x00,0x00004c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000421,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x005000,0xFFFFFF,20},  //
  {0x00,0x00004e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000821,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x005200,0xFFFFFF,20},  //
  {0x00,0x000050,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001021,0xFFFFFF,20},  //
  {0x09,0x003f00,0xFFFFFF,20},  //
  {0x0A,0x005200,0xFFFFFF,20},  //
  {0x00,0x000052,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002021,0xFFFFFF,20},  //
  {0x09,0x004100,0xFFFFFF,20},  //
  {0x0A,0x005400,0xFFFFFF,20},  //
  {0x00,0x000054,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004021,0xFFFFFF,20},  //
  {0x09,0x004140,0xFFFFFF,20},  //
  {0x0A,0x005600,0xFFFFFF,20},  //
  {0x00,0x000056,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000062,0xFFFFFF,20},  //
  {0x09,0x004080,0xFFFFFF,20},  //
  {0x0A,0x004b00,0xFFFFFF,20},  //
  {0x00,0x000058,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a2,0xFFFFFF,20},  //
  {0x09,0x004100,0xFFFFFF,20},  //
  {0x0A,0x004e00,0xFFFFFF,20},  //
  {0x00,0x00005a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000122,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004c80,0xFFFFFF,20},  //
  {0x00,0x00005c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000222,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x004d00,0xFFFFFF,20},  //
  {0x00,0x00005e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000422,0xFFFFFF,20},  //
  {0x09,0x004000,0xFFFFFF,20},  //
  {0x0A,0x005200,0xFFFFFF,20},  //
  {0x01,0x000000,0x000040,20},  // # set bit<6> to 0 to turn on RX
  {0x02,0x0088FF,0xFFFFFF,20},  // # default value, 88 for 4GHz LO
  {0x03,0x3F3FE2,0xFFFFFF,20},  // #
  {0x04,0x100000,0xFFFFFF,20},  // # default value
  {0x05,0x10F005,0xFFFFFF,20},  //
  {0x06,0x01888C,0xFFFFFF,20}  //
};

THMTS_RfRegConfig_t regconfig_rx2[] = {
  {0x00,0x000200,0x000300,20},  // # config channel 2 (0,1,2,3)
  {0x00,0x000000,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001fe1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004400,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000002,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001dc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000004,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001ba1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000006,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001981,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000008,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001761,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00000a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001541,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00000c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001321,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00000e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001101,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000010,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000ee1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000012,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000cc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000014,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000aa1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000016,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000881,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000018,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000661,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00001a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000441,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00001c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000221,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00001e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000020,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000089,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000022,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000109,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000024,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000209,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000026,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000409,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000028,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000809,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00002a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001009,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00002c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002009,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00002e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004009,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000030,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001049,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000032,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000051,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000034,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000091,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000036,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000111,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000038,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000211,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00003a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000411,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00003c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000811,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00003e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001011,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000040,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002011,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000042,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004011,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000044,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000061,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000046,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a1,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000048,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000121,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00004a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000221,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00004c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000421,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00004e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000821,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000050,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001021,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000052,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002021,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000054,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004021,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000056,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000062,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x000058,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a2,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00005a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000122,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00005c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000222,0xFFFFFF,20},  //
  {0x09,0x004580,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x00,0x00005e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000422,0xFFFFFF,20},  //
  {0x09,0x004600,0xFFFFFF,20},  //
  {0x0A,0x004400,0xFFFFFF,20},  //
  {0x01,0x000000,0x000040,20},  // # set bit<6> to 0 to turn on RX
  {0x02,0x0088FF,0xFFFFFF,20},  // # default value, 88 for 4GHz LO
  {0x03,0x3F3FE2,0xFFFFFF,20},  // #
  {0x04,0x100000,0xFFFFFF,20},  // # default value
  {0x05,0x10F005,0xFFFFFF,20},  //
  {0x06,0x01888C,0xFFFFFF,20}  //
};

THMTS_RfRegConfig_t regconfig_rx3[] = {
  {0x00,0x000300,0x000300,20},  // # config channel 3 (0,1,2,3)
  {0x00,0x000000,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001fe1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000002,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001dc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000004,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001ba1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000006,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001981,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000008,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001761,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00000a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001541,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00000c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001321,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00000e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x001101,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000010,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000ee1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000012,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000cc1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000014,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000aa1,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000016,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000881,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000018,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000661,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00001a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000441,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00001c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000221,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00001e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000020,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000089,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000022,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000109,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000024,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000209,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000026,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000409,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000028,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000809,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00002a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001009,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00002c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002009,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00002e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004009,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000030,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001049,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000032,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000051,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000034,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000091,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000036,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000111,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000038,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000211,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00003a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000411,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00003c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000811,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00003e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001011,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000040,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002011,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000042,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004011,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000044,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000061,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x000046,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a1,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000048,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000121,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x00004a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000221,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x00004c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000421,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004a00,0xFFFFFF,20},  //
  {0x00,0x00004e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000821,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000050,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x001021,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000052,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x002021,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000054,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x004021,0xFFFFFF,20},  //
  {0x09,0x003800,0xFFFFFF,20},  //
  {0x0A,0x004c00,0xFFFFFF,20},  //
  {0x00,0x000056,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000062,0xFFFFFF,20},  //
  {0x09,0x003c00,0xFFFFFF,20},  //
  {0x0A,0x004700,0xFFFFFF,20},  //
  {0x00,0x000058,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x0000a2,0xFFFFFF,20},  //
  {0x09,0x003c00,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00005a,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000122,0xFFFFFF,20},  //
  {0x09,0x003c00,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00005c,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000222,0xFFFFFF,20},  //
  {0x09,0x003c00,0xFFFFFF,20},  //
  {0x0A,0x004800,0xFFFFFF,20},  //
  {0x00,0x00005e,0x00007E,20},  // # config dagc_conf_addr, bit<6:1>
  {0x07,0x000001,0xFFFFFF,20},  //
  {0x08,0x000422,0xFFFFFF,20},  //
  {0x09,0x003a00,0xFFFFFF,20},  //
  {0x0A,0x004900,0xFFFFFF,20},  //
  {0x01,0x000000,0x000040,20},  // # set bit<6> to 0 to turn on RX
  {0x02,0x0088FF,0xFFFFFF,20},  // # default value, 88 for 4GHz LO
  {0x03,0x3F3FE2,0xFFFFFF,20},  // #
  {0x04,0x100000,0xFFFFFF,20},  // # default value
  {0x05,0x10F005,0xFFFFFF,20},  //
  {0x06,0x01888C,0xFFFFFF,20}  //
};

// This is a configuration for the THMTSV1 SW Normal
// The configuration is used to set up the SW Normal for the THMTSV1
THMTS_RfRegConfig_t regconfig_sw_normal[] = {
  {0x0B,0x001000,0x007000,20 }, //# R11, bit<15>：测试单元方向开关使能；bit<14>：中频PAD方向开关使能；bit<13>：LPF方向开关使能；bit<12>：AD方向开关使能；
  {0x14,0x000008,0x000038,20 }  //# R20, bit<9>：TX总关断，1关断，0使能；bit<5>：基带DA和PAD之间开关使能；bit<4>：MIX+PA和PAD之间开关使能；bit<3>：基带DA和MIX+PA之间开关使能
};

THMTS_RfRegConfig_t regconfig_dagc_enable[] = {
  {0x00,0x000080,0x000080,20}  //# R0, bit<7>：选择由基带或是SPI控制接收通道的增益和直流失调，1：基带控；0：SPI控；
};

THMTS_RfRegConfig_t regconfig_dagc_disable[] = {
  {0x00,0x000000,0x000080,20}  //# R0, bit<7>：选择由基带或是SPI控制接收通道的增益和直流失调，1：基带控；0：SPI控；
};

THMTS_RfRegConfig_t regconfig_trsw_enable[] = {
  {0x00,0x000001,0x000001,20}  //# R0, bit<7>：选择由基带或是SPI控制接收通道的增益和直流失调，1：基带控；0：SPI控；
};

THMTS_RfRegConfig_t regconfig_trsw_disable[] = {
  {0x00,0x000000,0x000001,20}  //# R0, bit<7>：选择由基带或是SPI控制接收通道的增益和直流失调，1：基带控；0：SPI控；
};


THMTS_RfRegConfig_t regconfig_clk_enable[] = {
  {0x00,0x000800,0x000800,20}  //# R0, bit<11>：基带时钟选择，1：选择clk_dac作为主时钟, 0：选择38.4MHz；
};

THMTS_RfRegConfig_t regconfig_clk_disable[] = {
  {0x00,0x000000,0x000800,20}  //# R0, bit<11>：基带时钟选择，1：选择clk_dac作为主时钟, 0：选择38.4MHz；
};

THMTS_RfRegConfig_t regconfig_clk_div_enable[] = {
  {0x00,0x000400,0x000400,20}  //# R0, bit<10>：CPU时钟分频使能，1：2分频, 0：不分频；
};

THMTS_RfRegConfig_t regconfig_clk_div_disable[] = {
  {0x00,0x000000,0x000400,20}  //# R0, bit<10>：CPU时钟分频使能，1：2分频, 0：不分频；
};

THMTS_RfRegConfig_t regconfig_tx_pd[] = {
  {0x14,0x000200,0x000200,20}  //# R20, bit<9>：TX总关断，1关断，0使能；
};

THMTS_RfRegConfig_t regconfig_rx0_pd[] = {
  {0x00,0x000000,0x000300,20},  //# R0, bit<9:8>：接收通道配置选择，对应0~3；
  {0x01,0x000040,0x000040,20}   //# R1, bit<6>：RX通道pd信号，0工作；
};

THMTS_RfRegConfig_t regconfig_rx1_pd[] = {
  {0x00,0x000100,0x000300,20},  //# R0, bit<9:8>：接收通道配置选择，对应0~3；
  {0x01,0x000040,0x000040,20}   //# R1, bit<6>：RX通道pd信号，0工作；
};

THMTS_RfRegConfig_t regconfig_rx2_pd[] = {
  {0x00,0x000200,0x000300,20},  //# R0, bit<9:8>：接收通道配置选择，对应0~3；
  {0x01,0x000040,0x000040,20}   //# R1, bit<6>：RX通道pd信号，0工作；
};

THMTS_RfRegConfig_t regconfig_rx3_pd[] = {
  {0x00,0x000300,0x000300,20},  //# R0, bit<9:8>：接收通道配置选择，对应0~3；
  {0x01,0x000040,0x000040,20}   //# R1, bit<6>：RX通道pd信号，0工作；
};

THMTS_RfRegConfig_t regconfig_dac_pd[] = {
  {0x4C,0x000000,0xFFFFFF,20},  //# R76, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd;
};

THMTS_RfRegConfig_t regconfig_adc0_pd[] = {
  {0x63,0x000000,0xFFFFFF,20}, //# R99, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd，低有效；
};

THMTS_RfRegConfig_t regconfig_adc1_pd[] = {
  {0x6B,0x000000,0xFFFFFF,20}, //# R107, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd，低有效；
};

THMTS_RfRegConfig_t regconfig_adc2_pd[] = {
  {0x73,0x000000,0xFFFFFF,20}, //# R115, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd，低有效；
};

THMTS_RfRegConfig_t regconfig_adc3_pd[] = {
  {0x7B,0x000000,0xFFFFFF,20}, //# R123, bit<0>：adc模拟部分pd，低有效；bit<1>：adc数字部分pd，低有效；
};

THMTS_RfRegConfig_t regconfig_da2lpf[] = {
  {0x14,0x000008,0x000038,20 }
};

THMTS_RfRegConfig_t regconfig_pad2lpf[] = {
  {0x14,0x000010,0x000038,20 }
};

THMTS_RfRegConfig_t regconfig_da2pad[] = {
  {0x14,0x000020,0x000038,20 }
};

THMTS_RfRegConfig_t regconfig_lpf2ad[] = {
  {0x0B,0x001000,0x007000,20 }
};

THMTS_RfRegConfig_t regconfig_lpf2pad[] = {
  {0x0B,0x002000,0x007000,20 }
};

THMTS_RfRegConfig_t regconfig_lpf2pad_and_lpf2ad[] = {
  {0x0B,0x003000,0x007000,20 }
};


THMTS_RfRegConfig_t regconfig_pad2ad[] = {
  {0x0B,0x004000,0x007000,20 }
};


#define RF_SPI QSPI1

void write_thmts_rf_reg(uint32_t addr, uint32_t data)
{
    uint32_t spi_data;
    uint32_t rw_flag = 0;
    QSPI_DirectionConfig(RF_SPI, QSPI_FMT_DIR_TX);
    spi_data = (addr << 24) + data + (rw_flag<<31);
    while (SET == QSPI_GetFlag(RF_SPI, QSPI_STATUS_TX_FULL)) {}
    QSPI_SendData(RF_SPI, spi_data);

    for( uint32_t i = 0; i < 38 * 200; i++ );

    return;
}

uint32_t read_thmts_rf_reg(uint32_t addr)
{
    uint32_t spi_data;
    uint32_t rw_flag = 1;
    QSPI_DirectionConfig(RF_SPI, QSPI_FMT_DIR_RX);
    spi_data = (addr << 24)  + (rw_flag<<31);
    while (SET == QSPI_GetFlag(RF_SPI, QSPI_STATUS_TX_FULL)) {}
    QSPI_SendData(RF_SPI, spi_data);
    while (SET == QSPI_GetFlag(RF_SPI, QSPI_STATUS_RX_EMPTY)) {}
    spi_data = QSPI_ReceiveData(RF_SPI);

    return spi_data;
}


void config_thmts_rf_reg(uint8_t addr, uint32_t data, uint32_t mask)
{
	uint32_t spi_data;
	spi_data = read_thmts_rf_reg(addr);
	spi_data &= (~mask);
	spi_data |= data;
	write_thmts_rf_reg(addr, spi_data);

	return;
}

void config_thmts_rf_regs(THMTS_RfRegConfig_t* p_cfg, uint32_t num)
{
	uint8_t addr;
	uint32_t data, mask;
	uint32_t i,j;

	for (i=0; i<num; i++)
	{
		addr = (p_cfg+i)->address;
		data = (p_cfg+i)->value;
		mask = (p_cfg+i)->mask;
		config_thmts_rf_reg(addr, data, mask);

		for (j=0; j<20000; j++);
	}
}




// 半自动锁相，PLL环路初始化
void config_thmts_pll_init_mfc()
{

    write_thmts_rf_reg(58,0X300FFF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(41,0X300FFF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(27,0X00001A);
   thmts_sleep_us(20);

   write_thmts_rf_reg(30,0X00027F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(31,0X6E0588);
   thmts_sleep_us(20);
   write_thmts_rf_reg(34,0X7DFADF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(37,0XA84020);
   thmts_sleep_us(20);
   write_thmts_rf_reg(38,0X80001F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(36,0X1FFF40);
   thmts_sleep_us(20);
   write_thmts_rf_reg(39,0X03845F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(47,0X00027F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(53,0X1FFF40);
   thmts_sleep_us(20);
   write_thmts_rf_reg(56,0X01805F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(55,0X80001F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(54,0XE0CD20);
   thmts_sleep_us(20);
   write_thmts_rf_reg(51,0X7DFADF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(45,0XF14627);
   thmts_sleep_us(20);

   write_thmts_rf_reg(45,0XF14727);
   thmts_sleep_us(20);

   write_thmts_rf_reg(28,0X82175B);
   thmts_sleep_us(200);
   write_thmts_rf_reg(28,0X83175B);
   thmts_sleep_us(200);

   write_thmts_rf_reg(58,0X300FFF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(41,0X300FFF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(27,0X00001A);
   thmts_sleep_us(20);

   write_thmts_rf_reg(30,0X00027F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(31,0X6E0588);
   thmts_sleep_us(20);
   write_thmts_rf_reg(34,0X7DFADF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(37,0XA84020);
   thmts_sleep_us(20);
   write_thmts_rf_reg(38,0XFFFFF0);
   thmts_sleep_us(20);
   write_thmts_rf_reg(36,0X1FFF40);
   thmts_sleep_us(20);
   write_thmts_rf_reg(39,0X03845F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(47,0X00027F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(53,0X1FFF40);
   thmts_sleep_us(20);
   write_thmts_rf_reg(56,0X01805F);
   thmts_sleep_us(20);
   write_thmts_rf_reg(55,0XFFFFF0);
   thmts_sleep_us(20);
   write_thmts_rf_reg(54,0XE0CD20);
   thmts_sleep_us(20);
   write_thmts_rf_reg(51,0X7DFADF);
   thmts_sleep_us(20);
   write_thmts_rf_reg(45,0XF14627);
   thmts_sleep_us(20);

   write_thmts_rf_reg(45,0XF14727);
   thmts_sleep_us(20);

   write_thmts_rf_reg(28,0X82175B);
   thmts_sleep_us(200);
   write_thmts_rf_reg(28,0X83175B);
   thmts_sleep_us(200);


}




// 半自动lo pll锁相
void config_thmts_rf_lopll_mfc(uint8_t NintSet)
{


	printf("begin config lo pll mfc regs\r\n");

	int tmep_n=0;

//	uint64_t Fre_Set,fdsm;
	uint32_t CT_REF,FT_REF;
	uint32_t VCO_CntCT,VCO_CntFT,REG28,REG30,Ct,Ft,Nint,VCO_R32CT,VCO_R32FT,VCO_R32FT_F,VCO_R32CT_F,VCO_R32_LF,VCO_R32RED,VCO_CntCTFF,VCO_R32_LF1,VCO_R32RED1,VCO_R32FT_F1,VCO_R32CT_F1,VCO_R33RED0,VCO_R33RED1;
	int i=0;
//	int j=0;

	REG28=0X831700;			//20-28    ��WIFI��Ŀ���岻һ��
	REG30=0X00027F;
	Ct=32;
	Ft=8;



	Nint= NintSet/2;
//	Nfrac= (uint64_t)(((Fre_Set*2*pow(2,23)) / fdsm) - (Nint*pow(2,23)));


	FT_REF = Nint*64;
	CT_REF = Nint*8;



	REG28= REG28 & 0XFF8FFF;
	REG28= REG28 | (1 << 12);  //ѡ��Ƶ��

	write_thmts_rf_reg(42,0XFFCC10);   //36-42
	write_thmts_rf_reg(42,0XFFCC13);

	REG28=REG28&0XFFFF00;
	REG28=REG28 | (Nint & 0Xff);

	write_thmts_rf_reg(28,REG28);
	thmts_sleep_us(50);

	REG28=REG28 & 0XFFF2FF;
	REG28=REG28 | 0X010200;    //D16+D9=1
	write_thmts_rf_reg(28,REG28 ); //AFC ??

	write_thmts_rf_reg(31,0X01F388);				//26-31   �п���
	write_thmts_rf_reg(29,0X030000);       //??SDM,step2.5   24-29
	write_thmts_rf_reg(29,0X070000);

	REG30= REG30 & 0X00000F;
	REG30= REG30 | (1 << 10);    //�ӳ�CT��������
	write_thmts_rf_reg(30,REG30 | ((Ct << 18) | (Ft << 14)));  //25-30

	write_thmts_rf_reg(29,0X070000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(29,0X050000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(29,0X070000);


//	write_thmts_rf_reg(0,34,0X7DDADF);			//27-34
//	write_thmts_rf_reg(0,34,0X7DFADF);//VCO BUFF ʹ��

	write_thmts_rf_reg(29,0X074000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(29,0X054000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(29,0X074000);

	thmts_sleep_us(200);
	VCO_CntCTFF  = read_thmts_rf_reg(33)&0X003FFF;
	// uart_save(0xB0,tmep_n++,VCO_CntCTFF);
	VCO_CntCT = VCO_CntCTFF;
	for(i=0;i<6;i++)
	{
		printf("epoch %d adjust CT---------\r\n",i);
		printf("i = %d , VCO_CntCT = %d , CT_REF = %d , Ct = %d\r\n" ,
				i,
				VCO_CntCT,
				CT_REF,
				Ct);

		if (VCO_CntCT >= CT_REF )
		{
			if (i==5)
			{
				if (Ct!=63)
				{
					Ct= Ct-1;
				}
			}
			else
			{
					Ct= Ct-pow(2,4-i);
			}
		}
		else
		{
			if (i==5)
			{
				Ct=Ct+0;
			}
			else
			{
				 Ct= Ct+pow(2,4-i);
			}
		}

		printf("i = %d , VCO_CntCT = %d , CT_REF = %d , Ct = %d\r\n" ,
				i,
				VCO_CntCT,
				CT_REF,
				Ct);
			write_thmts_rf_reg(30,REG30 | ((Ct << 18) | (Ft << 14)));

			write_thmts_rf_reg(29,0X070000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(29,0X050000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(29,0X070000);
			thmts_sleep_us(50);

			write_thmts_rf_reg(29,0X074000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(29,0X054000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(29,0X074000);
			thmts_sleep_us(10);


			thmts_sleep_us(20);
		  VCO_R32CT  = (read_thmts_rf_reg(32)&0X0003F0) >> 4;
			thmts_sleep_us(200);
			VCO_CntCT  = read_thmts_rf_reg(33)&0X003FFF;
	}
	// uart_save(0xA0,tmep_n++,VCO_R32CT);
	// uart_save(0xA1,tmep_n++,VCO_CntCT);

	REG30= REG30 & 0X00000F;
	REG30= REG30 | (1 << 13);
	write_thmts_rf_reg(30,REG30 | ((Ct << 18) | (Ft << 14)));
	write_thmts_rf_reg(29,0X070000);
	write_thmts_rf_reg(29,0X050000);
	write_thmts_rf_reg(29,0X070000);

	write_thmts_rf_reg(29,0X074000);
	write_thmts_rf_reg(29,0X054000);
	write_thmts_rf_reg(29,0X074000);

	thmts_sleep_us(200);
	VCO_CntFT  = read_thmts_rf_reg(33)&0X003FFF;

	for (i=0; i<4;i++)
	{
		printf("epoch %d adjust FT---------\r\n",i);
		printf("i = %d , VCO_CntFT = %d , FT_REF = %d , Ft = %d\r\n" ,
						i,
						VCO_CntFT,
						FT_REF,
						Ft);

		if (VCO_CntFT >= FT_REF)
		{
			if (i==3 )
			{
				if (Ft != 0)
				{
					 Ft= Ft-1;
				}
			}
			else
			{
				Ft= Ft - pow(2,2-i);
			}
		}
		else
		{
			if (i==3 )
			{
				if (Ft != 15)
				{
					 Ft= Ft+1;
				}
			}
			else
			{
				 Ft= Ft + pow(2,2-i);
			}
		}

		printf("i = %d , VCO_CntFT = %d , FT_REF = %d , Ft = %d\r\n" ,
						i,
						VCO_CntFT,
						FT_REF,
						Ft);

			write_thmts_rf_reg(30,REG30 | ((Ct << 18) | (Ft << 14)));
			write_thmts_rf_reg(29,0X070000);
			write_thmts_rf_reg(29,0X050000);
			write_thmts_rf_reg(29,0X070000);

			write_thmts_rf_reg(29,0X074000);
			write_thmts_rf_reg(29,0X054000);
			write_thmts_rf_reg(29,0X074000);

			thmts_sleep_us(200);
      VCO_R32FT  = (read_thmts_rf_reg(32)&0X003C00) >> 10;
			// uart_save(0xA2,tmep_n++,VCO_R32FT);
			thmts_sleep_us(200);
			VCO_CntFT  = read_thmts_rf_reg(33)&0X003FFF;
			// uart_save(0xA3,tmep_n++,VCO_CntFT);
	}
	write_thmts_rf_reg(31,0X6E0588);			//�бջ�

	thmts_sleep_us(2000);
	VCO_R32RED = read_thmts_rf_reg(32);
	VCO_R32CT_F  = (VCO_R32RED&0X0003F0) >> 4;
	// uart_save(0xA4,tmep_n++,VCO_R32CT_F);
	thmts_sleep_us(200);
	VCO_R32FT_F  = (VCO_R32RED&0X003C00) >> 10;
	// uart_save(0xA5,tmep_n++,VCO_R32FT_F);
	thmts_sleep_us(200);
	VCO_R32_LF  = (VCO_R32RED&0X000001);
	// uart_save(0xA6,tmep_n++,VCO_R32_LF);
	thmts_sleep_us(100);
	VCO_R33RED0 = read_thmts_rf_reg(33) & 0X003FFF;
	// uart_save(0xA7,tmep_n++,VCO_R33RED0);

	thmts_sleep_us(2000);

	VCO_R32RED1 = read_thmts_rf_reg(32);
	VCO_R32CT_F1  = (VCO_R32RED1&0X0003F0) >> 4;
	// uart_save(0xC1,tmep_n++,VCO_R32CT_F1);
	thmts_sleep_us(200);
	VCO_R32FT_F1  = (VCO_R32RED1&0X003C00) >> 10;
	// uart_save(0xC2,tmep_n++,VCO_R32FT_F1);
	thmts_sleep_us(200);
	VCO_R32_LF1  = (VCO_R32RED1&0X000001);
	// uart_save(0xC3,tmep_n++,VCO_R32_LF1);
	thmts_sleep_us(100);
	VCO_R33RED1 = read_thmts_rf_reg(33) & 0X003FFF;
	// uart_save(0xC4,tmep_n++,VCO_R33RED1);

	// uart_save(0xCC,tmep_n++,CT_REF);
	// uart_save(0xCF,tmep_n++,FT_REF);
	// uart_s(buftemp,tmep_n*4);



	//上述过程会将LO PLL锁定到7.987G
	// 需要进行二分频，然后设置为宽带
	write_thmts_rf_reg(38,0X80001F);//设置为宽带
	write_thmts_rf_reg(28,(REG28 | 0X400000));// bit22置1，环外除2开启

	uint32_t read_reg;
	read_reg = read_thmts_rf_reg(00);
	thmts_sleep_us(2000);


	printf("lo pll CT = %d , FT =%d \r\n" , Ct , Ft);
	printf("config lo pll mfc regs OK\r\n");


}







// 半自动clk pll锁相
void config_thmts_rf_clkpll_mfc( uint8_t NintSet)  //半自动CLK PLL
{
	printf("begin config clk pll mfc regs\r\n");

	int tmep_n=0;

//	uint64_t Fre_Set,fdsm;
	uint32_t CT_REF,FT_REF,VCO_Cnt_last;
    uint32_t VCO_CntCT,VCO_CntFT,REG45,REG47,Ct,Ft,Nint,VCO_R32CT,VCO_R32FT,VCO_R32FT_F,VCO_R32CT_F,VCO_R32_LF,VCO_R32RED,VCO_CntCTFF,VCO_R32_LF1,VCO_R32RED1,VCO_R32FT_F1,VCO_R32CT_F1,VCO_R33RED0,VCO_R33RED1;
	int i=0;
//	int j=0;

	REG45=0X811700;			//20-28    Óëbit17Î»ÓÚRFÍ¨µÀ¶¨Òå²»Ò»ÖÂ
	REG47=0X00027F;
	Ct=32;
	Ft=8;



	Nint= NintSet/2;
//	Nfrac= (uint64_t)(((Fre_Set*2*pow(2,23)) / fdsm) - (Nint*pow(2,23)));


	FT_REF = Nint*64;
	CT_REF = Nint*64;



	REG45= REG45 & 0XFF8FFF;
	REG45= REG45 | (1 << 12);  //Ñ¡·ÖÆµÆ÷

	write_thmts_rf_reg(42,0XFFCC10);   //36-42
	write_thmts_rf_reg(42,0XFFCC13);

	REG45=REG45&0XFFFF00;
	REG45=REG45 | (Nint & 0Xff);

	write_thmts_rf_reg(45,REG45);
	thmts_sleep_us(50);

	REG45=REG45 & 0XFFF2FF;
	REG45=REG45 | 0X010200;    //D16+D9=1
	write_thmts_rf_reg(45,REG45 ); //AFC ??

	write_thmts_rf_reg(48,0X01F388);				//26-31   ÇÐ¿ª»·
	write_thmts_rf_reg(46,0X030000);       //??SDM,step2.5   24-29
	write_thmts_rf_reg(46,0X070000);

	REG47= REG47 & 0X00000F;
	REG47= REG47 | (1 << 13);    //ÑÓ³¤CT¼ÆÊýÖÜÆÚ
	write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));  //25-30

	write_thmts_rf_reg(46,0X070000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(46,0X050000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(46,0X070000);


//	write_thmts_rf_reg(34,0X7DDADF);			//27-34
//	write_thmts_rf_reg(34,0X7DFADF);//VCO BUFF Ê¹ÄÜ

	write_thmts_rf_reg(46,0X074000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(46,0X054000);
	thmts_sleep_us(10);
	write_thmts_rf_reg(46,0X074000);

	thmts_sleep_us(200);
	VCO_CntCTFF  = read_thmts_rf_reg(50)&0X003FFF;
	//uart_save(0xB0,tmep_n++,VCO_CntCTFF);	 //  //uart0
	VCO_CntCT = VCO_CntCTFF;
	for(i=0;i<6;i++)
	{
	 if (abs(VCO_CntCT-CT_REF) > 20)
	 {

		if (VCO_CntCT >= CT_REF )
		{
			if (i==5)
			{
				if (Ct!=63)
				{
					Ct= Ct-1;
				///

				write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
				write_thmts_rf_reg(46,0X070000);  //Ð´100£¬ÉúÐ§CT,FTÖµ
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X050000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X070000);
				thmts_sleep_us(50);

				write_thmts_rf_reg(46,0X074000);  //Ð´101,¿ªÆô¼ÆÊýÆ÷
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X054000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X074000);
				thmts_sleep_us(10);

				thmts_sleep_us(200);
				VCO_CntCT  = read_thmts_rf_reg(50)&0X003FFF;
				if (VCO_CntCT < CT_REF)
				{
					if ((VCO_Cnt_last - CT_REF) < (CT_REF - VCO_CntCT))
					{
						Ct = Ct + 1;
					}
				}

				}
			}
			else
			{
					Ct= Ct-pow(2,4-i);
			}
		}
		else
		{
			if (i==5)
			{
				Ct=Ct+0;
				///

				write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
				write_thmts_rf_reg(46,0X070000);  //Ð´100£¬ÉúÐ§CT,FTÖµ
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X050000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X070000);
				thmts_sleep_us(50);

				write_thmts_rf_reg(46,0X074000);  //Ð´101,¿ªÆô¼ÆÊýÆ÷
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X054000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X074000);
				thmts_sleep_us(10);

				thmts_sleep_us(200);
				VCO_CntCT  = read_thmts_rf_reg(50)&0X003FFF;
				if (VCO_CntCT > CT_REF)
				{
						if ((CT_REF - VCO_Cnt_last) < (VCO_CntCT - CT_REF))
						{
								Ct = Ct - 1;
						}
				}
			}
			else
			{
				 Ct= Ct+pow(2,4-i);
			}
		}

			write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));

			write_thmts_rf_reg(46,0X070000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(46,0X050000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(46,0X070000);
			thmts_sleep_us(50);

			write_thmts_rf_reg(46,0X074000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(46,0X054000);
			thmts_sleep_us(10);
			write_thmts_rf_reg(46,0X074000);
			thmts_sleep_us(10);


			thmts_sleep_us(20);
		  VCO_R32CT  = (read_thmts_rf_reg(49)&0X0003F0) >> 4;
			thmts_sleep_us(200);
			VCO_CntCT  = read_thmts_rf_reg(50)&0X003FFF;
  	}
   }
	//uart_save(0xA0,tmep_n++,VCO_R32CT);  //  //uart1
	//uart_save(0xA1,tmep_n++,VCO_CntCT);	 //  //uart2

	REG47= REG47 & 0X00000F;
	REG47= REG47 | (1 << 13);
	write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
	write_thmts_rf_reg(46,0X070000);
	write_thmts_rf_reg(46,0X050000);
	write_thmts_rf_reg(46,0X070000);

	write_thmts_rf_reg(46,0X074000);
	write_thmts_rf_reg(46,0X054000);
	write_thmts_rf_reg(46,0X074000);

	thmts_sleep_us(200);
	VCO_CntFT  = read_thmts_rf_reg(50)&0X003FFF;

	for (i=0; i<4;i++)
	{
		if(abs(VCO_CntFT-FT_REF) >5 )
		{

		if (VCO_CntFT >= FT_REF)
		{
			if (i==3 )
			{
				if (Ft != 0)
				{
					 Ft= Ft-1;
					///
				write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
				write_thmts_rf_reg(46,0X070000);  //Ð´100£¬ÉúÐ§CT,FTÖµ
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X050000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X070000);
				thmts_sleep_us(50);

				write_thmts_rf_reg(46,0X074000);  //Ð´101,¿ªÆô¼ÆÊýÆ÷
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X054000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X074000);
				thmts_sleep_us(10);

				thmts_sleep_us(200);
				VCO_CntFT  = read_thmts_rf_reg(50)&0X003FFF;
					 if (VCO_CntFT < FT_REF)
                    {
                        if ((VCO_Cnt_last - FT_REF) < (FT_REF - VCO_CntFT))
                        {
                            Ft = Ft + 1;
                        }
                    }
				}
			}
			else
			{
				Ft= Ft - pow(2,2-i);
			}
		}
		else
		{
			if (i==3 )
			{
				if (Ft != 15)
				{
					 Ft= Ft+1;
					////
				write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
				write_thmts_rf_reg(46,0X070000);  //Ð´100£¬ÉúÐ§CT,FTÖµ
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X050000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X070000);
				thmts_sleep_us(50);

				write_thmts_rf_reg(46,0X074000);  //Ð´101,¿ªÆô¼ÆÊýÆ÷
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X054000);
				thmts_sleep_us(10);
				write_thmts_rf_reg(46,0X074000);
				thmts_sleep_us(10);

				thmts_sleep_us(200);
				VCO_CntFT  = read_thmts_rf_reg(50)&0X003FFF;
				           if (VCO_CntFT > FT_REF)
                    {
                        if ((FT_REF - VCO_Cnt_last) < (VCO_CntFT - FT_REF))
                        {
                            Ft = Ft - 1;
                        }
                    }
				}
			}
			else
			{
				 Ft= Ft + pow(2,2-i);
			}
		}
			write_thmts_rf_reg(47,REG47 | ((Ct << 18) | (Ft << 14)));
			write_thmts_rf_reg(46,0X070000);
			write_thmts_rf_reg(46,0X050000);
			write_thmts_rf_reg(46,0X070000);

			write_thmts_rf_reg(46,0X074000);
			write_thmts_rf_reg(46,0X054000);
			write_thmts_rf_reg(46,0X074000);

			thmts_sleep_us(200);
      VCO_R32FT  = (read_thmts_rf_reg(49)&0X003C00) >> 10;
			////uart_save(0xA2,tmep_n++,VCO_R32FT);
			thmts_sleep_us(200);
			VCO_CntFT  = read_thmts_rf_reg(50)&0X003FFF;
		//	//uart_save(0xA3,tmep_n++,VCO_CntFT);
	 }
  }
	write_thmts_rf_reg(48,0X6E0588);			//切闭环

	thmts_sleep_us(2000);

	write_thmts_rf_reg(55,0X80001F);//设置为宽带
	thmts_sleep_us(2000);

	uint32_t REG54;
	REG54 = read_thmts_rf_reg(54);
	write_thmts_rf_reg(54,(REG54 & 0xFFF7FF ));// bit11置0，输出3G


	SystemCoreClock = get_cpu_freq();
	DebugUart_Config();

	printf("SystemCoreClock after clk pll is %d Hz \r\n", SystemCoreClock);


	printf("clk pll CT = %d , FT =%d \r\n" , Ct , Ft);
	printf("config clk pll mfc regs OK\r\n");





}








// 全自动clk pll锁相
//读取 clkpll锁定成功标志
// 返回1: clkpll锁定成功
// 返回0：clkpll锁定失败
uint32_t config_thmts_rf_clkpll_afc()
{
    uint32_t pll_lock_flag;
    uint16_t j;
	uint32_t num = sizeof(regconfig_clkpll) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_clkpll, num);
    for (j=0; j<20000; j++);
    pll_lock_flag = read_thmts_rf_reg(0x31) & 0x00000001;
	return pll_lock_flag;
}


// 全自动lo pll锁相
//读取 lopll锁定成功标志
// 返回1: lopll锁定成功
// 返回0：lopll锁定失败
uint32_t config_thmts_rf_lopll_afc()
{
    uint32_t pll_lock_flag;
    uint16_t j;
	uint32_t num = sizeof(regconfig_lopll) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_lopll, num);
    for (j=0; j<20000; j++);
    pll_lock_flag = read_thmts_rf_reg(0x20) & 0x00000001;
	return pll_lock_flag;
}

//void config_thmts_rf_dac()
//{
//	uint32_t num = sizeof(regconfig_dac) / sizeof(THMTS_RfRegConfig_t);
//	config_thmts_rf_regs(regconfig_dac, num);
//	return;
//}

//修改config dac，返回寄存器0的bit12
// 配置完成DAC之后会切换到3G时钟，3G时钟分频出来124.8M的时钟
// 寄存器0的bit12，意义是使用晶振的38.4M时钟测量124.8M的时钟
// 如果测量结果准确则返回1 ， 否则返回0
uint32_t config_thmts_rf_dac()
{
	uint32_t num = sizeof(regconfig_dac) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_dac, num);

    for (uint16_t j=0; j<20000; j++);
    uint32_t read_reg = read_thmts_rf_reg(0x00);
    uint32_t dac_div_lock_flag = (read_reg>>12)&(0x00000001);
	return dac_div_lock_flag;
}

void config_thmts_rf_da2lpf()
{
	uint32_t num = sizeof(regconfig_da2lpf) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_da2lpf, num);
	return;
}

void config_thmts_rf_pad2lpf()
{
	uint32_t num = sizeof(regconfig_pad2lpf) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_pad2lpf, num);
	return;
}

void config_thmts_rf_da2pad()
{
	uint32_t num = sizeof(regconfig_da2pad) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_da2pad, num);
	return;
}

void config_thmts_rf_tx()
{
	uint32_t num = sizeof(regconfig_tx) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_tx, num);
	return;
}


void config_thmts_rf_tx_pd()
{
	config_thmts_rf_regs(regconfig_tx_pd, 1);
}


void config_thmts_rf_adc0()
{
	uint32_t num = sizeof(regconfig_adc0) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_adc0, num);
	return;
}


void config_thmts_rf_adc1()
{
	uint32_t num = sizeof(regconfig_adc1) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_adc1, num);
	return;
}


void config_thmts_rf_adc2()
{
	uint32_t num = sizeof(regconfig_adc2) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_adc2, num);
	return;
}

void config_thmts_rf_adc3()
{
	uint32_t num = sizeof(regconfig_adc3) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_adc3, num);
	return;
}

void config_thmts_rf_lpf2ad()
{
	uint32_t num = sizeof(regconfig_lpf2ad) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_lpf2ad, num);
	return;
}

void config_thmts_rf_lpf2pad()
{
	uint32_t num = sizeof(regconfig_lpf2pad) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_lpf2pad, num);
	return;
}

void config_thmts_rf_lpf2pad_and_lpf2ad()
{
  	uint32_t num = sizeof(regconfig_lpf2pad_and_lpf2ad) / sizeof(THMTS_RfRegConfig_t);
  	config_thmts_rf_regs(regconfig_lpf2pad_and_lpf2ad, num);
  	return;
}


void config_thmts_rf_pad2ad()
{
	uint32_t num = sizeof(regconfig_pad2ad) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_pad2ad, num);
	return;
}

void config_thmts_rf_rx0()
{
	uint32_t num = sizeof(regconfig_rx0) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_rx0, num);
	return;
}


void config_thmts_rf_rx1()
{
	uint32_t num = sizeof(regconfig_rx1) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_rx1, num);
	return;
}


void config_thmts_rf_rx2()
{
	uint32_t num = sizeof(regconfig_rx2) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_rx2, num);
	return;
}



void config_thmts_rf_rx3()
{
	uint32_t num = sizeof(regconfig_rx3) / sizeof(THMTS_RfRegConfig_t);
	config_thmts_rf_regs(regconfig_rx3, num);
	return;
}

void config_thmts_rf_dagc_enable(uint8_t enable)
{
	if(enable == 1)
		config_thmts_rf_regs(regconfig_dagc_enable, 1);
	else
		config_thmts_rf_regs(regconfig_dagc_disable, 1);

	return;
}

void config_thmts_rf_dagc_set_value(uint32_t rx_channel_sel, uint32_t dagc_value)
{
  uint16_t j;
  config_thmts_rf_reg(0x0, (rx_channel_sel << 8) & 0x000300, 0x000300); //config the rx channel
  for (j=0; j<10000; j++);
  config_thmts_rf_reg(0x2, (dagc_value << 16) & 0x3f0000 , 0x3f0000);
  for (j=0; j<10000; j++);

	return;
}

void config_thmts_rf_trsw_enable(uint8_t enable)
{
	if(enable == 1)
		config_thmts_rf_regs(regconfig_trsw_enable, 1);
	else
		config_thmts_rf_regs(regconfig_trsw_disable, 1);

	return;
}



/**
 * Config the clk output from rf.
 * @param div2_en : output clk frequency divided by 2 0x00[10].
 * @param hi_freq_clk_en : if 1 output clk frequency is 249.6, if 0 output clk frequency is 38.4 0x00[11].
 */

void config_thmts_rf_clkmode( uint8_t div2_en, uint8_t hi_freq_clk_en)
{

    if( hi_freq_clk_en )
    {
        uint32_t num = sizeof(regconfig_clk_enable) / sizeof(THMTS_RfRegConfig_t);
        config_thmts_rf_regs(regconfig_clk_enable, num);
    }
    else
    {
        uint32_t num = sizeof(regconfig_clk_disable) / sizeof(THMTS_RfRegConfig_t);
        config_thmts_rf_regs(regconfig_clk_disable, num);
    }

    if( div2_en )
    {
        uint32_t num = sizeof(regconfig_clk_div_enable) / sizeof(THMTS_RfRegConfig_t);
        config_thmts_rf_regs(regconfig_clk_div_enable, num);
    }
    else
    {
        uint32_t num = sizeof(regconfig_clk_div_disable) / sizeof(THMTS_RfRegConfig_t);
        config_thmts_rf_regs(regconfig_clk_div_disable, num);
    }

    return;
}


void thmts_sleep_us(uint32_t us)
{
	for( uint32_t i = 0; i < 38 * us; i++ );
}



void config_thmts_rf_pll_mfc()
{
	//初始化
	config_thmts_pll_init_mfc();


	uint32_t lo_lock_flag = 0;
	//半自动锁相 LO
	while(lo_lock_flag == 0)
	{
		config_thmts_rf_lopll_mfc(208);
		thmts_sleep_us(2000);
		lo_lock_flag = read_thmts_rf_reg(0x20) & 0x00000001;
	}


	uint32_t clk_lock_flag = 0;
	//半自动锁相 PLL
	while(clk_lock_flag == 0)
	{
		config_thmts_rf_clkpll_mfc(156);
		thmts_sleep_us(2000);
		clk_lock_flag = read_thmts_rf_reg(0x31) & 0x00000001;
	}



    // config tx channel
    uint32_t dac_div_clk_valid = 0;

	  dac_div_clk_valid = config_thmts_rf_dac();

//	  //时钟切换到124.8MHz
//    // Switch riscv clock freq to 124.8MHz
//    config_thmts_rf_clkmode(0, 1);
}




void config_thmts_rf_allinone(uint32_t rx_num)
{
	uint32_t i;



    config_thmts_rf_tx();
    config_thmts_rf_da2lpf();

    // config rx channel
    config_thmts_rf_rx0();
    config_thmts_rf_adc0();
#ifdef AD_MODE
    config_thmts_rf_lpf2ad();
#endif

#ifdef PAD_MODE
    config_thmts_rf_lpf2pad();
#endif

#ifdef AD_PAD_MODE
    config_thmts_rf_lpf2pad_and_lpf2ad();
#endif

    if (rx_num >= 2)
    {
    	config_thmts_rf_rx1();
    	config_thmts_rf_adc1();
#ifdef AD_MODE
    config_thmts_rf_lpf2ad();
#endif

#ifdef PAD_MODE
    config_thmts_rf_lpf2pad();
#endif

#ifdef AD_PAD_MODE
    config_thmts_rf_lpf2pad_and_lpf2ad();
#endif
    }

    if (rx_num >=3)
    {
    	config_thmts_rf_rx2();
    	config_thmts_rf_adc2();
#ifdef AD_MODE
    config_thmts_rf_lpf2ad();
#endif

#ifdef PAD_MODE
    config_thmts_rf_lpf2pad();
#endif

#ifdef AD_PAD_MODE
    config_thmts_rf_lpf2pad_and_lpf2ad();
#endif
    }

    if (rx_num == 4)
    {
    	config_thmts_rf_rx3();
    	config_thmts_rf_adc3();
#ifdef AD_MODE
    config_thmts_rf_lpf2ad();
#endif

#ifdef PAD_MODE
    config_thmts_rf_lpf2pad();
#endif

#ifdef AD_PAD_MODE
    config_thmts_rf_lpf2pad_and_lpf2ad();
#endif
    }


    // enable tx and rx sw
    config_thmts_rf_trsw_enable(1);

    // enable dagc
    config_thmts_rf_dagc_enable(1);





//    // Switch riscv clock freq to 124.8MHz
//    config_thmts_rf_clkmode(0, 1);

    return;
}
