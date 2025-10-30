#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ns_sdk_hal.h"

#define  sFLASH_ID              0XEF7019
#ifndef QSPI_XIP0_MEM_BASE
#define QSPI_XIP0_MEM_BASE QSPI_FLASH_BASE
#endif

/* flash operation cmd instruction */
#define FLASH_WriteEnable               0x06
#define FLASH_WriteDisable              0x04
#define FLASH_ReadStatusReg1		    0x05
#define FLASH_ReadStatusReg2		    0x35
#define FLASH_ReadStatusReg3		    0x15
#define FLASH_WriteStatusReg1           0x01
#define FLASH_WriteStatusReg2           0x31
#define FLASH_WriteStatusReg3           0x11
#define FLASH_ReadData                  0x03
#define FLASH_FastReadData              0x0B
#define FLASH_FastReadDual              0x3B
#define FLASH_FastReadQuad              0x6B
#define FLASH_PageProgram               0x02
#define FLASH_BlockErase                0xD8
#define FLASH_SectorErase               0x20
#define FLASH_BLOCK32K_Erase            0x52
#define FLASH_BLOCK64K_Erase            0xD8
#define FLASH_ChipErase                 0xC7
#define FLASH_PowerDown                 0xB9
#define FLASH_ReleasePowerDown          0xAB
#define FLASH_DeviceID                  0xAB
#define FLASH_ManufactDeviceID          0x90
#define FLASH_JedecDeviceID             0x9F
#define FLASH_Enable4ByteAddr           0xB7
#define FLASH_Exit4ByteAddr             0xE9

/* WIP(busy) flag */
#define WIP_Flag                  0x01
#define Dummy_Byte                0xFF

#define SPI_CS_ON  (QSPI_XIP_CSModeConfig(QSPI_XIP0, QSPI_XIP_CSMODE_MODE_HOLD))
#define SPI_CS_OFF (QSPI_XIP_CSModeConfig(QSPI_XIP0, QSPI_XIP_CSMODE_MODE_OFF))


extern void SPI_Init(void);
extern void SPI_WriteByte(uint8_t txdata);
extern uint8_t SPI_ReadByte(void);
extern void SPI_ReadJEDEC(void);
extern void SPI_Wip(void);
extern void SPI_WriteEnable(void);
extern void SPI_Quad(void);
extern void SPI_Erase(uint8_t cmd, uint32_t addr);
extern void SPI_PageWrite(uint8_t cmd, uint8_t* buf, uint32_t addr);
extern void SPI_PageRead(uint8_t cmd, uint8_t* buf, uint32_t addr);





#define NODE_CONFIG_FLASH_ADDR 0x000E0000
#define PHY_CONFIG_FLASH_ADDR  0x000E1000
#define MAC_CONFIG_FLASH_ADDR  0x000E2000

extern uint8_t flash_write_vector[256];
extern uint8_t flash_read_vector[256];


#define THMTS_CTRL_START_CONFIG_CMD 0x60
#define THMTS_CTRL_NODE_CONFIG_CMD 0x61
#define THMTS_CTRL_MAC_CONFIG_CMD 0x62
#define THMTS_CTRL_PHY_CONFIG_CMD 0x63

#define THMTS_CTRL_START_CONFIG_ACK 0x80
#define THMTS_CTRL_NODE_CONFIG_ACK 0x81
#define THMTS_CTRL_MAC_CONFIG_ACK 0x82
#define THMTS_CTRL_PHY_CONFIG_ACK 0x83

extern uint8_t ack_start;
extern uint8_t ack_node;
extern uint8_t ack_phy;
extern uint8_t ack_mac;



// 定义NODE CONFIG
#pragma pack(1)
typedef struct
{
    uint8_t dev_id;
    uint8_t group_id;
    uint8_t role;
} NODE_CONFIG_t;
#pragma pack()


// 定义PHY CONFIG
#pragma pack(1)
typedef struct
{
	uint8_t bit_rate;           // bit_rate 0:1M , 1:2M , 2:4M  , 3:8M
	uint8_t preamble_length;    // 同步头长度 ，暂时不用
	uint8_t freq_grid;          // 频格，暂时不用

	uint8_t rf_chan_num;        // 开启RF通道数量，(1~4)
	uint8_t AOA_enable;         // 是否是AOA工作模式 , 0代表关闭AOA模式 , 1代表打开AOA模式
	uint8_t gain_mode;          // 增益模式 , 0代表固定增益 , 1代表AGC模式
	uint8_t gain_value;         // 固定增益模式下，增益数值(0~44)

} PHY_CONFIG_t;
#pragma pack()


// 定义MAC CONFIG
#pragma pack(1)
typedef struct
{
    uint16_t TIMER_AAR;
    uint16_t TickPerSlot;
    uint16_t MAC_SCHEME_ID;
} MAC_CONFIG_t;
#pragma pack()

extern NODE_CONFIG_t node_config;
extern PHY_CONFIG_t  phy_config;
extern MAC_CONFIG_t  mac_config;


extern void write_flash_node_config();
extern void write_flash_phy_config();
extern void write_flash_mac_config();


extern void read_flash_node_config();
extern void read_flash_phy_config();
extern void read_flash_mac_config();





#endif
