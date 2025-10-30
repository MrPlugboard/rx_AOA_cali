#include "ns_sdk_hal.h"

#define GRAVITY 9.81  // 重力加速度

#define IMU_I2C_SLAVE_ADDR  0x18    // IMU从设备地址 
#define IMU_I2C_BURST_ADDR  0x12    // IMU Burst寄存器地址

#define IMU_I2C_TX_DMA_BUFF_SIZE 1024
#define IMU_I2C_RX_DMA_BUFF_SIZE 48 // IMU I2C Burst模式读取数据大小

#define IMU_SPI_TX_DMA_BUFF_SIZE 2+24 // IMU SPI Burst2模式发送数据大小
#define IMU_SPI_RX_DMA_BUFF_SIZE 2+24 // IMU SPI Burst2模式读取数据大小（IMU输出寄存器内容比输入读取指令滞后两个SPI周期）

#define ImuRxPeriodInMS     10     // IMU读取时间间隔

#define BMM150_REG_CHIP_ID                        0x40
#define BMM150_REG_DATA_X_LSB                     0x42
#define BMM150_REG_DATA_READY_STATUS              0x48
#define BMM150_REG_INTERRUPT_STATUS               0x4A
#define BMM150_REG_POWER_CONTROL                  0x4B
#define BMM150_REG_OP_MODE                        0x4C
#define BMM150_REG_INT_CONFIG                     0x4D
#define BMM150_REG_AXES_ENABLE                    0x4E
#define BMM150_REG_LOW_THRESHOLD                  0x4F
#define BMM150_REG_HIGH_THRESHOLD                 0x50
#define BMM150_REG_REP_XY                         0x51
#define BMM150_REG_REP_Z                          0x52


void IMU_I2C_rx_process(uint8_t *imu_rx_data);
void IMU_Init(void);
void IMU_SPI_Init(void);
void IMU_SPI_Read_Regs(void);
void IMU_SPI_Master_Read_Burst2(uint16_t *data0, uint64_t imu_timestamp);

void BMM_SPI_Init(void);


