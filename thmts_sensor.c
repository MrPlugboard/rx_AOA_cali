#include "string.h"
#include "thmts_sensor.h"
#include "thmts_tx_msg.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include "task.h"     /* RTOS task related API prototypes. */

#define SPI_CS_ON  (QSPI_CSModeConfig(QSPI2, QSPI_CSMODE_MODE_HOLD))
#define SPI_CS_OFF (QSPI_CSModeConfig(QSPI2, QSPI_CSMODE_MODE_OFF))

#define DUMMY 0x00FF
uint32_t spi_cnt = 0;

struct imu_i2c_slave_frame_publish_user
{
  uint32_t time_ms;
  float accel[3];
  float gyro[3];
  float temp;
  float attitude[3];
  uint32_t check_crc;
} __attribute__((packed));

struct spi_burst_data_t
{
    float  gyro[3];
    float  accel[3];
    uint16_t checksum;
} __attribute__((packed));
struct spi_burst_data_t  spi_burst_data;

struct spi_regs_data_t
{
    float  gyro[3];
    float  accel[3];
    float temp;
} __attribute__((packed));
struct spi_regs_data_t  spi_regs_data;

typedef struct{
	float gyro[3];
	float accel[3];
	float temperature;
	float attitude[3];
	uint32_t status;
	uint32_t check_crc;
}spi_burst_ext_data_t_mode2;
spi_burst_ext_data_t_mode2 spi_burst_data_mode2 = {0};

struct imu_i2c_slave_frame_publish_user imu_i2c_rev_publish_user = {0};

static const uint32_t crc32_tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size)
{
    for (uint32_t i=0; i<size; i++) {
        crc = crc32_tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    }

    return crc;
}

void IMU_I2C_rx_process(uint8_t *imu_rx_data)
{
    uint32_t cnt = 0, err_cnt = 0;
    uint32_t check_crc = 1;
    uint32_t check_sum_i2c = crc_crc32(check_crc,(uint8_t *)(imu_rx_data), IMU_I2C_RX_DMA_BUFF_SIZE - 4);

    if(check_sum_i2c== *(uint32_t*)&imu_rx_data[IMU_I2C_RX_DMA_BUFF_SIZE - 4])
    {
        memcpy((uint8_t *)(&imu_i2c_rev_publish_user), (uint8_t *)(imu_rx_data), IMU_I2C_RX_DMA_BUFF_SIZE); 
        printf("imu_data, time_ms: %d, accl_x: %d, accl_y: %d, accl_z: %d, gyro_x: %d, gyro_y: %d, gyro_z: %d, temp: %d, roll: %d, pitch: %d, yaw: %d\r\n", 
            imu_i2c_rev_publish_user.time_ms,
            (int32_t)(imu_i2c_rev_publish_user.accel[0]*100),
            (int32_t)(imu_i2c_rev_publish_user.accel[1]*100),
            (int32_t)(imu_i2c_rev_publish_user.accel[2]*100),
            (int32_t)(imu_i2c_rev_publish_user.gyro[0]),
            (int32_t)(imu_i2c_rev_publish_user.gyro[1]),
            (int32_t)(imu_i2c_rev_publish_user.gyro[2]),
            (int32_t)(imu_i2c_rev_publish_user.temp),
            (int32_t)(imu_i2c_rev_publish_user.attitude[0]),
            (int32_t)(imu_i2c_rev_publish_user.attitude[1]),
            (int32_t)(imu_i2c_rev_publish_user.attitude[2])
        );
    }
    else
    {
        printf("read data error... \n");
    }

    // if(check_sum_i2c== *(uint32_t*)&imu_rx_data[IMU_I2C_RX_DMA_BUFF_SIZE - 4])
    // {   
    //     memcpy((uint8_t *)(&imu_i2c_rev_publish_user), (uint8_t *)(imu_rx_data), IMU_I2C_RX_DMA_BUFF_SIZE); 
    //     printf("imu_data, accl_x: %d, accl_y: %d, accl_z: %d, gyro_x: %d, gyro_y: %d, gyro_z: %d\n", 
    //         (int32_t)(imu_i2c_rev_publish_user.accel[0]*100),
    //         (int32_t)(imu_i2c_rev_publish_user.accel[1]*100),
    //         (int32_t)(imu_i2c_rev_publish_user.accel[2]*100),
    //         (int32_t)(imu_i2c_rev_publish_user.gyro[0]*100),
    //         (int32_t)(imu_i2c_rev_publish_user.gyro[1]*100),
    //         (int32_t)(imu_i2c_rev_publish_user.gyro[2]*100));
    //     cnt++;
    // }
    // else
    // {
    //     err_cnt++;
    //     printf("read data error... \n");
    // }
}


/**
 * SPI双向传输数据
 */
uint16_t SPI_ReadWriteByte16(uint16_t cmd)
{

    while(SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_TX_FULL)){}
    QSPI2->TXDATA = cmd;
    while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_BUSY)){}
    while(SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_RX_EMPTY)){}
    return (uint16_t)(QSPI2->RXDATA);
}

uint8_t SPI_ReadWriteByte8(uint8_t cmd)
{
    while(SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_TX_FULL)){}
    QSPI2->TXDATA = cmd;
    while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_BUSY)){}
    while(SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_RX_EMPTY)){}
    return (uint8_t)(QSPI2->RXDATA);
}

// void SPI_WriteByte(uint8_t txdata)
// {
//     QSPI_DirectionConfig(QSPI2, QSPI_FMT_DIR_TX);
//     while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_TX_FULL)) {}
//     QSPI_SendData(QSPI2, txdata);
//     while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_BUSY)) {}
// }

// uint8_t SPI_ReadByte(void)
// {
//     uint8_t rxdata = 0;
//     QSPI_DirectionConfig(QSPI2, QSPI_FMT_DIR_RX);
//     QSPI_SendData(QSPI2, Dummy_Byte);
//     while (SET == QSPI_GetFlag(QSPI3, QSPI_STATUS_RX_EMPTY)) {}
//     rxdata = QSPI_ReceiveData(QSPI2) & 0xFF;
//     return rxdata;
// }

/**
 * SPI读取单个寄存器
 */
uint16_t IMU_SPI_ReadReg(uint16_t addr)
{
    SPI_ReadWriteByte16(addr);
    SPI_ReadWriteByte16(DUMMY);
    return SPI_ReadWriteByte16(DUMMY);
}

uint16_t IMU_SPI_ReadByte(void)
{
    uint16_t rxdata = 0;
    QSPI_DirectionConfig(QSPI2, QSPI_FMT_DIR_RX);
    QSPI_SendData(QSPI2, 0x00FF);
    while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_RX_EMPTY)) {}
    rxdata = QSPI_ReceiveData(QSPI2);
    return (uint16_t)(rxdata);
}

void IMU_Init(void)
{
    // 初始化额外片选CS，在三个传感器中选中IMU
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource5, 0);

    // 初始化拉低复位一次
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 1);
    vTaskDelay(pdMS_TO_TICKS(100)*30);  // 初始化读取前，需将原极IMU复位并等待3s，使IMU进入正常工作状态
}

void IMU_SPI_Init(void)
{
    uint16_t filter_ctrl, imu_id[3], odr;

    SPI_ReadWriteByte16(0xFE01);//选择配置窗口
    SPI_ReadWriteByte16(0x8655);//滤波器设置,15hz,数值参考手册
    filter_ctrl = IMU_SPI_ReadReg(0x0600);

    imu_id[0] = IMU_SPI_ReadReg(0x6C00);
    imu_id[1] = IMU_SPI_ReadReg(0x6E00);
    imu_id[2] = IMU_SPI_ReadReg(0x7000);

    SPI_ReadWriteByte16(0xC064);  // 配置odr频率为100Hz: C064  10Hz: 0xC00A
    SPI_ReadWriteByte16(0xC100);

    odr = IMU_SPI_ReadReg(0x4000);
    odr |= IMU_SPI_ReadReg(0x4100) << 8;

    // printf("spi filter_ctrl: %d, odr: %d\r\n", filter_ctrl, odr);

    SPI_ReadWriteByte16(0xFE00);//切换为数据窗口
	SPI_ReadWriteByte16(0x8D02);//读取Burst模式2
}

void IMU_SPI_Read_Regs(void)
{
    uint16_t imu_data[30]={0}; 
    /**
     * 偏移2位读取数据
     */
    imu_data[0]  = (uint16_t)SPI_ReadWriteByte16(0x1200);
    imu_data[1]  = (uint16_t)SPI_ReadWriteByte16(0x1400);
    imu_data[2]  = (uint16_t)SPI_ReadWriteByte16(0x1600);
    imu_data[3]  = (uint16_t)SPI_ReadWriteByte16(0x1800);
    imu_data[4]  = (uint16_t)SPI_ReadWriteByte16(0x1A00);
    imu_data[5]  = (uint16_t)SPI_ReadWriteByte16(0x1C00);
    imu_data[6]  = (uint16_t)SPI_ReadWriteByte16(0x1E00);
    imu_data[7]  = (uint16_t)SPI_ReadWriteByte16(0x2000);
    imu_data[8]  = (uint16_t)SPI_ReadWriteByte16(0x2200);
    imu_data[9]  = (uint16_t)SPI_ReadWriteByte16(0x2400);
    imu_data[10]  = (uint16_t)SPI_ReadWriteByte16(0x2600);
    imu_data[11]  = (uint16_t)SPI_ReadWriteByte16(0x2800);
    imu_data[12]  = (uint16_t)SPI_ReadWriteByte16(0x0e00);
    imu_data[13]  = (uint16_t)SPI_ReadWriteByte16(0x1000);
    imu_data[14]  = (uint16_t)SPI_ReadWriteByte16(DUMMY);
    imu_data[15]  = (uint16_t)SPI_ReadWriteByte16(DUMMY);


    spi_regs_data.gyro[0] = (float)((int32_t)(imu_data[2]<<16) | imu_data[3])/4096000.0f;
    spi_regs_data.gyro[1] = (float)((int32_t)(imu_data[4]<<16) | imu_data[5])/4096000.0f;
    spi_regs_data.gyro[2] = (float)((int32_t)(imu_data[6]<<16) | imu_data[7])/4096000.0f;

    spi_regs_data.accel[0] = (float)((int32_t)(imu_data[8]<<16) | imu_data[9])/327680000.0f;
    spi_regs_data.accel[1] = (float)((int32_t)(imu_data[10]<<16) | imu_data[11])/327680000.0f;
    spi_regs_data.accel[2] = (float)((int32_t)(imu_data[12]<<16) | imu_data[13])/327680000.0f;

    spi_regs_data.temp = -((float)(((imu_data[14]<<16) | imu_data[15]) - 172621824))/263.4/65536+25;

    printf("cnt: %d, %d, %d, %d, %d %d %d, %d\n", spi_cnt, 
        (int32_t)(spi_regs_data.gyro[0]),
        (int32_t)(spi_regs_data.gyro[1]),
        (int32_t)(spi_regs_data.gyro[2]),
        (int32_t)(spi_regs_data.accel[0]),
        (int32_t)(spi_regs_data.accel[1]),
        (int32_t)(spi_regs_data.accel[2]));
        spi_cnt++;
}

uint32_t spi_error = 0;
uint32_t time_cnt = 0;

void IMU_SPI_Master_Read_Burst2(uint16_t *imu_data, uint64_t imu_timestamp)
{
	/**
	*CRC校验
	*/
	uint32_t check_crc = 1;
    uint32_t data_crc = 0;
    uint32_t data_u32 = 0;

    // uint32_t timep[2] = {0};

	data_crc = crc_crc32(check_crc, (uint8_t *)(&(imu_data[2])), sizeof(spi_burst_ext_data_t_mode2) - 4);

	/**
	*小端模式
	*/
	if((uint32_t)(imu_data[25]<<16 | imu_data[24]) == data_crc)
	{
        data_u32 = (imu_data[3]<<16 | imu_data[2]);
        spi_burst_data_mode2.gyro[0] = *(float*)&data_u32;
        
        data_u32 = (imu_data[5]<<16 | imu_data[4]);
        spi_burst_data_mode2.gyro[1] = *(float*)&data_u32;
        
        data_u32 = (imu_data[7]<<16 | imu_data[6]);
        spi_burst_data_mode2.gyro[2] = *(float*)&data_u32;
        
        data_u32 = (imu_data[9]<<16 | imu_data[8]);
        spi_burst_data_mode2.accel[0] = *(float*)&data_u32;
        
        data_u32 = (imu_data[11]<<16 | imu_data[10]);
        spi_burst_data_mode2.accel[1] = *(float*)&data_u32;
        
        data_u32 = (imu_data[13]<<16 | imu_data[12]);
        spi_burst_data_mode2.accel[2] = *(float*)&data_u32;
        
        data_u32 = (imu_data[15]<<16 | imu_data[14]);
        spi_burst_data_mode2.temperature = *(float*)&data_u32;
        
        data_u32 = (imu_data[17]<<16 | imu_data[16]);
        spi_burst_data_mode2.attitude[0] = *(float*)&data_u32;
        
        data_u32 = (imu_data[19]<<16 | imu_data[18]);
        spi_burst_data_mode2.attitude[1] = *(float*)&data_u32;
        
        data_u32 = (imu_data[21]<<16 | imu_data[20]);
        spi_burst_data_mode2.attitude[2] = *(float*)&data_u32;
        
        spi_burst_data_mode2.status=((int32_t)(imu_data[23]<<16) | imu_data[22]);
        
        spi_burst_data_mode2.check_crc =((int32_t)(imu_data[25]<<16) | imu_data[24]);

        // memcpy(timep, &imu_timestamp, sizeof(timep));

        // if (timep[1] == 4294967295)
        // {
        //     // printf("HAHA!\r\n");
        // }

        // printf("cnt: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
        //     (int32_t)(spi_burst_data_mode2.gyro[0]),
        //     (int32_t)(spi_burst_data_mode2.gyro[1]),
        //     (int32_t)(spi_burst_data_mode2.gyro[2]),
        //     (int32_t)(spi_burst_data_mode2.accel[0]),
        //     (int32_t)(spi_burst_data_mode2.accel[1]),
        //     (int32_t)(spi_burst_data_mode2.accel[2]),
        //     (int32_t)(spi_burst_data_mode2.attitude[0]),
        //     (int32_t)(spi_burst_data_mode2.attitude[1]),
        //     (int32_t)(spi_burst_data_mode2.attitude[2]),
        //     (int32_t)(spi_burst_data_mode2.temperature));
        
        Sensor_IMU_data.utime = 0; // imu_timestamp / 63897600000;
        Sensor_IMU_data.gyro[0] = spi_burst_data_mode2.gyro[0] * M_PI / 180;  // 弧度/s
        Sensor_IMU_data.gyro[1] = spi_burst_data_mode2.gyro[1] * M_PI / 180;
        Sensor_IMU_data.gyro[2] = spi_burst_data_mode2.gyro[2] * M_PI / 180;
        Sensor_IMU_data.acc[0] = spi_burst_data_mode2.accel[0] * GRAVITY;  // m/s^2
        Sensor_IMU_data.acc[1] = spi_burst_data_mode2.accel[1] * GRAVITY;
        Sensor_IMU_data.acc[2] = spi_burst_data_mode2.accel[2] * GRAVITY;  
        Sensor_IMU_data.attitude[0] = spi_burst_data_mode2.attitude[1];  // roll
        Sensor_IMU_data.attitude[1] = spi_burst_data_mode2.attitude[0];  // pitch
        Sensor_IMU_data.attitude[2] = spi_burst_data_mode2.attitude[2];  // yaw
        Sensor_IMU_data.temperature = spi_burst_data_mode2.temperature;
        Sensor_IMU_data.type = 0x01;
        Sensor_IMU_data.status = (uint8_t)spi_burst_data_mode2.status;
        Sensor_IMU_data.msg_cnt++;

        // if (time_cnt >= 50)
        // {
        //     time_cnt = 0;
        // }
	}
    else
    {
        spi_error++;
    }
}

void BMM_SPI_Init(void)
{
    uint16_t power_ctrl_reg;

    LGPIO_Mode(LGPIO0, LGPIO_PinSource4, PP);
    LGPIO_PULLMode(LGPIO0, LGPIO_PinSource4, HIGH_Z);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource4, 1);

    LGPIO_Mode(LGPIO0, LGPIO_PinSource5, PP);
    LGPIO_PULLMode(LGPIO0, LGPIO_PinSource5, HIGH_Z);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource5, 1);

    LGPIO_Mode(LGPIO0, LGPIO_PinSource1, PP);
    LGPIO_PULLMode(LGPIO0, LGPIO_PinSource1, HIGH_Z);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource1, 0);


    // LGPIO_Mode(LGPIO0, LGPIO_PinSource7, PP);
    // LGPIO_PULLMode(LGPIO0, LGPIO_PinSource7, HIGH_Z);
    // LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 1);

    vTaskDelay(pdMS_TO_TICKS(100)*10);

    // 初始化拉低复位一次
//    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 1);
//    vTaskDelay(pdMS_TO_TICKS(100));
//    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 0);
//    vTaskDelay(pdMS_TO_TICKS(100));
//    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource7, 1);
//    vTaskDelay(pdMS_TO_TICKS(100)*30);  // 初始化读取前，需将原极IMU复位并等待3s，使IMU进入正常工作状态。

    // power up the sensor from suspend to sleep mode
//    SPI_ReadWriteByte16(0xFE01);
//    power_ctrl_reg = SPI_ReadWriteByte16(0x6C00);
//    power_ctrl_reg = SPI_ReadWriteByte16(0x00FF);
//    power_ctrl_reg = SPI_ReadWriteByte16(0x00FF);

//    power_ctrl_reg = SPI_ReadWriteByte16(0xCB00);
//    power_ctrl_reg = SPI_ReadWriteByte16(0x00FF);
//    power_ctrl_reg = SPI_ReadWriteByte16(0x00FF);
//    power_ctrl_reg |= 0x01;  // 第0比特(Power Control Bit)置1

    //SPI_CS_ON;

    uint16_t spi_data;
    uint16_t rw_flag = 1;
    QSPI_DirectionConfig(QSPI2, QSPI_FMT_DIR_RX);
    spi_data = (0x4B << 8)  + (rw_flag << 15);
    while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_TX_FULL)) {}
    QSPI_SendData(QSPI2, spi_data);
    while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_RX_EMPTY)) {}
    spi_data = QSPI_ReceiveData(QSPI2);

    // SPI_CS_OFF;
    
    // start-up time delay of 3ms
    // usleep(3);

    // read chip ID of the sensor
    // BMM150_REG_CHIP_ID

    // set configurations like powermode, odr and interrupt mapping

}










