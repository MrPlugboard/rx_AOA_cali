#include "ns_sdk_hal.h"
#include "thmts_bb_config.h"
#include "thmts_bb_firmware.h"
#include "thmts_config.h"


#include "thmts_phy_param.h"
#include "thmts_node_info.h"
#include "thmts_tx_msg.h"
#include "thmts_ranging.h"




uint32_t channel0_CIR[1022];
uint32_t channel1_CIR[1022];
uint32_t channel2_CIR[1022];
uint32_t channel3_CIR[1022];
uint16_t firstPeakIdx;



uint32_t AGC_result[3]={0};
uint32_t AGC_idx=0;


uint32_t tx_flen = sizeof(thmts_ranging_packet_t) - 4;

thmts_ranging_packet_t thmts_tx_frame;
thmts_ranging_packet_t thmts_rx_frame;

static uint32_t reg_val_in_gpio[2] = { 0 };

// 接收数据的全局变量
THMTS_RX_CONTENT_t thmts_rx_content;

void write_thmts_bb_reg(uint32_t addr, uint32_t data)
{
	*((volatile uint32_t*)(THMTS_BB_BASE + addr)) = data;

	if( addr != ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK )
			&& addr != ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE1_WE_MASK ) )
		return;
	else if( addr == ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) )
	{
		reg_val_in_gpio[0] = data;
		return;
	}
	else
	{
		reg_val_in_gpio[1] = data;
		return;
	}
    return;
}

uint32_t read_thmts_bb_reg(uint32_t addr)
{
    return 	*((volatile uint32_t*)(THMTS_BB_BASE + addr));
}


//void write_thmts_bb_reg_with_offset(uint32_t addr, uint8_t start_offset, uint8_t length, uint32_t data)
//{
//	uint32_t raw_reg_data;
//	if( addr == ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) )
//		raw_reg_data = reg_val_in_gpio[0];
//	else if( addr == ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE1_WE_MASK ) )
//		raw_reg_data = reg_val_in_gpio[1];
//	else raw_reg_data = read_thmts_bb_reg( addr );
//	uint32_t mask;
//	uint8_t i;
//
//	if( length > 16 )
//	{
//		length = 32 - length;
//
//		mask = 0xffffffff;
//		for( i = 0; i < length; i++ )
//		{
//			mask = ( mask >> 1 );
//		}
//		mask = (mask << start_offset);
//	}
//	else
//	{
//		mask = 0x00000000;
//		for( i = 0; i < length; i++ )
//		{
//			mask = ( mask << 1 ) | 0x00000001;
//		}
//		mask = (mask << start_offset);
//	}
//
//	write_thmts_bb_reg( addr, ( raw_reg_data & ( ~mask ) ) | ( ( data << start_offset ) & mask ) );
//
//	return;
//}



void write_thmts_bb_reg_with_offset(uint32_t addr, uint8_t start_offset, uint8_t length, uint32_t data)
{
	uint32_t raw_reg_data;
	if( addr == ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) )
		raw_reg_data = reg_val_in_gpio[0];
	else if( addr == ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE1_WE_MASK ) )
		raw_reg_data = reg_val_in_gpio[1];
	else raw_reg_data = read_thmts_bb_reg( addr );
	uint32_t mask;
	uint8_t i;

	mask = 0xffffffff;
	mask = mask << (32 -length-start_offset);
	mask = mask >> (32 -length);
	mask = mask << start_offset;

	write_thmts_bb_reg( addr, ( raw_reg_data & ( ~mask ) ) | ( ( data << start_offset ) & mask ) );

	return;
}





uint32_t read_thmts_bb_reg_with_offset( uint32_t addr, uint8_t offset, uint8_t length )
{
	uint32_t raw_reg_data;
	raw_reg_data = read_thmts_bb_reg( addr );

	if( length > 16 )   // large number of 1 in mask, then fill in 0 step by step.
	{
		length = 32 - length;

		uint32_t mask = 0xffffffff;
		uint8_t i;
		for( i = 0; i < length; i++ )
		{
			mask = ( mask >> 1 );   // Unsigned right shift.
		}

		return ( raw_reg_data >> offset ) & mask;
	}
	else    // Small number of 1 in mask, then fill in 1 step by step.
	{
		uint32_t mask = 0x00000000;
		uint8_t i;
		for( i = 0; i < length; i++ )
		{
			mask = ( mask << 1 ) | 0x00000001;
		}

		return ( raw_reg_data >> offset ) & mask;
	}
}





/*!
 * @brief Pull down the rstn wired to bb_top and then raise up.
*/
void reset_thmts_bb()
{
    apb_slv0_async_fifo_set_rst( 0 );
    delay_1ms( 1 );
    apb_slv0_async_fifo_set_rst( 1 );
    return;
}


/*
 * Reset the thurdz module, the membus and the ucore, set state to close.
*/
void reset_thmts_bb_rx_module()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0xffffffff );
	//for( uint32_t i = 0; i < 1000; i++ );
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0 );

    return;
}


// 接收模块关机，power down
void thmts_bb_rx_pd()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0x7fffffff );
	return;
}

// 接收模块开机，power on
void thmts_bb_rx_po()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0 );
	return;
}

void reset_thmts_bb_rx_ucore()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_RX_START_OFFSET, CHIP_THURDZ_RX_START_LENGTH, 0x0f );
	//for( uint32_t i = 0; i < 1000; i++ );
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_RX_START_OFFSET, CHIP_THURDZ_RX_START_LENGTH, 0x07 );

    return;
}

void reset_thmts_bb_tx_module()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0xffffffff );
	//for( uint32_t i = 0; i < 1000; i++ );
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0 );

    return;
}

// 发射模块关机，power down
void thmts_bb_tx_pd()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0xffffffff );
    return;
}

// 发射模块开机，power on
void thmts_bb_tx_po()
{
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0 );
	return;
}

void reset_thmts_bb_tx_ucore()
{
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_TX_START_OFFSET, CHIP_THURDZ_TX_START_LENGTH, 0x0f );
	//for( uint32_t i = 0; i < 1000; i++ );
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_TX_START_OFFSET, CHIP_THURDZ_TX_START_LENGTH, 0x07 );

    return;
}

// Reset instruction ram and data ram to zero.

void clear_thmts_bb_ucore_ram()
{
    for( uint32_t i = 0; i < 0x2000; i+=4 )
    {
    	write_thmts_bb_reg( i | 0x00000000, 0 );
    	write_thmts_bb_reg( i | 0x00010000, 0 );
    	write_thmts_bb_reg( i | 0x01000000, 0 );
    	write_thmts_bb_reg( i | 0x01010000, 0 );
    }
    return;
}


// 复位、下载固件、

void init_thmts_bb()
{
	reset_thmts_bb();

    // Reset the tpram, ucore, membus, acquire the im and dm token.
	write_thmts_bb_reg(CHIP_THURDZ_UCORE_CTRL_ADDR, 0x00010f0f);
	for( uint32_t i = 0; i < 1000; i++ );
	write_thmts_bb_reg(CHIP_THURDZ_UCORE_CTRL_ADDR, 0x00000707);

    /*
     * Enable the timer intr, pwm, pps output.
    */
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_ADDR, CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_OFFSET, CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_LENGTH, 1 );
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_ADDR, CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_OFFSET, CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_LENGTH, 1 );
	write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_WIDTH_ADDR, 1248000 );

    /*
     * Only use adc data from one rx antenna or not.
    */
	// 1,使用第一路ACCUM值给各个通道
	// 0,各自使用各自的ACCUM值
	//默认写成0
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_4TO1_EN_ADDR, CHIP_THURDZ_4TO1_EN_OFFSET, CHIP_THURDZ_4TO1_EN_LENGTH, 0 );

    /*
     * Use build-in loopback signal channel or not.
    */
	write_thmts_bb_reg_with_offset( CHIP_THURDZ_SELFLOOP_EN_ADDR, CHIP_THURDZ_SELFLOOP_EN_OFFSET, CHIP_THURDZ_SELFLOOP_EN_LENGTH, 0 );

    /*
     * Reset all hw modules, and set state to CLOSE.
    */
//	reset_thmts_bb_rx_module();
//	reset_thmts_bb_rx_ucore();
//	reset_thmts_bb_tx_module();
//	reset_thmts_bb_tx_ucore();


	thmts_bb_rx_pd();
	thmts_bb_tx_pd();

	clear_thmts_bb_ucore_ram();
	download_thmts_bb_ucore_fw();

    write_thmts_bb_reg_with_offset( CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_ADDR, CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_OFFSET, CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_LENGTH, 1 );
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_ADDR, CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_OFFSET,
                                CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_LENGTH, 1 ); // 原码



    return;
}


void enable_thmts_bb_loopback( uint8_t enable_flag )
{
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_SELFLOOP_EN_ADDR, CHIP_THURDZ_SELFLOOP_EN_OFFSET, CHIP_THURDZ_SELFLOOP_EN_LENGTH, enable_flag );
    return;
}


/*
 * Get systime from urdz timer, read the 32bit value and extend to 64bit.
*/

uint32_t get_thmts_bb_systime(int64_t* t)
{
    // Read systime from two membus and judge by apb_interface.
    uint32_t val = read_thmts_bb_reg( CHIP_THURDZ_UCORE_CTRL_ADDR );

    if( val & ( 0x01 << CHIP_THURDZ_RX_DM_TOKEN_OFFSET ) )
    {
        /*
         * Macro systime_addr is in ucore1 region.
        */
        int64_t systime = read_thmts_bb_reg( CHIP_THURDZ_SYSTIME_ADDR ^ 0x01000000 );
        *t = ( int64_t )systime << ( CHIP_THURDZ_FULL_STAMP_BITS - CHIP_THURDZ_SHORT_STAMP_BITS );
        return 1;
    }
    else if( val & ( 0x01 << CHIP_THURDZ_TX_DM_TOKEN_OFFSET ) )
    {
        int64_t systime = read_thmts_bb_reg( CHIP_THURDZ_SYSTIME_ADDR );
        *t = ( int64_t )systime << ( CHIP_THURDZ_FULL_STAMP_BITS - CHIP_THURDZ_SHORT_STAMP_BITS );
        return 1;
    }
    else
    {
        *t = -1;
        return 0;
    }

}


int64_t timestamp_add( int64_t a, int64_t b )
{
    int64_t c = a + b;
    if( c >= FULL_STAMP_MAX )
    {
        c -= FULL_STAMP_MAX;
    }
    else if( c < 0 )
    {
        c += FULL_STAMP_MAX;
    }
    return c;
}

int64_t timestamp_minus( int64_t a, int64_t b )
{
    if( a > b )
    {
        int64_t c = a - b;
        int64_t d = FULL_STAMP_MAX + b - a;
        if( c < d ) return c;
        else return -1 * d;
    }
    else
    {
        int64_t c = b - a;
        int64_t d = FULL_STAMP_MAX + a - b;
        if( c < d ) return -1 * c;
        else return d;
    }
}

int64_t timestamp_substract( int64_t a, int64_t b )
{
    return ( ( a - b ) < 0 ) ? ( a + FULL_STAMP_MAX - b ) : ( a - b );
}

int64_t us2timestamp( uint32_t us )
{
    int64_t stamp;
    stamp = us * CHIP_THURDZ_TICK_PER_SEC * 1e-6;
    return stamp;
}

//CRC16-CCITT calculation in software, as there is no rx data validation in hw.

uint16_t crc16_ccitt(uint8_t *di, uint32_t len)
{
    uint16_t crc_poly = 0x8408; //Bit sequence inversion of 0x1021
    uint16_t data_t = 0; //CRC register

    for(uint32_t i = 0; i < len; i++)
    {
        data_t ^= di[i]; //8-bit data

        for (uint8_t j = 0; j < 8; j++)
        {
            if (data_t & 0x0001)
                data_t = (data_t >> 1) ^ crc_poly;
            else
                data_t >>= 1;
        }
    }

    return data_t;
}

const uint16_t crc_table[256] ={
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t crc16_ccitt_enhanced(uint8_t *data, uint32_t len)
{
    uint16_t crc16 = 0x0000;
    uint16_t crc_h8, crc_l8;

    while( len-- ) {

        crc_h8 = (crc16 >> 8);
        crc_l8 = (crc16 << 8);
        crc16 = crc_l8 ^ crc_table[crc_h8 ^ *data];
        data++;
    }

    return crc16;
}


/**
 * Configure the pwm signal from urdz_timer.
 */
void enable_thmts_bb_pwm( uint32_t start_interval_us, uint32_t pwm_period, uint32_t pwm_on_period )
{
    int64_t curr_stamp = 0;
    get_thmts_bb_systime( &curr_stamp );

    int64_t next_stamp = timestamp_add( curr_stamp, us2timestamp( start_interval_us ) );
    next_stamp >>= 9;

    write_thmts_bb_reg_with_offset( CHIP_THURDZ_TIMER_PWM_OUT_ENA_ADDR, CHIP_THURDZ_TIMER_PWM_OUT_ENA_OFFSET, CHIP_THURDZ_TIMER_PWM_OUT_ENA_LENGTH, 1 );

    write_thmts_bb_reg( CHIP_THURDZ_TIMER_PWM_START_TIME_ADDR, next_stamp );
    write_thmts_bb_reg( CHIP_THURDZ_TIMER_PWM_PERIOD_ADDR, pwm_period );
    write_thmts_bb_reg( CHIP_THURDZ_TIMER_PWM_ON_PERIOD_ADDR, pwm_on_period );

    write_thmts_bb_reg_with_offset( CHIP_THURDZ_TIMER_PWM_ENA_ADDR, CHIP_THURDZ_TIMER_PWM_ENA_OFFSET, CHIP_THURDZ_TIMER_PWM_ENA_LENGTH, 1 );

    return;
}

void clear_thmts_bb_tpram(void)
{
    for( uint32_t i = 1; i < 16; i++ )
    {
        write_thmts_bb_reg( ( URDZ_PROTOCOL_BASE_ADDR | 0x80 | ZONE_APB_WE_MASK | ( i << 2 ) ), 0 );
    }
    return;
}

// p_data指向要写入数据的起始地址；offset为要写到位置相对于txbuff起始地址的偏移，单位为byte；len为要写入数据的长度，单位为byte。
void write_thmts_bb_txbuff(uint8_t *p_data, uint32_t offset, uint32_t length)
{

	memcpy((uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_TX_DATA_ADDR + offset), p_data , length);

    return;
}

// p_data指向读出数据存放的起始地址；offset为要读出位置相对于rxbuff起始地址的偏移，单位为byte；length为要读出数据的长度，单位为byte。
void read_thmts_bb_rxbuff(uint8_t *p_data, uint32_t offset, uint32_t length)
{
	memcpy(p_data , (uint8_t*)(THMTS_BB_BASE + CHIP_THURDZ_RX_DATA_ADDR + 0x5 + offset) , length);

	return;
}


void start_thmts_bb_tx(THMTS_PhyParamConfig_t *p_phycfg, thmts_ranging_packet_t *p_txdata, uint32_t txdata_length)
{
	//有一个配置：发射数据、发射数据长度、数据率、sfd、psr等信息
	//复位，准备好发射，启动发射。
	//返回
    //Reset each module in TX.
   // reset_thmts_bb_tx();
   // clear_thmts_bb_tpram();


	// 修改为只发射单通道
//    memcpy((uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_TX_DATA_ADDR), p_txdata , txdata_length);
//	memcpy((uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_TX_DATA_ADDR + 0x2000), p_txdata , txdata_length);


   // 发射长度配置字
   write_thmts_bb_reg(CHIP_THURDZ_TX_LENS_ADDR, p_phycfg->length_cfg_word);

   if ((p_phycfg->bit_rate != THMTS_BITRATE_850K) && (p_phycfg->bit_rate != THMTS_BITRATE_6M8))
   {
       write_thmts_bb_reg( CHIP_THURDZ_TX_PHR_ADDR, p_phycfg->phr_buff_word1 );
       write_thmts_bb_reg( CHIP_THURDZ_TX_PHR_ADDR + 0x04, p_phycfg->phr_buff_word2 );
       write_thmts_bb_reg( CHIP_THURDZ_TX_PHR_ADDR + 0x08, 0x55500000 );
   }
   else
   {

   }

   // 配置为一个发射通道
   write_thmts_bb_reg(CHIP_THURDZ_TX_CONFIG_START_ADDR, 1);

   write_thmts_bb_reg( CHIP_THURDZ_TX_CONFIG_START_ADDR + 0x04, p_phycfg->tx_config_word );

   // 配置发射频偏
   int32_t tx_freq_bias = 4000;
   write_thmts_bb_reg(0x02FD00A8, (uint32_t)(tx_freq_bias));



   // Release token and reset.
   // Trigger should be close enough w.r.t. ucore start, as ucore would read corresponding bits soon after enabled.
   write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_TX_START_OFFSET, CHIP_THURDZ_TX_START_LENGTH, 0x04 );
   write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_TRIGGER_ADDR, CHIP_THURDZ_TX_TRIGGER_OFFSET, CHIP_THURDZ_TX_TRIGGER_LENGTH, 1 );  // Automatically reset on this bit.

   write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_TX_START_OFFSET, CHIP_THURDZ_TX_START_LENGTH, 0x00 );

   return;
}

void clean_thmts_bb_tx(void)
{
    // Reset the ucore and acquire iram & dram token. MemBus is not reseted as some regs should be unchanged.
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_TX_START_OFFSET, CHIP_THURDZ_TX_START_LENGTH, 0x07 );
    // Clear delayed tx configuration if exists.
    write_thmts_bb_reg( CHIP_THURDZ_DLY_TX_STAMP_ADDR, 0x00 );
    write_thmts_bb_reg( CHIP_THURDZ_DLY_TX_STAMP_ADDR + 0x04, 0x00 );

    return;
}

void set_thmts_bb_delaytxtime(THMTS_PhyParamConfig_t *p_phycfg, uint32_t tick )
{
	tick = timestamp_substract(tick, p_phycfg->tx_delta_tick);

    write_thmts_bb_reg( CHIP_THURDZ_DLY_TX_STAMP_ADDR, tick );
    write_thmts_bb_reg( CHIP_THURDZ_DLY_TX_STAMP_ADDR + 0x04, 0x01 );

    return;
}

void set_thmts_bb_delayrxtime(uint32_t tick )
{
    write_thmts_bb_reg( CHIP_THURDZ_DLY_RX_STAMP_ADDR, tick );
    write_thmts_bb_reg( CHIP_THURDZ_DLY_RX_STAMP_ADDR + 0x04, 1 );

    return;
}

void set_thmts_bb_rxtimeout(uint32_t preamblesymbol_num)
{
    // It is assumed that under div-ratio0, the preamble symbol is always nearly 1us, by switching the chip clock from rf & adda.

	write_thmts_bb_reg( CHIP_THURDZ_RX_TIMEOUT_SYMBOL_ADDR, preamblesymbol_num );

    return;
}

// 设置延迟接收的时间、设置超时时间、复位接收模块、设置接收通道数、设置tpram中的接收参数、启动接收
// 收到接收中断后，清理接收单元，查看接收中断状态，读取接收数据长度、接收数据、rmark、cir等


void start_thmts_bb_rx(THMTS_PhyParamConfig_t *p_phycfg, uint32_t rx_timeout)
{
    set_thmts_bb_rxtimeout(rx_timeout);

    // 设置捕获门限
    // 默认值
//    write_thmts_bb_reg(0x02fd00dC , 0x000F0302);
//    write_thmts_bb_reg(0x02fd00dC , 0x000F0304);


//	write_thmts_bb_reg(0x02fd00e0 , 0);

    // 写0x1为单通道接收，写0x11为AOA模式
    if(p_phycfg->aoa_enable == 0)
    	write_thmts_bb_reg( CHIP_THURDZ_RX_CONFIG_START_ADDR, 0x01 );
    else
    	write_thmts_bb_reg( CHIP_THURDZ_RX_CONFIG_START_ADDR, 0x11 );

    write_thmts_bb_reg( CHIP_THURDZ_RX_CONFIG_START_ADDR + 0x04, p_phycfg->rx_config_word );

    // 配置接收频偏
    int32_t rx_freq_bias = -4000;
    write_thmts_bb_reg( 0x02FD00D4, (uint32_t)(rx_freq_bias));


    /*
     * Trigger should be close enough with respect to ucore start.
     */
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_RX_START_OFFSET, CHIP_THURDZ_RX_START_LENGTH, 0x04 );
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_TRIGGER_ADDR, CHIP_THURDZ_RX_TRIGGER_OFFSET, CHIP_THURDZ_RX_TRIGGER_LENGTH, 1 );  // Automatically reset on this bit.
    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_RX_START_OFFSET, CHIP_THURDZ_RX_START_LENGTH, 0x00 );

	return;
}

void clean_thmts_bb_rx(void)
{
    // Reset the ucore and acquire the memory token. MemBus is not reseted as some configurations inside should keep unchanged.

    write_thmts_bb_reg_with_offset( CHIP_THURDZ_UCORE_CTRL_ADDR, CHIP_THURDZ_RX_START_OFFSET, CHIP_THURDZ_RX_START_LENGTH, 0x07 );

    set_thmts_bb_rxtimeout(0);
    write_thmts_bb_reg( CHIP_THURDZ_DLY_RX_STAMP_ADDR, 0 );  // Write 0 to delayed rx stamp.
//    write_thmts_bb_reg( CHIP_THURDZ_DLY_RX_STAMP_ADDR + 4, 0 );  // Write 0 to disable delayed rx.

    write_thmts_bb_reg_with_offset(CHIP_THURDZ_DLY_RX_STAMP_ADDR + 4 , 0 , 4 , 0);
    return;
}

uint32_t get_thmts_bb_rxstatus()
{
	return read_thmts_bb_reg(CHIP_THURDZ_RX_STATUS_ADDR);
}

void get_thmts_bb_rxinfo(uint32_t* p_info)
{
    memcpy(p_info, (uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_RX_ISR_FLAG_ADDR) , 10*4);
    return;
}

// 接收字节数，不包括2个字节的crc
uint32_t get_thmts_bb_rxlength(THMTS_PhyParamConfig_t *p_phycfg)
{
	uint32_t rx_length;

	if (p_phycfg->bit_rate != THMTS_BITRATE_850K && p_phycfg->bit_rate != THMTS_BITRATE_6M8)
	{
		rx_length = ( ( ( read_thmts_bb_reg( CHIP_THURDZ_RX_PHR_VALUE_TMP_ADDR ) & 0x3FFC0000 ) >> 18 ) - 2 );
	}
	else
	{
		rx_length = ( ( ( read_thmts_bb_reg( CHIP_THURDZ_RX_PHR_VALUE_TMP_ADDR ) & 0x000007F0 ) >> 4 ) - 2 );
	}

	return rx_length;
}

//// 从rx地址读取数据到rx_data数组中
//// 长度len是5字节PHR长度 + payload长度 + 2字节CRC校验
//void read_thmts_bb_rxbuff(uint32_t len)
//{
////    uint32_t word_num;
////
////    if (len&0x3) word_num = (len>>2) + 1;
////    else word_num = (len>>2);
////
////    for (uint32_t i = 0; i < word_num; i++)
////    	*(uint32_t *)(rx_data+(i<<2)) = read_thmts_bb_reg( CHIP_THURDZ_RX_DATA_ADDR + (i<<2) );
//
//
//    memcpy(rx_data , (uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_RX_DATA_ADDR) , len);
//    //
//
//    return;
//
//}

//uint32_t rx_data_time = 0;
//void read_thmts_bb_rxbuff(thmts_ranging_frame_t *p_rxdata, uint32_t len)
//{
//
//	// len的长度是thmts_ranging_frame_t的长度 - 4(RSV的长度)
//
////	int64_t t1 = 0;
////	int64_t t2 = 0;
////	get_thmts_bb_systime(&t1); // 这个数字是<<9位，得到的64G时钟
////	uint64_t rtc_tmp = SysTimer_GetLoadValue();
//    memcpy(p_rxdata , (uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_RX_DATA_ADDR + 0x5) , len);
//    //
//
//
////    get_thmts_bb_systime(&t2);
////    rx_data_time = (uint32_t)(SysTimer_GetLoadValue() - rtc_tmp);
//
//
//
//    return;
//
//}






void read_thmts_bb_cir(uint32_t rx_id, uint32_t* buf, uint32_t offset, uint32_t len)
{
    memcpy(buf, (uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_CIR_BASE_ADDR + rx_id * CHIP_THURDZ_ACCUM_RES_STRIP + offset) , len*4);
    return;
}

void get_thmts_bb_rmark(THMTS_PhyParamConfig_t *p_phycfg, int64_t* p)
{

	int64_t sys_timer = read_thmts_bb_reg( CHIP_THURDZ_RX_STAMP_HI_ADDR );
	uint32_t buffer = read_thmts_bb_reg( CHIP_THURDZ_RX_STAMP_LO_ADDR );
	int8_t delta_sample_num = buffer & 0x000000ff;
	int64_t last_index = (buffer & 0x0000ff00) >> 8;
	int64_t mpf_index = (buffer & 0x00ff0000) >> 16;

	// Correction from LDE, unit is 1/64G.
	// 65408 = 1022 * 64, todo : peak may not on 1022 for someother signal params config.
	uint32_t lde_result = read_thmts_bb_reg( CHIP_THURDZ_RX_LDE_RES_ADDR );

	if( p_phycfg->preamble_type == THMTS_PREAMBLE_DS )
	{
		if( p_phycfg->rf_bandwidth == THMTS_RF_BANDWIDTH_500M )
			lde_result = 65408 - lde_result;
	    else if( p_phycfg->rf_bandwidth  == THMTS_RF_BANDWIDTH_1G )
	    	lde_result = 130944 - lde_result;
	}
	else if( p_phycfg->preamble_type == THMTS_PREAMBLE_HRP31 )
	{
        lde_result = 63488 - lde_result;
	}
    else if( p_phycfg->preamble_type == THMTS_PREAMBLE_HRP127 )
    {
        lde_result = 65024 - lde_result;
    }

	int64_t curr_rx_stamp;
	curr_rx_stamp = ( ( ( sys_timer ) * 8 -
	                    ( (8 - (last_index)) << p_phycfg->div_ratio_sel ) -
	                    ( ((int64_t)delta_sample_num) << p_phycfg->div_ratio_sel ) ) << 6 ) -
	                    ( (64 - mpf_index) << p_phycfg->div_ratio_sel );
	curr_rx_stamp = curr_rx_stamp - ( lde_result << p_phycfg->div_ratio_sel );
	if( curr_rx_stamp < 0 ) curr_rx_stamp += FULL_STAMP_MAX;

	*p = curr_rx_stamp;

	return;

}



//获取接收内容
// 在rx_complete之后调用
void thmts_get_rx_content(uint32_t cmd)
{

	uint32_t rx_status = get_thmts_bb_rxstatus();
	thmts_rx_content.rx_status = rx_status;


	get_thmts_bb_rxinfo(thmts_rx_content.rx_info);

	uint32_t rx_length = 0;
	thmts_rx_content.CRC_OK = 0;


    if((rx_status & 0xff) == 4)
    {
		rx_length = get_thmts_bb_rxlength(&thmts_phycfg);


		if (rx_length == tx_flen)   // 接收数据长度符合预期
		{
			if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_DS )
			{
			// For DS, phr is of 38bit length, there are 40bit's gap from the start of rx data.
				if( thmts_phycfg.bit_rate != THMTS_BITRATE_850K && thmts_phycfg.bit_rate != THMTS_BITRATE_6M8 )
				{
					thmts_rx_content.rx_length = rx_length;
					read_thmts_bb_rxbuff(&thmts_rx_frame ,0, rx_length);

					memcpy(&(thmts_rx_content.rx_frame_data) , &thmts_rx_frame , sizeof(thmts_rx_frame));


//					uint16_t crc_result = crc16_ccitt_enhanced(&thmts_rx_frame,rx_length);
//					uint8_t crc_hi;
//					uint8_t crc_lo;
//					crc_hi = (uint8_t)(crc_result>>8);
//					crc_lo = (uint8_t)(crc_result);
//
//
//					uint32_t CRC_RX = 0;
//					memcpy(&CRC_RX , (uint32_t *)(0x10132000 + 5 + sizeof(thmts_ranging_frame_t) - 4) , 2);
//
//
//					if(crc_hi == (CRC_RX&0xFF) && crc_lo == ((CRC_RX>>8)&0xFF))
	    			uint32_t CRC_BB = read_thmts_bb_reg(0x02fd0024);
	    			uint16_t CRC_BB_HIGH16 = (uint16_t)(CRC_BB >> 16);
	    			uint16_t CRC_BB_LOW16  = (uint16_t)(CRC_BB & 0xFFFF);

	    			if(CRC_BB_HIGH16 == CRC_BB_LOW16)
					{
						thmts_rx_content.CRC_OK = 1;
			    		get_thmts_bb_rmark(&thmts_phycfg ,  &node.ts_curr_rmark );
						thmts_rx_content.rmark = node.ts_curr_rmark;





						// 从数据中读取得到第一径位置
						firstPeakIdx = (uint16_t)((read_thmts_bb_reg(0x02FD001C) & 0xFFFFFFC0) >> 6);
			    		// 通过CRC校验才出CIR
			    		read_thmts_bb_cir(0, (uint32_t*)channel0_CIR, 0, 1022);
			    		read_thmts_bb_cir(1, (uint32_t*)channel1_CIR, 0, 1022);
			    		read_thmts_bb_cir(2, (uint32_t*)channel2_CIR, 0, 1022);






			    		// 通过CRC校验才出CIR
//			    		read_thmts_bb_cir(0, (uint32_t*)channel0_CIR, 0, 1022);
//			    		read_thmts_bb_cir(1, (uint32_t*)channel1_CIR, 0, 1022);
//			    		read_thmts_bb_cir(2, (uint32_t*)channel2_CIR, 0, 1022);
//			    		read_thmts_bb_cir(3, (uint32_t*)channel3_CIR, 0, 1022);



					}
				}
			}
		}
    }

	thmts_rx_content.BBCTRL = read_thmts_bb_reg(CHIP_THURDZ_BBEX_CTRL_ADDR);
	thmts_rx_content.PHR_info = read_thmts_bb_reg(0x02fd000C);
}














static int64_t chip_thurdz_tick_per_us = CHIP_THURDZ_TICK_PER_US;
int64_t us2tick( uint32_t us )
{
    int64_t stamp = ( us * chip_thurdz_tick_per_us );
    while( stamp >= FULL_STAMP_MAX ) stamp -= FULL_STAMP_MAX;
    return stamp;
}
void config_thmts_bb_exctrl(uint8_t cmd)
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, cmd);
	return;
}


void config_thmts_bb_txsw_on()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET, 1, 0);
	return;
}

void config_thmts_bb_txsw_off()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET, 1, 1);
	return;
}

void config_thmts_bb_rx0sw_on()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 0);
	return;
}

void config_thmts_bb_rx0sw_off()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 1);
	return;
}

void config_thmts_bb_rx012sw_on()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 3, 0);
	return;
}

void config_thmts_bb_rx012sw_off()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 3, 7);
	return;
}


void config_thmts_bb_rx0123sw_on()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 4, 0);
	return;
}

void config_thmts_bb_rx0123sw_off()
{
	write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 4, 15);
	return;
}



//根据不同的天线数量打开RF的接收通道和接收低噪放的开关
void config_thmts_bb_rx_sw_lna_on(uint8_t rf_chan_num)
{
	switch(rf_chan_num)
	{
	case 1:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 0);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xC5);
		break;
	case 3:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 3, 0);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0x05);
		break;
	case 4:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 4, 0);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0x01);
		break;
	default:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 0);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xC5);
		break;
	}
	return;
}



// 根据不同的天线数量关闭RF的接收通道和接收低噪放的开关
void config_thmts_bb_rx_sw_lna_off(uint8_t rf_chan_num)
{
	switch(rf_chan_num)
	{
	case 1:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 1);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xE5);
		break;
	case 3:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 3, 7);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xE5);
		break;
	case 4:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 4, 15);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xE5);
		break;
	default:
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET+1, 1, 1);
		write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xE5);
		break;
	}
	return;
}



void config_thmts_bb_tx_pa_on()
{
	config_thmts_bb_exctrl(0xf6);
}

void config_thmts_bb_tx_pa_off()// 单发射，功放关
{
	config_thmts_bb_exctrl(0xe6);
}

void config_thmts_bb_rx_lna1_on() // 单接收，通道1的LNA开
{
	config_thmts_bb_exctrl(0xc5);
}


void config_thmts_bb_rx_lna123_on()// 单接收，通道123的LNA开
{
	config_thmts_bb_exctrl(0x05);
}


void config_thmts_bb_rx_lna_off()
{
	// 单接收，全通道的LNA关
	config_thmts_bb_exctrl(0xE5);
}







