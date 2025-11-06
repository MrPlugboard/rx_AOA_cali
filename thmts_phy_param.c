#include "thmts_phy_param.h"
#include <stdio.h>
// For tx & rx physical layer parameters
THMTS_PhyParamConfig_t thmts_phycfg = {
    .sts_mode = THMTS_STSMODE_OFF,
    .preamble_type = THMTS_PREAMBLE_DS,
    .preamble_length = 128,
    .sfd_length = 8,
    .phr_mode = THMTS_PHRMODE_EXT,
    .bit_rate = THMTS_BITRATE_1M,
    .rf_channel = THMTS_RF_CHANNEL_4G,
    .rf_bandwidth = THMTS_RF_BANDWIDTH_500M,
    .freq_grid_idx = 0,
    .preamble_code_sel = 0,
    .sfd_sel = 0,
    .phr_code_sel = 0,
    .psdu_code_sel = 0,
    .div_ratio_sel = THMTS_DIVRATIO_1,

	.rf_chan_num = 4,        // 射频通道数量
    .accum_1to4_enable = 0,  // 0代表各个通道使用自己的ACCUM，1代表都使用第1路的ACCUM值
    .aoa_enable = 1,         // 0代表关闭AOA模式 , 1代表打开AOA模式

	.psdu_length = 0,

    //衍生字段
    .length_cfg_word = 0,
    .phr_info_bit = 0,
    .phr_buff_word1 = 0,
    .phr_buff_word2 = 0,


    .tx_config_word = 0,
	.rx_config_word = 0,
    .tx_delta_tick = 0
};


void Set_Channel_Config_By_Case( uint8_t case_idx )
{
    switch( case_idx )
    {
    case BITRATE_1M_B500M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_1M_B500M\r\n" );
        break;
    case BITRATE_2M_B500M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_2M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_2M_B500M\r\n" );
        break;
    case BITRATE_4M_B500M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_4M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_4M_B500M\r\n" );
        break;
    case BITRATE_8M_B500M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_8M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_8M_B500M\r\n" );
        break;
    case BITRATE_1M_B1000M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_1M_B1000M\r\n" );
        break;
    case BITRATE_2M_B1000M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_2M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_2M_B1000M\r\n" );
        break;
    case BITRATE_4M_B1000M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_4M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_4M_B1000M\r\n" );
        break;
    case BITRATE_8M_B1000M:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_8M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_8M_B1000M\r\n" );
        break;
    case BITRATE_850K_HRP31:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_850K;
    	thmts_phycfg.preamble_type = THMTS_PREAMBLE_HRP31;
    	thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_850K_HRP31\r\n" );
    	break;
    case BITRATE_850K_HRP127:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_850K;
    	thmts_phycfg.preamble_type = THMTS_PREAMBLE_HRP127;
    	thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_850K_HRP127\r\n" );
    	break;
    case BITRATE_6M8_HRP31:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_6M8;
    	thmts_phycfg.preamble_type = THMTS_PREAMBLE_HRP31;
    	thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_6M8_HRP31\r\n" );
    	break;
    case BITRATE_6M8_HRP127:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_6M8;
    	thmts_phycfg.preamble_type = THMTS_PREAMBLE_HRP127;
    	thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_6M8_HRP127\r\n" );
    	break;
    case BITRATE_1M_B500M_DIV2:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_2;
        printf( "configure bb to BITRATE_1M_B500M_DIV2\r\n" );
        break;
    case BITRATE_2M_B500M_DIV2:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_2M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_2;
        printf( "configure bb to BITRATE_2M_B500M_DIV2\r\n" );
        break;
    case BITRATE_4M_B500M_DIV2:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_4M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_2;
        printf( "configure bb to BITRATE_4M_B500M_DIV2\r\n" );
        break;
    case BITRATE_8M_B500M_DIV2:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_8M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_2;
        printf( "configure bb to BITRATE_8M_B500M_DIV2\r\n" );
        break;
    case BITRATE_1M_B1000M_DIV2:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_2;
        printf( "configure bb to BITRATE_1M_B1000M_DIV2\r\n" );
        break;
    case BITRATE_1M_B1000M_DIV4:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_1G;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_4;
        printf( "configure bb to BITRATE_1M_B1000M_DIV4\r\n" );
        break;

    default:
    	thmts_phycfg.bit_rate = THMTS_BITRATE_1M;
        thmts_phycfg.rf_bandwidth = THMTS_RF_BANDWIDTH_500M;
        thmts_phycfg.preamble_type = THMTS_PREAMBLE_DS;
        thmts_phycfg.div_ratio_sel = THMTS_DIVRATIO_1;
        printf( "configure bb to BITRATE_1M_B500M\r\n" );
        break;
    }

    //发射数据内容长度
    thmts_phycfg.psdu_length = sizeof(thmts_ranging_packet_t) - 4;
    // 发射内容长度加上2字节CRC校验
    uint32_t txdata_length = sizeof(thmts_ranging_packet_t)  - 2;

	uint32_t length_par = (thmts_phycfg.preamble_length << CHIP_THURDZ_TX_PRMB_LEN_OFFSET) | (thmts_phycfg.sfd_length << CHIP_THURDZ_TX_SFD_LEN_OFFSET) | (txdata_length & 0xfff);
	thmts_phycfg.length_cfg_word = length_par;

	uint32_t phr_info_bit = 0;
	uint32_t phr_buf_word = 0;

	// config phr buff word
	if ((thmts_phycfg.bit_rate != THMTS_BITRATE_850K) && (thmts_phycfg.bit_rate != THMTS_BITRATE_6M8))
	{
		switch ( thmts_phycfg.bit_rate )
		{
		case THMTS_BITRATE_1M:
			phr_info_bit |= 0xc0000000;
			break;
		case THMTS_BITRATE_2M:
			phr_info_bit |= 0x80000000;
			break;
		case THMTS_BITRATE_4M:
			phr_info_bit |= 0x40000000;
			break;
		case THMTS_BITRATE_8M:
			phr_info_bit |= 0x00000000;
			break;
		default:
			break;
		}
		phr_info_bit |= ((txdata_length & 0xfff) << 18);
		// Detecting bit & Reserve field.
		phr_info_bit |= 0x00020000;

		uint32_t parity = ((phr_info_bit>>24)&0xff) + ((phr_info_bit>>16)&0xff) + ((phr_info_bit>>8)&0xff);
		phr_info_bit |= (parity & 0xff);



		thmts_phycfg.phr_info_bit = phr_info_bit;
		if ((phr_info_bit>>31) & 0x1)
			phr_buf_word |= 0x3;
		else
			phr_buf_word |= 0x1;

		for (uint32_t i = 1; i <16; i++)
		{
			phr_buf_word <<= 2;
			if ((phr_info_bit>>(31-i)) & 0x1)
				phr_buf_word |= 0x3;
			else
				phr_buf_word |= 0x1;
		}
		thmts_phycfg.phr_buff_word1 = phr_buf_word;

		if ((phr_info_bit>>15) & 0x1)
			phr_buf_word |= 0x3;
		else
			phr_buf_word |= 0x1;

		for (uint32_t i = 17; i <32; i++)
		{
			phr_buf_word <<= 2;
			if ((phr_info_bit>>(31-i)) & 0x1)
				phr_buf_word |= 0x3;
			else
				phr_buf_word |= 0x1;
		}
		thmts_phycfg.phr_buff_word2 = phr_buf_word;
	}
	else
	{

	}


	uint32_t tx_config_word = 0;

	if (thmts_phycfg.preamble_type == THMTS_PREAMBLE_DS)
	{
		if (thmts_phycfg.rf_bandwidth == THMTS_RF_BANDWIDTH_500M)
			tx_config_word = 511;
		else if (thmts_phycfg.rf_bandwidth == THMTS_RF_BANDWIDTH_1G)
			tx_config_word = 1023;

		if( thmts_phycfg.bit_rate == THMTS_BITRATE_1M )
			tx_config_word |= (1 << 16);
		else if( thmts_phycfg.bit_rate == THMTS_BITRATE_2M ) tx_config_word |= (2 << 16);
		else if( thmts_phycfg.bit_rate == THMTS_BITRATE_4M ) tx_config_word |= (4 << 16);
		else if( thmts_phycfg.bit_rate == THMTS_BITRATE_8M ) tx_config_word |= (8 << 16);

		tx_config_word |= ( 1 << thmts_phycfg.div_ratio_sel ) << 24;
	}
	else
	{
	// todo: for HRP
	}

	//发射字与接收字配置一致
	thmts_phycfg.tx_config_word = tx_config_word;
	thmts_phycfg.rx_config_word = tx_config_word;





	//计算延迟发射delta tick
    uint32_t preamble_symbol_sample_count;

    if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_DS )
    {
        switch( thmts_phycfg.rf_bandwidth )
        {
        case THMTS_RF_BANDWIDTH_500M:
            preamble_symbol_sample_count = 1022;
            break;
        case THMTS_RF_BANDWIDTH_1G:
            preamble_symbol_sample_count = 2046;
            break;
        default:
            preamble_symbol_sample_count = 1022;
            break;
        }
    }
    else if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_HRP31 )
    {
        preamble_symbol_sample_count = 992;
    }
    else if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_HRP127 )
    {
        preamble_symbol_sample_count = 1016;
    }

    uint32_t preamble_symbol_count = thmts_phycfg.preamble_length + thmts_phycfg.sfd_length;

    if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_DS && thmts_phycfg.rf_bandwidth == THMTS_RF_BANDWIDTH_500M )
    {
        thmts_phycfg.tx_delta_tick = ( ( ( preamble_symbol_count * preamble_symbol_sample_count ) >> 3 ) << thmts_phycfg.div_ratio_sel ) + 258 + 6;

        if( thmts_phycfg.div_ratio_sel == THMTS_DIVRATIO_2 )
        {
        	thmts_phycfg.tx_delta_tick = 246 + 10;
        }
    }
    else if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_DS && thmts_phycfg.rf_bandwidth == THMTS_RF_BANDWIDTH_1G )
    {
        thmts_phycfg.tx_delta_tick = ( ( ( preamble_symbol_count * preamble_symbol_sample_count ) >> 3 ) << thmts_phycfg.div_ratio_sel ) + 258 + 256 + 6;


        if( thmts_phycfg.div_ratio_sel == THMTS_DIVRATIO_2 )
        {
        	thmts_phycfg.tx_delta_tick = ( 499 + 13 );
        }
        else if( thmts_phycfg.div_ratio_sel == THMTS_DIVRATIO_4 )
        {
        	thmts_phycfg.tx_delta_tick = ( 1496 + 39 );
        }
    }
    else if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_HRP31 )
    {
        thmts_phycfg.tx_delta_tick = ( ( preamble_symbol_count * preamble_symbol_sample_count ) >> 3 ) + 251 + 6;

    }
    else if( thmts_phycfg.preamble_type == THMTS_PREAMBLE_HRP127 )
    {
    	thmts_phycfg.tx_delta_tick = ( ( preamble_symbol_count * preamble_symbol_sample_count ) >> 3 ) + 257 + 6;
    }
}



