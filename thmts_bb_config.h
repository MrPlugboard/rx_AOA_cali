#ifndef THMTSV1_CHIP_H
#define THMTSV1_CHIP_H

#include "stdint.h"
#include "string.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CPU_BASE_ADDRESS                        0

#define URDZ_IM_32BITS_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00000000)
#define URDZ_DM_32BITS_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00010000)
#define URDZ_VM_32BITS_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00020000)

// for modules in zone0
#define URDZ_RESAMPLER_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00030000)
#define URDZ_DDC_BASE_ADDR                      (CPU_BASE_ADDRESS + 0x00040000)
#define URDZ_CORR_BASE_ADDR                     (CPU_BASE_ADDRESS + 0x00050000)
#define URDZ_ACCUM_BASE_ADDR                    (CPU_BASE_ADDRESS + 0x00060000)
#define URDZ_PEAK_FINDER_BASE_ADDR              (CPU_BASE_ADDRESS + 0x00070000)
#define URDZ_CMF_BASE_ADDR                      (CPU_BASE_ADDRESS + 0x00080000)
#define URDZ_EPL_MERGE_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00090000)
#define URDZ_VITERBI_BASE_ADDR                  (CPU_BASE_ADDRESS + 0x000A0000)
#define URDZ_CHANNEL_BASE_ADDR                  (CPU_BASE_ADDRESS + 0x00110000)
#define URDZ_DAGC_BASE_ADDR                     (CPU_BASE_ADDRESS + 0x00120000)
#define URDZ_RS_DECODER_BASE_ADDR               (CPU_BASE_ADDRESS + 0x00130000)
#define URDZ_DOWNSAMPLER_BASE_ADDR              (CPU_BASE_ADDRESS + 0x00140000)

// for modules in zone1
#define URDZ_PROTO_GEN_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00200000)
#define URDZ_ENCODER_CONV_BASE_ADDR             (CPU_BASE_ADDRESS + 0x00210000)
#define URDZ_SPREADER_BASE_ADDR                 (CPU_BASE_ADDRESS + 0x00220000)
#define URDZ_UPSAMPLER_BASE_ADDR                (CPU_BASE_ADDRESS + 0x00230000)
#define URDZ_UDC_BASE_ADDR                      (CPU_BASE_ADDRESS + 0x00240000)

// for both zone0 and zone1
#define URDZ_TIMER_BASE_ADDR                    (CPU_BASE_ADDRESS + 0x00400000)

// for large address space, now only used in zone0
#define URDZ_ADDA_INTERFACE_BASE_ADDR           (CPU_BASE_ADDRESS + 0x00D00000)
#define URDZ_RFSOC_REG_MAP_BASE_ADDR            (CPU_BASE_ADDRESS + 0x00E00000)

// for internal address space inside urdz_controller
#define URDZ_VPU_BASE_ADDR                      (CPU_BASE_ADDRESS + 0x00FA0000)
#define URDZ_CONSOLE_BASE_ADDR                  (CPU_BASE_ADDRESS + 0x00FB0000)
#define URDZ_INTC_BASE_ADDR                     (CPU_BASE_ADDRESS + 0x00FC0000)
#define URDZ_PROTOCOL_BASE_ADDR                 (CPU_BASE_ADDRESS + 0x00FD0000)
#define URDZ_MEMBUS_BASE_ADDR                   (CPU_BASE_ADDRESS + 0x00FE0000)
#define URDZ_VM_1024BITS_BASE_ADDR              (CPU_BASE_ADDRESS + 0x00FF0000)

#define RF_XINANJICHENG_RFIN_SW_CTRL_ADDR                       0x14
#define RF_XINANJICHENG_RFIN_EN_SWMIX_OFFSET                    4
#define RF_XINANJICHENG_RFIN_EN_SWDA_OFFSET                     3
#define RF_XINANJICHENG_RFIN_EN_SWPAD_OFFSET                    5

#define RF_XINANJICHENG_RFOUT_SW_CTRL_ADDR                      0x0b
#define RF_XINANJICHENG_RFOUT_EN_SWPAD_OFFSET                   14
#define RF_XINANJICHENG_RFOUT_EN_SWLPF_OFFSET                   13
#define RF_XINANJICHENG_RFOUT_EN_SWAD_OFFSET                    12

#define RF_XINANJICHENG_RFCLK_CTRL_ADDR                         0x00
#define RF_XINANJICHENG_RFCLK_CLK_SEL_OFFSET                    11
#define RF_XINANJICHENG_RFCLK_CLK_DIV_EN_OFFSET                 10


#define CHIP_THURDZ_TX_TUNNEL_NUM           2

#define ZONE0_WE_MASK                       0x00000000
#define ZONE1_WE_MASK                       0x01000000
#define ZONE_APB_WE_MASK                    0x02000000

#define CHIP_THURDZ_TX_TRIGGER_ADDR         ( URDZ_PROTOCOL_BASE_ADDR | 0x80 | ZONE_APB_WE_MASK )// 0x02FD0080  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 )
#define CHIP_THURDZ_TX_TRIGGER_OFFSET       1
#define CHIP_THURDZ_TX_TRIGGER_LENGTH       1

#define CHIP_THURDZ_RX_TRIGGER_ADDR         ( URDZ_PROTOCOL_BASE_ADDR | 0x80 | ZONE_APB_WE_MASK ) //0x02FD0080  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 )
#define CHIP_THURDZ_RX_TRIGGER_OFFSET       0
#define CHIP_THURDZ_RX_TRIGGER_LENGTH       1

#define CHIP_THURDZ_TX_LENS_ADDR            ( URDZ_PROTOCOL_BASE_ADDR | 0x84 | ZONE_APB_WE_MASK ) //0x02FD0084  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x04 )
#define CHIP_THURDZ_TX_PRMB_LEN_OFFSET      20
#define CHIP_THURDZ_TX_SFD_LEN_OFFSET       12
#define CHIP_THURDZ_TX_PSDU_LEN_OFFSET      0

#define CHIP_THURDZ_TX_SFD_ADDR				( URDZ_PROTOCOL_BASE_ADDR | 0x88 | ZONE_APB_WE_MASK ) // 0x02FD0088  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x08 )

#define CHIP_THURDZ_TX_PHR_ADDR             ( URDZ_PROTOCOL_BASE_ADDR | 0x8C | ZONE_APB_WE_MASK ) //0x02FD008C  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x0c, three words )

#define CHIP_THURDZ_TX_TUNNEL_ENABLE_ADDR   ( URDZ_PROTOCOL_BASE_ADDR | 0x98 | ZONE_APB_WE_MASK ) // 0x02FD0098  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x18 )
#define CHIP_THURDZ_TX_CONFIG_START_ADDR    ( URDZ_PROTOCOL_BASE_ADDR | 0x98 | ZONE_APB_WE_MASK )
#define CHIP_THURDZ_RX_CONFIG_START_ADDR    ( URDZ_PROTOCOL_BASE_ADDR | 0xC0 | ZONE_APB_WE_MASK )

#define CHIP_THURDZ_RX_TIMEOUT_SYMBOL_ADDR  ( URDZ_PROTOCOL_BASE_ADDR | 0xC8 | ZONE_APB_WE_MASK ) // 0x02FD00C8

#define CHIP_THURDZ_LB_CHANNEL_ADDR         ( URDZ_PROTOCOL_BASE_ADDR | 0x9C | ZONE_APB_WE_MASK ) // 0x02FD009C  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x1c, two words, one for carrier freq offset, another for sample freq offset )

#define CHIP_THURDZ_DLY_TX_STAMP_ADDR       ( URDZ_PROTOCOL_BASE_ADDR | 0xA0 | ZONE_APB_WE_MASK ) // 0x02FD00A0  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M2 + 0x20, two words )
#define CHIP_THURDZ_DLY_RX_STAMP_ADDR       ( URDZ_PROTOCOL_BASE_ADDR | 0xCC | ZONE_APB_WE_MASK ) // 0x02FD00CC

#define CHIP_THURDZ_TX_DATA_ADDR            ( URDZ_PROTO_GEN_BASE_ADDR | 0x8000 | ZONE1_WE_MASK ) // 0x01208000  // ( URDZ_PROTO_GEN_BASE_ADDR + 0x8000 )
#define CHIP_THURDZ_TX_DATA_STRIP           0x00002000

#define CHIP_THURDZ_TX_ISR_FLAG_ADDR        ( URDZ_PROTOCOL_BASE_ADDR | 0x40 | ZONE_APB_WE_MASK ) // 0x02FD0040  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M1 + 0x00 )
#define CHIP_THURDZ_TX_STAMP_ADDR           ( URDZ_PROTOCOL_BASE_ADDR | 0x44 | ZONE_APB_WE_MASK ) // 0x02FD0044  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M1 + 0x04 )

#define CHIP_THURDZ_UCORE_CTRL_ADDR         ZONE_APB_WE_MASK  // Reg controlled by apb directly.
#define CHIP_THURDZ_TPRAM_RST_OFFSET        16
#define CHIP_THURDZ_TX_MEMBUS_RST_OFFSET    11
#define CHIP_THURDZ_TX_UCORE_RST_OFFSET     10
#define CHIP_THURDZ_TX_DM_TOKEN_OFFSET      9
#define CHIP_THURDZ_TX_IM_TOKEN_OFFSET      8
#define CHIP_THURDZ_RX_MEMBUS_RST_OFFSET    3
#define CHIP_THURDZ_RX_UCORE_RST_OFFSET     2
#define CHIP_THURDZ_RX_DM_TOKEN_OFFSET      1
#define CHIP_THURDZ_RX_IM_TOKEN_OFFSET      0
#define CHIP_THURDZ_TX_START_OFFSET         8   // To release token and release reset.
#define CHIP_THURDZ_TX_START_LENGTH         4
#define CHIP_THURDZ_TX_START_MASK           0xf
#define CHIP_THURDZ_RX_START_OFFSET         0
#define CHIP_THURDZ_RX_START_LENGTH         4
#define CHIP_THURDZ_RX_START_MASK           0xf

#define CHIP_THURDZ_SELFLOOP_EN_ADDR        ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) // 0x00014004
#define CHIP_THURDZ_SELFLOOP_EN_OFFSET      0
#define CHIP_THURDZ_SELFLOOP_EN_LENGTH      1

#define CHIP_THURDZ_4TO1_EN_ADDR            ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) // 0x00014004
#define CHIP_THURDZ_4TO1_EN_OFFSET          1
#define CHIP_THURDZ_4TO1_EN_LENGTH          1

#define CHIP_THURDZ_RX_MODULE_RST_ADDR      ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) // 0x00014004  // Console's gpio. ( URDZ_GPIO_ADDR )
#define CHIP_THURDZ_RX_MODULE_RST_OFFSET    2
#define CHIP_THURDZ_RX_MODULE_RST_LENGTH    29
#define CHIP_THURDZ_RX_MODULE_RST_MASK      ( 0xffffffff )    // 27 is for corr.

#define CHIP_THURDZ_TX_MODULE_RST_ADDR      ( URDZ_CONSOLE_BASE_ADDR | 0x04 | ZONE1_WE_MASK ) // 0x01014004  // ( URDZ_GPIO_ADDR | 0x01000000 )
#define CHIP_THURDZ_TX_MODULE_RST_OFFSET    0
#define CHIP_THURDZ_TX_MODULE_RST_LENGTH    32
#define CHIP_THURDZ_TX_MODULE_RST_MASK      ( 0xffffffff )    // 28 is for spreader.

#define CHIP_THURDZ_RX_DATA_ADDR            ( URDZ_RS_DECODER_BASE_ADDR | 0x2000 | ZONE0_WE_MASK ) // 0x00132000  // ( URDZ_RS_DECODER_BASE_ADDR | 0x2000 )
#define CHIP_THURDZ_RX_PHR_ERR_FLAG_ADDR    ( URDZ_RS_DECODER_BASE_ADDR | 0x0010 | ZONE0_WE_MASK )
#define CHIP_THURDZ_RX_RS_ERR_FLAG_ADDR     ( URDZ_RS_DECODER_BASE_ADDR | 0x0014 | ZONE0_WE_MASK )
#define CHIP_THURDZ_RX_PHR_DATA_ADDR        ( URDZ_RS_DECODER_BASE_ADDR | 0x000C | ZONE0_WE_MASK )

#define CHIP_THURDZ_RX_ISR_FLAG_ADDR        ( URDZ_PROTOCOL_BASE_ADDR | 0x00 | ZONE_APB_WE_MASK ) // 0x02FD0000  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M0 + 0x00 )
#define CHIP_THURDZ_RX_STAMP_HI_ADDR        ( URDZ_PROTOCOL_BASE_ADDR | 0x04 | ZONE_APB_WE_MASK ) // 0x02FD0004  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M0 + 0x04 )
#define CHIP_THURDZ_RX_STAMP_LO_ADDR        ( URDZ_PROTOCOL_BASE_ADDR | 0x08 | ZONE_APB_WE_MASK ) // 0x02FD0008  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M0 + 0x08 )
#define CHIP_THURDZ_RX_PHR_VALUE_TMP_ADDR   ( URDZ_PROTOCOL_BASE_ADDR | 0x0C | ZONE_APB_WE_MASK ) // 0x02FD000C  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M0 + 0x0C )
#define CHIP_THURDZ_RX_LDE_RES_ADDR         ( URDZ_PROTOCOL_BASE_ADDR | 0x1C | ZONE_APB_WE_MASK ) // 0x02FD001C
#define CHIP_THURDZ_CIR_NOISE_MEAN_ADDR     ( URDZ_PROTOCOL_BASE_ADDR | 0x0C | ZONE_APB_WE_MASK ) // 0x02FD000C
#define CHIP_THURDZ_CIR_PEAK_ADDR           ( URDZ_PROTOCOL_BASE_ADDR | 0x10 | ZONE_APB_WE_MASK ) // 0x02FD0010
#define CHIP_THURDZ_CIR_ACCUM_NUM_ADDR      ( URDZ_PROTOCOL_BASE_ADDR | 0x18 | ZONE_APB_WE_MASK ) // 0x02FD0018

#define CHIP_THURDZ_LB_CHANNEL_CTRL_ADDR    ( URDZ_CHANNEL_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00240000  // ( URDZ_CHANNEL_BASE_ADDR )
#define CHIP_THURDZ_LB_CHANNEL_CF_WORD_ADDR ( URDZ_CHANNEL_BASE_ADDR | 0x04 | ZONE0_WE_MASK ) // 0x00240004  // ( URDZ_CHANNEL_BASE_ADDR + ( URDZ_CHANNEL_REG_DOPPLER_FREQ_WORD << 2 ) )
#define CHIP_THURDZ_LB_CHANNEL_SF_WORD_ADDR ( URDZ_CHANNEL_BASE_ADDR | 0x08 | ZONE0_WE_MASK ) // 0x00240008  // ( URDZ_CHANNEL_BASE_ADDR + ( URDZ_CHANNEL_REG_SAMPLE_OFFSET_FREQ_WORD << 2 ) )
#define CHIP_THURDZ_LB_CHANNEL_SNR_SEL_ADDR ( URDZ_CHANNEL_BASE_ADDR | 0x0C | ZONE0_WE_MASK ) // 0x0024000C  // ( URDZ_CHANNEL_BASE_ADDR + ( URDZ_CHANNEL_REG_KSNR << 2 ) )
#define CHIP_THURDZ_LB_CHANNEL_GN_WORD_ADDR ( URDZ_CHANNEL_BASE_ADDR | 0x10 | ZONE0_WE_MASK ) // 0x00240010  // ( URDZ_CHANNEL_BASE_ADDR + ( URDZ_CHANNEL_REG_GAIN << 2 ) )

#define CHIP_THURDZ_TICK_PER_SEC            63897600000LL
//  #define CHIP_THURDZ_TICK_PER_SEC            127795200000LL
//  #define CHIP_THURDZ_TICK_PER_US             127795LL
#define CHIP_THURDZ_TICK_PER_US             63898LL
#define CHIP_THURDZ_FULL_STAMP_BITS         41
#define CHIP_THURDZ_SHORT_STAMP_BITS        32

#define CHIP_THURDZ_RF_MODULE_CTRL_ADDR     ( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_AGC_CTRL_ADDR			( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_AGC_CTRL_OFFSET			0
#define CHIP_THURDZ_AGC_CTRL_LENGTH			6

#define CHIP_THURDZ_RFSW_ADDR				( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_RFSW_OFFSET				6
#define CHIP_THURDZ_RFSW_LENGTH				5

#define CHIP_THURDZ_ANT_CTRL_ADDR			( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_ANT_CTRL_OFFSET			20
#define CHIP_THURDZ_ANT_CTRL_LENGTH			4

#define CHIP_THURDZ_BBEX_CTRL_ADDR			( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_BBEX_CTRL_OFFSET        16
#define CHIP_THURDZ_BBEX_CTRL_LENGTH        8

#define CHIP_THURDZ_MTS_TRUNC_ADDR			( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_MTS_TRUNC_OFFSET		16
#define CHIP_THURDZ_MTS_TRUNC_LENGTH		4

// for THMTS-Chip, 2025/04/25, cxw
#define CHIP_THURDZ_BBEX_CTRL_ADDR			( URDZ_MEMBUS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00fe0000
#define CHIP_THURDZ_BBEX_CTRL_OFFSET        16
#define CHIP_THURDZ_BBEX_CTRL_LENGTH        8

#define CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_ADDR                  ( URDZ_TIMER_BASE_ADDR | 0x00 | ZONE1_WE_MASK ) // 0x01fb0000  // ( URDZ_TIMER_BASE_ADDR )
#define CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_OFFSET                1
#define CHIP_THURDZ_TIMER_INTR_OUTPUT_ENA_LENGTH                1

#define CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_ADDR                   ( URDZ_TIMER_BASE_ADDR | 0x00 | ZONE1_WE_MASK ) // 0x01fb0000  // ( URDZ_TIMER_BASE_ADDR )
#define CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_OFFSET                 2
#define CHIP_THURDZ_TIMER_PPS_OUTPUT_ENA_LENGTH                 1

#define CHIP_THURDZ_TIMER_PPS_WIDTH_ADDR                        ( URDZ_TIMER_BASE_ADDR | 0x08 | ZONE1_WE_MASK ) // 0x01fb0008

#define CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR                   ( URDZ_TIMER_BASE_ADDR | 0x04 | ZONE1_WE_MASK ) // 0x01fb0004

#define CHIP_THURDZ_TIMER_PWM_ENA_ADDR                          ( URDZ_TIMER_BASE_ADDR | 0x00 | ZONE1_WE_MASK )
#define CHIP_THURDZ_TIMER_PWM_ENA_OFFSET                        3
#define CHIP_THURDZ_TIMER_PWM_ENA_LENGTH                        1

#define CHIP_THURDZ_TIMER_PWM_OUT_ENA_ADDR                      ( URDZ_TIMER_BASE_ADDR | 0x00 | ZONE1_WE_MASK )
#define CHIP_THURDZ_TIMER_PWM_OUT_ENA_OFFSET                    4
#define CHIP_THURDZ_TIMER_PWM_OUT_ENA_LENGTH                    1

#define CHIP_THURDZ_TIMER_PWM_PERIOD_ADDR                       ( URDZ_TIMER_BASE_ADDR | 0x0C | ZONE1_WE_MASK )

#define CHIP_THURDZ_TIMER_PWM_ON_PERIOD_ADDR                    ( URDZ_TIMER_BASE_ADDR | 0x10 | ZONE1_WE_MASK )

#define CHIP_THURDZ_TIMER_PWM_START_TIME_ADDR                   ( URDZ_TIMER_BASE_ADDR | 0x14 | ZONE1_WE_MASK )


#define CHIP_THURDZ_SYSTIME_ADDR            ( URDZ_TIMER_BASE_ADDR | 0x20 | ZONE1_WE_MASK ) // 0x01fb0020  // ( URDZ_TIMER_BASE_ADDR + ( URDZ_TIMER_REG_CUR_TIMER << 2 ) )
#define CHIP_THURDZ_SYSTIME_OFFSET          0
#define CHIP_THURDZ_SYSTIME_LENGTH          32

// Set to zero when entering ucore1, set to corresponding stamp when return from ucore1.
#define CHIP_THURDZ_TMARK_REPORT_ADDR       ( URDZ_PROTOCOL_BASE_ADDR | 0x44 | ZONE_APB_WE_MASK ) // 0x02fd0044  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M1 + 0x04 )

// Set to zero when entering ucore1, set to 1 when reach delayed tx stamp, set to 0 when tx complete.
#define CHIP_THURDZ_TX_INTR_CAUSE_ADDR      ( URDZ_PROTOCOL_BASE_ADDR | 0x4C | ZONE_APB_WE_MASK ) // 0x02fd004c  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M1 + 0x0c )
#define CHIP_THURDZ_TX_INTR_CAUSE_OFFSET    0
#define CHIP_THURDZ_TX_INTR_CAUSE_LENGTH    1
#define CHIP_THURDZ_TX_COMPLETE             0
#define CHIP_THURDZ_TX_REACH_DELAYED_STAMP  1

#define CHIP_THURDZ_RX_STATUS_ADDR          ( URDZ_PROTOCOL_BASE_ADDR | 0x3C | ZONE_APB_WE_MASK ) // 0x02fd003c

#define CHIP_THURDZ_TX_ABOUT_TIME_ADDR      ( URDZ_PROTOCOL_BASE_ADDR | 0x4C | ZONE_APB_WE_MASK ) // 0x02fd004c  // ( URDZ_PROTOCOL_BASE_ADDR + URDZ_PROTOCOL_OFFSET_M1 + 0x0c )

#define CHIP_THURDZ_INRUN_TX_DATA_ADDR      ( URDZ_PROTOCOL_BASE_ADDR | 0x88 | ZONE_APB_WE_MASK ) // 0x02fd0088
#define CHIP_THURDZ_TX_TPRAM_FREE_ADDR      ( URDZ_PROTOCOL_BASE_ADDR | 0x50 | ZONE_APB_WE_MASK ) // 0x02fd0050

#define CHIP_THURDZ_TX_DM_ADDR              ( URDZ_DM_32BITS_BASE_ADDR | 0x00 | ZONE1_WE_MASK ) // 0x01010000  // URDZ_DM_32BITS_BASE_ADDR
#define CHIP_THURDZ_TX_DM_32B_DEPTH         16384
#define CHIP_THURDZ_RX_DM_ADDR              ( URDZ_DM_32BITS_BASE_ADDR | 0x00 | ZONE0_WE_MASK ) // 0x00010000  // URDZ_DM_32BITS_BASE_ADDR
#define CHIP_THURDZ_RX_DM_32B_DEPTH         16384

#define CHIP_THURDZ_CIR_BASE_ADDR           ( URDZ_ACCUM_BASE_ADDR | 0x8000 | ZONE0_WE_MASK )

#define CHIP_THURDZ_ACCUM_RES_STRIP             0x2000

#define CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_ADDR                          0x00d00000
#define CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_OFFSET                        5
#define CHIP_THURDZ_ADDA_INTERFACE_FORMAT_SEL_LENGTH                        1

#define CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_ADDR                            0x00d00000
#define CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_OFFSET                          7
#define CHIP_THURDZ_ADDA_INTERFACE_CLOCK_EN_LENGTH                          1

#define FULL_STAMP_WIDTH                                    41
#define SHORT_STAMP_WIDTH                                   32
#define FULL_STAMP_MAX                                      (( int64_t )( ( ( int64_t )0x01 ) << FULL_STAMP_WIDTH ))

#define CHIP_THURDZ_ASSERT( x ) do{ if( !(x) ) { error( __LINE__ ); } } while(0)

#define TICK_PER_MS                                     63897600LL
#define TICK_PER_10US                                   638976LL

#define THMTS_BB_BASE   APB_SLV0_ASYNC_FIFO_MEM_BASE

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define THMTS_RF_CHANNEL_3P5G 	0
#define THMTS_RF_CHANNEL_4G   	1
#define THMTS_RF_CHANNEL_7G   	2
#define THMTS_RF_CHANNEL_8G   	3

#define THMTS_RF_BANDWIDTH_500M 0
#define THMTS_RF_BANDWIDTH_1G   1

#define THMTS_PREAMBLE_DS       0
#define THMTS_PREAMBLE_HRP31    1
#define	THMTS_PREAMBLE_HRP127   2
#define	THMTS_PREMABLE_RADAR    3

#define THMTS_BITRATE_15P625K 	0
#define THMTS_BITRATE_31P25K	1
#define THMTS_BITRATE_62P5K		2
#define THMTS_BITRATE_125K		3
#define THMTS_BITRATE_250K		4
#define THMTS_BITRATE_500K		5
// DS under 500M and 1G bandwidth.
#define THMTS_BITRATE_1M		6
#define THMTS_BITRATE_2M		7
#define THMTS_BITRATE_4M		8
#define THMTS_BITRATE_8M		9
// 802.15.4 HRP standard.
#define THMTS_BITRATE_850K		10
#define THMTS_BITRATE_6M8		11

#define	THMTS_PHRMODE_STD		0
#define	THMTS_PHRMODE_EXT		1

#define	THMTS_STSMODE_OFF 				0
#define	THMTS_STSMODE_DUAL_AFTER_SHR	1
#define	THMTS_STSMODE_DUAL_AFTER_PSDU	2
#define	THMTS_STSMODE_DUAL_NO_PSDU		3
#define	THMTS_STSMODE_QUAD_NO_PSDU		4

#define	THMTS_DIVRATIO_1		0
#define	THMTS_DIVRATIO_2 		1
#define	THMTS_DIVRATIO_4 		2
#define	THMTS_DIVRATIO_8 		3
#define	THMTS_DIVRATIO_16 		4


extern uint32_t channel0_CIR[1022];
extern uint32_t channel1_CIR[1022];
extern uint32_t channel2_CIR[1022];
extern uint32_t channel3_CIR[1022];
extern uint16_t firstPeakIdx;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	uint8_t rf_channel;
	uint8_t rf_bandwidth;
	uint8_t preamble_type;
	uint8_t bit_rate;
	uint8_t phr_mode;
	uint8_t sts_mode;
    uint8_t div_ratio_sel;

	int8_t freq_grid_idx;
	uint32_t preamble_length;
	uint32_t sfd_length;

    // Extend for DS standard, by wm, 20231128.
    uint8_t preamble_code_sel;
    uint8_t sfd_sel;
    uint8_t phr_code_sel;
    uint8_t psdu_code_sel;

    uint8_t rf_chan_num;
    uint8_t accum_1to4_enable;
    uint8_t aoa_enable;

    uint32_t psdu_length;

    //衍生字段
    uint32_t length_cfg_word;
    uint32_t phr_info_bit;
    uint32_t phr_buff_word1;
    uint32_t phr_buff_word2;
    uint32_t tx_config_word;
    uint32_t rx_config_word;
    uint32_t tx_delta_tick;

} THMTS_PhyParamConfig_t;


#pragma pack(4)
typedef struct
{
	int16_t CIR_Q;
	int16_t CIR_I;
} CIR_struct_t;

//extern CIR_struct_t channel1_CIR[1022];
//extern CIR_struct_t channel2_CIR[1022];
//extern CIR_struct_t channel3_CIR[1022];
//extern CIR_struct_t channel4_CIR[1022];

#pragma pack()

//#pragma pack(1)
//typedef struct
//{
//    uint16_t dev_id;
//    uint16_t slot_id;
//    int64_t rx_stamp;
//    int64_t tx_stamp;
//    uint8_t data[16];
//    uint32_t RSV;
//} thmts_ranging_frame_t;
//#pragma pack()
#pragma pack(1)
typedef struct
{
    uint8_t  slot_id;
    uint8_t  subfrm_id;
    uint8_t  frm_id;
    uint8_t  group_id;
    uint64_t tx_stamp;
} thmts_uwbpacket_head_t;
#pragma pack()

#define USER_DATA_LENGTH 256

#pragma pack(1)
typedef struct
{
    thmts_uwbpacket_head_t head;
    uint64_t master_pps_time;
    uint16_t rx_stamp_bitmap;
    uint64_t rx_stamp;
    uint16_t  user_data[USER_DATA_LENGTH];
    uint32_t RSV;
} thmts_ranging_packet_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    thmts_uwbpacket_head_t head;
    uint8_t  data[150];
    uint32_t RSV;
} thmts_comm_packet_t;
#pragma pack()


#pragma pack(1)
typedef struct
{
    uint32_t rx_status;                // 接收到的信息低8bit代表接收状态 1: 接收超时  4: 接收成功  15: PHR出错
                                       // 次低字节代表AGC增益字
                                       //高16bit代表CIR累积次数

    uint32_t CRC_result;               //接收完成之后读取的CRC结果,高16bit为BB计算出来的CRC结果，低16bit为发射端传递的CRC结果

    uint32_t rx_info[10];              // 接收寄存器信息，从BB中读取
    uint64_t rmark;                    // 自己收到信息的接收时刻

	uint32_t BBCTRL;                   // 接收控制字
	uint32_t PHR_info;                 // 接收PHR
	uint32_t CRC_OK;
	int32_t node_adjust;

    uint16_t rx_length;                // 接收到的数据包rx_info长度
    uint16_t RSV;                      // 保留字段

    int tof_int;                       // 测距结果，等于真实距离(m) * 100，转化为int

    thmts_ranging_packet_t rx_frame_data;     // 接收到的帧信息


} THMTS_RX_CONTENT_t;
#pragma pack()








extern uint32_t tx_flen;
extern thmts_ranging_packet_t thmts_tx_frame;
extern thmts_ranging_packet_t thmts_rx_frame;


//接收内容的全局结构体
extern THMTS_RX_CONTENT_t thmts_rx_content;

/* ===== THMTS exported variables ===== */


/* ===== THMTS exported functions ===== */
//void write_thmts_rf_reg(uint32_t addr, uint32_t data);
//uint32_t read_thmts_rf_reg(uint32_t addr);
//void config_thmts_rf_clkmode( uint8_t div2_en, uint8_t hi_freq_clk_en);


void write_thmts_bb_reg(uint32_t addr, uint32_t data);
void write_thmts_bb_reg_with_offset(uint32_t addr, uint8_t start_offset, uint8_t length, uint32_t data);

uint32_t read_thmts_bb_reg(uint32_t addr);
uint32_t read_thmts_bb_reg_with_offset( uint32_t addr, uint8_t offset, uint8_t length );


int64_t timestamp_add( int64_t a, int64_t b );
int64_t timestamp_minus( int64_t a, int64_t b );
int64_t timestamp_substract( int64_t a, int64_t b );
int64_t us2timestamp( uint32_t us );
int64_t us2tick( uint32_t us );

uint16_t crc16_ccitt(uint8_t *di, uint32_t len);
uint16_t crc16_ccitt_enhanced(uint8_t *data, uint32_t len);

void reset_thmts_bb();
void reset_thmts_bb_tx_module();
void reset_thmts_bb_tx_ucore();
void reset_thmts_bb_rx_module();
void reset_thmts_bb_rx_ucore();

void clear_thmts_bb_tpram(void);


// 接收模块关机，power down
void thmts_bb_rx_pd();
// 接收模块开机，power on
void thmts_bb_rx_po();
// 发射模块关机，power down
void thmts_bb_tx_pd();
// 发射模块开机，power on
void thmts_bb_tx_po();


void init_thmts_bb();
void enable_thmts_bb_pwm(uint32_t start_interval_us, uint32_t pwm_period, uint32_t pwm_on_period);
void enable_thmts_bb_loopback( uint8_t enable_flag );

uint32_t get_thmts_bb_systime(int64_t* t);

void set_thmts_bb_delaytxtime(THMTS_PhyParamConfig_t *p_phycfg, uint32_t tick );
void start_thmts_bb_tx(THMTS_PhyParamConfig_t *p_phycfg, thmts_ranging_packet_t *p_txdata, uint32_t txdata_length);
void clean_thmts_bb_tx(void);

void write_thmts_bb_txbuff(uint8_t *p_data, uint32_t offset, uint32_t length);
void read_thmts_bb_rxbuff(uint8_t *p_data, uint32_t offset, uint32_t length);

void set_thmts_bb_delayrxtime(uint32_t tick );
void set_thmts_bb_rxtimeout(uint32_t preamblesymbol_num);
void start_thmts_bb_rx(THMTS_PhyParamConfig_t *p_phycfg, uint32_t rx_timeout);
void clean_thmts_bb_rx(void);

uint32_t get_thmts_bb_rxstatus();
void get_thmts_bb_rxinfo(uint32_t* p_info);
void get_thmts_bb_rmark(THMTS_PhyParamConfig_t *p_phycfg, int64_t* p);
uint32_t get_thmts_bb_rxlength(THMTS_PhyParamConfig_t *p_phycfg);
//void read_thmts_bb_rxbuff(thmts_ranging_frame_t *p_rxdata, uint32_t len);
void read_thmts_bb_cir(uint32_t rx_id, uint32_t* buf, uint32_t offset, uint32_t len);

void thmts_get_rx_content(uint32_t cmd);

void config_thmts_bb_exctrl(uint8_t cmd);
void config_thmts_bb_tx_pa_on();
void config_thmts_bb_tx_pa_off();
void config_thmts_bb_rx_lna1_on();
void config_thmts_bb_rx_lna123_on();
void config_thmts_bb_rx_lna_off();

void config_thmts_bb_txsw_on();
void config_thmts_bb_txsw_off();
void config_thmts_bb_rx0sw_on();
void config_thmts_bb_rx0sw_off();
void config_thmts_bb_rx012sw_on();
void config_thmts_bb_rx012sw_off();
void config_thmts_bb_rx0123sw_on();
void config_thmts_bb_rx0123sw_off();
void config_thmts_bb_rx_sw_lna_on(uint8_t rf_chan_num);
void config_thmts_bb_rx_sw_lna_off(uint8_t rf_chan_num);

#define DAGC_ENABLE         (write_thmts_bb_reg_with_offset(CHIP_THURDZ_DLY_RX_STAMP_ADDR + 4 , 4 , 4 , 1))
#define DAGC_DISABLE        (write_thmts_bb_reg_with_offset(CHIP_THURDZ_DLY_RX_STAMP_ADDR + 4 , 4 , 4 , 0))
#define SET_FIX_GAIN(gain)  (write_thmts_bb_reg_with_offset(CHIP_THURDZ_AGC_CTRL_ADDR , CHIP_THURDZ_AGC_CTRL_OFFSET , CHIP_THURDZ_AGC_CTRL_LENGTH , gain))

#define PA_ENABLE           (write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xF6))
#define PA_LNA_DISABLE          (write_thmts_bb_reg_with_offset(CHIP_THURDZ_BBEX_CTRL_ADDR, CHIP_THURDZ_BBEX_CTRL_OFFSET, CHIP_THURDZ_BBEX_CTRL_LENGTH, 0xE5))


#define BB_TX_MODULE_POWER_DOWN         (write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0xffffffff ))
#define BB_TX_MODULE_POWER_ON           (write_thmts_bb_reg_with_offset( CHIP_THURDZ_TX_MODULE_RST_ADDR, CHIP_THURDZ_TX_MODULE_RST_OFFSET, CHIP_THURDZ_TX_MODULE_RST_LENGTH, 0 ))
#define BB_RX_MODULE_POWER_DOWN         (write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0x7fffffff ))
#define BB_RX_MODULE_POWER_ON           (write_thmts_bb_reg_with_offset( CHIP_THURDZ_RX_MODULE_RST_ADDR, CHIP_THURDZ_RX_MODULE_RST_OFFSET, CHIP_THURDZ_RX_MODULE_RST_LENGTH, 0 ))

#define RF_TX_POWER_ON      (write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET, 1, 0))
#define RF_TX_POWER_DOWN    (write_thmts_bb_reg_with_offset(CHIP_THURDZ_RFSW_ADDR, CHIP_THURDZ_RFSW_OFFSET, 1, 1))

#define ENABLE_BB_LOOPBACK   (write_thmts_bb_reg_with_offset( CHIP_THURDZ_SELFLOOP_EN_ADDR, CHIP_THURDZ_SELFLOOP_EN_OFFSET, CHIP_THURDZ_SELFLOOP_EN_LENGTH, 1 ))
#define DISABLE_BB_LOOPBACK  (write_thmts_bb_reg_with_offset( CHIP_THURDZ_SELFLOOP_EN_ADDR, CHIP_THURDZ_SELFLOOP_EN_OFFSET, CHIP_THURDZ_SELFLOOP_EN_LENGTH, 0 ))

extern uint32_t rx_data_time;
//for AGC debug output
extern uint32_t AGC_idx;
extern uint32_t AGC_result[3];

#endif // THMTSV1_CHIP_H
