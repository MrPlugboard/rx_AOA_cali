/*
 * thmts_rf_config.h
 *
 *  Created on: 2025年4月21日
 *      Author: csj
 */

#ifndef THMTS_RF_CONFIG_H_
#define THMTS_RF_CONFIG_H_

#include "ns_sdk_hal.h"

typedef struct
{
    unsigned int address;
    unsigned int value;
    unsigned int mask;
    unsigned int delay;
} THMTS_RfRegConfig_t;


extern uint32_t config_thmts_rf_lopll();
extern uint32_t config_thmts_rf_clkpll();
extern uint32_t config_thmts_rf_dac();
extern void config_thmts_rf_da2lpf();
extern void config_thmts_rf_da2pad();
extern void config_thmts_rf_tx();
extern void config_thmts_rf_adc0();
extern void config_thmts_rf_lpf2ad();
extern void config_thmts_rf_lpf2pad();
extern void config_thmts_rf_lpf2pad_and_lpf2ad();
extern void config_thmts_rf_pad2ad();
extern void config_thmts_rf_rx0();
extern void config_thmts_rf_clkmode(uint8_t div2_en, uint8_t hi_freq_clk_en);
extern void config_thmts_rf_dagc_set_value(uint32_t rx_channel_sel, uint32_t dagc_value);
extern void thmts_sleep_us(uint32_t us);


extern void config_thmts_pll_init_mfc();   // 半自动锁相，PLL环路初始化
extern void config_thmts_rf_lopll_mfc(uint8_t NintSet);   // 半自动lo pll锁相
extern void config_thmts_rf_clkpll_mfc( uint8_t NintSet);  // 半自动clk pll锁相

extern void config_thmts_rf_pll_mfc();      // 半自动锁相

extern void config_thmts_rf_allinone(uint32_t rx_num);





#endif /* THMTS_RF_CONFIG_H_ */
