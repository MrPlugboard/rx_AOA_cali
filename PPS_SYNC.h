#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "thmts_bb_config.h"
#include "thmts_ranging.h"
#include "stdlib.h"

#define PPS_DELAY_IN_US 500
#define PPS_CHECK_PERIOD_IN_MS 1000
#define PPS_TR_CHECK_CNT 5

#pragma pack(1)
typedef struct{
    uint64_t tx_time_set[PPS_TR_CHECK_CNT];   //for Slave calculate drift and offset
    uint64_t rx_time_set[PPS_TR_CHECK_CNT];   //for Slave calculate drift and offset
    uint64_t tx_time_record;
    uint64_t rx_time_record;
    uint8_t tr_cnt;
    double clk_drift_coff;
    double clk_offset_coff;
    uint64_t master_pps_time_last;
} pps_adjust_t;
#pragma pack()
typedef pps_adjust_t* pps_adjust_ptr;

extern pps_adjust_t pps_adjust;
extern uint64_t pps_start_time;
extern uint8_t First_start_pps;
extern double tof_compensate;
extern uint8_t pps_config_done;
extern uint64_t master_pps_time_next;
extern uint64_t delta_tx;
extern uint64_t delta_s;
extern uint32_t pps_timer_cnt;
extern uint8_t pps_update;


void pps_start(pps_adjust_ptr pps_adjust_p,uint16_t role,double tof);
void pps_clk_compute(pps_adjust_ptr pps_adjust_p);
