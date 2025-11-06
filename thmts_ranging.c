#include "thmts_ranging.h"

altds_twr_t altds_twr;
uint32_t tof_int;
uint64_t poll1_tx_stamp_t[16] = {0};
uint64_t poll1_rx_stamp_t[16] = {0};
uint64_t resp_rx_stamp_t[16] = {0};
uint64_t resp_tx_stamp_t = 0;
uint64_t poll2_tx_stamp_t[16] = {0};
uint64_t poll2_rx_stamp_t[16] = {0};
uint64_t thmts_tx_frame_rx_stamp[3];
double tof[9]={0};
uint32_t tof_int_cm=0;
double tof_double_cm;

uint32_t rx_cnt=0;
uint32_t crc_error_cnt=0;
uint32_t rx_timeout_cnt=0;
uint32_t rx_phr_error_cnt=0;
uint32_t rx_ok_cnt=0;
uint32_t crc_ok_cnt=0;
int valid_count=0;

uint8_t altds_dstwr_check(altds_twr_ptr altds_twr_p)
{
	uint8_t altds_completed = 0;
    	if(altds_twr_p->poll_rx_time != 0 &&
    			altds_twr_p->poll_tx_time != 0 &&
				altds_twr_p->poll2_rx_time != 0 &&
				altds_twr_p->poll2_tx_time != 0 &&
				altds_twr_p->resp_rx_time != 0 &&
				altds_twr_p->resp_tx_time != 0){
    		altds_completed = 1;
    }
    return altds_completed;
}

void altds_dstwr_clear(altds_twr_ptr altds_twr_p)
{
    if (altds_twr_p != NULL) {
        memset(altds_twr_p, 0, sizeof(altds_twr_t));
    }
}

double altds_dstwr_compute(altds_twr_ptr altds_twr_p)
{
	double tof = 0;
	int64_t round1, round2, reply1, reply2 = 0;
	round1 = timestamp_minus(altds_twr_p->resp_rx_time,altds_twr_p->poll_tx_time);
	round2 = timestamp_minus(altds_twr_p->poll2_rx_time,altds_twr_p->resp_tx_time);
	reply1 = timestamp_minus(altds_twr_p->resp_tx_time,altds_twr_p->poll_rx_time);
	reply2 = timestamp_minus(altds_twr_p->poll2_tx_time,altds_twr_p->resp_rx_time);
	tof = ( (double)(round1 * round2 - reply1 * reply2) ) / ( (double)(round1 + round2 + reply1 + reply2) ) * C_speed / 124.8 / 64 / 8 / 1000000;
	return tof;
}

