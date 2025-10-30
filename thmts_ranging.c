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

//void start_dstwr_compute()
//{
//	for(int i = 0; i < 2; i++){
//		if(i==node.dev_id)
//		{
//			continue;
//		}
//		altds_twr_t twr;
//		twr.poll_tx_time = poll1_tx_stamp_t[i];
//		twr.poll_rx_time = poll1_rx_stamp_t[i];
//		twr.resp_tx_time = resp_tx_stamp_t;
//		twr.resp_rx_time = resp_rx_stamp_t[i];
//		twr.poll2_tx_time = poll2_tx_stamp_t[i];
//		twr.poll2_rx_time = poll2_rx_stamp_t[i];
//
//		// uint32_t poll_tx_hi = (uint32_t)(twr.poll_tx_time >> 32);
//		// uint32_t poll_tx_lo = (uint32_t)(twr.poll_tx_time);
//		// uint32_t poll_rx_hi = (uint32_t)(twr.poll_rx_time >> 32);
//		// uint32_t poll_rx_lo = (uint32_t)(twr.poll_rx_time);
//		// uint32_t resp_tx_hi = (uint32_t)(twr.resp_tx_time >> 32);
//		// uint32_t resp_tx_lo = (uint32_t)(twr.resp_tx_time);
//		// uint32_t resp_rx_hi = (uint32_t)(twr.resp_rx_time >> 32);
//		// uint32_t resp_rx_lo = (uint32_t)(twr.resp_rx_time);
//		// uint32_t poll2_tx_hi = (uint32_t)(twr.poll2_tx_time >> 32);
//		// uint32_t poll2_tx_lo = (uint32_t)(twr.poll2_tx_time);
//		// uint32_t poll2_rx_hi = (uint32_t)(twr.poll2_rx_time >> 32);
//		// uint32_t poll2_rx_lo = (uint32_t)(twr.poll2_rx_time);
//
////				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff],
////					"poll_tx = %3u, %10u | poll_rx = %3u, %10u | resp_tx = %3u, %10u | resp_rx = %3u, %10u | poll2_tx = %3u, %10u | poll2_rx = %3u, %10u |\r\n",
////					poll_tx_hi, poll_tx_lo,
////					poll_rx_hi, poll_rx_lo,
////					resp_tx_hi, resp_tx_lo,
////					resp_rx_hi, resp_rx_lo,
////					poll2_tx_hi, poll2_tx_lo,
////					poll2_rx_hi, poll2_rx_lo
////				);
//
//		if(altds_dstwr_check(&twr))
//		{
//			tof=altds_dstwr_compute(&twr);
//			tof_int=tof*100;
//		}
//		altds_dstwr_clear(&twr);
//	}
//	memset(poll1_tx_stamp_t, 0, sizeof(poll1_tx_stamp_t));
//	memset(poll1_rx_stamp_t, 0, sizeof(poll1_rx_stamp_t));
//	memset(resp_rx_stamp_t, 0, sizeof(resp_rx_stamp_t));
//	memset(poll2_tx_stamp_t, 0, sizeof(poll2_tx_stamp_t));
//	memset(poll2_rx_stamp_t, 0, sizeof(poll2_rx_stamp_t));
//	resp_tx_stamp_t = 0;
//}
