#include "PPS_SYNC.h"
#include "stdio.h"

pps_adjust_t pps_adjust;
uint64_t pps_time_last=0;
uint64_t pps_start_time=0;
uint8_t First_start_pps=1;
double tof_compensate;
uint8_t pps_config_done=0;
uint64_t master_pps_time_next;
uint64_t delta_tx;
uint64_t delta_s;
uint32_t pps_timer_cnt=1;
uint8_t pps_update = 0;


//void pps_start(pps_adjust_ptr pps_adjust_p, uint16_t role, double tof)
//{
//	if(role==MASTER)
//	{
//		if(First_start_pps==1)
//		{
//			get_thmts_bb_systime( &pps_start_time );
//			pps_start_time = timestamp_add(pps_start_time,PPS_DELAY_IN_US * CHIP_THURDZ_TICK_PER_SEC / 1e6);
//			pps_start_time = pps_start_time >> 9;
//			write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
//			First_start_pps=0;
//		}
//		else
//		{
//			pps_start_time = pps_start_time <<9;
//			pps_start_time = timestamp_add(pps_start_time,PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3);
//			pps_start_time = pps_start_time >> 9;
//			write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
//		}
//		pps_adjust_p->master_pps_time_last=pps_start_time;
//	}
//	else if(role==SLAVE)
//	{
////		pps_start_time=pps_adjust.master_pps_time_last;
////		pps_start_time=pps_start_time<<9;
////		if(pps_start_time < (FULL_STAMP_MAX >> 3))
////		{
////			pps_start_time += FULL_STAMP_MAX;
////		}
////		pps_start_time=pps_start_time*pps_adjust_p->clk_drift_coff;
////		pps_start_time = timestamp_add(pps_start_time , pps_adjust_p->clk_offset_coff);
////		tof_compensate=tof/C_speed*CHIP_THURDZ_TICK_PER_SEC;
////		pps_start_time = timestamp_substract(pps_start_time,tof_compensate); //slave behind
////		uint64_t local_time;
////		get_thmts_bb_systime( &local_time );
////		uint64_t PPS_PERIOD_compensate_int=PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3;
////		//uint64_t PPS_PERIOD_compensate_int_drift=PPS_PERIOD_compensate_int * pps_adjust_p->clk_drift_coff;
////		while(timestamp_minus(local_time,pps_start_time)>0) //过了
////		{
////			pps_start_time = timestamp_add(pps_start_time , PPS_PERIOD_compensate_int);
////		}
//		//pps_start_time = pps_start_time >> 9;
//		//write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
//		uint64_t master_pps_time_next = pps_adjust.master_pps_time_last <<9;
//		master_pps_time_next = timestamp_add(master_pps_time_next,PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3);//Clock of Master is accurate
//		uint64_t delta_tx = timestamp_substract(master_pps_time_next , pps_adjust.tx_time_record);
//		tof_compensate=tof/C_speed*CHIP_THURDZ_TICK_PER_SEC;
//		delta_tx = timestamp_substract(delta_tx,tof_compensate); //slave behind
//		uint64_t delta_s=delta_tx * pps_adjust.clk_drift_coff;
//		uint64_t pps_start_time = timestamp_add( delta_s,  pps_adjust.rx_time_record);
//		uint64_t local_time;
//		get_thmts_bb_systime( &local_time );
//		uint64_t PPS_PERIOD_compensate_int=PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3;
//		while(timestamp_minus(local_time,pps_start_time)>0) //过了
//		{
//			pps_start_time = timestamp_add(pps_start_time , PPS_PERIOD_compensate_int);
//		}
//		pps_start_time = pps_start_time >>9;
//		write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
//		pps_config_done=0;
//	}
//
//}

void pps_clk_compute(pps_adjust_ptr pps_adjust_p)
{
	uint64_t delta_rx_time = timestamp_substract(pps_adjust_p->rx_time_set[PPS_TR_CHECK_CNT-1] , pps_adjust_p->rx_time_set[0]);
	uint64_t delta_tx_time = timestamp_substract(pps_adjust_p->tx_time_set[PPS_TR_CHECK_CNT-1] , pps_adjust_p->tx_time_set[0]);
	double clk_frift_coff_new;
	if(delta_tx_time!=0)
	{
		clk_frift_coff_new=(double)delta_rx_time / (double)delta_tx_time;
		if(pps_adjust_p->clk_drift_coff==0)
		{
			pps_adjust_p->clk_drift_coff=clk_frift_coff_new;
		}
		else
		{
			pps_adjust_p->clk_drift_coff = (pps_adjust_p->clk_drift_coff+clk_frift_coff_new)/2;
		}
		//pps_adjust_p->clk_drift_coff=clk_frift_coff_new;

	}
//	pps_adjust_p->clk_offset_coff = pps_adjust_p->rx_time_set[PPS_TR_CHECK_CNT-1] - pps_adjust_p->clk_drift_coff * pps_adjust_p->tx_time_set[PPS_TR_CHECK_CNT-1];

    uint64_t rx_sum=0;uint64_t tx_sum=0;
    double tx_ave=0;double rx_ave=0;
    uint8_t circle_count_tx=0;
    for (int iter=0;iter<PPS_TR_CHECK_CNT-1;iter++)
    {
    	if(pps_adjust_p->tx_time_set[iter+1] < pps_adjust_p->tx_time_set[iter])
    	{
    		circle_count_tx=PPS_TR_CHECK_CNT-iter-1;
    	}
    	tx_sum=tx_sum + pps_adjust_p->tx_time_set[iter];
    }
    tx_sum= tx_sum + pps_adjust_p->tx_time_set[PPS_TR_CHECK_CNT-1] + circle_count_tx * FULL_STAMP_MAX;
    tx_ave = tx_sum / PPS_TR_CHECK_CNT;
    tx_ave = timestamp_add(tx_ave,0);

    uint8_t circle_count_rx=0;
    for (int iter=0;iter<PPS_TR_CHECK_CNT-1;iter++)
    {
    	if(pps_adjust_p->rx_time_set[iter+1] < pps_adjust_p->rx_time_set[iter])
    	{
    		circle_count_rx=PPS_TR_CHECK_CNT-iter-1;
    	}
    	rx_sum=rx_sum + pps_adjust_p->rx_time_set[iter];
    }
    rx_sum= rx_sum + pps_adjust_p->rx_time_set[PPS_TR_CHECK_CNT-1] + circle_count_rx * FULL_STAMP_MAX;
    rx_ave = rx_sum / PPS_TR_CHECK_CNT;
    rx_ave = timestamp_add(rx_ave,0);
    if(circle_count_tx==0 && circle_count_rx==0)
    {
    	pps_adjust_p->clk_offset_coff = rx_ave - pps_adjust_p->clk_drift_coff * tx_ave;
    }
}
