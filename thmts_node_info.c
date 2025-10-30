#include "thmts_node_info.h"
#include "thmts_config.h"

THMTS_NodeInfo_t node;

void init_node()
{
	node.role = node_config.role;
	node.dev_id= node_config.dev_id;
	node.group_id= node_config.group_id;
	node.state = NODE_STATE_IDLE;

    node.curr_slot_idx=0;
    node.curr_subfrm_idx=0;
    node.curr_frm_idx=0;
    node.curr_user_data_frm_idx=0;

	node.uwb_tx_busy=0;
	node.uwb_rx_busy=0;
	node.uwb_rx_OK=0;
	node.comm_frm_tx_flag =0;
	node.confirm_slot_cnt=0;    // 进入Confirm状态之后过去的时隙数
	node.running_center_slot_cnt =0 ;
	node.running_group_master_slot_cnt=0;
	node.confirm_center_rcv=0;

	for(int i = 0; i < USER_DATA_LENGTH; i++) {
		node.user_data[i] = USER_DATA_LENGTH*node.user_data_cnt + i;
	}
	memset(node.user_data_rx,0,sizeof(node.user_data_rx));
	node.user_data_rx_valid = 0;
	node.user_data_cnt = 0;
	node.user_data_frm_cnt = 0;
	node.user_data_invalid_frm_cnt = 0;

	node.ts_into_timer=-1;		// 进入timer中断是uwb-bb的time stamp
	node.ts_curr_rmark=-1;

	node.arr_adjust=0;
	node.tcnt_adjust=0;
	node.arr_adjust_flag=0;

	memset(node.trx_stamps_subfrm0_ping, 0, sizeof(node.trx_stamps_subfrm0_ping));
	memset(node.trx_stamps_subfrm1_ping, 0, sizeof(node.trx_stamps_subfrm1_ping));
	memset(node.trx_stamps_subfrm2_ping, 0, sizeof(node.trx_stamps_subfrm2_ping));
	
}

void start_thmts_tx(THMTS_NodeInfo_t* node_ptr, THMTS_PhyParamConfig_t* thmts_phycfg_ptr, thmts_ranging_packet_t* thmts_tx_frame_ptr)
{
	uint64_t txdly_ts;
	txdly_ts = timestamp_add( node_ptr->ts_into_timer, 55 * TICK_PER_10US );

	thmts_tx_frame_ptr->head.slot_id = node_ptr->dev_id;
	thmts_tx_frame_ptr->head.frm_id = node_ptr->curr_frm_idx;
	thmts_tx_frame_ptr->head.group_id = node_ptr->group_id;
	thmts_tx_frame_ptr->head.subfrm_id = node_ptr->curr_subfrm_idx;
	thmts_tx_frame_ptr->head.tx_stamp = (txdly_ts >> 9) << 9;

//	BB_TX_MODULE_POWER_DOWN;
//	BB_TX_MODULE_POWER_ON;

	set_thmts_bb_delaytxtime(thmts_phycfg_ptr, (txdly_ts>>9));

	//memcpy((uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_TX_DATA_ADDR), thmts_tx_frame_ptr , 12);

	memcpy((uint32_t*)(THMTS_BB_BASE + CHIP_THURDZ_TX_DATA_ADDR), thmts_tx_frame_ptr ,  sizeof(thmts_ranging_packet_t));

	start_thmts_bb_tx(thmts_phycfg_ptr, thmts_tx_frame_ptr, sizeof(thmts_ranging_packet_t));
	PA_ENABLE;
	RF_TX_POWER_ON;

	node_ptr->uwb_tx_busy = 1;
}

void start_thmts_rx(THMTS_NodeInfo_t* node_ptr, THMTS_PhyParamConfig_t* thmts_phycfg_ptr)
{
	if (node_ptr->uwb_rx_busy == 0) {
		//BB_RX_MODULE_POWER_DOWN;
		BB_RX_MODULE_POWER_ON;
		config_thmts_bb_rx_sw_lna_on(thmts_phycfg_ptr->rf_chan_num);
		start_thmts_bb_rx(thmts_phycfg_ptr, 1024);			// 超时时间设置为1ms

		node_ptr->uwb_rx_busy = 1;
	}
}
