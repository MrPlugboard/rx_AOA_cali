//  节点信息结构体

#ifndef THMTS_NODE_INFO_H
#define THMTS_NODE_INFO_H

// incldue necessary headers
#include <stdint.h>
#include "flash.h"
#include "thmts_bb_config.h"

//subfrm_type
#define AOA_FRAME      0
#define STAMP_FRAME    1
#define DIST_FRAME     2
#define BEACON_FRAME   3
#define IDLE_FRAME     4
#define COMM_FRAME     5
#define COMM_FRAME2    6

typedef struct
{
	uint8_t total_slot_num;
	uint8_t slot_len_50us;
	uint8_t subfrm_type;
} subfrm_desc_t;

typedef struct
{
	uint8_t total_frm_num;
	uint8_t total_subfrm_num;
	subfrm_desc_t* p_subfrm_descs;
} supfrm_desc_t;

#define NODE_ROLE_ANCHOR_MASTER 0
#define NODE_ROLE_ANCHOR_SLAVE_NORMAL 1
#define NODE_ROLE_ANCHOR_SLAVE_BACKUP 2
#define NODE_ROLE_TAG 3
#define NODE_ROLE_SELFTESTER_IN 4
#define NODE_ROLE_SELFTESTER_OUT 5
#define NODE_ROLE_NONE 6

//node state type
#define NODE_STATE_IDLE 0
#define NODE_STATE_CAPTURE 1
#define NODE_STATE_CONFIRM 2
#define NODE_STATE_RUNNING 3
#define NODE_STATE_CLOSING 4
#define NODE_STATE_CLOSED 5
#define NODE_STATE_SELFLOOP 6


typedef struct
{
	uint8_t dev_id;
	uint8_t group_id;
	uint8_t role;					// 0 : master, 1: slave

	supfrm_desc_t* p_supfrm_desc;

	uint8_t state;					// 0 : idle, 1 : search, 2 : connected

    uint8_t curr_slot_idx;
    uint8_t curr_subfrm_idx;
    uint8_t curr_frm_idx;
    uint8_t curr_user_data_frm_idx;      //for tx

    uint8_t comm_frm_tx_flag;   // 是否在时间同步帧中进行发射，对于备用主节点而言可以用于标识是否激活
    uint16_t confirm_slot_cnt;
    uint16_t running_center_slot_cnt;
    uint16_t running_group_master_slot_cnt;
    uint8_t confirm_center_rcv; //标识在confirm状态中是否曾经收到过中心节点的消息
	uint16_t user_data[USER_DATA_LENGTH];

	uint16_t user_data_rx[USER_DATA_LENGTH];
	uint8_t user_data_rx_valid;
	uint32_t user_data_cnt;               //内部一个永远+1的值，主从节点可以不同步，自己维护+1就行
	uint32_t user_data_frm_cnt;           //累计100次，作为计算错帧率的上限
	uint32_t user_data_invalid_frm_cnt;   //在这累计100次里面，误帧的个数，当user_data_frm_cnt为100的时候，两者一起清零重新累计

	uint8_t uwb_tx_busy;
	uint8_t uwb_rx_busy;
	uint8_t uwb_rx_OK;

	uint64_t ts_into_timer;		// 进入timer中断是uwb-bb的time stamp
	uint64_t ts_curr_rmark;

	int32_t arr_adjust;
	int32_t tcnt_adjust;
	uint32_t arr_adjust_flag;
	//调整钟差标志    0:  初始值  1：收到数据，计算出来第一次时间同步结果   2：进入第一次调整状态
	//只对从节点有效  3:  第一次调整完成之后的正常工作状态                 4：时钟偏差超过10us（78个clk），需要调整ARR，调整完成回到状态3

	uint64_t trx_stamps_subfrm0_ping[2][2];
	uint64_t trx_stamps_subfrm1_ping[2][2];
	uint64_t trx_stamps_subfrm2_ping[2][2];

} THMTS_NodeInfo_t;

// 全局节点变量声明（extern）
extern THMTS_NodeInfo_t node;
void init_node();

//
//#define  NODE_ROLE_ANCHOR_MASTER				0
//#define  NODE_ROLE_ANCHOR_SLAVE_NORMAL  		1
//#define  NODE_ROLE_ANCHOR_SLAVE_BACKUP  		2
//
//#define  NODE_ROLE_TAG								3
//#define  NODE_ROLE_SELFTESTER_IN					4
//#define  NODE_ROLE_SELFTESTER_OUT					5


#endif // THMT_NODE_INFO_H
