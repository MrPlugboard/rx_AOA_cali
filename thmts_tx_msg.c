#include "thmts_tx_msg.h"


// 控制发射信息的开关
uint8_t SWITCH_THMTS_TX_SLOT_INFO_T = 0;     // 控制TX info是否发射的开关,   1:上传TXinfo , 0:不上传TXinfo
uint8_t SWITCH_THMTS_RX_SLOT_INFO_T = 0;     // 控制RX info是否发射的开关,   1:上传RXinfo , 0:不上传RXinfo
uint8_t SWITCH_THMTS_RX_CIR_CH0_INFO_T = 0;  // 控制通道0 CIR 是否发射的开关, 1:上传通道0 CIR , 0:不上传通道0 CIR
uint8_t SWITCH_THMTS_RX_CIR_CH1_INFO_T = 0;  // 控制通道1 CIR 是否发射的开关, 1:上传通道1 CIR , 0:不上传通道1 CIR
uint8_t SWITCH_THMTS_RX_CIR_CH2_INFO_T = 0;  // 控制通道2 CIR 是否发射的开关, 1:上传通道2 CIR , 0:不上传通道2 CIR
uint8_t SWITCH_THMTS_RX_CIR_CH3_INFO_T = 0;  // 控制通道3 CIR 是否发射的开关, 1:上传通道3 CIR , 0:不上传通道3 CIR
uint8_t SWITCH_THMTS_RANGING_INFO_T    = 0;  // 控制测距消息是否发射的开关,   1:上传测距结果     0:不上传测距结果
uint8_t SWITCH_THMTS_RANGE_AND_DATA_T  = 0;  // 控制数据包和测距值消息是否发射的开关,   1:上传结果     0:不上传结果
uint8_t SWITCH_THMTS_SENSOR_IMU_DATA_T = 0;  // 控制IMU传感器数据是否发射的开关, 1:上传IMU数据, 0:不上传IMU数据
//uint8_t SWITCH_THMTS_SENSOR_IMU_DATA_T = 1;  // 控制IMU传感器数据是否发射的开关, 1:上传IMU数据, 0:不上传IMU数据





// 定义不同类型的msg
thmts_tx_slot_info_t TX_slot_info;         // TX info msg   0x91
thmts_rx_slot_info_t RX_slot_info;         // RX info msg   0x92
thmts_rx_CIR_info_t  RX_CIR_info_channel0; // 通道0 CIR msg  0x93
thmts_rx_CIR_info_t  RX_CIR_info_channel1; // 通道1 CIR msg  0x93
thmts_rx_CIR_info_t  RX_CIR_info_channel2; // 通道2 CIR msg  0x93
thmts_rx_CIR_info_t  RX_CIR_info_channel3; // 通道3 CIR msg  0x93
thmts_RANGING_info_t ranging_info;         // 测距      msg  0x94
thmts_range_and_data_t range_and_data_info; // 测距和数据包 msg 0x95
thmts_sensor_imu_data_t  Sensor_IMU_data;  // IMU传感器数据

// 定义各通道的CIR存储结构体
//uint32_t channel0_CIR[1022];
//uint32_t channel1_CIR[1022];
//uint32_t channel2_CIR[1022];
//uint32_t channel3_CIR[1022];




//------------------------------------------------------------------------------

void read_and_send_CIR_msg(QueueHandle_t msgQueue)
{
//	read_thmts_bb_cir(0, (uint32_t*)channel0_CIR, 0, 1022);
//	read_thmts_bb_cir(1, (uint32_t*)channel1_CIR, 0, 1022);
//	read_thmts_bb_cir(2, (uint32_t*)channel2_CIR, 0, 1022);
//	read_thmts_bb_cir(3, (uint32_t*)channel3_CIR, 0, 1022);


//	if (SWITCH_THMTS_RX_CIR_CH0_INFO_T)
//	{
//		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
//		ptMsg->msg_id      = 1;
//		ptMsg->msg_type    = TYPE_THMTS_RX_CIR_INTO_T;   //发射 CIR info
//		ptMsg->msg_status  = 1;
//		ptMsg->msg_length  = sizeof(thmts_rx_CIR_info_t);
//
//		RX_CIR_info_channel0.ch_id = 0;
//		memcpy(&(RX_CIR_info_channel0.CIR) , channel0_CIR , 1022*4);
////									memcpy(&(RX_CIR_info_channel0.CIR[0]) , &(channel0_CIR[1012]) , 10*4);
////									memcpy(&(RX_CIR_info_channel0.CIR[10]) , &(channel0_CIR[0]) , 10*4);
//
//		ptMsg->msg_ptr     = &RX_CIR_info_channel0;
//		xQueueSend(msgQueue, &ptMsg, portMAX_DELAY);
//	}
//
//	if (SWITCH_THMTS_RX_CIR_CH1_INFO_T)
//	{
//		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
//		ptMsg->msg_id      = 1;
//		ptMsg->msg_type    = TYPE_THMTS_RX_CIR_INTO_T;   //发射 CIR info
//		ptMsg->msg_status  = 1;
//		ptMsg->msg_length  = sizeof(thmts_rx_CIR_info_t);
//
//		RX_CIR_info_channel1.ch_id = 1;
//		memcpy(&(RX_CIR_info_channel1.CIR) , channel1_CIR , 1022*4);
////									memcpy(&(RX_CIR_info_channel1.CIR[0]) , &(channel1_CIR[1012]) , 10*4);
////									memcpy(&(RX_CIR_info_channel1.CIR[10]) , &(channel1_CIR[0]) , 10*4);
//
//		ptMsg->msg_ptr     = &RX_CIR_info_channel1;
//		xQueueSend(msgQueue, &ptMsg, portMAX_DELAY);
//	}
//
//	if (SWITCH_THMTS_RX_CIR_CH2_INFO_T)
//	{
//		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
//		ptMsg->msg_id      = 1;
//		ptMsg->msg_type    = TYPE_THMTS_RX_CIR_INTO_T;   //发射 CIR info
//		ptMsg->msg_status  = 1;
//		ptMsg->msg_length  = sizeof(thmts_rx_CIR_info_t);
//
//		RX_CIR_info_channel2.ch_id = 2;
//		memcpy(&(RX_CIR_info_channel2.CIR) , channel2_CIR , 1022*4);
////									memcpy(&(RX_CIR_info_channel2.CIR[0]) , &(channel2_CIR[1012]) , 10*4);
////									memcpy(&(RX_CIR_info_channel2.CIR[10]) , &(channel2_CIR[0]) , 10*4);
//
//		ptMsg->msg_ptr     = &RX_CIR_info_channel2;
//		xQueueSend(msgQueue, &ptMsg, portMAX_DELAY);
//	}
//
//	if (SWITCH_THMTS_RX_CIR_CH3_INFO_T)
//	{
//		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
//		ptMsg->msg_id      = 1;
//		ptMsg->msg_type    = TYPE_THMTS_RX_CIR_INTO_T;   //发射 CIR info
//		ptMsg->msg_status  = 1;
//		ptMsg->msg_length  = sizeof(thmts_rx_CIR_info_t);
//
//		RX_CIR_info_channel3.ch_id = 3;
//		memcpy(&(RX_CIR_info_channel3.CIR) , channel3_CIR , 1022*4);
////									memcpy(&(RX_CIR_info_channel3.CIR[0]) , &(channel3_CIR[1012]) , 10*4);
////									memcpy(&(RX_CIR_info_channel3.CIR[10]) , &(channel3_CIR[0]) , 10*4);
//
//		ptMsg->msg_ptr     = &RX_CIR_info_channel3;
//		xQueueSend(msgQueue, &ptMsg, portMAX_DELAY);
//	}
}


void read_and_send_IMU_data(QueueHandle_t msgQueue)
{
	if (SWITCH_THMTS_SENSOR_IMU_DATA_T)
	{
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = TYPE_THMTS_SENSOR_IMU_DATA_T;   // IMU传感器数据
		ptMsg->msg_status  = 1;
		ptMsg->msg_length  = sizeof(thmts_sensor_imu_data_t);
		ptMsg->msg_ptr     = &Sensor_IMU_data;
		xQueueSend(msgQueue, &ptMsg, portMAX_DELAY);
	}
}



