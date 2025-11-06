#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "ns_sdk_hal.h"

#include "thmts_bb_config.h"
#include "thmts_rf_config.h"
#include "thmts_bsp.h"

#include "thmts_trxctrl_serial_cmd_type.h"
#include "thmts_ranging.h"
#include "thmts_pdoa.h"
#include "PPS_SYNC.h"

#include "thmts_phy_param.h"
#include "thmts_config.h"
#include "thmts_node_info.h"
#include "thmts_tx_msg.h"

#include "thmts_sensor.h"


uint8_t reset_cnt = 0;

uwb_config_t uwb_config = {
    .pdoa1 = -7555,
    .pdoa2 = 2133,
    .pdoa3 = 0,
    .role = 1,        // 根据实际需求设置：0=ANCHOR, 1=TAG
    .rsv = 0,
    .aoaOffset = 0.0f,
    .aoaCali = {0},
    .SN = "default",  // 或合适的SN号
    .rangeCali = {0}
};

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define TASKDLYMS                       pdMS_TO_TICKS(100)
#define mainQUEUE_LENGTH                (256)

static TaskHandle_t Task_ProcessUwbTR_Handler;					// 5
static TaskHandle_t Task_StartUwbTR_Handler;					// 7
static TaskHandle_t Task_ProcessUsartRxDma_Handler;             // 6
static TaskHandle_t Task_ParseUsartRxCmd_Handler;               // 3
static TaskHandle_t Task_ParseUsartTxMsg_Handler;               // 4
static TaskHandle_t Task_ProcessSpiRxDma_Handler;               // 5
static TaskHandle_t Task_ProcessSpiRxMsg_Handler;               // 1
static TaskHandle_t Task_ProcessRanging_Handler;                // 1

/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t xQueue = NULL;
SemaphoreHandle_t xRxcmdBufferMutex;
SemaphoreHandle_t xSemaphore_timer;

void task_processUwbTR(void* pvParameters);
void task_startUwbTR(void* pvParameters);
void task_processUsartRxDma(void* pvParameters);
void task_parseUsartRxCmd(void* pvParameters);
void task_parseUsartTxMsg(void* pvParameters);
void task_processSpiRxDma(void* pvParameters);
void task_processSpiRxMsg(void* pvParameters);
void task_processRanging(void* pvParameters);

/////////////////////////////////////////////////////////////////////////////////////////

uint32_t timer_cnt = 0;
void BASIC_TIMER0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulReturn;

    ulReturn = taskENTER_CRITICAL_FROM_ISR();

    if( SET == Basic_Timer_InterruptFlagGet(BASIC_TIMER0, BASIC_TIMER_DIER_UIE)) {

        if (timer_cnt==0)
        {
        	get_thmts_bb_systime( &node.ts_into_timer );
            vTaskNotifyGiveFromISR(Task_StartUwbTR_Handler, &xHigherPriorityTaskWoken);
        }
        else
        {
            vTaskNotifyGiveFromISR(Task_ProcessUsartRxDma_Handler, &xHigherPriorityTaskWoken);
        }

    	if( pps_timer_cnt ==0 && node.state == NODE_STATE_RUNNING)
    	{
    		pps_update = 1;
    		vTaskNotifyGiveFromISR(Task_ProcessUwbTR_Handler, &xHigherPriorityTaskWoken);
    	}

		if(node.arr_adjust_flag == 5)
		{
			Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1);
			node.arr_adjust_flag = 3;
//				printf("node.arr_adjust = %d" , node.arr_adjust);
		}

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        timer_cnt++;
        if (timer_cnt == TickPerSlot) timer_cnt = 0;

        pps_timer_cnt++;
        if (pps_timer_cnt == PPS_CHECK_PERIOD_IN_MS) pps_timer_cnt = 0;
    }
    taskEXIT_CRITICAL_FROM_ISR(ulReturn);
    Basic_Timer_InterruptFlagClear(BASIC_TIMER0, BASIC_TIMER_DIER_UIE);
}


uint8_t tx_complete = 0;
uint8_t rx_complete = 0;
void THURDZ_BB_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulReturn;

    ulReturn = taskENTER_CRITICAL_FROM_ISR();

	// Read the tpram once, if there exists isr, then a non-zero value will be read out and flush the 32bit reg.
	uint32_t rx_isr_flag = read_thmts_bb_reg( CHIP_THURDZ_RX_ISR_FLAG_ADDR );
	uint32_t tx_isr_flag = read_thmts_bb_reg( CHIP_THURDZ_TX_ISR_FLAG_ADDR );

	if ( rx_isr_flag ) rx_complete = 1;
	if ( tx_isr_flag ) tx_complete = 1;

    vTaskNotifyGiveFromISR(Task_ProcessUwbTR_Handler, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    taskEXIT_CRITICAL_FROM_ISR(ulReturn);

    return;
}

// dma发送数据 buff
uint8_t UartTxDma_Buff0[UART_TX_DMA_BUFF_SIZE];
uint8_t UartTxDma_Buff1[UART_TX_DMA_BUFF_SIZE];
uint8_t *p_UartTxDma_Buff = UartTxDma_Buff0;

// dma接收数据 buff
uint8_t UartRxDma_Buff0[UART_RX_DMA_BUFF_SIZE];
uint8_t UartRxDma_Buff1[UART_RX_DMA_BUFF_SIZE];
uint8_t *p_UartRxDma_Buff = UartRxDma_Buff0;

// dma发送imu spi数据 buff
uint16_t SpiImuTxDma_Buff[IMU_SPI_TX_DMA_BUFF_SIZE] = {0x8000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

// dma接收imu spi数据 buff
uint16_t SpiImuRxDma_Buff0[IMU_SPI_RX_DMA_BUFF_SIZE + 4] = {0};
uint16_t SpiImuRxDma_Buff1[IMU_SPI_RX_DMA_BUFF_SIZE + 4] = {0};
uint16_t *p_SpiRxDma_Buff = SpiImuRxDma_Buff0;

void UDMA_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulReturn;

    ulReturn = taskENTER_CRITICAL_FROM_ISR();

    if (UDMA_PA2M_GetITStatus(USART1_USART_DMA_TX_DMA_IRQ , PA2M_FTRANS_IRQ_STAT))
    {
        UDMA_PA2M_ClearITStatus(USART1_USART_DMA_TX_DMA_IRQ , PA2M_FTRANS_IRQ_CLEAR_STAT);
        vTaskNotifyGiveFromISR(Task_ParseUsartTxMsg_Handler, &xHigherPriorityTaskWoken);
    }

    if (UDMA_PA2M_GetITStatus(QSPI2_RX_DMA_DMA_IRQ, PA2M_FTRANS_IRQ_STAT))
    {
        UDMA_PA2M_ClearITStatus(QSPI2_RX_DMA_DMA_IRQ, PA2M_FTRANS_IRQ_CLEAR_STAT);
        vTaskNotifyGiveFromISR(Task_ProcessSpiRxMsg_Handler, &xHigherPriorityTaskWoken);
    }

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}



//IMU中断 ，通知芯片有IMU数据
uint64_t imu_data_timestamp = 0;

void LGPIO0_IMU_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulReturn;

    ulReturn = taskENTER_CRITICAL_FROM_ISR();

    // if(LGPIO_GetITStatus(LGPIO0, LGPIO_PinSource6, LGPIO_IT_FALL) == LGPIO_PinSource6)
    if(LGPIO_GetIRQStatus(LGPIO0, LGPIO_PinSource6) == LGPIO_PinSource6)
    {
        // get_thmts_bb_systime(&imu_data_timestamp);
        LGPIO_ITClear(LGPIO0, LGPIO_PinSource6, LGPIO_IT_FALL);
        vTaskNotifyGiveFromISR(Task_ProcessSpiRxDma_Handler, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    taskEXIT_CRITICAL_FROM_ISR(ulReturn);

    return;
}

// 全局变量定义区域
uint32_t slot_cnt = 0;

uint8_t debug_ranging_buf[500];
uint8_t debug_ranging_buf2[500];
uint8_t debug_ranging_buf3[3000];

thmts_ctrl_cmd_proc_msg_t thmts_ctrl_cmd_proc_msg;



int main()
{
	First_start_pps = 1;
    TimerHandle_t xExampleSoftwareTimer = NULL;
    xSemaphore_timer = xSemaphoreCreateBinary();
	xRxcmdBufferMutex = xSemaphoreCreateMutex();

    prvSetupHardware();

	if (xRxcmdBufferMutex == NULL) {
        printf("Unable to create xRxcmdBufferMutex due to low memory.\n");
        while (1);
    }

	xQueue = xQueueCreate(/* The number of items the queue can hold. */
	                 mainQUEUE_LENGTH,
	                 /* The size of each item the queue holds. */
	                 sizeof(thmts_tx_msg_info_t *));

	if (xQueue == NULL) {
		printf("Unable to create xQueue due to low memory.\n");
		while (1);
	}

	// init cmd parser
    thmts_ctrl_cmd_proc_msg_t thmts_ctrl_cmd_proc_msg;
    thmts_ctrl_cmd_proc_init( &thmts_ctrl_cmd_proc );

    //串口任务合并成一个
    xTaskCreate((TaskFunction_t)task_processUsartRxDma, (const char*)"task_processUsartRxDma",
                (uint16_t)512, (void*)NULL, (UBaseType_t)6,
                (TaskHandle_t*)&Task_ProcessUsartRxDma_Handler);

    xTaskCreate((TaskFunction_t)task_parseUsartRxCmd, (const char*)"task_parseUsartRxCmd",
                (uint16_t)256, (void*)NULL, (UBaseType_t)2,
                (TaskHandle_t*)&Task_ParseUsartRxCmd_Handler);

    xTaskCreate((TaskFunction_t)task_parseUsartTxMsg, (const char*)"task_parseUsartTxMsg",
                (uint16_t)256, (void*)NULL, (UBaseType_t)4,
                (TaskHandle_t*)&Task_ParseUsartTxMsg_Handler);

    xTaskCreate((TaskFunction_t)task_processUwbTR, (const char*)"task_processUwbTR",
                (uint16_t)512, (void*)NULL, (UBaseType_t)5,
                (TaskHandle_t*)&Task_ProcessUwbTR_Handler);

    xTaskCreate((TaskFunction_t)task_startUwbTR, (const char*)"task_startUwbTR",
                (uint16_t)512, (void*)NULL, (UBaseType_t)7,
                (TaskHandle_t*)&Task_StartUwbTR_Handler);


    xTaskCreate((TaskFunction_t)task_processSpiRxDma, (const char*)"task_processSpiRxDma",
                 (uint16_t)512, (void*)NULL, (UBaseType_t)3,
                 (TaskHandle_t*)&Task_ProcessSpiRxDma_Handler);

    xTaskCreate((TaskFunction_t)task_processSpiRxMsg, (const char*)"task_processSpiRxMsg",
                 (uint16_t)512, (void*)NULL, (UBaseType_t)1,
                 (TaskHandle_t*)&Task_ProcessSpiRxMsg_Handler);

    xTaskCreate((TaskFunction_t)task_processRanging, (const char*)"task_processRanging",
                 (uint16_t)512*6, (void*)NULL, (UBaseType_t)1,
                 (TaskHandle_t*)&Task_ProcessRanging_Handler);

    printf("Before StartScheduler\r\n");

    vTaskStartScheduler();

    printf("OS should never run to here\r\n");

    while (1);

    printf( "SIMULATION SUCCESS\r\n" );
}


void start_UWB_TR();

void task_startUwbTR(void* pvParameters)
{
	printf("Enter to task BasicTimer11\r\n");

	while(1)
	{   // wait for timer interrupt
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		start_UWB_TR();
	}
}



uint16_t adjust_tick_period = 0;
uint16_t adjust_tick_cnt = 0;

uint16_t testPHRCnt = 0;
uint32_t PHRdata[1000];


void processUwbTx();
void processUwbRx();
void process();

void task_processUwbTR(void* pvParameters)
{
	while(1)
	{
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		if (tx_complete)
		{
			processUwbTx();
		}

		if (rx_complete)
		{
			processUwbRx();
		}

		if(pps_update)
		{
			processPPS();
		}

		if(node.curr_slot_idx == 0 && node.curr_subfrm_idx == 3)
		{
			xTaskNotifyGive(Task_ProcessRanging_Handler);
		}
    }
}




uint16_t prepareTxMsg(thmts_tx_msg_info_t *ptMsgReceived, uint8_t *pTxBuffer);

void task_parseUsartTxMsg(void* pvParameters)
{

	printf("Enter to task_parseUsartTxcmd.\r\n");

	uint8_t FIRST_START_DMA = 1;
	uint16_t tx_len = 0;

    while (1)
    {
    	thmts_tx_msg_info_t *ptMsgReceived;
    	if(xQueueReceive(xQueue, &ptMsgReceived, portMAX_DELAY) == pdPASS)
		{
    		tx_len = prepareTxMsg(ptMsgReceived, p_UartTxDma_Buff);
		}

    	if(FIRST_START_DMA == 0)
    	{
    		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    	}
    	else
    	{
    		//第一次进入DMA发射
    		FIRST_START_DMA = 0;
    	}

    	uint16_t epoch = 0;
    	epoch = tx_len/1024;
    	uint16_t len_mod = tx_len%1024;
    	uint32_t idx = 0;
    	uint32_t send_cnt_per_epoch = 1024;
    	for(idx = 0 ; idx < epoch ;idx++)
    	{
    		UDMA_USART1_TX(1024, p_UartTxDma_Buff + 1024*idx);  // 把当前buff地址给到DMA，启动TX，然后切换buff
    		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    	}
    	if(len_mod > 0)
    	{
    		UDMA_USART1_TX(len_mod, p_UartTxDma_Buff + 1024*epoch);  // 把当前buff地址给到DMA，启动TX，然后切换buff
    	}



    	//切换DMA
		if (p_UartTxDma_Buff==UartTxDma_Buff0)
		{
			p_UartTxDma_Buff = UartTxDma_Buff1;
		}
		else
		{
			p_UartTxDma_Buff = UartTxDma_Buff0;
		}
    }
}


void task_processUsartRxDma(void* pvParameters)
{
	uint8_t *p_tmp = NULL;
	uint32_t data_size_inbuff;

	printf("Enter to task_processUsartRxDma.\r\n");

    while (1)
    {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

	    if(USART_GetFlag(USART1, USART_STATUS_RX_BUSY) == RESET)
	    {
	    	DMA_Stop(USART1_USART_DMA_RX_DMA_CH);

	    	data_size_inbuff = UART_RX_DMA_BUFF_SIZE - DMA_Get_Counter(USART1_USART_DMA_RX_DMA_CH);

	        if (data_size_inbuff > 0)
	        {
	        	p_tmp = p_UartRxDma_Buff;
	        	if (p_UartRxDma_Buff==UartRxDma_Buff0)
	        	{
	        		p_UartRxDma_Buff = UartRxDma_Buff1;
	        	}
	        	else
	        	{
	        		p_UartRxDma_Buff = UartRxDma_Buff0;
	        	}
	        }

	        UDMA_USART1_RX(p_UartRxDma_Buff);	// 重新启动DMA

            // 后续替换为向消息解析任务的循环buff写入
	        if (data_size_inbuff > 0)
	        {
		        BaseType_t xLockTaken = xSemaphoreTake(xRxcmdBufferMutex, portMAX_DELAY);

				if (xLockTaken == pdTRUE)
				{
					if (p_tmp==UartRxDma_Buff0)
					{
						thmts_ctrl_cmd_proc_insert_rx_bytes( &thmts_ctrl_cmd_proc, UartRxDma_Buff0, data_size_inbuff );
					}
					else
					{
						thmts_ctrl_cmd_proc_insert_rx_bytes( &thmts_ctrl_cmd_proc, UartRxDma_Buff1, data_size_inbuff );
					}
					//通知RX cmd解析任务
					xTaskNotifyGive(Task_ParseUsartRxCmd_Handler);
				}
				xSemaphoreGive(xRxcmdBufferMutex);
	        }
	        data_size_inbuff = 0;
	    }
    }
}


thmts_tx_msg_info_t  thmts_tx_msg_info;

uint8_t node_config_flag;
uint8_t phy_config_flag;


uint8_t flash_data[256];
uint8_t read_data[256];


void process_received_message(thmts_ctrl_cmd_proc_msg_t *msg_info);


void task_parseUsartRxCmd(void* pvParameters)
{
	uint8_t *p_tmp = NULL;
	uint32_t data_size_inbuff;
	uint16_t j;

	uint16_t cmdBufLen = 0;

	BaseType_t   xReturn;

	printf("Enter to task_parseUsartRxcmd.\r\n");

    while (1)
    {
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
//		printf("---------rec cmd \r\n");


		BaseType_t xLockTaken = xSemaphoreTake(xRxcmdBufferMutex, portMAX_DELAY);

		if (xLockTaken == pdTRUE)
		{
			thmts_ctrl_cmd_proc_exec( &thmts_ctrl_cmd_proc );
		}
		xSemaphoreGive(xRxcmdBufferMutex);


		while(thmts_ctrl_cmd_proc.curr_msg_node_cnt > 0)
		{
			thmts_ctrl_cmd_proc_get_rx_msg( &thmts_ctrl_cmd_proc, &thmts_ctrl_cmd_proc_msg );
			if( thmts_ctrl_cmd_proc_msg.msg_buf_ptr )
			{

				process_received_message(&thmts_ctrl_cmd_proc_msg);


				thmts_ctrl_cmd_proc_pop_rx_msg( &thmts_ctrl_cmd_proc );
			}

		}

    }
}




void task_processSpiRxDma(void* pvParameters)
{
    printf("Enter to task_processSpiRxDma.\r\n");

	IMU_Init();      // IMU设备初始化
    IMU_SPI_Init();  // IMU寄存器参数配置
    LGPIO_ITConfig(LGPIO0, LGPIO_PinSource6, LGPIO_IT_FALL, ENABLE);  // IMU DRDY引脚中断使能

    while (1)
    {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        SPI2_CS_ON;

        UDMA_QSPI2_RX(IMU_SPI_RX_DMA_BUFF_SIZE, p_SpiRxDma_Buff);
        UDMA_QSPI2_TX(IMU_SPI_TX_DMA_BUFF_SIZE, SpiImuTxDma_Buff);

        SPI2_Dma_Enable(QSPI2, QSPI_CR_DMA_ENABLE, QSPI_CR_DMA_TX_ENABLE, QSPI_CR_DMA_RX_ENABLE);
    }
}

void task_processSpiRxMsg(void* pvParameters)
{
    printf("Enter to task_processSpiRxMsg.\r\n");

    while (1)
    {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        // 等待DMA接收完全停止
        while (SET == QSPI_GetFlag(QSPI2, QSPI_STATUS_BUSY)) {}

        SPI2_Dma_Enable(QSPI2, QSPI_CR_DMA_DISABLE, QSPI_CR_DMA_TX_DISABLE, QSPI_CR_DMA_RX_DISABLE);

        SPI2_CS_OFF;

        // 更换乒乓缓存接收buffer的地址，处理原buffer接收到的数据
		p_SpiRxDma_Buff = (p_SpiRxDma_Buff == SpiImuRxDma_Buff0) ? SpiImuRxDma_Buff1 : SpiImuRxDma_Buff0;
		IMU_SPI_Master_Read_Burst2((p_SpiRxDma_Buff == SpiImuRxDma_Buff0) ? SpiImuRxDma_Buff1 : SpiImuRxDma_Buff0, imu_data_timestamp);
		read_and_send_IMU_data(xQueue);
    }
}

// 填充altds_twr_t结构体，共12个时间戳，编号从1开始
double tof_fill_and_compute(uint64_t* ts_all,uint8_t poll_tx, uint8_t poll_rx,uint8_t resp_tx,uint8_t resp_rx,uint8_t poll2_tx,uint8_t poll2_rx)
{
	altds_twr_t twr={0};
	twr.poll_tx_time=ts_all[poll_tx-1];
	twr.poll_rx_time=ts_all[poll_rx-1];
	twr.resp_tx_time=ts_all[resp_tx-1];
	twr.resp_rx_time=ts_all[resp_rx-1];
	twr.poll2_tx_time=ts_all[poll2_tx-1];
	twr.poll2_rx_time=ts_all[poll2_rx-1];
	if(altds_dstwr_check(&twr))
	{
		return altds_dstwr_compute(&twr); 
	}
	return 0;
}

uint8_t check_valid = 0;
uint32_t tof_two_nodes_compute_new(uint8_t i, uint8_t j, uint8_t ping_pong_flag)
{
	uint32_t tof_int = 0;
	uint64_t ts_all[12]={0};
	double tof_temp=0;

	ts_all[0]=node.trx_stamps_subfrm0_ping[i][i];
	ts_all[1]=node.trx_stamps_subfrm0_ping[i][j];
	ts_all[2]=node.trx_stamps_subfrm0_ping[j][i];
	ts_all[3]=node.trx_stamps_subfrm0_ping[j][j];
	ts_all[4]=node.trx_stamps_subfrm1_ping[i][i];
	ts_all[5]=node.trx_stamps_subfrm1_ping[i][j];
	ts_all[6]=node.trx_stamps_subfrm1_ping[j][i];
	ts_all[7]=node.trx_stamps_subfrm1_ping[j][j];
	ts_all[8]=node.trx_stamps_subfrm2_ping[i][i];
	ts_all[9]=node.trx_stamps_subfrm2_ping[i][j];
	ts_all[10]=node.trx_stamps_subfrm2_ping[j][i];
	ts_all[11]=node.trx_stamps_subfrm2_ping[j][j];

//	ts_error_cnt = check_ts_error(ts_all);
//	comm_error_cnt = check_comm_error(ts_all);

	tof[0]=tof_fill_and_compute(ts_all,1,3,4,2,5,7);  //123
	tof[1]=tof_fill_and_compute(ts_all,1,3,8,6,9,11); //145
	tof[2]=tof_fill_and_compute(ts_all,5,7,8,6,9,11); //345
	tof[3]=tof_fill_and_compute(ts_all,4,2,5,7,8,6);  //234
	tof[4]=tof_fill_and_compute(ts_all,1,3,8,6,5,7);  //143
	tof[5]=tof_fill_and_compute(ts_all,4,2,9,11,8,6); //254
	tof[6]=tof_fill_and_compute(ts_all,1,3,4,2,9,11); //125
	tof[7]=tof_fill_and_compute(ts_all,9,11,4,2,5,7); //523
	tof[8]=tof_fill_and_compute(ts_all,8,6,1,3,4,2);  //412

	// 选取tof数组中非零值的中位数
	double valid_tof[9];
	valid_count = 0;
	for (int idx = 0; idx < 9; idx++) {
		if (tof[idx] > 0 && tof[idx] < TOF_UPPER_BOUND) {
			valid_tof[valid_count++] = tof[idx];
		}
	}
	double median_tof = 0;
	if (valid_count == 0) {
		median_tof = 0;
	} else {
		// 冒泡排序
		for (int i = 0; i < valid_count - 1; i++) {
			for (int j = 0; j < valid_count - 1 - i; j++) {
				if (valid_tof[j] > valid_tof[j + 1]) {
					double tmp = valid_tof[j];
					valid_tof[j] = valid_tof[j + 1];
					valid_tof[j + 1] = tmp;
				}
			}
		}
		median_tof = valid_tof[valid_count / 2];
	}

	tof_temp = median_tof;
	tof_double_cm = tof_temp;

	check_valid=1;
	tof_int = (tof_temp * 100 - tof_bias) > 0 ? (tof_temp * 100 - tof_bias) : 0;
	if(tof_int > TOF_UPPER_BOUND * 100)
	{
		tof_int=0;
	}
	if(tof_int==0)
	{
		check_valid=0;
	}

	tof_double_cm = 1.0*tof_int/100.0;

	return tof_int;
}


void task_processRanging(void* pvParameters)
{
	
    printf("Enter to task_processRanging.\r\n");

    while (1)
    {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    	uint32_t txPoint_buff = 0;
    	// 发送结构体指针

    	thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
    	ptMsg->msg_id      = 1;
    	ptMsg->msg_type    = 0x41;
    	ptMsg->msg_status  = 1;
    	ptMsg->msg_ptr     = debug_ranging_buf3;

    	AGC_idx=0;

		tof_int_cm=tof_two_nodes_compute_new(0, 1, 1);

		double phi21 = cal_pdoa(CIR_data0, CIR_data1);
		double phi31 = cal_pdoa(CIR_data0, CIR_data2);
		double phi32 = cal_pdoa(CIR_data1, CIR_data2);

		//这里的计算结果就是三组PDOA内容，单位是角度（°），这里输出的话就是10Hz的输出频度

//		txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff], " tof_int between %d and %d = %d cm, valid_cnt = %d \r\n" , 1-node.dev_id,node.dev_id, tof_int_cm,valid_count);

		//TODO: RSSI获取
		int32_t rssi=10;
		
		// 根据PDOA计算AOA
		double azimuth = aoa_compute_from_pdoa(phi21, phi31, phi32, &uwb_config);

		AOA_index++;
//		// 上报信息：交互角色（TAG） 序号 测距 根据当前校准表计算的AOA 接收信号强度 原始PDOA1 原始PDOA2

		int32_t azimuth_integer=(int32_t)azimuth;
		int32_t azimuth_decimal=abs(((int32_t)(azimuth*100))%100);

		#ifdef SWITCH_OUTPUT_CIR
			if(azimuth_decimal<10)
			{
				//输出协议中要求pdoa统一为phi12和phi13，单位为pi/6434
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
					"TAG %7d %6d %3d.0%1d %4d %5d %5d",
					AOA_index,tof_int_cm,azimuth_integer,azimuth_decimal,rssi,(int32_t)(-PDOA_scale*phi21),(int32_t)(-PDOA_scale*phi31));

				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR0: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data0[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR1: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data1[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR2: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data2[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"\n");
			}
			else
			{
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
					"TAG %7d %6d %3d.%2d %4d %5d %5d",
					AOA_index,tof_int_cm,azimuth_integer,azimuth_decimal,rssi,(int32_t)(-PDOA_scale*phi21),(int32_t)(-PDOA_scale*phi31));
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR0: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data0[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR1: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data1[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"CIR2: ");
				for (uint8_t cir_idx=0;cir_idx<CIR_FOR_PDOA_LENGTH;cir_idx++)
				{
					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
										"%5d ",CIR_data2[0][cir_idx]);
				}
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],"\n");
			}
		#else
			if(azimuth_decimal<10)
			{
				//输出协议中要求pdoa统一为phi12和phi13，单位为pi/6434
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
					"TAG %7d %6d %3d.0%1d %4d %5d %5d\n",
					AOA_index,tof_int_cm,azimuth_integer,azimuth_decimal,rssi,(int32_t)(-PDOA_scale*phi21),(int32_t)(-PDOA_scale*phi31));
			}
			else
			{
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
					"TAG %7d %6d %3d.%2d %4d %5d %5d\n",
					AOA_index,tof_int_cm,azimuth_integer,azimuth_decimal,rssi,(int32_t)(-PDOA_scale*phi21),(int32_t)(-PDOA_scale*phi31));
			}
		#endif


		// 打印
		uint8_t ts_output=0;
		uint8_t tof_output=0;
		uint8_t rx_info_output=0;

		if(ts_output)
		{
			uint32_t ts_hi,ts_lo;

			ts_hi = (uint32_t)(node.trx_stamps_subfrm0_ping[0][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm0_ping[0][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm0_ping[0][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm0_ping[0][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm0_ping[1][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm0_ping[1][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm0_ping[1][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm0_ping[1][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u | \r\n",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm1_ping[0][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm1_ping[0][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);
			
			ts_hi = (uint32_t)(node.trx_stamps_subfrm1_ping[0][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm1_ping[0][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm1_ping[1][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm1_ping[1][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm1_ping[1][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm1_ping[1][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |\r\n",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm2_ping[0][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm2_ping[0][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);
			
			ts_hi = (uint32_t)(node.trx_stamps_subfrm2_ping[0][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm2_ping[0][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm2_ping[1][0] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm2_ping[1][0]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u |",
			ts_hi, ts_lo);

			ts_hi = (uint32_t)(node.trx_stamps_subfrm2_ping[1][1] >> 32);
			ts_lo = (uint32_t)(node.trx_stamps_subfrm2_ping[1][1]);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"%3u %10u | \r\n",
			ts_hi, ts_lo);
		}
		memset(node.trx_stamps_subfrm0_ping, 0, sizeof(node.trx_stamps_subfrm0_ping));
		memset(node.trx_stamps_subfrm1_ping, 0, sizeof(node.trx_stamps_subfrm1_ping));
		memset(node.trx_stamps_subfrm2_ping, 0, sizeof(node.trx_stamps_subfrm2_ping));
		memset(thmts_tx_frame_rx_stamp,0,sizeof(thmts_tx_frame_rx_stamp));


		if(tof_output==1)
		{
			uint32_t tof_temp=0;
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"tof: ");
			for (int i = 0; i < 9; i++)
			{
				tof_temp=tof[i]*100;
				txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
				"%9d |",
				tof_temp);
			}
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff],
			"\r\n");
		}


		if(rx_info_output==1)
		{
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff], "total rx = %d, ok = %d , timeout = %d , phr error = %d , crc error = %d\r\n",
					rx_cnt,
					crc_ok_cnt,
					rx_timeout_cnt,
					rx_phr_error_cnt,
					rx_ok_cnt - crc_ok_cnt);
			txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf3[txPoint_buff], "AGC_num = %d | %d | %d\r\n", AGC_result[0],AGC_result[1],AGC_result[2]);
		}



		memset(poll1_tx_stamp_t, 0, sizeof(poll1_tx_stamp_t));
		memset(poll1_rx_stamp_t, 0, sizeof(poll1_rx_stamp_t));
		memset(resp_rx_stamp_t, 0, sizeof(resp_rx_stamp_t));
		memset(poll2_tx_stamp_t, 0, sizeof(poll2_tx_stamp_t));
		memset(poll2_rx_stamp_t, 0, sizeof(poll2_rx_stamp_t));

		memset(tof, 0, sizeof(tof));
		tof_int_cm=0;
		
		resp_tx_stamp_t = 0;
		tof_int=0;
		if(txPoint_buff > 0 )
		{
			ptMsg->msg_length  = txPoint_buff;
			xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
		}
		else
		{
			vPortFree(ptMsg);
		}
    }
}

void vApplicationTickHook(void)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
    1 in FreeRTOSConfig.h.

    "Give" the semaphore on every 500th tick interrupt. */

    /* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
    normally be performed before leaving the interrupt (because during the
    execution of the interrupt a task of equal or higher priority than the
    running task was unblocked).  The syntax required to context switch from
    an interrupt is port dependent, so check the documentation of the port you
    are using.

    In this case, the function is running in the context of the tick interrupt,
    which will automatically check for the higher priority task to run anyway,
    so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    printf("malloc failed\n");
    while (1);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    printf("Stack Overflow\n");
    while (1);
}
/*-----------------------------------------------------------*/

extern UBaseType_t uxCriticalNesting;
void vApplicationIdleHook(void)
{
    // volatile size_t xFreeStackSpace;
    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    /* By now, the kernel has allocated everything it is going to, so
    if there is a lot of heap remaining unallocated then
    the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
    reduced accordingly. */
}


uint8_t rx_flag=0;//用于统计

void start_UWB_TR()
{

	uint32_t txPoint_buff = 0;
	// 发送结构体指针

	thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
	ptMsg->msg_id      = 1;
	ptMsg->msg_type    = 0x41;
	ptMsg->msg_status  = 1;
	ptMsg->msg_ptr     = debug_ranging_buf2;

	int32_t delta_test = 0;
	thmts_tx_frame.RSV = 0x0;

	switch (node.state)
	{
		case NODE_STATE_IDLE :
			if ( node.dev_id == 0 && node.group_id == 0) //中心节点
			{
				start_thmts_tx(&node, &thmts_phycfg, &thmts_tx_frame);

				node.state = NODE_STATE_RUNNING;
			}
			else
			{
				node.state = NODE_STATE_CAPTURE;
			}
			break;
		case NODE_STATE_CAPTURE :
			if (node.arr_adjust_flag == 0) {
				if (node.uwb_rx_busy == 0) {
					//BB_RX_MODULE_POWER_DOWN;
					BB_RX_MODULE_POWER_ON;
					config_thmts_bb_rx_sw_lna_on(thmts_phycfg.rf_chan_num);
					start_thmts_bb_rx(&thmts_phycfg, 1024000); 		// 超时时间设置为1s

					node.uwb_rx_busy = 1;
				}
			}
			else if (node.arr_adjust_flag == 1)
			{
				delta_test = node.arr_adjust;

				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR*TickPerSlot - 1 + delta_test - 2);
				node.arr_adjust_flag = 2;

				timer_cnt = 0;
			}
			else if(node.arr_adjust_flag == 2)
			{
				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1);
				node.state = NODE_STATE_CONFIRM;
				node.confirm_slot_cnt = 0;
				node.arr_adjust_flag = 3;
			}
			break;
		case NODE_STATE_CONFIRM :
			node.confirm_slot_cnt++;
			if(node.role == NODE_ROLE_ANCHOR_SLAVE_NORMAL)
			{
				if(node.confirm_slot_cnt > 60)
				{
					init_node();
					node.state = NODE_STATE_CAPTURE;
					break;
				}
				if(thmts_rx_frame.head.slot_id == 0 && thmts_rx_frame.head.group_id ==0 ) //收				
				{
					node.state = NODE_STATE_RUNNING;
					//node.comm_frm_tx_flag=0;
				}
				else
				{
					start_thmts_rx(&node, &thmts_phycfg);
				}
			}
///////////////////////////////////////////////////////////////////////
/// thmts_rx_frame是否搞成pingpong buffer
///////////////////////////////////////////////////////////////////////
			if(node.arr_adjust_flag == 4)
			{
				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1 + node.arr_adjust - 2);
				node.arr_adjust_flag = 5;
			}
			else
			{
				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1);
			}
			break;
		case NODE_STATE_RUNNING :
			if(node.role == NODE_ROLE_ANCHOR_SLAVE_NORMAL)
			{
				node.running_group_master_slot_cnt++;
			}
	    	if(thmts_rx_frame.head.group_id ==0 && thmts_rx_frame.head.slot_id ==0 ) //收到簇主节点的消息之后
	    	{
	    		node.running_group_master_slot_cnt = 0;
	    	}

			if(node.curr_slot_idx == 3) //提前写部分PSDU
			{
				// memcpy(thmts_tx_frame.user_data, node.user_data, USER_DATA_LENGTH);
				// thmts_tx_frame.rx_stamp = thmts_tx_frame_rx_stamp;
				// write_thmts_bb_txbuff(&thmts_tx_frame + sizeof(thmts_uwbpacket_head_t), sizeof(thmts_uwbpacket_head_t) ,sizeof(thmts_ranging_packet_t)-sizeof(thmts_uwbpacket_head_t));
			}
			if(node.p_supfrm_desc->p_subfrm_descs[node.curr_subfrm_idx].subfrm_type != IDLE_FRAME)
			{
				if(node.curr_slot_idx == node.dev_id)
				{
					//BB_TX_MODULE_POWER_DOWN;
					BB_TX_MODULE_POWER_ON;
//					memcpy(thmts_tx_frame.user_data, node.user_data, USER_DATA_LENGTH);
					if(node.dev_id==0)
					{
						if(node.curr_subfrm_idx ==1 || node.curr_subfrm_idx ==2)
						{
							thmts_tx_frame.rx_stamp = thmts_tx_frame_rx_stamp[node.curr_subfrm_idx-1];
						}
					}
					else
					{
						thmts_tx_frame.rx_stamp = thmts_tx_frame_rx_stamp[node.curr_subfrm_idx];
					}
					thmts_tx_frame.master_pps_time = pps_adjust.master_pps_time_last;
//					write_thmts_bb_txbuff(&thmts_tx_frame + sizeof(thmts_uwbpacket_head_t), sizeof(thmts_uwbpacket_head_t) ,sizeof(thmts_ranging_packet_t)-sizeof(thmts_uwbpacket_head_t));
					start_thmts_tx(&node, &thmts_phycfg, &thmts_tx_frame);

//					uint32_t poll_tx_hi = (uint32_t)(thmts_tx_frame_rx_stamp >> 32);
//					uint32_t poll_tx_lo = (uint32_t)(thmts_tx_frame_rx_stamp);
//					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf2[txPoint_buff],
//					"poll_tx = %3u, %10u \r\n",
//					poll_tx_hi, poll_tx_lo
//				);

				}
				else
				{
					start_thmts_rx(&node, &thmts_phycfg);
				}
			}
			else
			{
				if (node.curr_subfrm_idx == 3 && node.curr_slot_idx == 0)
				{
					node.user_data_frm_cnt++;
					if(node.user_data_rx_valid == 0){
						node.user_data_invalid_frm_cnt++;
					}
					int FER = node.user_data_invalid_frm_cnt * 100 / node.user_data_frm_cnt;
//					txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf2[txPoint_buff], "frame cnt =%d , user_data_rx_valid = %d , user_data_frm_cnt = %d , user_data_invalid_frm_cnt = %d, FER = %d%% \r\n",
//							node.user_data_cnt, node.user_data_rx_valid, node.user_data_frm_cnt, node.user_data_invalid_frm_cnt, FER);

					if (SWITCH_THMTS_RANGE_AND_DATA_T)
					{
						thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
						ptMsg->msg_id      = 1;
						ptMsg->msg_type    = TYPE_THMTS_RANGE_AND_DATA_T;   //发射 SLOT info
						ptMsg->msg_status  = 1;
						ptMsg->msg_length  = sizeof(thmts_range_and_data_t);

						memcpy(range_and_data_info.data,node.user_data_rx,USER_DATA_LENGTH*sizeof(node.user_data[0]));
						range_and_data_info.data_valid = node.user_data_rx_valid;
						range_and_data_info.frame_cnt = node.user_data_cnt;
						range_and_data_info.tof_valid = check_valid;
						range_and_data_info.tof_value = tof_double_cm;

						ptMsg->msg_ptr     = &range_and_data_info;
						xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
					}
					node.user_data_rx_valid = 0;
					if(node.user_data_frm_cnt == 100)
					{
						node.user_data_frm_cnt = 0;
						node.user_data_invalid_frm_cnt = 0;
					}
				}
			}


			if(node.running_group_master_slot_cnt > 60) //超过三个超帧没收到簇内主节点消息
			{
				node.state = NODE_STATE_CONFIRM;
			}

			///////////////////////////////////////
			/// 时钟调整的方案： 各个非0的簇主节点也需要调整
			if(node.arr_adjust_flag == 4)
			{
				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1 + node.arr_adjust - 2);
				node.arr_adjust_flag = 5;
			}
			else
			{
				Basic_Timer_AutoReloadValueConfig(BASIC_TIMER0 , TIMER_AAR - 1);
			}
			break;
		default	: ;
	}

	if(node.p_supfrm_desc->p_subfrm_descs[node.curr_subfrm_idx].subfrm_type != IDLE_FRAME)
	{
		if(node.curr_slot_idx==2 || node.curr_slot_idx==3)
		{
			rx_flag=0;
		}
		else if(node.curr_slot_idx != node.dev_id)
		{
			rx_flag=1;
		}
	}
	else
	{
		rx_flag=0;
	}



	// 统一对slot_id进行管理
	node.curr_slot_idx++;
	if (node.curr_slot_idx == node.p_supfrm_desc->p_subfrm_descs[node.curr_subfrm_idx].total_slot_num) {
		node.curr_slot_idx = 0;
		node.curr_subfrm_idx++;
		if (node.curr_subfrm_idx == node.p_supfrm_desc->total_subfrm_num) {
			node.curr_subfrm_idx = 0;
			node.curr_frm_idx++;
			node.curr_user_data_frm_idx++;
			node.user_data_cnt++;
			if (node.curr_frm_idx == node.p_supfrm_desc->total_frm_num) {
				node.curr_frm_idx = 0;
			}
		}
	}

	if(txPoint_buff > 0 )
	{
		ptMsg->msg_length  = txPoint_buff;
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
	}
	else
	{
		vPortFree(ptMsg);
	}

}


void processUwbTx()
{
	tx_complete = 0;

	RF_TX_POWER_DOWN;

	PA_LNA_DISABLE;         //打开接收，关闭PA和LNA

	clean_thmts_bb_tx();

	BB_TX_MODULE_POWER_DOWN;
	//
	node.uwb_tx_busy = 0;

	switch (node.curr_subfrm_idx)
	{
		case 0x00:
		{
			node.trx_stamps_subfrm0_ping[node.dev_id][node.dev_id] = thmts_tx_frame.head.tx_stamp;
			break;
		}
		case 0x01:
		{
			node.trx_stamps_subfrm1_ping[node.dev_id][node.dev_id] = thmts_tx_frame.head.tx_stamp;
			break;
		}
		case 0x02:
		{
			node.trx_stamps_subfrm2_ping[node.dev_id][node.dev_id] = thmts_tx_frame.head.tx_stamp;
			break;
		}
		default:
			break;
	}

	if(node.curr_subfrm_idx == 1)
	{
		resp_tx_stamp_t = thmts_tx_frame.head.tx_stamp;
	}
	memset(&thmts_tx_frame,0,sizeof(thmts_tx_frame));
	if(node.curr_subfrm_idx == 2)
	{
		for(int i = 0; i < USER_DATA_LENGTH; i++) {
			node.user_data[i] = USER_DATA_LENGTH*node.user_data_cnt + i;
		}
	}
	memcpy(thmts_tx_frame.user_data, node.user_data, USER_DATA_LENGTH*sizeof(node.user_data[0]));




	if (SWITCH_THMTS_TX_SLOT_INFO_T)
	{
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = TYPE_THMTS_TX_SLOT_INFO_T;   //发射 SLOT info
		ptMsg->msg_status  = 1;
		ptMsg->msg_length  = sizeof(thmts_tx_slot_info_t);

		ptMsg->msg_ptr     = &TX_slot_info;
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
	}
}






void processUwbRx()
{
	uint32_t txPoint_buff = 0;
	uint32_t PHR_info_RX = 0;
	// 发送结构体指针

	thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
	ptMsg->msg_id      = 1;
	ptMsg->msg_type    = 0x41;
	ptMsg->msg_status  = 1;
	ptMsg->msg_ptr     = debug_ranging_buf;

	rx_complete = 0;

	clean_thmts_bb_rx();



	config_thmts_bb_rx_sw_lna_off(4);
	node.uwb_rx_busy = 0;

	// 获取接收信息
	thmts_get_rx_content(0x00);

	BB_RX_MODULE_POWER_DOWN;

	adjust_tick_cnt++;



	if(thmts_rx_content.CRC_OK == 1)
	{


		uint32_t CRC_BB = read_thmts_bb_reg(0x02fd0024);
		uint16_t CRC_BB_HIGH16 = (uint16_t)(CRC_BB >> 16);
		uint16_t CRC_BB_LOW16  = (uint16_t)(CRC_BB & 0xFFFF);

		uint32_t CRC_RX = 0;
		memcpy(&CRC_RX , (uint32_t *)(0x10132000 + 5 + sizeof(thmts_ranging_packet_t) - 4) , 2);


		if(thmts_rx_frame.head.slot_id == 0)
		{
			node.curr_frm_idx = thmts_rx_frame.head.frm_id;
			node.curr_subfrm_idx = thmts_rx_frame.head.subfrm_id;
			node.curr_slot_idx = thmts_rx_frame.head.slot_id+1; //slot 15没有节点发送消息，因此不会出现溢出
		}


		if(node.curr_frm_idx == node.group_id) //所属簇的Frame
		{
			thmts_tx_frame.rx_stamp_bitmap |= (0x01 << thmts_rx_frame.head.slot_id);
			thmts_tx_frame.rx_stamp = thmts_rx_content.rmark;
		}

		//进行调整时钟偏差
		uint64_t delta_ts = timestamp_minus(node.ts_curr_rmark, node.ts_into_timer);

		delta_ts = timestamp_minus( delta_ts, 55 * TICK_PER_10US );
		node.arr_adjust = ( (delta_ts >> 6) >> 3 ) >> 4;

    	thmts_rx_content.node_adjust = node.arr_adjust;

		if( (node.arr_adjust_flag == 3) && (node.role == NODE_ROLE_ANCHOR_SLAVE_NORMAL) && (node.arr_adjust >= 40 || node.arr_adjust <= -40))
		{
			//这种情况判定为rx提取错误，不进行调整
			//存在如下问题：当节点长时间丢失之后，其误差超过这个门限，这时候会丢失信号
			if((node.arr_adjust <= 1000 && node.arr_adjust >= -1000))
			{
				node.arr_adjust_flag = 4;
				adjust_tick_period = adjust_tick_cnt;
				adjust_tick_cnt = 0;
			}

			// 正常工作状态情况下，对于从节点，如果误差超过10us，则需要进行一次ARR调整
			// 状态设置为4 ， 进入调整
		}

		if ( node.state == NODE_STATE_CAPTURE)  // 搜索状态
		{
			node.arr_adjust_flag = 1; //下一次进入调钟周期
		}

		if (node.state == NODE_STATE_RUNNING)
		{
			if(thmts_rx_frame.head.slot_id == 0)
			{
				pps_adjust.master_pps_time_last = thmts_rx_frame.master_pps_time;
				pps_adjust.tx_time_record = thmts_rx_frame.head.tx_stamp;
				pps_adjust.rx_time_record=node.ts_curr_rmark;
			}
			//进行时间戳收集与处理
			switch (thmts_rx_frame.head.subfrm_id)
			{
				case 0x00:
				{
					node.trx_stamps_subfrm0_ping[thmts_rx_frame.head.slot_id][thmts_rx_frame.head.slot_id] = thmts_rx_frame.head.tx_stamp;
					node.trx_stamps_subfrm0_ping[node.dev_id][thmts_rx_frame.head.slot_id] = node.ts_curr_rmark;
					if(node.dev_id==0)
					{
						node.trx_stamps_subfrm0_ping[thmts_rx_frame.head.slot_id][node.dev_id] = thmts_rx_frame.rx_stamp;	
					}
					thmts_tx_frame_rx_stamp[0] = node.ts_curr_rmark;
					break;
				}
				case 0x01:
				{
					if(node.dev_id==1)
					{
						node.trx_stamps_subfrm0_ping[thmts_rx_frame.head.slot_id][node.dev_id] = thmts_rx_frame.rx_stamp;
					}
					if(node.dev_id==0)
					{
						node.trx_stamps_subfrm1_ping[thmts_rx_frame.head.slot_id][node.dev_id] = thmts_rx_frame.rx_stamp;	
					}
					node.trx_stamps_subfrm1_ping[thmts_rx_frame.head.slot_id][thmts_rx_frame.head.slot_id] = thmts_rx_frame.head.tx_stamp;
					node.trx_stamps_subfrm1_ping[node.dev_id][thmts_rx_frame.head.slot_id] = node.ts_curr_rmark;
					thmts_tx_frame_rx_stamp[1] = node.ts_curr_rmark;
					break;
				}
				case 0x02:
				{
					if(node.dev_id==1)
					{
						node.trx_stamps_subfrm1_ping[thmts_rx_frame.head.slot_id][node.dev_id] = thmts_rx_frame.rx_stamp;
					}
					if(node.dev_id==0)
					{
						node.trx_stamps_subfrm2_ping[thmts_rx_frame.head.slot_id][node.dev_id] = thmts_rx_frame.rx_stamp;
					}
					node.trx_stamps_subfrm2_ping[thmts_rx_frame.head.slot_id][thmts_rx_frame.head.slot_id] = thmts_rx_frame.head.tx_stamp;
					node.trx_stamps_subfrm2_ping[node.dev_id][thmts_rx_frame.head.slot_id] = node.ts_curr_rmark;
					thmts_tx_frame_rx_stamp[2] = node.ts_curr_rmark;
					break;
				}
				default:
					break;
			}


			//采集CIR，处理PDOA数据
			int subfrm_idx = thmts_rx_frame.head.subfrm_id;
			if(subfrm_idx >= 0 && subfrm_idx <= 2)
			{
			    collect_cir_from(channel0_CIR, CIR_data0, 0,firstPeakIdx);
			    collect_cir_from(channel1_CIR, CIR_data1, 0,firstPeakIdx);
			    collect_cir_from(channel2_CIR, CIR_data2, 0,firstPeakIdx);
			}


			node.user_data_rx_valid = 1;
			memcpy(node.user_data_rx, thmts_rx_frame.user_data,USER_DATA_LENGTH * sizeof(node.user_data[0]));

			// 用于slave储存多组时间戳矫正clk drift
			if (node.role == NODE_ROLE_ANCHOR_SLAVE_NORMAL)
			{
				pps_adjust.rx_time_set[pps_adjust.tr_cnt] = node.ts_curr_rmark;
				pps_adjust.tx_time_set[pps_adjust.tr_cnt] = thmts_rx_frame.head.tx_stamp;
				if (pps_adjust.tr_cnt == (PPS_TR_CHECK_CNT-1))
				{
					pps_clk_compute(&pps_adjust);
					pps_adjust.tr_cnt = 0;
				}
				else
				{
					pps_adjust.tr_cnt++;
				}
			}
			pps_adjust.rx_time_set[pps_adjust.tr_cnt] = node.ts_curr_rmark;
		}



	}
	else
	{
		uint32_t testCnt = 0;

		uint16_t firsrERRORpos = 0;
		for(uint16_t index = 0 ; index < sizeof(thmts_rx_frame.rx_stamp) ; index++)
		{
			if(thmts_rx_frame.rx_stamp != (uint8_t)(index & 0xFF))
			{
				firsrERRORpos = index;
				break;
			}


		}
//    	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "first CRC err pos =%d\r\n",
//    			firsrERRORpos);
	}


//            // rx complete，接收模块关机
//            BB_RX_MODULE_POWER_DOWN;

	//打印接收数据

	uint32_t rx_status = thmts_rx_content.rx_status;
	uint16_t CIR_accum_cnt= (rx_status>>16)&0xFFFF;
	uint16_t AGC_num = (rx_status>>8)&0xFF;


    if(reset_cnt == 1)
    {
    	rx_cnt = 0;
    	rx_timeout_cnt = 0;
    	rx_phr_error_cnt = 0;
    	rx_ok_cnt = 0;
    	crc_error_cnt = 0;
    	crc_ok_cnt = 0;

    	reset_cnt = 0;

    }
    else
    {
    	if(rx_flag) //只在0和1进行收发
    	{

    		rx_cnt++;
    		AGC_result[AGC_idx]=AGC_num;
    		AGC_idx++;

			if((rx_status & 0xFF) == 1)
				rx_timeout_cnt++;
			else if((rx_status & 0xFF) == 15)
				rx_phr_error_cnt++;
			else if((rx_status & 0xFF) == 4)
			{
				rx_ok_cnt++;
				if(thmts_rx_content.CRC_OK == 1)
					crc_ok_cnt++;
			}
    	}



//            	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "rx_status = %d\r\n",
//            			(rx_status & 0xFF));

//    	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "node_adjust = %d , AGC_num = %d\r\n",
//    			node.arr_adjust,
//				AGC_num);
////
////
//    	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "adjust_tick_period = %d Tick , adjust_tick_cnt = %d\r\n",
//    			adjust_tick_period,
//				adjust_tick_cnt);





//    	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "total rx = %d, ok = %d , timeout = %d , phr error = %d , crc error = %d , rx_subfrm_id = %d ,rx_slot_id = %d, node.state = %d\r\n",
//    			rx_cnt,
//				crc_ok_cnt,
//				rx_timeout_cnt,
//				rx_phr_error_cnt,
//				rx_ok_cnt - crc_ok_cnt,
//				node.curr_subfrm_idx,
//				node.curr_slot_idx,
//				node.state
//				);

//		uint32_t timeStamp_hi,timeStamp_lo;
//		timeStamp_hi = (uint32_t)(thmts_tx_frame.head.tx_stamp >> 32);
//		timeStamp_lo = (uint32_t)(thmts_tx_frame.head.tx_stamp);
//		txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "tx_stamp = %3u , %10u |\r\n" , timeStamp_hi , timeStamp_lo);

    	uint32_t BBCTRL = thmts_rx_content.BBCTRL;
    	uint32_t PHR_info = thmts_rx_content.PHR_info;

    	PHR_info_RX = read_thmts_bb_reg(CHIP_THURDZ_RX_DATA_ADDR);


//    	txPoint_buff += sprintf((uint8_t *)&debug_ranging_buf[txPoint_buff], "PHR = %8x , PHR_info_RX = %8x\r\n",
//				PHR_info,
//				PHR_info_RX);




    }






    // 上报接收消息
    if (SWITCH_THMTS_RX_SLOT_INFO_T)
	{
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		thmts_rx_slot_info_t thmts_rx_slot_info;
		memcpy(&thmts_rx_slot_info,thmts_rx_content.rx_info,sizeof(thmts_rx_content.rx_info));
		thmts_rx_slot_info.rmark= node.ts_curr_rmark;
		thmts_rx_slot_info.dev_id = node.dev_id;
		thmts_rx_slot_info.slot_id = slot_cnt;
		thmts_rx_slot_info.rx_length = thmts_rx_content.rx_length;
		thmts_rx_slot_info.rx_data = thmts_rx_frame;
		thmts_rx_slot_info.crc_res = 0;
		thmts_rx_slot_info.rx_status = rx_status;

		thmts_rx_slot_info.rx_cnt = rx_cnt;
		thmts_rx_slot_info.crc_ok_cnt = crc_ok_cnt;
		thmts_rx_slot_info.rx_timeout_cnt = rx_timeout_cnt;
		thmts_rx_slot_info.rx_phr_error_cnt = rx_phr_error_cnt;
		thmts_rx_slot_info.crc_error_cnt = rx_ok_cnt - crc_ok_cnt;

		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = TYPE_THMTS_RX_SLOT_INFO_T;
		ptMsg->msg_status  = 1;
		ptMsg->msg_ptr     = &thmts_rx_slot_info;
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);


	}


	// rx complete，接收模块关机
	BB_RX_MODULE_POWER_DOWN;


	if(txPoint_buff > 0 )
	{
		ptMsg->msg_length  = txPoint_buff;
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
	}
	else
	{
		vPortFree(ptMsg);
	}




}

void processPPS()
{
	pps_update = 0;
	if(node.role==NODE_ROLE_ANCHOR_MASTER)
	{
		if(First_start_pps==1)
		{
			get_thmts_bb_systime( &pps_start_time );
			pps_start_time = timestamp_add(pps_start_time,PPS_DELAY_IN_US * CHIP_THURDZ_TICK_PER_SEC / 1e6);
			pps_start_time = pps_start_time >> 9;
			write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
			First_start_pps=0;
		}
		else
		{
			pps_start_time = pps_start_time <<9;
			pps_start_time = timestamp_add(pps_start_time,PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3);
			pps_start_time = pps_start_time >> 9;
			write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
		}
		pps_adjust.master_pps_time_last=pps_start_time;
	}
	else if(node.role==NODE_ROLE_ANCHOR_SLAVE_NORMAL)
	{
		master_pps_time_next = pps_adjust.master_pps_time_last <<9;
		master_pps_time_next = timestamp_add(master_pps_time_next,PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3);//Clock of Master is accurate
		delta_tx = timestamp_substract(master_pps_time_next , pps_adjust.tx_time_record);
//		tof_compensate= 259.14 / C_speed * CHIP_THURDZ_TICK_PER_SEC;
		tof_compensate= 256.59 / C_speed * CHIP_THURDZ_TICK_PER_SEC;
		delta_tx = timestamp_substract(delta_tx,tof_compensate); //slave behind
//		pps_adjust.clk_drift_coff = 1;
		delta_s = delta_tx * pps_adjust.clk_drift_coff;
		pps_start_time = timestamp_add( delta_s,  pps_adjust.rx_time_record);
		uint64_t local_time;
		get_thmts_bb_systime( &local_time );
		uint64_t PPS_PERIOD_compensate_int=PPS_CHECK_PERIOD_IN_MS * CHIP_THURDZ_TICK_PER_SEC / 1e3;
		while(timestamp_minus(local_time,pps_start_time)>0) //过了
		{
			pps_start_time = timestamp_add(pps_start_time , PPS_PERIOD_compensate_int);
		}
		pps_start_time = pps_start_time >>9;
		write_thmts_bb_reg( CHIP_THURDZ_TIMER_PPS_START_TIME_ADDR, pps_start_time );
	}
}



uint32_t imu_cnt = 0;

uint16_t prepareTxMsg(thmts_tx_msg_info_t *ptMsgReceived, uint8_t *pTxBuffer)
{

	// 接收结构体指针
	uint8_t msg_type;
	uint16_t tx_len = 0;
	uint16_t j = 0;

	msg_type = ptMsgReceived->msg_type;

	switch(msg_type)
	{
		case 0x41:
		{

			uint8_t *data = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = ptMsgReceived->msg_length;
			memcpy(pTxBuffer , data , tx_len);
//					printf("0x41\r\n");

			break;
		}


		case TYPE_THMTS_TX_SLOT_INFO_T:
		{
			int msg_length = sizeof(thmts_tx_slot_info_t) + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , sizeof(thmts_tx_slot_info_t));

			pTxBuffer[msg_length + 0] = TYPE_THMTS_TX_SLOT_INFO_T;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			break;
		}
		case TYPE_THMTS_RX_SLOT_INFO_T:
		{
			int msg_length = sizeof(thmts_rx_slot_info_t) + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , sizeof(thmts_rx_slot_info_t));

			pTxBuffer[msg_length + 0] = TYPE_THMTS_RX_SLOT_INFO_T;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			break;
		}
		case TYPE_THMTS_RX_CIR_INTO_T:
		{
			int msg_length = sizeof(thmts_rx_CIR_info_t) + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , sizeof(thmts_rx_CIR_info_t));

			pTxBuffer[msg_length + 0] = TYPE_THMTS_RX_CIR_INTO_T;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint32_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			break;
		}
		case TYPE_THMTS_RANGING_INTO_T:
		{
			int msg_length = sizeof(thmts_RANGING_info_t) + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(p_UartTxDma_Buff + 4 , payload , sizeof(thmts_rx_CIR_info_t));

			pTxBuffer[msg_length + 0] = TYPE_THMTS_RANGING_INTO_T;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint32_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			break;
		}

		case TYPE_THMTS_RANGE_AND_DATA_T:
		{
			int msg_length = sizeof(thmts_range_and_data_t) + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , sizeof(thmts_range_and_data_t));

			pTxBuffer[msg_length + 0] = TYPE_THMTS_RANGE_AND_DATA_T;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			break;
		}

		case THMTS_CTRL_START_CONFIG_ACK:
		{
			int msg_length = ptMsgReceived->msg_length + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , ptMsgReceived->msg_length);

			pTxBuffer[msg_length + 0] = THMTS_CTRL_START_CONFIG_CMD;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc
			break;
		}

		case THMTS_CTRL_NODE_CONFIG_ACK:
		{
			int msg_length = ptMsgReceived->msg_length + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , ptMsgReceived->msg_length);

			pTxBuffer[msg_length + 0] = THMTS_CTRL_NODE_CONFIG_ACK;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc
			break;
		}

		case THMTS_CTRL_PHY_CONFIG_ACK:
		{
			int msg_length = ptMsgReceived->msg_length + 4;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4 , payload , ptMsgReceived->msg_length);

			pTxBuffer[msg_length + 0] = THMTS_CTRL_PHY_CONFIG_ACK;
			pTxBuffer[msg_length + 1] = 1;                           //msg_status
			pTxBuffer[msg_length + 2] = 1;                           //msg_cnt

			uint16_t crc = 0;
			for(j = 2 ; j < msg_length +3 ; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc
			break;
		}

		case TYPE_THMTS_SENSOR_IMU_DATA_T:
		{
			int msg_length = sizeof(thmts_sensor_imu_data_t) + 1;
			uint8_t *payload = ((uint8_t *)(ptMsgReceived->msg_ptr));
			tx_len = msg_length + 12;  // +4, +6 for debug
			//tx_len = msg_length + 4;
			pTxBuffer[0] = 0xEB;
			pTxBuffer[1] = 0x90;

			pTxBuffer[2] = (uint8_t)(msg_length&0xFF);
			pTxBuffer[3] = (uint8_t)(msg_length>>8);

			memcpy(pTxBuffer + 4, payload, sizeof(thmts_sensor_imu_data_t));

			uint32_t crc = 0;
			for(j = 2 ; j < msg_length + 3; j++)
				crc += pTxBuffer[j];

			pTxBuffer[msg_length + 3] = (uint8_t)(crc&0xFF);         //crc

			pTxBuffer[msg_length + 4] = 'H';
			pTxBuffer[msg_length + 5] = 'A';
			pTxBuffer[msg_length + 6] = 'H';
			pTxBuffer[msg_length + 7] = 'A';
			sprintf((uint8_t *)&pTxBuffer[msg_length + 8], "%02d", ++imu_cnt);
			pTxBuffer[msg_length + 10] = '\r';  // debug
			pTxBuffer[msg_length + 11] = '\n';  // debug

			if (imu_cnt >= 50)
			{
				imu_cnt = 0;
			}

			break;
		}



		default: break;

	}
	vPortFree(ptMsgReceived);

	return tx_len;

}



void process_received_message(thmts_ctrl_cmd_proc_msg_t *msg_info)
{
	// The primary switch statement to handle different message types.
	switch (msg_info->msg_type)
	{
	case 14:
	{
		uint8_t* data = (uint8_t*)thmts_ctrl_cmd_proc_msg.msg_buf_ptr;

		reset_cnt = 1;
		printf("type = 14 ,data = %d cmd info\r\n" , data[0]);
		break;
	}
	case 0x70:
	{
		uint8_t* data = (uint8_t*)thmts_ctrl_cmd_proc_msg.msg_buf_ptr;

		reset_cnt = 1;
		printf("type = 0x70 rec pdoa cali info\r\n");
		break;
	}


	case 113:
	{
		printf("type = 0x71 , cmd info\r\n");

		uint8_t* data = (uint8_t*)thmts_ctrl_cmd_proc_msg.msg_buf_ptr;
		uint8_t cmd_byte0 = 0x0;
		uint8_t cmd_byte1 = 0x0;

		cmd_byte0 = data[0];
		cmd_byte1 = data[1];

		SWITCH_THMTS_TX_SLOT_INFO_T = cmd_byte1 & 0x1;
		SWITCH_THMTS_RX_SLOT_INFO_T = (cmd_byte1>>1) & 0x1;
		SWITCH_THMTS_RANGING_INFO_T = (cmd_byte1>>2) & 0x1;


		SWITCH_THMTS_RX_CIR_CH0_INFO_T = (cmd_byte1>>4) & 0x1;
		SWITCH_THMTS_RX_CIR_CH1_INFO_T = (cmd_byte1>>5) & 0x1;
		SWITCH_THMTS_RX_CIR_CH2_INFO_T = (cmd_byte1>>6) & 0x1;
		SWITCH_THMTS_RX_CIR_CH3_INFO_T = (cmd_byte1>>7) & 0x1;

		printf("SWITCH_THMTS_TX_SLOT_INFO_T =%d\r\n" , SWITCH_THMTS_TX_SLOT_INFO_T);
		printf("SWITCH_THMTS_RX_SLOT_INFO_T = %d\r\n", SWITCH_THMTS_RX_SLOT_INFO_T);
		printf("cmd byte0 = %d , cmd byte1 = %d\r\n" , cmd_byte0 , cmd_byte1);


		break;
	}
	case THMTS_CTRL_START_CONFIG_CMD:
	{
		printf("receive start config\r\n");

		node.state = -1;
		ack_start=1;
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = THMTS_CTRL_START_CONFIG_ACK;
		ptMsg->msg_status  = 1;
		ptMsg->msg_ptr     = &ack_start;
		ptMsg->msg_length = sizeof(ack_start);
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
		break;
	}
	case THMTS_CTRL_PHY_CONFIG_CMD:
	{
		printf("receive config phy\r\n");

		uint8_t* data = (uint8_t*)thmts_ctrl_cmd_proc_msg.msg_buf_ptr;
		memcpy(&phy_config , data , sizeof(PHY_CONFIG_t));

		write_flash_phy_config();
		read_flash_phy_config();

		phy_config_flag = 1;
		for(uint8_t i = 0; i < sizeof(PHY_CONFIG_t) ; i++)
		{
			if(flash_read_vector[i] != data[i])
			{
				phy_config_flag = 0;
				break;
			}
		}
		if(phy_config_flag == 1)
			printf("phy config cmd OK!\r\n");
		else
			printf("phy config cmd failure!\r\n");



		ack_phy=phy_config_flag;
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = THMTS_CTRL_PHY_CONFIG_ACK;
		ptMsg->msg_status  = 1;
		ptMsg->msg_ptr     = &ack_phy;
		ptMsg->msg_length = sizeof(ack_phy);
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
		break;
	}
	case THMTS_CTRL_NODE_CONFIG_CMD: // 接收配置node的指令
	{
		printf("receive config node\r\n");

		uint8_t* data = (uint8_t*)thmts_ctrl_cmd_proc_msg.msg_buf_ptr;
		memcpy(&node_config , data , sizeof(NODE_CONFIG_t));

		write_flash_node_config();
		read_flash_node_config();

		node_config_flag = 1;
		for(uint8_t i = 0; i < sizeof(NODE_CONFIG_t) ; i++)
		{
			if(flash_read_vector[i] != data[i])
			{
				node_config_flag = 0;
				break;
			}
		}
		if(node_config_flag == 1)
			printf("node config cmd OK!\r\n");
		else
			printf("node config cmd failure!\r\n");



		ack_node=node_config_flag;
		thmts_tx_msg_info_t *ptMsg = pvPortMalloc(sizeof(thmts_tx_msg_info_t));
		ptMsg->msg_id      = 1;
		ptMsg->msg_type    = THMTS_CTRL_NODE_CONFIG_ACK;
		ptMsg->msg_status  = 1;
		ptMsg->msg_ptr     = &ack_node;
		ptMsg->msg_length = sizeof(ack_node);
		xQueueSend(xQueue, &ptMsg, portMAX_DELAY);
		break;

	}
	default: break;

	}
}
