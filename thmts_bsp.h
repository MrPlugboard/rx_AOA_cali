#include "ns_sdk_hal.h"
#include "thmts_bb_config.h"
#include "thmts_rf_config.h"
#include "flash.h"
#include "thmts_node_info.h"

extern void BASIC_TIMER0_IRQHandler(void);
extern void THURDZ_BB_IRQHandler(void);
extern void UDMA_IRQHandler(void);
extern void LGPIO0_IMU_IRQHandler(void);

extern THMTS_PhyParamConfig_t thmts_phycfg;

// dma发送数据 buff
extern uint8_t *p_UartTxDma_Buff;
// dma接收数据 buff
extern uint8_t *p_UartRxDma_Buff;


#define	TIMER_PRESCALER 	16
extern uint16_t TIMER_AAR;         // 决定每个TICK的长度
extern uint16_t TickPerSlot;       // 每个Slot占几个TICK，可配值的变量
extern uint16_t MAC_SCHEME_ID;

#define SPI2_CS_ON  (QSPI_CSModeConfig(QSPI2, QSPI_CSMODE_MODE_HOLD))
#define SPI2_CS_OFF (QSPI_CSModeConfig(QSPI2, QSPI_CSMODE_MODE_OFF))

void iomux_config( void );

void TIMER_Config( void );
void SPI1_Config( void );
void SPI2_Config(void);
void USART1_config(void);
void SPI2_Dma_Enable(QSPI_TypeDef* QSPIx, uint32_t CrDma, uint32_t TxDma, uint32_t RxDma);

void DebugUart_Config( void );



#define UART_TX_DMA_BUFF_SIZE 4096+64
//#define UART_RX_DMA_BUFF_SIZE 512
#define UART_RX_DMA_BUFF_SIZE 4096

void UDMA_USART1_TX_Init( void );
void UDMA_USART1_RX_Init( void );
void UDMA_USART1_TX(uint32_t size, uint8_t *addr);
void UDMA_USART1_RX( uint8_t *addr );
void UDMA_QSPI2_TX(uint32_t size, uint16_t *addr);
void UDMA_QSPI2_RX(uint32_t size, uint16_t *addr);



uint32_t DMA_Get_Counter(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx);

void DMA_Stop(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx);
void DMA_Start(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx);

void UDMA_QSPI2_TX_Stop();
void UDMA_QSPI2_RX_Stop();


void prvSetupHardware(void);
