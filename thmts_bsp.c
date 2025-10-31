#include "thmts_bsp.h"
#include "ns_conf.h"

uint16_t TIMER_AAR;         // 决定每个TICK的长度
uint16_t TickPerSlot;       // 每个Slot占几个TICK，可配值的变量
uint16_t MAC_SCHEME_ID;
subfrm_desc_t subfrm_descs[5] = {
    { .total_slot_num = 4, .slot_len_50us = 30, .subfrm_type = STAMP_FRAME }, // 1.5ms = 30*50us
    { .total_slot_num = 4, .slot_len_50us = 30, .subfrm_type = STAMP_FRAME },
    { .total_slot_num = 4, .slot_len_50us = 30, .subfrm_type = STAMP_FRAME },
    { .total_slot_num = 4, .slot_len_50us = 30, .subfrm_type = IDLE_FRAME },
    { .total_slot_num = 4, .slot_len_50us = 30, .subfrm_type = IDLE_FRAME }
};
supfrm_desc_t supfrm_desc = {
    .total_frm_num = 1,
    .total_subfrm_num = 5,
    .p_subfrm_descs = subfrm_descs
};

void DebugUart_Config( void )
{
    USART_InitTypeDef usart_init_t;
    USART_StructInit(&usart_init_t);
#if defined(USART_SETUP_BIT_LENGTH_8)
    usart_init_t.USART_BaudRate = 115200;
    usart_init_t.USART_WordLength = USART_SETUP_BIT_LENGTH_8;
    usart_init_t.USART_StopBits = USART_TXCTRL_CFG_STOP_BIT_1BIT;
    usart_init_t.USART_TX_CTL = USART_TXCTRL_TXEN_ENABLE;
    usart_init_t.USART_RX_CTL = USART_RXCTRL_RXEN_ENABLE;
    usart_init_t.USART_Parity = USART_SETUP_PARITY_EN_DISABLE;
    usart_init_t.USART_HardwareFlowControl = USART_SETUP_RTSE_DISABLE |USART_SETUP_CTSE_DISABLE;
#else
    usart_init_t.USART_BaudRate = 115200;
    usart_init_t.USART_WordLength = USART_WordLength_8b;
    usart_init_t.USART_StopBits = USART_STOP_BIT_1;
    usart_init_t.USART_TX_CTL = USART_TXCTRL_ENABLE;
    usart_init_t.USART_RX_CTL = USART_RXCTRL_ENABLE;
    usart_init_t.USART_Parity = USART_PARITY_DISABLE;
    usart_init_t.USART_HardwareFlowControl = USART_HardwareFlowControl_NONE;
#endif
    USART_Init(SOC_DEBUG_UART, &usart_init_t);
}

void SPI1_Config(void)
{
    QSPI_InitTypeDef spi_init_struct;
    /* deinitilize SPI and the parameters */
    QSPI_StructInit(&spi_init_struct);
    /* QSPI1 parameter config */  /*Adjust the init sequence to prevent the "x" state */
    spi_init_struct.ProtolMode = QSPI_FMT_PROTO_SINGLE;
    spi_init_struct.DataSize = QSPI_FMT_LEN_32B;
    spi_init_struct.CPOL = QSPI_SCKMODE_CPOL_LOW;
    spi_init_struct.CPHA = QSPI_SCKMODE_CPHA_EDGE1;
    spi_init_struct.SCKDIV = QSPI_SCKDIV_PRESCALER_64;
    spi_init_struct.CSMode = QSPI_CSMODE_MODE_AUTO;
    spi_init_struct.Endian = QSPI_FMT_ENDIAN_MSB;
    spi_init_struct.SSM = QSPI_CR_SSM_HARD;
    spi_init_struct.DevMode = QSPI_CR_MODE_MASTER;
    QSPI_Init(QSPI1, &spi_init_struct);
}


void SPI2_Config(void)
{
    QSPI_InitTypeDef spi_init_struct;
    /* deinitilize SPI and the parameters */
    QSPI_StructInit(&spi_init_struct);
    /* QSPI2 parameter config */  /*Adjust the init sequence to prevent the "x" state */
    spi_init_struct.ProtolMode = QSPI_FMT_PROTO_SINGLE;
    spi_init_struct.DataSize = QSPI_FMT_LEN_16B;
    spi_init_struct.CPOL = QSPI_SCKMODE_CPOL_HIGH;
    spi_init_struct.CPHA = QSPI_SCKMODE_CPHA_EDGE2;
    spi_init_struct.SCKDIV = QSPI_SCKDIV_PRESCALER_64;
    spi_init_struct.CSMode = QSPI_CSMODE_MODE_AUTO;
    spi_init_struct.Endian = QSPI_FMT_ENDIAN_MSB;
    spi_init_struct.SSM = QSPI_CR_SSM_HARD;
    spi_init_struct.DevMode = QSPI_CR_MODE_MASTER; // | QSPI_CR_DMA_ENABLE;
    // spi_init_struct.Tx_DMA = QSPI_CR_DMA_TX_ENABLE;
    // spi_init_struct.Rx_DMA = QSPI_CR_DMA_RX_ENABLE;
    QSPI_Init(QSPI2, &spi_init_struct);
}

void SPI2_Dma_Enable(QSPI_TypeDef* QSPIx, uint32_t CrDma, uint32_t TxDma, uint32_t RxDma)
{
    /* Check the parameters */
    assert_param(IS_QSPI_ALL_PERIPH(QSPIx));
    assert_param(IS_QSPI_SPI_CR_DMA(CrDma));
    assert_param(IS_QSPI_SPI_CR_DMA_TX(TxDma));
    assert_param(IS_QSPI_SPI_CR_DMA_RX(RxDma));

    uint32_t reg_cr = 0U;

    /* Configure QSPIxMSTR bits and SSM bits according to  DevMode and  SSM value*/
    reg_cr = QSPIx->CR;
    reg_cr &= ~ (BITS(0,6));
    reg_cr |= ((QSPI_CR_MODE_MASTER | CrDma) | QSPI_CR_SSM_HARD | QSPI_CR_CSOE_ENABLE | QSPI_CR_CSI_OFF);
    reg_cr |= (TxDma | RxDma);
    QSPIx->CR = reg_cr;
}

/**
  * \brief configure the TIMER peripheral
  */
void TIMER_Config(void)
{
    Basic_TIMER_Init_TypeDef timer_initpara;

    /* initialize TIMER init parameter struct */
    Basic_Timer_StructParaInit(&timer_initpara);
    /* BASIC_TIMER0 configuration */

    timer_initpara.prescaler = TIMER_PRESCALER - 1;
    timer_initpara.period = TIMER_AAR- 1;

    Basic_Timer_Init(BASIC_TIMER0, &timer_initpara);

    Basic_Timer_InterruptEnable(BASIC_TIMER0, BASIC_TIMER_DIER_UIE);
    Basic_Timer_Enable(BASIC_TIMER0);
}


void iomux_config(void)
{
	// for QSPI1
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_SCK_IOF_OVAL, QSPI1_SCK_PAD_SEL, QSPI1_SCK_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_CS_0_IOF_OVAL, QSPI1_CS_0_PAD_SEL, QSPI1_CS_0_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI1_DQ_0_IOF_IVAL, QSPI1_DQ_0_PAD_SEL, QSPI1_DQ_0_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_DQ_0_IOF_OVAL, QSPI1_DQ_0_PAD_SEL, QSPI1_DQ_0_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI1_DQ_1_IOF_IVAL, QSPI1_DQ_1_PAD_SEL, QSPI1_DQ_1_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_DQ_1_IOF_OVAL, QSPI1_DQ_1_PAD_SEL, QSPI1_DQ_1_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI1_DQ_2_IOF_IVAL, QSPI1_DQ_2_PAD_SEL, QSPI1_DQ_2_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_DQ_2_IOF_OVAL, QSPI1_DQ_2_PAD_SEL, QSPI1_DQ_2_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI1_DQ_3_IOF_IVAL, QSPI1_DQ_3_PAD_SEL, QSPI1_DQ_3_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI1_DQ_3_IOF_OVAL, QSPI1_DQ_3_PAD_SEL, QSPI1_DQ_3_HS_SEL, 0, 0);

    // for QSPI2
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_SCK_IOF_OVAL, QSPI2_SCK_PAD_SEL, QSPI2_SCK_HS_SEL, 0, 0);
    // iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI2_CS_0_IOF_IVAL, QSPI2_CS_0_PAD_SEL, QSPI2_CS_0_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_CS_0_IOF_OVAL, QSPI2_CS_0_PAD_SEL, QSPI2_CS_0_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI2_DQ_0_IOF_IVAL, QSPI2_DQ_0_PAD_SEL, QSPI2_DQ_0_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_DQ_0_IOF_OVAL, QSPI2_DQ_0_PAD_SEL, QSPI2_DQ_0_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI2_DQ_1_IOF_IVAL, QSPI2_DQ_1_PAD_SEL, QSPI2_DQ_1_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_DQ_1_IOF_OVAL, QSPI2_DQ_1_PAD_SEL, QSPI2_DQ_1_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI2_DQ_2_IOF_IVAL, QSPI2_DQ_2_PAD_SEL, QSPI2_DQ_2_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_DQ_2_IOF_OVAL, QSPI2_DQ_2_PAD_SEL, QSPI2_DQ_2_HS_SEL, 0, 0);

    iomux_ls_iof_ival_cfg(IOMUX_BASE, QSPI2_DQ_3_IOF_IVAL, QSPI2_DQ_3_PAD_SEL, QSPI2_DQ_3_HS_SEL, 0, 0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE, QSPI2_DQ_3_IOF_OVAL, QSPI2_DQ_3_PAD_SEL, QSPI2_DQ_3_HS_SEL, 0, 0);

    // for spi_cs of BMM
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_1_IOF_IVAL,LGPIO0_IO_PORT_PINS_1_PAD_SEL,LGPIO0_IO_PORT_PINS_1_HS_SEL,0,0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_1_IOF_OVAL,LGPIO0_IO_PORT_PINS_1_PAD_SEL,LGPIO0_IO_PORT_PINS_1_HS_SEL,0,0);

    // for BMM DRY
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_2_IOF_IVAL,LGPIO0_IO_PORT_PINS_2_PAD_SEL,LGPIO0_IO_PORT_PINS_2_HS_SEL,0,0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_2_IOF_OVAL,LGPIO0_IO_PORT_PINS_2_PAD_SEL,LGPIO0_IO_PORT_PINS_2_HS_SEL,0,0);

    // for spi_cs of BMP
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_4_IOF_IVAL,LGPIO0_IO_PORT_PINS_4_PAD_SEL,LGPIO0_IO_PORT_PINS_4_HS_SEL,0,0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_4_IOF_OVAL,LGPIO0_IO_PORT_PINS_4_PAD_SEL,LGPIO0_IO_PORT_PINS_4_HS_SEL,0,0);

    // for spi_cs of IMU
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_5_IOF_IVAL,LGPIO0_IO_PORT_PINS_5_PAD_SEL,LGPIO0_IO_PORT_PINS_5_HS_SEL,0,0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_5_IOF_OVAL,LGPIO0_IO_PORT_PINS_5_PAD_SEL,LGPIO0_IO_PORT_PINS_5_HS_SEL,0,0);

    // for IMU RST
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_7_IOF_IVAL,LGPIO0_IO_PORT_PINS_7_PAD_SEL,LGPIO0_IO_PORT_PINS_7_HS_SEL,0,0);
    iomux_ls_iof_oval_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_7_IOF_OVAL,LGPIO0_IO_PORT_PINS_7_PAD_SEL,LGPIO0_IO_PORT_PINS_7_HS_SEL,0,0);

    // for IMU DRDY
    iomux_ls_iof_ival_cfg(IOMUX_BASE,LGPIO0_IO_PORT_PINS_6_IOF_IVAL,LGPIO0_IO_PORT_PINS_6_PAD_SEL,LGPIO0_IO_PORT_PINS_6_HS_SEL,0,0);


    // for USART1
    iomux_ls_iof_oval_cfg(IOMUX_BASE,USART1_TX_IOF_OVAL,USART1_TX_PAD_SEL,USART1_TX_HS_SEL,0,0);
    iomux_ls_iof_ival_cfg(IOMUX_BASE,USART1_RX_IOF_IVAL,USART1_RX_PAD_SEL,USART1_RX_HS_SEL,0,0);
}





void USART1_config(void){
    USART_InitTypeDef usart_init_t = {0};
    USART_StructInit(&usart_init_t);
    usart_init_t.USART_BaudRate = 921600;//921600;//115200;
    usart_init_t.USART_WordLength = USART_SETUP_BIT_LENGTH_8;
    usart_init_t.USART_StopBits = USART_TXCTRL_CFG_STOP_BIT_1BIT;
    usart_init_t.USART_TX_CTL = USART_TXCTRL_TXEN_ENABLE;   // 无论是接收还是发送都可以保证在DMA传输完成前结束
    usart_init_t.USART_RX_CTL = USART_RXCTRL_RXEN_ENABLE;   //
    usart_init_t.USART_Parity = USART_SETUP_PARITY_EN_DISABLE;
    usart_init_t.USART_HardwareFlowControl = USART_SETUP_RTSE_DISABLE | USART_SETUP_CTSE_DISABLE;
    usart_init_t.USART_RX_IDLE_Timeout = 0xFF;
    usart_init_t.USART_RX_WM_Timeout = 0xFF;
    usart_init_t.USART_TX_Data_Size = USART_TX_DATASIZE_NUM_BYTE;
    usart_init_t.USART_RX_Data_Size = USART_RX_DATASIZE_NUM_BYTE;
    usart_init_t.USART_DMA_EN = USART_SETUP_DMA_EN_ENABLE;
    usart_init_t.USART_RX_DMA_EN = USART_SETUP_RX_DMA_EN_ENABLE;
    USART_Init(USART1, &usart_init_t);
}


uint8_t debugComTxBuf1[1024] = {0};

uint8_t receive_data[512];
uint8_t data[512];

UDMA_PAM2MTypeDef UDMA_PAM2MStruct = {0};

UDMA_PAM2MTypeDef UDMA_PAM2MStruct_QSPI2_Tx = {0};
UDMA_PAM2MTypeDef UDMA_PAM2MStruct_QSPI2_Rx = {0};



//void UDMA_USART1_TX_Init( void )
//{
//    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_ENABLE;
//    UDMA_PAM2MStruct.UDMA_DstBaseAddr = ADDR32(&(USART1->TXDATA));
//    UDMA_PAM2MStruct.UDMA_SrcBaseAddr = ADDR32(debugComTxBuf1);
//    UDMA_PAM2MStruct.UDMA_BufferSize = UART_TX_DMA_BUFF_SIZE;
//    UDMA_PAM2MStruct.UDMA_DstInc = PA2M_MDNA_DISABLE;
//    UDMA_PAM2MStruct.UDMA_SrcInc = PA2M_MSNA_ENABLE;
//    UDMA_PAM2MStruct.UDMA_Width = PA2M_MDWIDTH_8BIT;
//    UDMA_PAM2MStruct.UDMA_Mode = PA2M_MODE_NORMAL;
//    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_TX;
//    UDMA_PAM2M_Init(USART1_USART_DMA_TX_DMA_CH, &UDMA_PAM2MStruct);
//}

void UDMA_USART1_TX(uint32_t size, uint8_t *addr){
    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct.UDMA_DstBaseAddr = ADDR32(&(USART1->TXDATA));
    UDMA_PAM2MStruct.UDMA_SrcBaseAddr = ADDR32(addr);
    UDMA_PAM2MStruct.UDMA_BufferSize = size;
    UDMA_PAM2MStruct.UDMA_DstInc = PA2M_MDNA_DISABLE;
    UDMA_PAM2MStruct.UDMA_SrcInc = PA2M_MSNA_ENABLE;
    UDMA_PAM2MStruct.UDMA_Width = PA2M_MDWIDTH_8BIT;
    UDMA_PAM2MStruct.UDMA_Mode = PA2M_MODE_NORMAL;
    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_TX;
    UDMA_PAM2M_Init(USART1_USART_DMA_TX_DMA_CH, &UDMA_PAM2MStruct);
}

void UDMA_USART1_RX_Init( void )
{
    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct.UDMA_DstBaseAddr = ADDR32(receive_data);
    UDMA_PAM2MStruct.UDMA_SrcBaseAddr = ADDR32(&(USART1->RXDATA));
    UDMA_PAM2MStruct.UDMA_BufferSize = UART_RX_DMA_BUFF_SIZE;
    UDMA_PAM2MStruct.UDMA_DstInc = PA2M_MDNA_ENABLE;
    UDMA_PAM2MStruct.UDMA_SrcInc = PA2M_MSNA_DISABLE;
    UDMA_PAM2MStruct.UDMA_Width = PA2M_MDWIDTH_8BIT;
    UDMA_PAM2MStruct.UDMA_Mode = PA2M_MODE_NORMAL;
    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_RX;
    UDMA_PAM2M_Init(USART1_USART_DMA_RX_DMA_CH, &UDMA_PAM2MStruct);
}

void UDMA_USART1_RX( uint8_t *addr )
{
    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct.UDMA_DstBaseAddr = ADDR32(addr);
    UDMA_PAM2MStruct.UDMA_SrcBaseAddr = ADDR32(&(USART1->RXDATA));
    UDMA_PAM2MStruct.UDMA_BufferSize = UART_RX_DMA_BUFF_SIZE;
    UDMA_PAM2MStruct.UDMA_DstInc = PA2M_MDNA_ENABLE;
    UDMA_PAM2MStruct.UDMA_SrcInc = PA2M_MSNA_DISABLE;
    UDMA_PAM2MStruct.UDMA_Width = PA2M_MDWIDTH_8BIT;
    UDMA_PAM2MStruct.UDMA_Mode = PA2M_MODE_NORMAL;
    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_RX;
    UDMA_PAM2M_Init(USART1_USART_DMA_RX_DMA_CH, &UDMA_PAM2MStruct);
}


void DMA_Stop(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx)
{
    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_DISABLE;
    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_RX;
    UDMA_PAM2M_Init(UDMAy_CHannelx, &UDMA_PAM2MStruct);
}

void DMA_Start(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx)
{
    UDMA_PAM2MStruct.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct.UDMA_PER_SEL = UDMA_SEL_USART1_USART_DMA_RX;
    UDMA_PAM2M_Init(UDMAy_CHannelx, &UDMA_PAM2MStruct);
}





uint32_t DMA_Get_Counter(UDMA_PA_CHxCfg_TypeDef* UDMAy_CHannelx)
{
    return UDMAy_CHannelx->MSIZE;
}



void UDMA_QSPI2_TX(uint32_t size, uint16_t *addr)
{
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_DstBaseAddr = ADDR32(&(QSPI2->TXDATA));
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_SrcBaseAddr = ADDR32(addr);
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_BufferSize = size * 2;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_DstInc = PA2M_MDNA_DISABLE;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_SrcInc = PA2M_MSNA_ENABLE;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_Width = PA2M_MDWIDTH_16BIT;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_Mode = PA2M_MODE_NORMAL;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_PER_SEL = UDMA_SEL_QSPI2_TX_DMA;
    UDMA_PAM2M_Init(QSPI2_TX_DMA_DMA_CH, &UDMA_PAM2MStruct_QSPI2_Tx);
}

void UDMA_QSPI2_RX(uint32_t size, uint16_t *addr)
{
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_TransEn = PA2M_TRANS_ENABLE;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_DstBaseAddr = ADDR32(addr);
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_SrcBaseAddr = ADDR32(&(QSPI2->RXDATA));
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_BufferSize = size * 2;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_DstInc = PA2M_MDNA_ENABLE;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_SrcInc = PA2M_MSNA_DISABLE;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_Width = PA2M_MDWIDTH_16BIT;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_Mode = PA2M_MODE_NORMAL;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_PER_SEL = UDMA_SEL_QSPI2_RX_DMA;
    UDMA_PAM2M_Init(QSPI2_RX_DMA_DMA_CH, &UDMA_PAM2MStruct_QSPI2_Rx);
}

void UDMA_QSPI2_TX_Stop()
{
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_TransEn = PA2M_TRANS_DISABLE;
    UDMA_PAM2MStruct_QSPI2_Tx.UDMA_PER_SEL = UDMA_SEL_QSPI2_TX_DMA;
    UDMA_PAM2M_Init(QSPI2_TX_DMA_DMA_CH, &UDMA_PAM2MStruct_QSPI2_Tx);
}

void UDMA_QSPI2_RX_Stop()
{
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_TransEn = PA2M_TRANS_DISABLE;
    UDMA_PAM2MStruct_QSPI2_Rx.UDMA_PER_SEL = UDMA_SEL_QSPI2_RX_DMA;
    UDMA_PAM2M_Init(QSPI2_RX_DMA_DMA_CH, &UDMA_PAM2MStruct_QSPI2_Rx);
}




void prvSetupHardware(void)
{

	iomux_config();

	/* Init QSPI1 for RF & ADC & DAC configuration. */
	#ifdef MISC_HAS_QSPI1_HAS_CLK
	qspi1_clk_en(ENABLE);
	#endif
	#ifdef MISC_HAS_QSPI1_RST
	qspi1_set_rst(DISABLE);
	qspi1_set_rst(ENABLE);
	#endif

	// step1 : 配置clk_pll和lo_pll
	// 检测本次启动时工作时钟频率：重新上电，系统时钟为19.2M；JTAG Debug下重新下载启动，系统时钟为124.8M
	SystemCoreClock = get_cpu_freq();
	DebugUart_Config();
	printf("First config SystemCoreClock is %d Hz \r\n", SystemCoreClock);

	// 根据当前时钟频率为124.8MHz，则按照该频率配置qspi1，然后将时钟配置为19.2M
	if(SystemCoreClock > 2*38400000)
	{
		qspi1_cfg_clk_div( 96 );
		SPI1_Config();
    	config_thmts_rf_clkmode(0, 0); // 切换到19.2M
	}

	SystemCoreClock = get_cpu_freq();
	DebugUart_Config();
	//这个应该一定会配置为19.2M
	printf("SystemCoreClock before RF config = %d Hz\r\n", SystemCoreClock);

	// 按照19.2MHz重新配置SPI
	qspi1_cfg_clk_div( 24 );
    SPI1_Config();


    int delta_SysClock = 100000;
    uint8_t epoch = 0;
    while(delta_SysClock > 24960)  // 200ppm，相对于124.8M
    {
    	epoch++;
    	printf("---------Epoch %d config RF \r\n" , epoch);
        // 进行基于半自动选段的PLL配置，包括LO PLL、CLK PLL和DAC配置
		config_thmts_rf_pll_mfc();

		// 将工作时钟切换到124.8MHz
	    config_thmts_rf_clkmode(0, 1);

		// 测量当前实际的工作时钟频率，并和标称值进行比较
		SystemCoreClock =  get_cpu_freq();
		DebugUart_Config();
		delta_SysClock = abs(SystemCoreClock - 124800000);

		printf("SystemCoreClock after  RF config = %d Hz , delta_SysClock = %d Hz\r\n",
				SystemCoreClock,
				delta_SysClock);

		// 如果小于预设的偏差门限，则使用124.8MHz配置qspi1，然后退出时钟配置过程
		// 否则，则重新进行配置
		if(delta_SysClock <= 24960)  //退出初始化时钟
		{
			qspi1_cfg_clk_div( 96 );
	        SPI1_Config();
	        break;
		}
		else
		{
	    	config_thmts_rf_clkmode(0, 0); // 工作时钟切换到19.2M，clk_pll配置后输出的低频时钟为19.2M
	    	qspi1_cfg_clk_div( 24 );
	        SPI1_Config();
	    }
    }




    // step2: 从flash中读取node、phy和mac的配置参数
    // init flash
	#ifdef MISC_HAS_QSPI_XIP0_HAS_CLK
    qspi_xip0_clk_en(ENABLE);
	#endif
	#ifdef MISC_HAS_QSPI_XIP0_CLK_DIV
    qspi_xip0_clk_div(0);
	#endif
	#ifdef MISC_HAS_QSPI_XIP0_RST
    qspi_xip0_set_rst(DISABLE);
    qspi_xip0_set_rst(ENABLE);
	#endif
	#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT == 1
    DisableDCache();
	#endif

    SPI_Init();
    SPI_ReadJEDEC();


#define TEST_MODE

#ifdef TEST_MODE
    // node配置写入flash
    node_config.role = NODE_ROLE_ANCHOR_MASTER;          //主节点
    node_config.dev_id = 0;
    node_config.group_id = 0;

////    // node配置写入flash
//    node_config.role = NODE_ROLE_ANCHOR_SLAVE_NORMAL;          //从节点
//    node_config.dev_id = 1;
//    node_config.group_id = 0;


    write_flash_node_config();

    // mac配置写入flash
    mac_config.TIMER_AAR=7800;
    mac_config.TickPerSlot=5;
    mac_config.MAC_SCHEME_ID=1;
    write_flash_mac_config();

    // phy配置写入flash
	phy_config.bit_rate = 3;              // bit_rate选择8M
	phy_config.preamble_length = 128;     // 同步头长度为128，暂时不用
	phy_config.freq_grid = 0;             // 频格，暂时不用
//	phy_config.rf_chan_num = 1;           // RF通道配置为1个
//	phy_config.AOA_enable = 0;            // 关闭AOA工作模式

	phy_config.rf_chan_num = 4;           // RF通道配置为3个
	phy_config.AOA_enable = 1;            // 打开AOA工作模式

	phy_config.gain_mode = 1;             // 设置为AGC模式
	phy_config.gain_value = 44;           // 固定增益的数值

    write_flash_phy_config();
    write_flash_mac_config();
    write_flash_node_config();
#endif
    // 从flash中读取配置
    read_flash_node_config();
    read_flash_phy_config();
    read_flash_mac_config();

    //配置node变量
    init_node();

	//配置mac变量
	TIMER_AAR = mac_config.TIMER_AAR;
    TickPerSlot = mac_config.TickPerSlot;
    MAC_SCHEME_ID=1;
    switch (MAC_SCHEME_ID)
    {
    case 1:
    {
        // MAC方案1的配置

        node.p_supfrm_desc = &supfrm_desc;
        break;
    }
    case 2:
        // MAC方案2的配置
        break;
    default:
        break;
    }

	//配置phy变量
	thmts_phycfg.aoa_enable  = phy_config.AOA_enable;
	thmts_phycfg.rf_chan_num = phy_config.rf_chan_num;
    uint32_t case_idx = phy_config.bit_rate;
	case_idx = 0x3;
	thmts_phycfg.preamble_length = 128;
    Set_Channel_Config_By_Case( case_idx );

	// step 3: 配置RF，tx、rx、adc
	config_thmts_rf_allinone(thmts_phycfg.rf_chan_num);

	// step 4: 配置uwb-bb
	apb_slv0_async_fifo_clk_en( ENABLE );
    apb_slv0_clk_div( 0 );

    // init uwb-bb and download firmware
	init_thmts_bb();
   	enable_thmts_bb_pwm(1000000, 65536, 32768);

//	BB_TX_MODULE_POWER_ON;
//	BB_TX_MODULE_POWER_DOWN;
//
//	BB_RX_MODULE_POWER_ON;
//	BB_RX_MODULE_POWER_DOWN;

   	if (node.role == NODE_ROLE_SELFTESTER_IN)
   		ENABLE_BB_LOOPBACK;
   	else
   		DISABLE_BB_LOOPBACK;

	// 关闭发射通道
	RF_TX_POWER_DOWN;
	// 关闭接收通道
	config_thmts_bb_rx_sw_lna_off(4);

	if(phy_config.gain_mode == 1)  //AGC模式
	{
		DAGC_ENABLE;
	}
	else  //固定增益模式
	{
		DAGC_DISABLE;
		SET_FIX_GAIN(phy_config.gain_value);
	}

	// step 5: 配置其它外设
	#ifdef MISC_HAS_USART1_HAS_CLK
	usart1_clk_en(ENABLE);
	#endif
	#ifdef MISC_HAS_USART1_RST
	usart1_set_rst(DISABLE);
	usart1_set_rst(ENABLE);
	#endif
	// config UART1 and DMA
	USART1_config();



    /* Init LGPIO configuration.*/
    #ifdef MISC_HAS_LGPIO0_HAS_CLK
    lgpio0_clk_en(ENABLE);
    #endif
    #ifdef MISC_HAS_LGPIO0_RST
    lgpio0_set_rst(DISABLE);
    lgpio0_set_rst(ENABLE);
    #endif

    /* Init QSPI2 for IMU configuration. */
    #ifdef MISC_HAS_QSPI2_HAS_CLK
    qspi2_clk_en(ENABLE);
    #endif
    #ifdef MISC_HAS_QSPI2_RST
    qspi2_set_rst(DISABLE);
    qspi2_set_rst(ENABLE);
    #endif

    SPI2_Config();

    /* addition cs gpio for bmm, bmp and imu*/
    LGPIO_Mode(LGPIO0, LGPIO_PinSource1, PP);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource1, 1);

    LGPIO_Mode(LGPIO0, LGPIO_PinSource4, PP);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource4, 1);

    LGPIO_Mode(LGPIO0, LGPIO_PinSource5, PP);
    LGPIO_WriteBit(LGPIO0, LGPIO_PinSource5, 1);

    /* rst gpio for imu*/
    LGPIO_Mode(LGPIO0, LGPIO_PinSource7, PP);



	#ifdef MISC_HAS_UDMA0_HAS_CLK
	udma0_clk_en(ENABLE);
	#endif
	#ifdef MISC_HAS_UDMA0_RST
	udma0_set_rst(DISABLE);
	udma0_set_rst(ENABLE);
	#endif
    UDMA_USART1_RX(p_UartRxDma_Buff);

	#ifdef MISC_HAS_BASIC_TIMER0_HAS_CLK
    basic_timer0_clk_en(ENABLE);
	#endif
	#ifdef MISC_HAS_BASIC_TIMER0_RST
    basic_timer0_set_rst(DISABLE);
    basic_timer0_set_rst(ENABLE);
	#endif
    TIMER_Config();

    // step 6 : 使能中断
    // Init ECLIC for event capture.
    __enable_irq();

//    // Init USART0 for data inout.  UART0 is exactly Debug UART.
//	USART_ITConfig( USART0, USART_INT_EN_RXIE, ENABLE );

    ECLIC_Register_IRQ(UDMA0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
        ECLIC_LEVEL_TRIGGER, 1, 1,
        UDMA_IRQHandler);
    UDMA_PA2M_ITConfig(USART1_USART_DMA_TX_DMA_IRQ, PA2M_FTRANS_IRQ_EN, ENABLE);
    UDMA_PA2M_ITConfig(QSPI2_RX_DMA_DMA_IRQ, PA2M_FTRANS_IRQ_EN, ENABLE);

    /* register interrupt BASIC_TIMER0_IRQn */
    ECLIC_Register_IRQ(BASIC_TIMER0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                                    ECLIC_LEVEL_TRIGGER, 1, 0,
                                    BASIC_TIMER0_IRQHandler);

    /* register interrupt THURDZ_BB_IRQn */
	ECLIC_Register_IRQ( THURDZ_BB_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                        ECLIC_POSTIVE_EDGE_TRIGGER, 1, 3,
						THURDZ_BB_IRQHandler );


   /* register interrupt LGPIO0_IMU_IRQHandler */
	ECLIC_Register_IRQ(LGPIO0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
									ECLIC_POSTIVE_EDGE_TRIGGER, 1, 2,
									LGPIO0_IMU_IRQHandler);
	// IMU初始化配置完成后再使能中断，不在这里使能

	return;
}



