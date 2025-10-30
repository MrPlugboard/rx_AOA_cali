#include "flash.h"





/**
  * \brief configure the SPI peripheral
  */
void SPI_Init(void)
{
    QSPI_XIP_InitTypeDef spixip_init_struct = {0};
    /* deinitilize SPI and the parameters */
    QSPI_XIP_StructInit(&spixip_init_struct);

    spixip_init_struct.ProtolMode = QSPI_XIP_FMT_PROTO_SINGLE;
    spixip_init_struct.DataSize = QSPI_XIP_FMT_LEN_8B;
    spixip_init_struct.CPOL = QSPI_XIP_SCKMODE_CPOL_LOW;
    spixip_init_struct.CPHA = QSPI_XIP_SCKMODE_CPHA_EDGE1;
    spixip_init_struct.SCKDIV = QSPI_XIP_SCKDIV_PRESCALER_8;
    spixip_init_struct.Endian = QSPI_XIP_FMT_ENDIAN_MSB;
    spixip_init_struct.SSM = QSPI_XIP_CR_SSM_HARD;
    spixip_init_struct.DevMode = QSPI_XIP_CR_MODE_MASTER;
    spixip_init_struct.Force = (QSPI_XIP_FORCE_EN | QSPI_XIP_FORCE_WP);
    QSPI_XIP_Init(QSPI_XIP0, &spixip_init_struct);
}

void SPI_WriteByte(uint8_t txdata)
{
    QSPI_XIP_DirectionConfig(QSPI_XIP0, QSPI_XIP_FMT_DIR_TX);
    while (SET == QSPI_XIP_GetFlag(QSPI_XIP0, QSPI_XIP_STATUS_TX_FULL)) {}
    QSPI_XIP_SendData(QSPI_XIP0, txdata);
    while (SET == QSPI_XIP_GetFlag(QSPI_XIP0, QSPI_XIP_STATUS_BUSY)) {}
}

uint8_t SPI_ReadByte(void)
{
    uint8_t rxdata = 0;
    QSPI_XIP_DirectionConfig(QSPI_XIP0, QSPI_XIP_FMT_DIR_RX);
    QSPI_XIP_SendData(QSPI_XIP0, Dummy_Byte);
    while (SET == QSPI_XIP_GetFlag(QSPI_XIP0, QSPI_XIP_STATUS_RX_EMPTY)) {}
    rxdata = QSPI_XIP_ReceiveData(QSPI_XIP0) & 0xFF;
    return rxdata;
}

void SPI_ReadJEDEC(void)
{
    SPI_CS_ON;
    SPI_WriteByte(FLASH_JedecDeviceID);
    for (int i = 0; i < 3; i++) {
        printf("JEDEC[%d]:%#x\n", i, SPI_ReadByte());
    }
    SPI_CS_OFF;
}

void SPI_Wip(void)
{
    uint8_t temp = 0;
    SPI_CS_ON;
    SPI_WriteByte(FLASH_ReadStatusReg1);
    do {
        temp = SPI_ReadByte() & WIP_Flag;
    } while (temp);
    SPI_CS_OFF;
}

void SPI_WriteEnable(void)
{
    SPI_CS_ON;
    SPI_WriteByte(FLASH_WriteEnable);
    SPI_CS_OFF;
}

void SPI_Quad(void)
{
    uint32_t value = QSPI_XIP0->FMT;
    value &= ~QSPI_XIP_FMT_PROTO_MASK;
    QSPI_XIP0->FMT = value | QSPI_XIP_FMT_PROTO_SINGLE;
    SPI_WriteEnable();
    SPI_CS_ON;
    SPI_WriteByte(FLASH_WriteStatusReg2);
    SPI_WriteByte(0x1 << 1);
    SPI_CS_OFF;
    SPI_Wip();
}

void SPI_Erase(uint8_t cmd, uint32_t addr)
{
    SPI_WriteEnable();
    SPI_CS_ON;
    SPI_WriteByte(cmd);
    SPI_WriteByte(addr >> 16);
    SPI_WriteByte(addr >> 8);
    SPI_WriteByte(addr);
    SPI_CS_OFF;
    SPI_Wip();
}

void SPI_PageWrite(uint8_t cmd, uint8_t* buf, uint32_t addr)
{
    SPI_WriteEnable();
    SPI_CS_ON;
    SPI_WriteByte(cmd);
    SPI_WriteByte(addr >> 16);
    SPI_WriteByte(addr >> 8);
    SPI_WriteByte(addr);
    for (int i = 0; i < 256; i++) {
        SPI_WriteByte(buf[i]);
    }
    SPI_CS_OFF;
    SPI_Wip();
}

void SPI_PageRead(uint8_t cmd, uint8_t* buf, uint32_t addr)
{
    SPI_CS_ON;
    SPI_WriteByte(cmd);
    SPI_WriteByte(addr >> 16);
    SPI_WriteByte(addr >> 8);
    SPI_WriteByte(addr);
    SPI_WriteByte(Dummy_Byte);
    uint32_t value = QSPI_XIP0->FMT;
    value &= ~QSPI_XIP_FMT_PROTO_MASK;
    if (FLASH_FastReadDual == cmd) {
        QSPI_XIP0->FMT = value | QSPI_XIP_FMT_PROTO_DUAL;
    }
    if (FLASH_FastReadQuad == cmd) {
        QSPI_XIP0->FMT = value | QSPI_XIP_FMT_PROTO_QUAD;
    }
    for (int i = 0; i < 256; i++) {
        buf[i] = SPI_ReadByte();
    }
    SPI_CS_OFF;
}


//void print_data(uint8_t* data)
//{
//    for (int i = 0; i < 16; i++) {
//        printf("%#x ", data[i]);
//    }
//    printf("\n");
//}

/**
 * For FLASH self-test.
 */
uint8_t write[256] = {0};
uint8_t read[256] = {0};

//// /* spi xip read-write flash via normal mode */
//void spi_normal_mode_test(void){
//    for (int i = 0; i < 256; i++) {
//        write[i] = i;
//    }
//
//    SPI_Erase(FLASH_SectorErase, 0x0);
//    SPI_PageWrite(FLASH_PageProgram, write, 0x0);
//    SPI_PageRead(FLASH_FastReadData, read, 0x0);
//    if (0 == memcmp(write, read, 256)) {
//        printf("1 line data cmp pass\n");
//    } else {
//        printf("1 line data cmp error\n");
//        simulation_fail();
//        printf("write\n");
//        print_data(write);
//        printf("read\n");
//        print_data(read);
//        while (1) {}
//    }
//
//    memset(read, 0x00, 256);
//
//    SPI_PageRead(FLASH_FastReadDual, read, 0x0);
//    if (0 == memcmp(write, read, 256)) {
//        printf("2 line data cmp pass\n");
//    } else {
//        printf("2 line data cmp error\n");
//        simulation_fail();
//        printf("write\n");
//        print_data(write);
//        printf("read\n");
//        print_data(read);
//        while (1) {}
//    }
//
//    memset(read, 0x00, 256);
//    SPI_Quad();
//    QSPI_XIP0->FORCE &= ~(QSPI_XIP_FORCE_EN | QSPI_XIP_FORCE_WP);
//    SPI_PageRead(FLASH_FastReadQuad, read, 0x0);
//    if (0 == memcmp(write, read, 256)) {
//        printf("4 line data cmp pass\n");
//    } else {
//        printf("4 line data cmp error\n");
//        simulation_fail();
//        printf("write\n");
//        print_data(write);
//        printf("read\n");
//        print_data(read);
//        while (1) {}
//    }
//}


/**
 * Write one page to flash.
 * @param app : Identifier of the application in flash.
 * @param frame_num : The page number.
 * @param data : Pointer to the data to be written.
 */
//uint8_t write_flash( uint8_t app, uint16_t frame_num, uint8_t* data )
//{
//	uint32_t word, addrx = 0;
//	uint16_t i = 0;
//	char res = 0;
//
//	if( frame_num >= 900 )  // The maximum number of application size is 900 * 256B = 230400B.
//		return 0;
//
//	if( app == 1 )
//	{
//		addrx = APPLICATION_1_ADDR + (frame_num << 8);
//	}
//	else if( app == 2 )
//	{
//		addrx = APPLICATION_2_ADDR + (frame_num << 8);
//	}
//	else if( app == 3 )
//	{
//		addrx = APPLICATION_3_ADDR + (frame_num << 8);
//	}
//	else
//	{
//		res = 0;
//		return res;
//	}
//
//    /**
//     * If enter a new block( 64KB ), erase it.
//     */
//    if( 0 == ( addrx & 65535 ) )
//    {
//        SPI_Erase( FLASH_BLOCK64K_Erase, addrx );
//    }
//
//    /**
//     * Write one page.
//     */
//    SPI_PageWrite( FLASH_PageProgram, data, addrx );
//
//	/**
//	 * Readback and compare.
//	 */
//	uint8_t read_data[256];
//	SPI_PageRead( FLASH_FastReadData, read_data, addrx );
//	for( uint32_t i = 0; i < 256; i++ )
//	{
//		if( data[ i ] != read_data[ i ] )
//		{
//			res = 0;
//			return res;
//		}
//	}
//
//	return 1;
//}

/**
 * Erase the application area.
 * @param app : Identifier of the application in flash.
 */
//uint8_t erase_flash()
//{
//
//	SPI_Erase( FLASH_BLOCK64K_Erase, 65536 );
//
//    return 1;
//}









//读写flash page的数组
uint8_t flash_write_vector[256];
uint8_t flash_read_vector[256];

NODE_CONFIG_t node_config;
PHY_CONFIG_t  phy_config;
MAC_CONFIG_t  mac_config;

uint8_t ack_start;
uint8_t ack_node;
uint8_t ack_phy;
uint8_t ack_mac;


void write_flash_node_config()
{
	SPI_Erase( FLASH_SectorErase, NODE_CONFIG_FLASH_ADDR );
	memcpy(flash_write_vector , &node_config , sizeof(NODE_CONFIG_t));
	SPI_PageWrite( FLASH_PageProgram, flash_write_vector, NODE_CONFIG_FLASH_ADDR );
}
void write_flash_phy_config()
{
	SPI_Erase( FLASH_SectorErase, PHY_CONFIG_FLASH_ADDR );
	memcpy(flash_write_vector , &phy_config , sizeof(PHY_CONFIG_t));
	SPI_PageWrite( FLASH_PageProgram, flash_write_vector, PHY_CONFIG_FLASH_ADDR );
}
void write_flash_mac_config()
{
	SPI_Erase( FLASH_SectorErase, MAC_CONFIG_FLASH_ADDR );
	memcpy(flash_write_vector , &mac_config , sizeof(MAC_CONFIG_t));
	SPI_PageWrite( FLASH_PageProgram, flash_write_vector, MAC_CONFIG_FLASH_ADDR );
}


void read_flash_node_config()
{
	SPI_PageRead( FLASH_FastReadData, flash_read_vector, NODE_CONFIG_FLASH_ADDR );
	memcpy(&node_config , flash_read_vector , sizeof(NODE_CONFIG_t));
}
void read_flash_phy_config()
{
	SPI_PageRead( FLASH_FastReadData, flash_read_vector, PHY_CONFIG_FLASH_ADDR );
	memcpy(&phy_config , flash_read_vector , sizeof(PHY_CONFIG_t));
}
void read_flash_mac_config()
{
	SPI_PageRead( FLASH_FastReadData, flash_read_vector, MAC_CONFIG_FLASH_ADDR );
	memcpy(&mac_config , flash_read_vector , sizeof(MAC_CONFIG_t));
}





