#ifndef THMTS_TRXCTRL_SERIAL_CMD_TYPE_
#define THMTS_TRXCTRL_SERIAL_CMD_TYPE_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ringbuffer.h"

#define THMTS_CTRL_SERIAL_CMD_HEAD_LEN                   2
#define THMTS_CTRL_SERIAL_CMD_TAIL_LEN                   0
#define THMTS_CTRL_SERIAL_CMD_HEAD                       0x90eb
#define THMTS_CTRL_SERIAL_CMD_TAIL                       0xdeadbeef
#define THMTS_CTRL_SERIAL_CMD_HEAD_BYTE( n )             ( ( THMTS_CTRL_SERIAL_CMD_HEAD >> ( n << 3 ) ) & 0xff )
#define THMTS_CTRL_SERIAL_CMD_TAIL_BYTE( n )             ( ( THMTS_CTRL_SERIAL_CMD_TAIL >> ( n << 3 ) ) & 0xff )

#define THMTS_CTRL_GET_CMD_REGION( msgType )             ( ( msgType >> 6 ) & 0x03 )
#define THMTS_CTRL_GET_CMD_TYPE( msgType )               ( ( msgType >> 4 ) & 0x03 )
#define THMTS_CTRL_GET_CMD_IDX( msgType )                ( ( msgType ) & 0x0f )

#define THMTS_CTRL_SERIAL_BUF_SIZE                       128

typedef struct
{
    uint8_t msg_type;   // MsgType 8bit.
    uint8_t msg_status; // revisionId 8bit.
    uint8_t msg_id;     // msgCnt 8bit.
    uint8_t rdy_flag;   // Used for msg TX.

    uint8_t resv8;
    uint8_t violate_flag;   // Used for other type of serial msg.
    uint16_t msg_length;    // msgLength - 4 16bit.
    uint32_t CRC;

    void* msg_buf_ptr;  // Pointer to a region with size equals msg_length, its content is the same as msg.
    void* next_msg_ptr; // For RX msg list.
} thmts_ctrl_cmd_proc_msg_t;

#pragma pack(1)
typedef struct
{
    uint8_t msg_type;
    uint8_t msg_status;
    uint8_t msg_id;
    uint16_t msg_length;
} thmts_ctrl_cmd_tx_msg_desc_t;
#pragma pack()

typedef struct
{
    ring_buf_t cmd_ringbuf;
    
    uint8_t curr_msg_node_cnt;
    uint8_t curr_msg_type;
    uint8_t curr_msg_status;
    uint8_t curr_msg_id;
    uint16_t curr_msg_byte_num;
    uint8_t curr_msg_crc;
    uint16_t curr_msg_length;
    uint8_t* curr_msg_buf_ptr;
    thmts_ctrl_cmd_proc_msg_t* msg_node_list_head;
    thmts_ctrl_cmd_proc_msg_t* msg_node_list_tail;

} thmts_ctrl_cmd_proc_t;

bool thmts_ctrl_cmd_proc_init( thmts_ctrl_cmd_proc_t* p);

/*!
 * @brief Push bytes recv from serial port into ringbuffer.
 * This logic should be called upon serial data RX.
*/
void thmts_ctrl_cmd_proc_insert_rx_bytes( thmts_ctrl_cmd_proc_t* p, uint8_t* buf, uint32_t len );

/*!
 * @brief Parse bytes in ringbuffer, allocate memory for good message, send out bytes in tx buffer.
 * This logic should be called in non-intr like main().
*/
void thmts_ctrl_cmd_proc_exec( thmts_ctrl_cmd_proc_t* p );

/*!
 * @brief In FIFO manner, get one message (node).
 * This logic should be called in non-intr like main().
*/
void thmts_ctrl_cmd_proc_get_rx_msg( thmts_ctrl_cmd_proc_t* p, thmts_ctrl_cmd_proc_msg_t* p_msg );

/*!
 * @brief Free the memory of the oldest frame.
 * This logic should be called in non-intr like main().
*/
void thmts_ctrl_cmd_proc_pop_rx_msg( thmts_ctrl_cmd_proc_t* p );


extern thmts_ctrl_cmd_proc_t thmts_ctrl_cmd_proc;

#endif
