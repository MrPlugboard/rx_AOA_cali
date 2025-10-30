#include "thmts_trxctrl_serial_cmd_type.h"
#include <stdio.h>

thmts_ctrl_cmd_proc_t thmts_ctrl_cmd_proc;

bool thmts_ctrl_cmd_proc_init( thmts_ctrl_cmd_proc_t* p )
{
    void* serial_cmd_buf_ptr = malloc( THMTS_CTRL_SERIAL_BUF_SIZE );
    if( serial_cmd_buf_ptr )
    {
        if( ring_buf_init( &p->cmd_ringbuf, serial_cmd_buf_ptr, THMTS_CTRL_SERIAL_BUF_SIZE )  )
        {
            p->curr_msg_buf_ptr = NULL;
            p->curr_msg_byte_num = 0;
            p->curr_msg_crc = 0;
            p->curr_msg_id = 0;
            p->curr_msg_length = 0xffff;
            p->curr_msg_node_cnt = 0;
            p->curr_msg_status = 0;
            p->curr_msg_type = 0;
            p->msg_node_list_head = NULL;
            p->msg_node_list_tail = NULL;
            return true;
        }
        else
        {
            free( serial_cmd_buf_ptr );
            return false;
        }
    }
    else return false;
}

inline void thmts_ctrl_cmd_proc_insert_rx_bytes( thmts_ctrl_cmd_proc_t* p, uint8_t* buf, uint32_t len )
{
    ring_buf_put( &( p->cmd_ringbuf ), buf, len );
}

static inline void thmts_ctrl_cmd_proc_per_byte( thmts_ctrl_cmd_proc_t* p, uint8_t msg_byte )
{
    if( p->curr_msg_byte_num <THMTS_CTRL_SERIAL_CMD_HEAD_LEN )
    {
        if( msg_byte == THMTS_CTRL_SERIAL_CMD_HEAD_BYTE( p->curr_msg_byte_num ) )
        p->curr_msg_byte_num++;
        else
        p->curr_msg_byte_num = 0;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN )
    {
        p->curr_msg_length = msg_byte;
        p->curr_msg_crc += msg_byte;
        p->curr_msg_byte_num++;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 )
    {
        p->curr_msg_length |= ( msg_byte << 8 );
        p->curr_msg_crc += msg_byte;
        p->curr_msg_byte_num++;

        if( p->curr_msg_length >= 2048 || ( p->curr_msg_buf_ptr = ( void* )malloc( p->curr_msg_length - 4 ) ) == NULL )
        {
            p->curr_msg_byte_num = 0;
            p->curr_msg_crc = 0;
            p->curr_msg_length = 0xffff;    // Set to max value.
            p->curr_msg_type = 0;
            p->curr_msg_status = 0;
            p->curr_msg_id = 0;
            p->curr_msg_buf_ptr = NULL;
        }
    }
    else if( p->curr_msg_byte_num <= THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 + p->curr_msg_length - 4 )
    {
        p->curr_msg_crc += msg_byte;
        p->curr_msg_buf_ptr[ p->curr_msg_byte_num - THMTS_CTRL_SERIAL_CMD_HEAD_LEN - 2 ] = msg_byte;
        p->curr_msg_byte_num++;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 + p->curr_msg_length - 4 + 1 )
    {
        p->curr_msg_type = msg_byte;
        p->curr_msg_crc += msg_byte;
        p->curr_msg_byte_num++;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 + p->curr_msg_length - 4 + 2 )
    {
        p->curr_msg_status = msg_byte;
        p->curr_msg_crc += msg_byte;
        p->curr_msg_byte_num++;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 + p->curr_msg_length - 4 + 3 )
    {
        p->curr_msg_id = msg_byte;
        p->curr_msg_crc += msg_byte;
        p->curr_msg_byte_num++;
    }
    else if( p->curr_msg_byte_num == THMTS_CTRL_SERIAL_CMD_HEAD_LEN + 1 + p->curr_msg_length - 4 + 4 )
    {
        if( p->curr_msg_crc == msg_byte )
        {
            thmts_ctrl_cmd_proc_msg_t* p_msg_node = ( thmts_ctrl_cmd_proc_msg_t* )malloc( sizeof( thmts_ctrl_cmd_proc_msg_t ) );
            if( p_msg_node )
            {
                p_msg_node->msg_buf_ptr = p->curr_msg_buf_ptr;
                p_msg_node->msg_id = p->curr_msg_id;
                p_msg_node->msg_length = p->curr_msg_length - 4;
                p_msg_node->msg_status = p->curr_msg_status;
                p_msg_node->msg_type = p->curr_msg_type;
                p_msg_node->CRC = msg_byte;
                p_msg_node->next_msg_ptr = NULL;
                printf("p_msg_node->msg_type = %d , \r\n" , p_msg_node->msg_type);
                if( p->msg_node_list_tail ) 
                {
                    p->msg_node_list_tail->next_msg_ptr = p_msg_node;
                    p->msg_node_list_tail = p_msg_node;
                }
                else p->msg_node_list_head = p->msg_node_list_tail = p_msg_node;

                p->curr_msg_node_cnt++;
            }
            else
            {
                free( p->curr_msg_buf_ptr );
            }
        }
        else
        {
            free( p->curr_msg_buf_ptr );
        }

        p->curr_msg_byte_num = 0;
        p->curr_msg_crc = 0;
        p->curr_msg_length = 0;
        p->curr_msg_type = 0;
        p->curr_msg_status = 0;
        p->curr_msg_id = 0;
        p->curr_msg_buf_ptr = NULL;
    }
}

void thmts_ctrl_cmd_proc_exec( thmts_ctrl_cmd_proc_t* p )
{
    if( p->cmd_ringbuf.front != p->cmd_ringbuf.rear )
    {
        unsigned int left = p->cmd_ringbuf.rear - p->cmd_ringbuf.front;
        unsigned int left_unwrap = ( left < ( p->cmd_ringbuf.size - ( p->cmd_ringbuf.front & p->cmd_ringbuf.size - 1 ) ) ? left : 
                                    ( p->cmd_ringbuf.size - ( p->cmd_ringbuf.front & p->cmd_ringbuf.size - 1 ) ) );

        for( uint32_t i = 0; i < left_unwrap; i++ )
        {
            thmts_ctrl_cmd_proc_per_byte( p, p->cmd_ringbuf.buf[ ( p->cmd_ringbuf.front & p->cmd_ringbuf.size - 1) + i ] );
        }

        for( uint32_t i = 0; i < ( left - left_unwrap ); i++ )
        {
            thmts_ctrl_cmd_proc_per_byte( p, p->cmd_ringbuf.buf[ i ] );
        }

        p->cmd_ringbuf.front += left;
    }
}

void thmts_ctrl_cmd_proc_get_rx_msg( thmts_ctrl_cmd_proc_t* p, thmts_ctrl_cmd_proc_msg_t* p_msg )
{
    if( !p->msg_node_list_head )  
    {
        memset( ( void* )p_msg, 0, sizeof( thmts_ctrl_cmd_proc_msg_t ) );
    }
    else
    {
        memcpy( ( void* )p_msg, ( void* )( p->msg_node_list_head ), sizeof( thmts_ctrl_cmd_proc_msg_t ) );
    }
}

void thmts_ctrl_cmd_proc_pop_rx_msg( thmts_ctrl_cmd_proc_t* p )
{
    if( p->msg_node_list_head )
    {
        free( p->msg_node_list_head->msg_buf_ptr );
        thmts_ctrl_cmd_proc_msg_t* next_msg_ptr = p->msg_node_list_head->next_msg_ptr;
        free( p->msg_node_list_head );
        if( !next_msg_ptr ) p->msg_node_list_head = p->msg_node_list_tail = NULL;
        else p->msg_node_list_head = next_msg_ptr;

        p->curr_msg_node_cnt--;
    }
}

