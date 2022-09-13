

#include "post_data.h"
#include "hx_control_model_create.h"



static post_packet_t post_packet = {
    .head = 0xfafa,
    .type = 0x10,
    .id = 0x02,
    .pos.x = 0x13,
    .pos.y = 0x26,
    .pos.z = 0x37,
    .battery = 0x56,
    .heart = 0x90,
    .blood = 0x80,
    .tail = 0xfbfb,
    .guard = 0x00
    };

void set_post_all(void)  //设置初始值（测试用）
{
    
}


void post_data(void)
{
    char data[] = {0xFA,0xFA,0x10,0x02,0x13,0x26,0x37,0x56,0x90,0x80,0xFB,0xFB,0x00};
    hx_client_tx(data);
}