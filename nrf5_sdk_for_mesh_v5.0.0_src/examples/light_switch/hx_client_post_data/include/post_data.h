
#ifndef PAST_DATA_H__
#define PAST_DATA_H__

#include <stdint.h>

typedef struct 
{
  int8_t x;
  int8_t y;
  int8_t z;
}pos_t;


typedef struct  
{
  uint16_t head;
  uint8_t type;
  uint8_t id;
  pos_t pos;
  //uint32_t _time;
  uint8_t battery;
  uint8_t heart;
  uint8_t blood;
  uint16_t tail;
  uint8_t guard;
}post_packet_t;

void set_post_all(void);  //设置初始值（测试用）
void post_data(void);

#endif  /* PAST_DATA_H__ */