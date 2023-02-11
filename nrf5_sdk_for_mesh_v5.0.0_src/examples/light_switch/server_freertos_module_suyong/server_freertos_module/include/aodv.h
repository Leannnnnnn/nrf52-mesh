#ifndef AODV_H
#define AODV_H

#include <stdint.h>
#include <stdbool.h>

#define AODV_RREQ_TTL 10
#define AODV_RT_SIZE 50

typedef struct {
    bool in_use;//路由是否有效
    uint8_t destination_address;//目的节点地址
    uint8_t destination_sequence_number;//目的节点序列号
    bool known_sequence_number;//目的节点序列号是否正确
    uint8_t hop_count;//跳数
    uint8_t next_hop;//下一跳

} AodvRoutingEntry;

typedef enum {
    RREQ = 0,
    RREP = 1,
    RERR = 2,
} AodvType;

typedef struct {
    uint8_t id;//路由请求消息标识，用它和发起节点地址就可以唯一标识一个RREQ信息
    uint8_t source_address;//发起节点地址
    uint8_t source_sequence_number;//发起节点序列号
    uint8_t destination_address;//目标节点地址
    uint8_t destination_sequence_number;//目标节点序列号，发起节点在以前通往目标节点的路由信息中能找到最新的序列号
    uint8_t hot_count;//跳数
    uint8_t ttl;
    bool known_sequence_number;
} AodvRreq;

typedef struct {
    uint8_t hop_count;
    uint8_t source_address;
    uint8_t destination_address;
    uint8_t destination_sequence_number;
} AodvRrep;

typedef struct {
    uint8_t destination_address;//因断开连接而不可达的目的节点地址
    uint8_t destination_sequence_number;
} AodvRerr;



typedef struct {
    uint8_t node_sequence_number;
    uint8_t node_address;
     AodvRoutingEntry routing_table[AODV_RT_SIZE];
}Node;




int aodv_send_rreq( uint8_t source_address,uint8_t destination_address) {//已知源节点和目标节点，形成rreq表
bool aodv_seen_rreq(AodvRreq *rreq);
AodvRreq *aodv_receive_rreq(uint8_t *data);
void aodv_print_rreq(const char* action, AodvRreq *rreq);

int aodv_send_rrep(AodvRrep *rrep);
int aodv_send_rrep_as_destination( AodvRreq *rreq);
int aodv_send_rrep_as_intermediate(AodvRreq *rreq);
AodvRrep *aodv_receive_rrep(uint8_t *data);
void aodv_print_rrep(const char* action, AodvRrep *rrep);

int aodv_send_rerr(uint8_t exclude_address, AodvRerr *rerr);
int aodv_send_rerr2(
    uint8_t exclude_address,
    uint8_t destination_address,
    uint8_t destination_sequence_number);
void aodv_initiate_rerr( uint8_t destination_address);
AodvRerr *aodv_receive_rerr(uint8_t *data);
void aodv_print_rerr(const char* action, AodvRerr *rerr);

void aodv_routing_table_init(Node **node, uint8_t size);
void aodv_routing_table_print();
void aodv_routing_table_update_if_required(uint8_t to,
    uint8_t via,
    uint8_t hop_count,
    uint8_t destination_sequence_number,
    bool known_sequence_number);
bool aodv_routing_table_has_latest_route(AodvRreq *rreq);
bool aodv_routing_table_remove_stale_route(uint8_t to, uint8_t sequence_number);
uint8_t aodv_routing_table_lookup(uint8_t address);

#endif
