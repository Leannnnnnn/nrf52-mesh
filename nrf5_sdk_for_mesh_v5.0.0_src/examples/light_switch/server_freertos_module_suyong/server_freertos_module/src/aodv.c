#include "aodv.h"
#include <stdio.h>
#include <stdlib.h>
# include "linkaddr.h"

static AodvRoutingEntry routing_table[AODV_RT_SIZE];

//固定组播地址
#define BT_MESH_ADDR_ALL_NODES    0xfff//所有节点组地址
#define BT_MESH_ADDR_PROXIES      0xfffc
#define BT_MESH_ADDR_FRIENDS      0xfffd
#define BT_MESH_ADDR_RELAYS       0xfffe


/*RREQ DEFINITIONS*/
#define RREQ_SDU_MAX_SIZE 15//最大消息包长度15字节
#define  RREQ_RING_SEARCH_MAX_TTL//RREQ重传最大TTL

static uint8_t rreq.id = 0;

/* RREP DEFINITIONS */
#define RREP_SDU_MAX_SIZE 12

/*Error Messages*/
#define ELOCAL 139 
#define ENORREQ 140 /* RREQ interval 过期*/
#define ENORREP 141 /* RREP interval 过期*/
#define ENODRREQ 142 /*RREQ被删除*/
#define ENORREQSENT 143 /* 重复的RREQ被丢弃*/

static Node node;

// 元组（node_last_sent_rreq_id，own_address）标识可以删除的自发送rreq，只要它们出现在AODV_PATH_DISCOVERY_TIME中
static uint8_t node_last_seen_rreq_id = 0;
static uint8_t node_last_seen_source_address = 0;

uint8_t next_hop;//从这个节点传过来的（传递RREQ给本节点的那个相邻节点）

int initnode(uint8_t node_address){
    node.node_address = node_address;
    node.node_sequence_number = 0;
}

/*    RREQ Function          */

/* 
* @brief 已知源节点和目标节点，生成RREQ消息，并广播
*/
 int aodv_send_rreq(uint8_t source_address,uint8_t destination_address) 
 {
    static AodvRreq rreq = {0,0,0,0,0,0,0,false};//初始化消息
    
    uint16_t network_next_hop = BT_MESH_ADDR_ALL_NODES;//默认网络下一层是广播到所有节点
 

    // 检查路由表中是否存在关于目的节点的信息
    if(node.routing_table[destination_address].in_use) {//如果已经有有效路由，就从路由表中获取目标节点的已知的最新序列号
        rreq.destination_sequence_number = node.routing_table[destination_address].destination_sequence_number;
        rreq.known_sequence_number = node.routing_table[destination_address].known_sequence_number;
    } else {//如果尚未获得任何目的节点序列号
        rreq.destination_sequence_number = 0;
        rreq.known_sequence_number = false;
    }
    //生成rreq消息
    rreq.id = ++rreq.id;//RREQ ID 将当前节点以前用过的id加一 
    rreq.source_address = node.node_address;
    rreq.source_sequence_number = ++node.node_sequence_number;//发起节点自己的序列号，放进RREQ消息时先自增一
    rreq.destination_address = destination_address;
    rreq.hot_count = 0;
    rreq.ttl = AODV_RREQ_TTL;

   //发送 
    if(rreq.ttl <= 0) {
        return 0;
    }

    // 标记 RREQ ID 为已发送
    node_last_seen_rreq_id = rreq.id;
    node_last_seen_source_address = rreq.source_address;

    static uint8_t buffer[sizeof(AodvRreq) + 1];

    buffer[0] = RREQ;
    buffer[1] = (rreq->known_sequence_number << 0);
    buffer[2] = rreq->id;
    buffer[3] = rreq->source_address;
    buffer[4] = rreq->source_sequence_number;
    buffer[5] = rreq->destination_address;
    buffer[6] = rreq->destination_sequence_number;
    buffer[7] = rreq->ttl;


    aodv_print_rreq("Send", &rreq);//打印rreq
    return  broadcast_send(buffer);//有待修改
}




bool aodv_seen_rreq(AodvRreq *rreq) {//是否已经发过这个rreq
    return node_last_seen_rreq_id == rreq->id && node_last_seen_source_address == rreq->source_address;
}



//处理和转发路由请求
AodvRreq *aodv_receive_rreq(uint8_t *data) {//收到data，并分解为rreq各字段
    static AodvRreq rreq;

    rreq.known_sequence_number = data[1] & 0x01;
    rreq.id = data[2];
    rreq.source_address = data[3];
    rreq.source_sequence_number = data[4];
    rreq.destination_address = data[5];
    rreq.destination_sequence_number = data[6];
    rreq.hot_count = data[7];
    rreq.ttl = data[8];

    aodv_print_rreq("Recv", &rreq);
    broadcast_send(*data);
   

    //如果是目的节点接收到rreq
    if(rreq.destination_address == node.node_address){
        if( aodv_routing_table_has_latest_route(rreq)){//检查是否有最新的有效路由
            //如果有，根据路由度量选择最好的路由
            if(rreq.hot_count < node.routing_table[rreq.source_address].hop_count){//更新路由
                 node.routing_table[rreq.source_address].next_hop = next_hop ;//最短路线的上一个节点
                 node.routing_table[rreq.source_address].hop_count = rreq.hot_count;
            }
        }
        else {//否则就创建方向路由
            node.routing_table->in_use = true;
            if (rreq.source_sequence_number > node.routing_table[rreq.source_address].destination_sequence_number){
            node.routing_table[rreq.source_address].destination_sequence_number = rreq.source_sequence_number;
        }
            node.routing_table->next_hop = next_hop;
            node.routing_table->hop_count = rreq.hot_count;
        }


        aodv_send_rrep_as_destination(&rreq);//发送rrep

    }
    //如果是中间节点接收到rreq
   else{
        if( aodv_routing_table_has_latest_route(rreq)){//检查是否有最新的有效路由
            aodv_send_rrep_as_intermediate(&rreq);//发送rrep
        }
        else{
        //创建方向路由
        node.routing_table[rreq.source_address].in_use = true;
        if (rreq.source_sequence_number > node.routing_table[rreq.source_address].destination_sequence_number){
            node.routing_table[rreq.source_address].destination_sequence_number = rreq.source_sequence_number;
        }
        node.routing_table->next_hop = next_hop;
        node.routing_table->hop_count = rreq.hot_count;
        //更新rreq并继续转发
        rreq.ttl--;
        rreq.hot_count++;
       
        aodv_print_rreq("Send", &rreq);//打印rreq
        return  broadcast_send(&rreq)//广播

        
        }
   }
    
    
    
}

void aodv_print_rreq(const char* action, AodvRreq *rreq) {
    printf("%s RREQ: ID: %d | Source: %d/%d | Destination: %d/%d/%s | Hop_count:%d | TTL: %d\n",
        action,
        rreq->id,
        rreq->source_address,
        rreq->source_sequence_number,
        rreq->destination_address,
        rreq->destination_sequence_number,
        rreq->known_sequence_number ? "known  " : "unknown",
        rreq->hot_count,
        rreq->ttl);
}




/*        RREP Function      */


int aodv_send_rrep( AodvRrep *rrep) {//发送rrep
    // 检查是否有路由
    static uint8_t next_hop = 0;
    next_hop = aodv_routing_table_lookup(rrep->destination_address);//rrep的下一跳是在路由表里查询反向路由
    if(next_hop == 0) {//如果没有，就报错
        printf("Error: RREP could not be sent - no route to node\n");
        return 0;
    }
    //缓存
   static uint8_t buffer[sizeof(AodvRrep) + 1];

    buffer[0] = RREP;
    buffer[1] = rrep->hop_count;
    buffer[2] = rrep->source_address;
    buffer[3] = rrep->destination_address;
    buffer[4] = rrep->destination_sequence_number;

     // 创建单播地址
    static linkaddr_t addr;
    addr.u8[0] = next_hop;
    addr.u8[1] = 0;

   
    aodv_print_rrep("Send", rrep);
    return unicast_send(uc, &addr);//单播有待修改
}




//生成RREP的两种情况：
//它自己就是目标节点
//它有到目的节点的有效路由
int aodv_send_rrep_as_destination( AodvRreq *rreq) {//已知rreq，目的节点发送rrep
   if(rreq->known_sequence_number
            && rreq->destination_sequence_number > routing_table[linkaddr_node_addr.u8[0]].sequence_number) {
        routing_table[linkaddr_node_addr.u8[0]].sequence_number = rreq->destination_sequence_number;
    };//修改路由表
    static AodvRrep rrep;
    if(rreq->destination_address == node.node_address && rreq->destination_sequence_number == node.node_sequence_number ){
        node.node_sequence_number++;
    }
    
     //构造rrep
    rrep.hop_count = 0;
    rrep.source_address = node.node_sequence_number;
    rrep.destination_address = rreq->source_address;
    rrep.destination_sequence_number = node.routing_table[rreq->source_address].destination_sequence_number;

    return aodv_send_rrep( &rrep);
}

int aodv_send_rrep_as_intermediate( AodvRreq *rreq) {//中间节点生成rrep
    static AodvRrep rrep;
//编写rrep
    rrep.hop_count = node.routing_table[rreq->destination_address].hop_count;//中继节点放置它到目的节点的跳数到RREP中的count字段
    rrep.source_address = rreq->destination_address;
    rrep.destination_address = rreq->source_address;
    rrep.destination_sequence_number = node.routing_table[rreq->source_address].destination_sequence_number;

    return aodv_send_rrep(&rrep);
}

//接受和转发路由回复
AodvRrep *aodv_receive_rrep(uint8_t *data) {
    static AodvRrep rrep;
    
    rrep.hop_count = data[1];
    rrep.source_address = data[2];
    rrep.destination_address = data[3];
    rrep.destination_sequence_number = data[4];


    rrep.hop_count++;

    return aodv_send_rrep( &rrep);//继续转发
    
}

void aodv_print_rrep(const char* action, AodvRrep *rrep) {
    printf("%s RREP: Hop Count: %d | Source: %d | Destination: %d/%d\n",
        action,
        rrep->hop_count,
        rrep->source_address,
        rrep->destination_address,
        rrep->destination_sequence_number);
}


/*        RERR Function      */
int aodv_send_rerr( uint8_t exclude_address, AodvRerr *rerr) {
    //缓存
    static uint8_t buffer[sizeof(AodvRerr) + 1];

    buffer[0] = RERR;
    buffer[1] = rerr->destination_address;
    buffer[2] = rerr->destination_sequence_number;
  

    static linkaddr_t addr;
    static uint8_t i = 0;
    for(i = 0; i < AODV_RT_SIZE; i++) {
        if(node.routing_table[i].in_use
                && node.routing_table[i].known_sequence_number
                && node.routing_table[i].next_hop != linkaddr_node_addr.u8[0]
                && node.routing_table[i].next_hop != exclude_address) {
           addr.u8[0] = routing_table[i].next_hop;
            addr.u8[1] = 0;
            

           
            aodv_print_rerr("Send", rerr);
            return unicast_send(&addr);;
            
        }
    }

    return 1;
}

int aodv_send_rerr_data(
        uint8_t exclude_address,
        uint8_t destination_address,
        uint8_t destination_sequence_number) {
    static AodvRerr rerr;

    rerr.destination_address = destination_address;
    rerr.destination_sequence_number = destination_sequence_number;

    return aodv_send_rerr(exclude_address, &rerr);
}

void aodv_initiate_rerr( uint8_t destination_address) {
    // 如果有路由，使其无效，并通知其邻居
    static uint8_t sequence_number;
    //当发现无效路由时，增加序列号
    sequence_number = node.routing_table[destination_address].destination_sequence_number + 1;

    if(aodv_routing_table_remove_stale_route(destination_address, sequence_number)) {//如果删除陈旧路由
        aodv_send_rerr_data( 0, destination_address, sequence_number);//发送rerr
    }

    // 删除路由
    if(node.routing_table[destination_address].in_use) {
        node.routing_table[destination_address].in_use = false;
        printf("Removed route to %d via %d, reason: direct route stale\n",
            destination_address,
            node.routing_table[destination_address].next_hop);
    }
}

AodvRerr *aodv_receive_rerr(uint8_t *data) {
    static AodvRerr rerr;
    
    rerr.destination_address = data[1];
    rerr.destination_sequence_number = data[2];

    aodv_print_rerr("Recv", &rerr);
    return &rerr;
}

void aodv_print_rerr(const char* action, AodvRerr *rerr) {
    printf("%s RERR: Destination: %d/%d\n",
        action,
        rerr->destination_address,
        rerr->destination_sequence_number);
}


/* Routing Table Function*/
void aodv_routing_table_init( uint8_t size) {//路由表初始化
    
    static uint8_t i;
    for(i = 0; i < AODV_RT_SIZE; i++) {
        node.routing_table[i].in_use = false;
        }

     //将自己加入路由表中
    routing_table[linkaddr_node_addr.u8[0]].in_use = true;
    routing_table[linkaddr_node_addr.u8[0]].next_hop = linkaddr_node_addr.u8[0];
    routing_table[linkaddr_node_addr.u8[0]].distance = 0;
    routing_table[linkaddr_node_addr.u8[0]].sequence_number = 0;
    routing_table[linkaddr_node_addr.u8[0]].known_sequence_number = true;
}

void aodv_routing_table_print() {//打印路由表
    printf("---------------------------------------------------------------------------\n");
    printf("%-15s%-15s%-15s%-15s%-15s\n", "Destination", "Next Hop", "Distance", "SN", "SN Known");
    static uint8_t i;
    for(i = 0; i < AODV_RT_SIZE; i++) {
        if(node.routing_table[i].in_use) {
            printf("%-15d%-15d%-15d%-15d%-15s\n",
            i,
            node.routing_table[i].next_hop,
            node.routing_table[i].hop_count,
            node.routing_table[i].destination_sequence_number,
            node.routing_table[i].known_sequence_number ? "yes": "no");
        }
    }
    printf("---------------------------------------------------------------------------\n");
}

void aodv_routing_table_update_if_required(//路由表更新 从a节点到b的路由
        uint8_t a,
        uint8_t b,
        uint8_t hop_count,
        uint8_t destination_sequence_number,
        bool known_sequence_number) {
    static bool update = false;

    // 确定是否更新
    if(node.routing_table[a].in_use) {
        if(node.routing_table[a].known_sequence_number) {
            // 比较序列号
           
            static int8_t sequence_diff;//序列号差
            sequence_diff = destination_sequence_number - node.routing_table[a].destination_sequence_number;

            if(sequence_diff > 0) {//如果目的节点序列号更新
                update = true;//则需要更新
                printf("Updated route to %d via %d, reason: newer sequence number\n", a, b);
            } else if(sequence_diff == 0) {//如果一样
                // 则比较跳数/距离
                if(hop_count < node.routing_table[a].hop_count) {//如果新的跳数较小，则更新
                    update = true;
                    printf("Updated route to %d via %d, reason: shorter distance\n", a, b);
                } else {
                    update = false;
                }
            } else {
                update = false;
            }
        } else {
            update = true;
            printf("Updated route to %d via %d, reason: unknown sequence number\n", a, b);
        }
    } else {
        update = true;
        printf("Added route to %d via %d, reason: not existent\n", a, b);
    }

    // Update routing table
    if(update) {
        node.routing_table[a].in_use = true;
        node.routing_table[a].hop_count = hop_count;
        node.routing_table[a].next_hop = b;
        node.routing_table[a].destination_sequence_number = destination_sequence_number;
        node.routing_table[a].known_sequence_number = known_sequence_number;
    }
}

bool aodv_routing_table_has_latest_route(AodvRreq rreq) {//判断路由表是否具有最新路由
    // 路由表必须有效(in_use),目的节点序列号已知，并且大于等于rreq中的目标节点序列号
    return node.routing_table[rreq.destination_address].in_use
            && node.routing_table[rreq.destination_address].known_sequence_number
            && (int8_t)(node.routing_table[rreq.destination_address].destination_sequence_number - rreq.destination_sequence_number) >= 0;
}

bool aodv_routing_table_remove_stale_route(uint8_t to, uint8_t sequence_number) {//判断路由表是否删除陈旧路由
    // 不仅检查序列号，还要检查是否需要删除
    if(to != next_hop
            && node.routing_table[to].in_use
            && node.routing_table[to].known_sequence_number
            && sequence_number > node.routing_table[to].destination_sequence_number) {
        node.routing_table[to].in_use = false;
        printf("Removed route to %d via %d, reason: indicated as stale\n", to, node.routing_table[to].next_hop);
        return true;
    }

    return false;
}

uint8_t aodv_routing_table_lookup(uint8_t address) {//查询路由表的下一跳
    
    if( address != linkaddr_node_addr.u8[0]//不能查询自己
            &&node.routing_table[address].in_use) {
        return node.routing_table[address].next_hop;//则返回路由表的下一跳
    }

    return 0;
}



 
