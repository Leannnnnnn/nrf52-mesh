#include "contiki-net.h"
#include <string.h>

static const struct packetbuf_attrlist attributes[] =
  {
    BROADCAST_ATTRIBUTES PACKETBUF_ATTR_LAST
  };

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static void
recv_from_abc(struct abc_conn *bc)
{
  linkaddr_t sender;
  struct broadcast_conn *c = (struct broadcast_conn *)bc;

  linkaddr_copy(&sender, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  
  PRINTF("%d.%d: broadcast: from %d.%d\n",
	 linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
	 sender.u8[0], sender.u8[1]);
  if(c->u->recv) {
    c->u->recv(c, &sender);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_by_abc(struct abc_conn *bc, int status, int num_tx)
{
  struct broadcast_conn *c = (struct broadcast_conn *)bc;

  PRINTF("%d.%d: sent to %d.%d status %d num_tx %d\n",
	 linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],
	 packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],
         packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],
         status, num_tx);
  if(c->u->sent) {
    c->u->sent(c, status, num_tx);
  }
}
/*---------------------------------------------------------------------------*/
static const struct abc_callbacks broadcast = {recv_from_abc, sent_by_abc};
/*---------------------------------------------------------------------------*/
void
broadcast_open(struct broadcast_conn *c, uint16_t channel,
	  const struct broadcast_callbacks *u)
{
  abc_open(&c->c, channel, &broadcast);
  c->u = u;
  channel_set_attributes(channel, attributes);
}
/*---------------------------------------------------------------------------*/
void
broadcast_close(struct broadcast_conn *c)
{
  abc_close(&c->c);
}
/*---------------------------------------------------------------------------*/
int
broadcast_send(struct broadcast_conn *c)
{
  PRINTF("%d.%d: broadcast_send\n",
	 linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1]);
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  return abc_send(&c->c);
}
/*---------------------------------------------------------------------------*/
/** @} */