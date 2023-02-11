#ifndef UNICAST_H_
#define UNICAST_H_

#include "broadcast.h"

struct unicast_conn;

#define UNICAST_ATTRIBUTES   { PACKETBUF_ADDR_RECEIVER, PACKETBUF_ADDRSIZE }, \
                        BROADCAST_ATTRIBUTES

struct unicast_callbacks {
  void (* recv)(struct unicast_conn *c, const linkaddr_t *from);
  void (* sent)(struct unicast_conn *ptr, int status, int num_tx);
};

struct unicast_conn {
  struct broadcast_conn c;
  const struct unicast_callbacks *u;
};

void unicast_open(struct unicast_conn *c, uint16_t channel,
	      const struct unicast_callbacks *u);
void unicast_close(struct unicast_conn *c);

int unicast_send(struct unicast_conn *c, const linkaddr_t *receiver);

#endif /* UNICAST_H_ */
/** @} */
/** @} */