#ifndef BROADCAST_H_
#define BROADCAST_H_


#include "linkaddr.h"

struct broadcast_conn;

#define BROADCAST_ATTRIBUTES  { PACKETBUF_ADDR_SENDER, PACKETBUF_ADDRSIZE }, \
                        ABC_ATTRIBUTES

/**
 * \brief     Callback structure for broadcast
 *
 */
struct broadcast_callbacks {
  /**在广播模块收到数据包时调用. */
  void (* recv)(struct broadcast_conn *ptr, const linkaddr_t *sender);
  void (* sent)(struct broadcast_conn *ptr, int status, int num_tx);
};

struct broadcast_conn {
  struct abc_conn c;
  const struct broadcast_callbacks *u;
};

/**
 * \brief      设置已经确定的广播连接
 * \param c    A pointer to a struct broadcast_conn
 * \param channel The channel on which the connection will operate
 * \param u    A struct broadcast_callbacks with function pointers to functions that will be called when a packet has been received
 *
 *             This function sets up a broadcast connection on the
 *             specified channel. The caller must have allocated the
 *             memory for the struct broadcast_conn, usually by declaring it
 *             as a static variable.
 *
 *             The struct broadcast_callbacks pointer must point to a structure
 *             containing a pointer to a function that will be called
 *             when a packet arrives on the channel.
 *
 */
void broadcast_open(struct broadcast_conn *c, uint16_t channel,
	       const struct broadcast_callbacks *u);

/**
 * \brief     关闭广播连接
 * \param c    A pointer to a struct broadcast_conn
 *
 *             此函数通常做为退出处理程序调用
 *
 */
void broadcast_close(struct broadcast_conn *c);

/**
 * \brief      发送已识别的广播数据包
 * \param c    The broadcast connection on which the packet should be sent
 * \retval     Non-zero if the packet could be sent, zero otherwise
 *
 *             This function sends an identified best-effort broadcast
 *             packet. The packet must be present in the packetbuf
 *             before this function is called.
 *
 *             The parameter c must point to a broadcast connection that
 *             must have previously been set up with broadcast_open().
 *
 */
int broadcast_send(struct broadcast_conn *c);

#endif /* BROADCAST_H_ */
/** @} */
/** @} */