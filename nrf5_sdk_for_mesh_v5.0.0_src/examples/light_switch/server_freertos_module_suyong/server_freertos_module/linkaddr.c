#include "linkaddr.h"
#include <string.h>

linkaddr_t linkaddr_node_addr;//表示本节点地址
#if LINKADDR_SIZE == 2
const linkaddr_t linkaddr_null = { { 0, 0 } };
#endif /*LINKADDR_SIZE == 2*/

void linkaddr_copy(linkaddr_t *dest, const linkaddr_t *src)
{
    memcpy(dest, src, LINKADDR_SIZE);
}//使用memcpy将src所指向的linkaddr拷贝给dest

void linkaddr_set_node_addr(linkaddr_t *t)
{
  linkaddr_copy(&linkaddr_node_addr, t);
}//设置本节点地址