
#ifndef LINKADDR_H_
#define LINKADDR_H_


//地址定义 16位短地址
typedef union {
  unsigned char u8[LINKADDR_SIZE];
#if LINKADDR_SIZE == 2
  uint16_t u16;
#endif /* LINKADDR_SIZE == 2 */
} linkaddr_t;

/**
 * \brief      Copy a Rime address
 * \param dest The destination
 * \param from The source
 *
 *             This function copies a Rime address from one location
 *             to another.
 *
 */
void linkaddr_copy(linkaddr_t *dest, const linkaddr_t *from);


/**
 * \brief      Set the address of the current node
 * \param addr The address
 *
 *             This function sets the Rime address of the node.
 *
 */
void linkaddr_set_node_addr(linkaddr_t *addr);


#endif /* LINKADDR_H_ */
/** @} */
/** @} */