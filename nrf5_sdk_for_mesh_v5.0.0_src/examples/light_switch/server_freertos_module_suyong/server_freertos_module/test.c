#include "aodv.h"
#include "string.h"

static Node node[50];

int mesh_aodv_test1(void){

 aodv_send_rreq(node[1].node_address,node[2].node_address);

}

