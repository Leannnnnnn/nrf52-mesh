
#ifndef HX_CONTROL_MODEL_CREATE_H__
#define HX_CONTROL_MODEL_CREATE_H__


#include "hx_model_control.h"

void hx_models_init(void);
void hx_client_tx(uint8_t *str);
void hx_client_tx_int(int16_t num);  //发送整形数字



#endif  //HX_CONTROL_MODEL_CREATE_H__