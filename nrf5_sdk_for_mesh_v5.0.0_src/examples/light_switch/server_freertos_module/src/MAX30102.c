
/*---------------------------------------------
 * 
 * MAX30205心率血氧传感器驱动程序，基于IIC传输
 *
 *---------------------------------------------*/


#include "MAX30102.h"
#include "nrf_drv_gpiote.h"


static bool int_flag = false;

void int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

    int_flag = true;

}

void max30102_int_pin_set(void)
{

    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(45, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);  //设置为下降沿触发
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_INT, &in_config, int_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_INT, true);

}


static bool max30102_verify(void)
{
    uint8_t id = 0;
    max30102_i2c_read(PART_ID, &id, 1);
    if(EXPECTED_PARTID != id){
        return false;
    }
    return true;
}


bool max30102_init(void)
{ 
    uint8_t data;

    bool ret = max30102_verify();

    max30102_i2c_write(MODE_CONFIGURATION,0x40);  //reset the device

    delay_ms(5);

    max30102_i2c_write(INTERRUPT_ENABLE1,0xE0);
    max30102_i2c_write(INTERRUPT_ENABLE2,0x00);  //interrupt enable: FIFO almost full flag, new FIFO Data Ready,
                                                 //ambient light cancellation overflow, power ready flag, 
                                                 //internal temperature ready flag

    max30102_i2c_write(FIFO_WR_POINTER,0x00);
    max30102_i2c_write(FIFO_OV_COUNTER,0x00);
    max30102_i2c_write(FIFO_RD_POINTER,0x00);   //clear the pointer

    max30102_i2c_write(FIFO_CONFIGURATION,0x4F); //FIFO configuration: sample averaging(1),FIFO rolls on full(0), FIFO almost full value(15 empty data samples when interrupt is issued)  

    max30102_i2c_write(MODE_CONFIGURATION,0x03);  //MODE configuration:SpO2 mode

    max30102_i2c_write(SPO2_CONFIGURATION,0x2A); //SpO2 configuration:ACD resolution:15.63pA,sample rate control:200Hz, LED pulse width:215 us 

    max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x2f);	//IR LED
    max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x2f); //RED LED current

    max30102_i2c_write(TEMPERATURE_CONFIG,0x01);   //temp

    max30102_i2c_read(INTERRUPT_STATUS1,&data,1);
    max30102_i2c_read(INTERRUPT_STATUS2,&data,1);  //clear status

    delay_ms(5);

    nrf_gpio_cfg_input(PIN_INT, NRF_GPIO_PIN_PULLUP);//设置管脚位上拉输入,中断触发引脚

    return ret;
     
}


void max30102_fifo_read(float *output_data)
{
    uint8_t receive_data[6];
    uint32_t data[2];
    max30102_i2c_read(FIFO_DATA,receive_data,6);
    data[0] = ((receive_data[0]<<16 | receive_data[1]<<8 | receive_data[2]) & 0x03ffff);
    data[1] = ((receive_data[3]<<16 | receive_data[4]<<8 | receive_data[5]) & 0x03ffff);
    *output_data = data[0];
    *(output_data+1) = data[1];
 
}


uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
{
    float input_data_sum_aver = 0;
    uint16_t i,temp;
    
    
    for(i=0;i<cache_nums;i++)
    {
    input_data_sum_aver += *(input_data+i);
    }
    input_data_sum_aver = input_data_sum_aver/cache_nums;
    for(i=0;i<cache_nums;i++)
    {
        if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
        {
            temp = i;
            break;
        }
    }
    i++;
    for(;i<cache_nums;i++)
    {
        if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
        {
            temp = i - temp;
            break;
        }
    }
    if((temp>14)&&(temp<100))
    {
        return 3000/temp;  //60/temp间隔/采样率
    }
    else
    {
        return 0;
    }
}


float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
    float ir_max=*ir_input_data,ir_min=*ir_input_data;
    float red_max=*red_input_data,red_min=*red_input_data;
    float R;
    uint16_t i;
    for(i=1;i<cache_nums;i++)
    {
        if(ir_max<*(ir_input_data+i))
        {
            ir_max=*(ir_input_data+i);
        }
        if(ir_min>*(ir_input_data+i))
        {
            ir_min=*(ir_input_data+i);
        }
        if(red_max<*(red_input_data+i))
        {
            red_max=*(red_input_data+i);
        }
        if(red_min>*(red_input_data+i))
        {
            red_min=*(red_input_data+i);
        }
    }
    
    R=((ir_max+ir_min)*(red_max-red_min))/((red_max+red_min)*(ir_max-ir_min));
    return ((-45.060)*R*R + 30.354*R + 94.845);
}



bool hr_flag_check(void)  //查询中断状态
{
    return int_flag; 
}

void hr_flag_clear(void)  //清除中断状态
{
    int_flag = false;
}
 

