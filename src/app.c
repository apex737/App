/*
 * app.c
 *
 *  Created on: Dec 29, 2025
 *      Author: user
 */


#include "app.h"

// 전역 변수
volatile float angle = 0.0;
typedef enum {
	LISTEN, WAIT_FOR_DATA, TDOA, UART
} state_t;

state_t c_state = LISTEN;
uint32_t cnt = 0;
extern Buffer_Status_t buf_status;

void app_init(app_handle_t* pApp)
{
	adc_init(pApp->hadc, pApp->htim);
	tdoa_init();
//	uart_init(pApp->huart);
}


void app_main(void)
{
	adc_awd_enable();

    while(1)
    {
       switch(c_state)
       {
		   case LISTEN:
			   if(buf_status.awd_triggered){
				   // Flag All Clear
				   buf_status.awd_triggered = false;
				   buf_status.half_xfer = false;
				   buf_status.xfer_cplt = false;
				   buf_status.tdoa_running = false;

				   c_state = WAIT_FOR_DATA;
			   }
			   break;

		   case WAIT_FOR_DATA:
			   if(buf_status.half_xfer)
			   {
				   split_adc_data(0);
				   buf_status.half_xfer = false;
				   c_state = TDOA;
			   }
			   else if(buf_status.xfer_cplt)
			   {
				   split_adc_data(BUFFER_OFFSET);
				   buf_status.xfer_cplt = false;
				   c_state = TDOA;
			   }
			   break;

		   case TDOA:
			   angle = tdoa_process(mic1_buf, mic2_buf);
			   buf_status.tdoa_running = false;
			   c_state = UART;
			   break;

		   case UART:
//			   transmit_angle();
			   cnt++;
			   adc_awd_enable();
//			   buf_status.awd_triggered = false;
			   c_state = LISTEN;
			   break;

       }
    }
}

