/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cyapicallbacks.h"
#include <cytypes.h>

#include "tsk_analog.h"
#include "tsk_fault.h"
#include "alarmevent.h"
#include "config.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "helper/FastPID.h"

#include <stdlib.h>
#include "math.h"


xTaskHandle tsk_analog_TaskHandle;
uint8 tsk_analog_initVar = 0u;

SemaphoreHandle_t adc_ready_Semaphore;

xQueueHandle adc_data;

TimerHandle_t xCharge_Timer;





/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "cli_common.h"
#include "telemetry.h"
#include "tsk_priority.h"
#include <device.h>


#define INITIAL 0 /* Initial value of the filter memory. */

typedef struct
{
	uint16_t rms;
	uint64_t sum_squares;
} rms_t;




FastPID v_pid;

rms_t current_idc;



/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */



#define IPRIM_MA_CNT 61;
volatile int32_t sp;
void set_curr_dac(uint32_t dacValue){
    uint8 msb, lsb;
    
    msb = (uint8)(dacValue >> 3);
    lsb = (uint8)((dacValue << 0) & 0x07);
    
    DAC_H_SetValue(msb);
    DAC_L_SetValue(lsb);   
}
int16_t setpoint = 1000;

CY_ISR(ADC_data_ready_ISR) {

    int32_t count;
	count = ADC_Vout_GetResult32();
    if(count<0) count =0;
    
    
    sp = pid_step(&v_pid,configuration.spt_Vout_cnt>>1,count>>1);
    set_curr_dac(sp);
    
    tt.n.Vout.value = ((55860*count)/(0xFFFF /10));
 //   tt.n.Iprim.value = (uint32_t)rms_filter(&current_idc,ADC_Iprim_GetResult16()) * IPRIM_MA_CNT;
    

   
    //xSemaphoreGiveFromISR(adc_ready_Semaphore, NULL);
}

void initialize_analogs(void) {
	
    DAC_L_Start();
    DAC_H_Start();
    Opamp_1_Start();
    ADC_Vout_Start();
    ADC_Vout_StartConvert();
    ADC_Iprim_Start();

	ADC_data_ready_StartEx(ADC_data_ready_ISR);

}





uint8_t callback_pid(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    
    pid_new(&v_pid,configuration.v_pid_p,configuration.v_pid_i,configuration.v_pid_d,ADC_Vout_CFG1_SRATE,11,false);
    pid_set_anti_windup(&v_pid,0,MAX_DAC_VAL);
    pid_set_limits(&v_pid,0,MAX_DAC_VAL);
    if(v_pid._cfg_err==true){
        ttprintf("PID value error\r\n");   
    }
    
    return pdPASS;
}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void tsk_analog_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

	//adc_data = xQueueCreate(128, sizeof(ADC_sample));
    
    /* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */

	adc_ready_Semaphore = xSemaphoreCreateBinary();

	initialize_analogs();
    
    CyGlobalIntEnable;

    
    alarm_push(ALM_PRIO_INFO,warn_task_analog, ALM_NO_VALUE);

	/* `#END` */

	for (;;) {
		/* `#START TASK_LOOP_CODE` */
		//xSemaphoreTake(adc_ready_Semaphore, portMAX_DELAY);

        vTaskDelay(5);

		/* `#END` */
	}
}
/* ------------------------------------------------------------------------ */
void tsk_analog_Start(void) {
	/*
	 * Insert task global memeory initialization here. Since the OS does not
	 * initialize ANY global memory, execute the initialization here to make
	 * sure that your task data is properly 
	 */
	/* `#START TASK_GLOBAL_INIT` */

	/* `#END` */

	if (tsk_analog_initVar != 1) {

		/*
	 	* Create the task and then leave. When FreeRTOS starts up the scheduler
	 	* will call the task procedure and start execution of the task.
	 	*/
		xTaskCreate(tsk_analog_TaskProc, "Analog", STACK_ANALOG, NULL, PRIO_ANALOG, &tsk_analog_TaskHandle);
		tsk_analog_initVar = 1;
	}
}
/* ------------------------------------------------------------------------ */
/* ======================================================================== */
/* [] END OF FILE */
