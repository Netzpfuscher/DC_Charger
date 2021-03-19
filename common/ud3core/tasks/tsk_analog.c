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

int32_t iout_offset;

#define IOUT_ZERO_CNT 5
#define IOUT_MA_CNT 5


adc_sample_t ADC;



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

#define N_SCOPE_POINTS 256

typedef struct __scope__ {
    int32_t Vout;
    int32_t Iout;
    int32_t spt;
    int32_t out;
} SCOPE;

SCOPE scope[N_SCOPE_POINTS];


void set_curr_dac(uint32_t dacValue){
    uint8 msb, lsb;
    
    msb = (uint8)(dacValue >> 3);
    lsb = (uint8)((dacValue << 0) & 0x07);
    
    DAC_H_SetValue(msb);
    DAC_L_SetValue(lsb);   
}
int16_t setpoint = 1000;
uint32_t cnt=0;

enum scope_states{
    SCOPE_IDLE,
    SCOPE_SAMPLING,
    SCOPE_FINISH
};

volatile enum scope_states scope_state = SCOPE_IDLE;
uint32_t scope_speed = 1;
uint32_t scope_speed_cnt = 1;

int32_t samp;



CY_ISR(ADC_data_ready_ISR) {
    
    
    int32_t count;
	//count = ADC_Vout_GetResult32();
    count = ADC.v_out_raw;

    if(count<0) count =0;

    sp = pid_step(&v_pid,configuration.spt_Vout_cnt>>1,count>>1);
    set_curr_dac(sp);
    
    
    tt.n.Vout.value = ((55860*count)/(0xFFFF /10));
    
    
    
    
    //count = ADC_iout_GetResult16();
    count = count + (ADC.i_out_raw - iout_offset);
    count /= 2;
    //count = count - iout_offset;
    
    tt.n.Iout.value = (count * IOUT_MA_CNT)/10;
    

    scope_speed_cnt--;
    
    if(scope_state == SCOPE_SAMPLING && scope_speed_cnt==0){
        scope[cnt].out = sp;
        scope[cnt].Vout = tt.n.Vout.value;
        scope[cnt].Iout = tt.n.Iout.value;
        scope[cnt].spt = configuration.spt_Vout;
        if(cnt<N_SCOPE_POINTS){
            cnt++;
        }else{
            cnt=0;
            scope_state = SCOPE_FINISH;
        }
    }
    
    if(scope_speed_cnt==0){
        scope_speed_cnt = scope_speed;
    }
        

}

void initialize_analogs(void) {
	
    DAC_L_Start();
    DAC_H_Start();
    Opamp_1_Start();
    ADC_Vout_Start();
    ADC_Vout_StartConvert();
    VDAC_iout_Start();
    ADC_iout_Start();
    ADC_iout_StartConvert();
    
    Filter_Start();
    Filter_SetCoherency(Filter_CHANNEL_B, Filter_KEY_MID);
    Filter_SetDalign(Filter_STAGEB_DALIGN,Filter_ENABLED);
    Filter_SetDalign(Filter_HOLDB_DALIGN,Filter_ENABLED);
    
    Filter_SetCoherency(Filter_CHANNEL_A, Filter_KEY_MID);
    Filter_SetDalign(Filter_STAGEA_DALIGN,Filter_ENABLED);
    Filter_SetDalign(Filter_HOLDA_DALIGN,Filter_ENABLED);
    
	ADC_data_ready_StartEx(ADC_data_ready_ISR);
 

}

void initialize_DMA(void) {
    
    /* Defines for DMA_iout */
    #define DMA_iout_BYTES_PER_BURST 2
    #define DMA_iout_REQUEST_PER_BURST 1
    #define DMA_iout_SRC_BASE (CYDEV_PERIPH_BASE)
    #define DMA_iout_DST_BASE (CYDEV_PERIPH_BASE)

    /* Variable declarations for DMA_iout */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_iout_Chan;
    uint8 DMA_iout_TD[1];

    /* DMA Configuration for DMA_iout */
    DMA_iout_Chan = DMA_iout_DmaInitialize(DMA_iout_BYTES_PER_BURST, DMA_iout_REQUEST_PER_BURST, 
        HI16(DMA_iout_SRC_BASE), HI16(DMA_iout_DST_BASE));
    DMA_iout_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_iout_TD[0], 2, DMA_iout_TD[0], 0);
    CyDmaTdSetAddress(DMA_iout_TD[0], LO16((uint32)ADC_iout_SAR_WRK0_PTR), LO16((uint32)Filter_STAGEA_PTR));
    CyDmaChSetInitialTd(DMA_iout_Chan, DMA_iout_TD[0]);
    CyDmaChEnable(DMA_iout_Chan, 1);
    
        /* Defines for DMA_vout */
    #define DMA_vout_BYTES_PER_BURST 2
    #define DMA_vout_REQUEST_PER_BURST 1
    #define DMA_vout_SRC_BASE (CYDEV_PERIPH_BASE)
    #define DMA_vout_DST_BASE (CYDEV_PERIPH_BASE)

    /* Variable declarations for DMA_vout */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_vout_Chan;
    uint8 DMA_vout_TD[1];

    /* DMA Configuration for DMA_vout */
    DMA_vout_Chan = DMA_vout_DmaInitialize(DMA_vout_BYTES_PER_BURST, DMA_vout_REQUEST_PER_BURST, 
        HI16(DMA_vout_SRC_BASE), HI16(DMA_vout_DST_BASE));
    DMA_vout_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_vout_TD[0], 2, DMA_vout_TD[0], DMA_vout__TD_TERMOUT_EN);
    CyDmaTdSetAddress(DMA_vout_TD[0], LO16((uint32)ADC_Vout_DEC_SAMP_PTR), LO16((uint32)Filter_STAGEB_PTR));
    CyDmaChSetInitialTd(DMA_vout_Chan, DMA_vout_TD[0]);
    CyDmaChEnable(DMA_vout_Chan, 1);


        /* Defines for DMA_filter_a */
    #define DMA_filter_a_BYTES_PER_BURST 2
    #define DMA_filter_a_REQUEST_PER_BURST 1
    #define DMA_filter_a_SRC_BASE (CYDEV_PERIPH_BASE)
    #define DMA_filter_a_DST_BASE (CYDEV_SRAM_BASE)

    /* Variable declarations for DMA_filter_a */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_filter_a_Chan;
    uint8 DMA_filter_a_TD[1];

    /* DMA Configuration for DMA_filter_a */
    DMA_filter_a_Chan = DMA_filter_a_DmaInitialize(DMA_filter_a_BYTES_PER_BURST, DMA_filter_a_REQUEST_PER_BURST, 
        HI16(DMA_filter_a_SRC_BASE), HI16(DMA_filter_a_DST_BASE));
    DMA_filter_a_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_filter_a_TD[0], 2, DMA_filter_a_TD[0], 0);
    CyDmaTdSetAddress(DMA_filter_a_TD[0], LO16((uint32)Filter_HOLDA_PTR), LO16((uint32)&ADC.i_out_raw));
    CyDmaChSetInitialTd(DMA_filter_a_Chan, DMA_filter_a_TD[0]);
    CyDmaChEnable(DMA_filter_a_Chan, 1);

    /* Defines for DMA_filter_b */
    #define DMA_filter_b_BYTES_PER_BURST 2
    #define DMA_filter_b_REQUEST_PER_BURST 1
    #define DMA_filter_b_SRC_BASE (CYDEV_PERIPH_BASE)
    #define DMA_filter_b_DST_BASE (CYDEV_SRAM_BASE)

    /* Variable declarations for DMA_filter_b */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_filter_b_Chan;
    uint8 DMA_filter_b_TD[1];

    /* DMA Configuration for DMA_filter_b */
    DMA_filter_b_Chan = DMA_filter_b_DmaInitialize(DMA_filter_b_BYTES_PER_BURST, DMA_filter_b_REQUEST_PER_BURST, 
        HI16(DMA_filter_b_SRC_BASE), HI16(DMA_filter_b_DST_BASE));
    DMA_filter_b_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_filter_b_TD[0], 2, DMA_filter_b_TD[0],  DMA_filter_b__TD_TERMOUT_EN);
    CyDmaTdSetAddress(DMA_filter_b_TD[0], LO16((uint32)Filter_HOLDB_PTR), LO16((uint32)&ADC.v_out_raw));
    CyDmaChSetInitialTd(DMA_filter_b_Chan, DMA_filter_b_TD[0]);
    CyDmaChEnable(DMA_filter_b_Chan, 1);

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
    
    initialize_DMA();
	initialize_analogs();
    
    CyGlobalIntEnable;

    vTaskDelay(200);
    
    
    
    for(uint8_t i=0;i<IOUT_ZERO_CNT;i++){
        iout_offset = iout_offset + ADC.i_out_raw;   
        vTaskDelay(200);
    }
    iout_offset /= IOUT_ZERO_CNT;
    
    
    alarm_push(ALM_PRIO_INFO,warn_task_analog, ALM_NO_VALUE);
    
    

	/* `#END` */

	for (;;) {
		/* `#START TASK_LOOP_CODE` */
		//xSemaphoreTake(adc_ready_Semaphore, portMAX_DELAY);
        
        tt.n.avg_power.value = ((tt.n.Vout.value/100) * (tt.n.Iout.value/10))/1000;

        vTaskDelay(50);

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
