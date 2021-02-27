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

#include "tsk_thermistor.h"
#include "tsk_fault.h"
#include "alarmevent.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

xTaskHandle tsk_thermistor_TaskHandle;
uint8 tsk_thermistor_initVar = 0u;

/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "tsk_analog.h"
#include "cli_common.h"
#include "telemetry.h"
#include "tsk_priority.h"

#define THERM_DAC_VAL 23

#define TEMP1_FAULT 0x00FF
#define TEMP2_FAULT 0xFF00

#define TEMP_FAULT_LOW 0xFFFF

#define TEMP_FAULT_COUNTER_MAX 5

//temperature *128
/*
* Die NTC Tabelle, bestehend aus 33 Temperaturstützpunkten.
* Einheit:0.1 °C
*/
 
int NTC_table[33] = {
  1835, 1493, 1151, 969, 847, 754, 678, 615, 
  560, 511, 466, 425, 386, 350, 316, 282, 250, 
  218, 187, 156, 125, 93, 61, 28, -6, -43, 
  -83, -126, -176, -235, -311, -428, -545
};
 
 
/*
* \brief    Konvertiert das ADC Ergebnis in einen Temperaturwert.
*
*           Mit p1 und p2 wird der Stützpunkt direkt vor und nach dem
*           ADC Wert ermittelt. Zwischen beiden Stützpunkten wird linear
*           interpoliert. Der Code ist sehr klein und schnell.
*           Es wird lediglich eine Ganzzahl-Multiplikation verwendet.
*           Die Division kann vom Compiler durch eine Schiebeoperation
*           ersetzt werden.
*
*           Im Temperaturbereich von -10°C bis 50°C beträgt der Fehler
*           durch die Verwendung einer Tabelle 0.169°C
*
* \param    adc_value  Das gewandelte ADC Ergebnis
* \return              Die Temperatur in 0.1 °C
*
*/
 
int NTC_ADC2Temperature(unsigned int adc_value){
 
  int p1,p2;
  /* Stützpunkt vor und nach dem ADC Wert ermitteln. */
  p1 = NTC_table[ (adc_value >> 7)  ];
  p2 = NTC_table[ (adc_value >> 7)+1];
 
  /* Zwischen beiden Punkten linear interpolieren. */
  return p1 - ( (p1-p2) * (adc_value & 0x007F) ) / 128;
};
 
 
/*
  Copyright (C) Preis Ingenieurbüro GmbH
 
  Licensed under the Apache License, Version 2.0 (the "License").
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */

void initialize_thermistor(void) {

}


int16_t get_temp_counts(uint8_t channel){
    //Therm_Mux_Select(channel);
    vTaskDelay(20);
    //ADC_therm_StartConvert();
    vTaskDelay(20);
    //return ADC_therm_GetResult16()-80;  //compensate for 100mV Offset
    return 0;
}

uint16 run_temp_check(void) {
	//this function looks at all the thermistor temperatures, compares them against limits and returns any faults
	uint16 fault = 0;
    
	tt.n.temp1.value = NTC_ADC2Temperature(ADC_NTC_GetResult16()) / 10;

	if (tt.n.temp1.value > configuration.temp1_max && configuration.temp1_max) {
		fault |= TEMP1_FAULT;
	}

	if (tt.n.temp1.value > configuration.temp1_setpoint) {
		//Fan_Write(1);
	} else if (tt.n.temp1.value < configuration.temp1_setpoint) {
		//Fan_Write(0);
	}
	return fault;
}

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void tsk_thermistor_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */
    uint16_t temp_fault_counter=0;
    
	initialize_thermistor();
    
    PWM_FAN_Start();
    

	/* `#END` */
    alarm_push(ALM_PRIO_INFO,warn_task_thermistor, ALM_NO_VALUE);
	for (;;) {
		/* `#START TASK_LOOP_CODE` */
        
        uint16_t ret = run_temp_check();
		if (ret) {
			temp_fault_counter++;
			if (temp_fault_counter > TEMP_FAULT_COUNTER_MAX) {
                if(ret&0x00FF){
                    if(sysfault.temp1==0){
                        alarm_push(ALM_PRIO_CRITICAL, warn_temp1_fault, tt.n.temp1.value);
                    }
                    sysfault.temp1 = 1;
                }
			}
		} else {
			temp_fault_counter = 0;
		}

		/* `#END` */

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
/* ------------------------------------------------------------------------ */
void tsk_thermistor_Start(void) {
	/*
	 * Insert task global memeory initialization here. Since the OS does not
	 * initialize ANY global memory, execute the initialization here to make
	 * sure that your task data is properly 
	 */
	/* `#START TASK_GLOBAL_INIT` */

	/* `#END` */

	if (tsk_thermistor_initVar != 1) {

		/*
	 	* Create the task and then leave. When FreeRTOS starts up the scheduler
	 	* will call the task procedure and start execution of the task.
	 	*/
		xTaskCreate(tsk_thermistor_TaskProc, "Therm", STACK_THERMISTOR, NULL, PRIO_THERMISTOR, &tsk_thermistor_TaskHandle);
		tsk_thermistor_initVar = 1;
	}
}
/* ------------------------------------------------------------------------ */
/* ======================================================================== */
/* [] END OF FILE */
