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

#include "cli_common.h"
#include "telemetry.h"
#include <project.h>
#include <stdint.h>
#include <stdlib.h>
#include "helper/printf.h"
#include "helper/debug.h"

#include "cyapicallbacks.h"
#include <cytypes.h>

#include "tasks/tsk_analog.h"
#include "tasks/tsk_overlay.h"
#include "tasks/tsk_priority.h"
#include "tasks/tsk_uart.h"
#include "tasks/tsk_cli.h"
#include "tasks/tsk_min.h"
#include "tasks/tsk_fault.h"
#include "min_id.h"
#include "helper/teslaterm.h"
#include "math.h"
#include "alarmevent.h"
#include "version.h"
#include "system.h"


#define UNUSED_VARIABLE(N) \
	do {                   \
		(void)(N);         \
	} while (0)
        
	
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_TuneFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_TTupdateFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_OfftimeFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_i2tFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_baudrateFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_SPIspeedFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_VisibleFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_MchFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_MchCopyFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_ivoUART(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_Vout(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);

void update_ivo_uart();

cli_config configuration;
cli_parameter param;

enum uart_ivo{
    UART_IVO_NONE=0,
    UART_IVO_TX=1,
    UART_IVO_RX=10,
    UART_IVO_RX_TX=11
};


/*****************************************************************************
* Initializes parameters with default values
******************************************************************************/
void init_config(){
    configuration.watchdog = 1;

    configuration.temp1_max = 40;
    configuration.temp1_setpoint = 30;
    configuration.baudrate = 460800;
    configuration.spi_speed = 16;
    strncpy(configuration.nt_name,"NT_1", sizeof(configuration.nt_name));
    configuration.minprot = pdFALSE;
    configuration.chargedelay = 1000;
    configuration.ivo_uart = UART_IVO_NONE;
    configuration.spt_Vout = 0;
    configuration.spt_Iprim = 0;
    
    configuration.v_pid_p = 10.0;
    configuration.v_pid_i = 1.0;
    configuration.v_pid_d = 0;

    update_ivo_uart();
    shutdown_Write(1);
    
}

// clang-format off

/*****************************************************************************
* Parameter struct
******************************************************************************/

parameter_entry confparam[] = {
    //       Parameter Type ,Visible,"Text   "         , Value ptr                     ,Min     ,Max    ,Div    ,Callback Function           ,Help text
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"watchdog"        , configuration.watchdog        , 0      ,1      ,0      ,callback_ConfigFunction     ,"Watchdog Enable")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"temp1_max"       , configuration.temp1_max       , 0      ,100    ,0      ,callback_TTupdateFunction   ,"Max temperature 1 [*C]")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"temp1_setpoint"  , configuration.temp1_setpoint  , 0      ,100    ,0      ,NULL                        ,"Setpoint for fan [*C]")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"charge_delay"    , configuration.chargedelay     , 1      ,60000  ,0      ,callback_ConfigFunction     ,"Delay for the charge relay [ms]")  
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"ud_name"         , configuration.nt_name         , 0      ,0      ,0      ,NULL                        ,"Name of the power supply [15 chars]")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"min_enable"      , configuration.minprot         , 0      ,1      ,0      ,NULL                        ,"Use MIN-Protocol")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"baudrate"        , configuration.baudrate        , 1200   ,4000000,0      ,callback_baudrateFunction   ,"Serial baudrate")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"ivo_uart"        , configuration.ivo_uart        , 0      ,11     ,0      ,callback_ivoUART            ,"[RX][TX] 0=not inverted 1=inverted")
    ADD_PARAM(PARAM_DEFAULT ,pdTRUE ,"v_out"           , configuration.spt_Vout        , 0      ,500000 ,1000   ,callback_Vout               ,"Voltage setpoint [V]")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"i_prim"          , configuration.spt_Iprim       , 0      ,50000  ,1000   ,NULL                        ,"Primary current setpoint [A]")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"v_pid_p"         , configuration.v_pid_p         , 0      ,10000  ,0      ,callback_pid                ,"Voltage PID kp")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"v_pid_i"         , configuration.v_pid_i         , 0      ,10000  ,0      ,callback_pid                ,"Voltage PID ki")
    ADD_PARAM(PARAM_CONFIG  ,pdTRUE ,"v_pid_d"         , configuration.v_pid_d         , 0      ,10000  ,0      ,callback_pid                ,"Voltage PID kd")
};


   
void eeprom_load(TERMINAL_HANDLE * handle){
    EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,handle);
    update_ivo_uart();
    update_visibilty();
    uart_baudrate(configuration.baudrate);
    init_telemetry();
    callback_pid(confparam,0,handle);
}



void update_visibilty(void){


}

// clang-format on

uint8_t callback_Vout(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    float cnt = configuration.spt_Vout / VOUT_MVOLT_CNT;
    configuration.spt_Vout_cnt = floor(cnt);
    if(configuration.spt_Vout_cnt){
        shutdown_Write(0);
    }else{
        shutdown_Write(1);
    }
    return TERM_CMD_EXIT_SUCCESS;   
}

/*****************************************************************************
* Callback for invert option UART
******************************************************************************/

void update_ivo_uart(){
    IVO_UART_Control=UART_IVO_NONE;
    switch(configuration.ivo_uart){
    case UART_IVO_NONE:
        break;
    case UART_IVO_TX:
        set_bit(IVO_UART_Control,1);
        break;
    case UART_IVO_RX:
        set_bit(IVO_UART_Control,0);
        break;
    case UART_IVO_RX_TX:
        set_bit(IVO_UART_Control,0);
        set_bit(IVO_UART_Control,1);
        break;
    }
}

uint8_t callback_ivoUART(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    if(configuration.ivo_uart == UART_IVO_NONE || configuration.ivo_uart == UART_IVO_TX || configuration.ivo_uart == UART_IVO_RX || configuration.ivo_uart == UART_IVO_RX_TX){
        update_ivo_uart();   
        return 1;
    }else{
        ttprintf("Only the folowing combinations are allowed\r\n");
        ttprintf("0  = no inversion\r\n");
        ttprintf("1  = rx inverted, tx not inverted\r\n");
        ttprintf("10 = rx not inverted, tx inverted\r\n");
        ttprintf("11 = rx and tx inverted\r\n");
        return 0;
    }
}


/*****************************************************************************
* Callback if the offtime parameter is changed
******************************************************************************/
uint8_t callback_VisibleFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    update_visibilty();
    return 1;
}


/*****************************************************************************
* Callback if the maximum current is changed
******************************************************************************/
uint8_t callback_TTupdateFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle) {
    

    uint8 sfflag = system_fault_Read();
    system_fault_Control = 0; //halt tesla coil operation during updates!
    
    
    system_fault_Control = sfflag;
    
    if (portM->term_mode!=PORT_TERM_VT100) {
        uint8_t include_chart;
        if (portM->term_mode==PORT_TERM_TT) {
            include_chart = pdTRUE;
        } else {
            include_chart = pdFALSE;
        }
        init_tt(include_chart,handle);
    }
	return 1;
}


/*****************************************************************************
* Callback if the baudrate is changed
******************************************************************************/
void uart_baudrate(uint32_t baudrate){
    float divider = (float)(BCLK__BUS_CLK__HZ/8)/(float)baudrate;
    uint16_t divider_selected=1;
    
    uint32_t down_rate = (BCLK__BUS_CLK__HZ/8)/floor(divider);
    uint32_t up_rate = (BCLK__BUS_CLK__HZ/8)/ceil(divider);
   
    float down_rate_error = (down_rate/(float)baudrate)-1;
    float up_rate_error = (up_rate/(float)baudrate)-1;
    
    UART_Stop();
    if(fabs(down_rate_error) < fabs(up_rate_error)){
        //selected round down divider
        divider_selected = floor(divider);
    }else{
        //selected round up divider
        divider_selected = ceil(divider);
    }

    UART_CLK_SetDividerValue(divider_selected);
    
    tt.n.rx_datarate.max = baudrate / 8;
    tt.n.tx_datarate.max = baudrate / 8;
    
    UART_Start();  
    
}

uint8_t callback_baudrateFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    uart_baudrate(configuration.baudrate);
 
    return 1;
}


/*****************************************************************************
* Callback if a configuration relevant parameter is changed
******************************************************************************/
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    uint8 sfflag = system_fault_Read();
    system_fault_Control = 0; //halt tesla coil operation during updates!
    WD_enable(configuration.watchdog);


    update_visibilty();
    
    recalc_telemetry_limits();
    
	system_fault_Control = sfflag;
    return 1;
}

/*****************************************************************************
* Default function if a parameter is changes (not used)
******************************************************************************/
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    
    return 1;
}


void con_info(TERMINAL_HANDLE * handle){
    #define COL_A 9
    #define COL_B 15
    ttprintf("\r\nConnected clients:\r\n");
    TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
    ttprintf("Num");
    TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
    ttprintf("| Remote IP\r\n");
    
    for(uint8_t i=0;i<NUM_MIN_CON;i++){
        if(socket_info[i].socket==SOCKET_CONNECTED){
            TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
            ttprintf("\033[36m%d", i);
            TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
            ttprintf("\033[32m%s\r\n", socket_info[i].info);
        }
    }
    TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
}

void con_numcon(TERMINAL_HANDLE * handle){
    uint8_t cnt=0;
    for(uint8_t i=0;i<NUM_MIN_CON;i++){
        if(socket_info[i].socket==SOCKET_CONNECTED){
            cnt++;
        }
    }
    ttprintf("CLI-Sessions: %u/%u\r\n",cnt ,NUM_MIN_CON);
}

/*****************************************************************************
* Displays the statistics of the min protocol
******************************************************************************/
uint8_t con_minstat(TERMINAL_HANDLE * handle){
    TERM_sendVT100Code(handle, _VT100_CLS,0);
    TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE,0);
  
    while(Term_check_break(handle,200)){
        TERM_sendVT100Code(handle, _VT100_CURSOR_POS1,0);
        ttprintf("MIN monitor    (press [CTRL+C] for quit)\r\n");
        ttprintf("%sDropped frames        : %lu\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0), min_ctx.transport_fifo.dropped_frames);
        ttprintf("%sSpurious acks         : %lu\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),min_ctx.transport_fifo.spurious_acks);
        ttprintf("%sResets received       : %lu\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),min_ctx.transport_fifo.resets_received);
        ttprintf("%sSequence mismatch drop: %lu\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),min_ctx.transport_fifo.sequence_mismatch_drop);
        ttprintf("%sMax frames in buffer  : %u\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),min_ctx.transport_fifo.n_frames_max);
        ttprintf("%sCRC errors            : %u\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),min_ctx.transport_fifo.crc_fails);
        ttprintf("%sRemote Time           : %u\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),time.remote);
        ttprintf("%sDiff Time             : %i\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),time.diff);
        ttprintf("%sResync count          : %u\r\n", TERM_getVT100Code(_VT100_ERASE_LINE_END, 0),time.resync);
    }
    TERM_sendVT100Code(handle, _VT100_CURSOR_ENABLE,0);
    return 1; 
}

/*****************************************************************************
* Prints the ethernet connections
******************************************************************************/
uint8_t CMD_con(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("con [info|numcon|min]\r\n");
    }
    
    if(strcmp(args[0], "info") == 0){
        con_info(handle);
        return TERM_CMD_EXIT_SUCCESS;
    }

    if(strcmp(args[0], "numcon") == 0){
        con_numcon(handle);
        return TERM_CMD_EXIT_SUCCESS;
    }

    if(strcmp(args[0], "min") == 0){
        con_minstat(handle);
        return TERM_CMD_EXIT_SUCCESS;
    }
    return TERM_CMD_EXIT_SUCCESS;
}


/*****************************************************************************
* Sends the features to teslaterm
******************************************************************************/
uint8_t CMD_features(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	for (uint8_t i = 0; i < sizeof(version)/sizeof(char*); i++) {
       send_features(version[i],handle); 
    }
    return TERM_CMD_EXIT_SUCCESS; 
}


uint8_t CMD_RELAY(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	if(argCount==0) return 0;
    
    uint16_t temp = atoi(args[0]);
    
    Relay1_Write(temp);
    
    return TERM_CMD_EXIT_SUCCESS; 
}

uint8_t CMD_FAN(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	if(argCount==0) return 0;
    
    uint16_t temp = atoi(args[0]);
    
    PWM_FAN_WriteCompare(temp);   
    
    return TERM_CMD_EXIT_SUCCESS; 
}

uint8_t CMD_DAC(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	if(argCount==0) return 0;
    
    uint16_t dacValue = atoi(args[0]);
    dacValue /=2;
    uint8 msb, lsb;
    
    msb = (uint8)(dacValue >> 3);
    lsb = (uint8)((dacValue << 0) & 0x07);
    
    DAC_H_SetValue(msb);
    DAC_L_SetValue(lsb);
    
    ttprintf("Set lsb: %u msb: %u\r\n",lsb,msb);
    

    return TERM_CMD_EXIT_SUCCESS; 
}

/*****************************************************************************
* Sends the configuration to teslaterm
******************************************************************************/
uint8_t CMD_config_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    char buffer[80];
	for (uint8_t current_parameter = 0; current_parameter < sizeof(confparam) / sizeof(parameter_entry); current_parameter++) {
        if(confparam[current_parameter].parameter_type == PARAM_CONFIG  && confparam[current_parameter].visible){
            print_param_buffer(buffer, confparam, current_parameter);
            send_config(buffer,confparam[current_parameter].help, handle);
        }
    }
    send_config("NULL","NULL", handle);
    return TERM_CMD_EXIT_SUCCESS; 
}




/*****************************************************************************
* Get a value from a parameter or print all parameters
******************************************************************************/
uint8_t CMD_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    
    if(argCount==0){
        print_param_help(confparam, PARAM_SIZE(confparam), handle);
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(strcmp(args[0], "-?") == 0){
        ttprintf("Usage: get [parameter]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
	for (uint8_t current_parameter = 0; current_parameter < sizeof(confparam) / sizeof(parameter_entry); current_parameter++) {
		if (strcmp(args[0], confparam[current_parameter].name) == 0) {
			//Parameter found:
			print_param(confparam,current_parameter,handle);
			return TERM_CMD_EXIT_SUCCESS;
		}
	}
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
	ttprintf("E: unknown param\r\n");
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
	return 0;
}

/*****************************************************************************
* Set a new value to a parameter
******************************************************************************/
uint8_t CMD_set(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {

	if(argCount<2){
        ttprintf("Usage: set [parameter] [value]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
  
	for (uint8_t current_parameter = 0; current_parameter < sizeof(confparam) / sizeof(parameter_entry); current_parameter++) {
		if (strcmp(args[0], confparam[current_parameter].name) == 0) {
			//parameter name found:

			if (updateDefaultFunction(confparam, args[1],current_parameter, handle)){
                if(confparam[current_parameter].callback_function){
                    if (confparam[current_parameter].callback_function(confparam, current_parameter, handle)){
                        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
                        ttprintf("OK\r\n");
                        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
                        return TERM_CMD_EXIT_SUCCESS;
                    }else{
                        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
                        ttprintf("ERROR: Callback\r\n");
                        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
                        return TERM_CMD_EXIT_SUCCESS;
                    }
                }else{
                    TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
                    ttprintf("OK\r\n");
                    TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
                    return TERM_CMD_EXIT_SUCCESS;
                }
			} else {
				TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
				ttprintf("NOK\r\n");
				TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
				return TERM_CMD_EXIT_SUCCESS;
			}
		}
	}
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
	ttprintf("E: unknown param\r\n");
	TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
	return TERM_CMD_EXIT_SUCCESS;
}


/*****************************************************************************
* Saves confparams to eeprom
******************************************************************************/
uint8_t CMD_eeprom(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: eprom [load|save]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    EEPROM_1_UpdateTemperature();
	uint8 sfflag = system_fault_Read();
	system_fault_Control = 0; //halt tesla coil operation during updates!
	if(strcmp(args[0], "save") == 0){
        EEPROM_check_hash(confparam,PARAM_SIZE(confparam),handle);
	    EEPROM_write_conf(confparam, PARAM_SIZE(confparam),0, handle);

		system_fault_Control = sfflag;
		return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "load") == 0){
        uint8 sfflag = system_fault_Read();
        system_fault_Control = 0; //halt tesla coil operation during updates!
		EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,handle);

	    system_fault_Control = sfflag;
		return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Switches the bus on/off
******************************************************************************/
uint8_t CMD_bus(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: bus [on|off]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }

	if(strcmp(args[0], "on") == 0){
		bus_command = BUS_COMMAND_ON;
		ttprintf("BUS ON\r\n");
        return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "off") == 0){
		bus_command = BUS_COMMAND_OFF;
		ttprintf("BUS OFF\r\n");
        return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Loads the default parametes out of flash
******************************************************************************/
uint8_t CMD_load_defaults(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    ttprintf("Default parameters loaded\r\n");
    init_config();
    return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Reset of the controller
******************************************************************************/
uint8_t CMD_reset(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    CySoftwareReset();
    return TERM_CMD_EXIT_SUCCESS;
}


/*****************************************************************************
* Signal debugging
******************************************************************************/

void send_signal_state(uint8_t signal, uint8_t inverted, TERMINAL_HANDLE * handle){
    if(inverted) signal = !signal; 
    if(signal){
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
        ttprintf("true \r\n");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);  
    }else{
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
        ttprintf("false\r\n");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);;
    }
}
void send_signal_state_wo(uint8_t signal, uint8_t inverted, TERMINAL_HANDLE * handle){
    if(inverted) signal = !signal; 
    if(signal){
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
        ttprintf("true ");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE); 
    }else{
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
        ttprintf("false");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
    }
}

void send_signal_state_new(uint8_t signal, uint8_t inverted, TERMINAL_HANDLE * handle){
    if(inverted) signal = !signal; 
    if(signal){
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
        ttprintf("true \r\n");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE); 
    }else{
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
        ttprintf("false\r\n");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
    }
}
void send_signal_state_wo_new(uint8_t signal, uint8_t inverted, TERMINAL_HANDLE * handle){
    if(inverted) signal = !signal; 
    if(signal){
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_RED);
        ttprintf("true ");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
    }else{
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_GREEN);
        ttprintf("false");
        TERM_sendVT100Code(handle, _VT100_FOREGROUND_COLOR, _VT100_WHITE);
    }
}

uint8_t CMD_signals(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    TERM_sendVT100Code(handle, _VT100_CLS, 0);
    TERM_sendVT100Code(handle, _VT100_CURSOR_DISABLE, 0);
    do{
        TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
        ttprintf("Signal state [CTRL+C] for quit:\r\n");
        ttprintf("**************************\r\n");
        ttprintf(" Crystal clock: ");
        send_signal_state_new((CY_GET_XTND_REG8((void CYFAR *)CYREG_FASTCLK_XMHZ_CSR) & 0x80u),pdTRUE,handle);
        
        ttprintf("Sysfault driver undervoltage: ");
        send_signal_state_new(sysfault.uvlo,pdFALSE,handle);
        ttprintf("Sysfault Temp 1: ");
        send_signal_state_new(sysfault.temp1,pdFALSE,handle);
        ttprintf("Sysfault charging: ");
        send_signal_state_new(sysfault.charge,pdFALSE,handle);
        ttprintf("Sysfault watchdog: ");
        send_signal_state_wo_new(sysfault.watchdog,pdFALSE,handle);
        ttprintf(" updating: ");
        send_signal_state_new(sysfault.update,pdFALSE,handle);
        ttprintf("Sysfault bus undervoltage: ");
        send_signal_state_new(sysfault.bus_uv,pdFALSE,handle);
        ttprintf("Sysfault interlock: ");
        send_signal_state_wo_new(sysfault.interlock,pdFALSE,handle);
        ttprintf(" link: ");
        send_signal_state_wo_new(sysfault.link_state,pdFALSE,handle);
        ttprintf(" combined: ");
        send_signal_state_new(system_fault_Read(),pdTRUE,handle);
        
        ttprintf("Relay 1: ");
        send_signal_state_wo_new(relay_read_bus(),pdFALSE,handle);
        ttprintf("Fan: ");
        send_signal_state_wo_new(Fan_Read(),pdFALSE,handle);
        ttprintf(" Temp 1: %i*C \r\n", tt.n.temp1.value);
       // uint32_t count = ADC_Vout_GetResult32();
        //uint32_t mvolt = ((55860*count)/0xFFFF)*10;
        ttprintf("ADC Vout: %u* mV\r\n", tt.n.Vout.value);
        
    }while(Term_check_break(handle,250));
    
    TERM_sendVT100Code(handle, _VT100_RESET_ATTRIB, 0);

    return TERM_CMD_EXIT_SUCCESS;
}

