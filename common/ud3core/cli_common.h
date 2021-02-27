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

#ifndef CLI_COMMON_H
#define CLI_COMMON_H

#include <device.h>
#include "cli_basic.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "TTerm.h"


void init_config();
void eeprom_load(TERMINAL_HANDLE * handle);

uint8_t command_cls(char *commandline, port_str *ptr);

void uart_baudrate(uint32_t baudrate);
void spi_speed(uint32_t speed);
void update_visibilty(void);

uint8_t CMD_signals(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_con(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_alarms(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_bus(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_calib(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_features(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_config_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_eeprom(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_load_defaults(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_set(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_reset(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

uint8_t CMD_RELAY(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_FAN(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_DAC(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

extern parameter_entry confparam[];
volatile uint8_t qcw_reg;

struct config_struct{
    uint8_t watchdog;
    uint8_t temp1_max;
    uint16_t temp1_setpoint;
    char nt_name[16];
    uint8_t minprot;
    uint32_t baudrate;
    uint8_t spi_speed;
    uint16_t chargedelay;
    uint8_t ivo_uart;
    int32_t spt_Vout;
    int32_t spt_Vout_cnt;
    int32_t spt_Iprim;
    float v_pid_p;
    float v_pid_i;
    float v_pid_d;
};
typedef struct config_struct cli_config;

struct parameter_struct{
    uint16_t    pw;
    uint16_t    pwp;
};
typedef struct parameter_struct cli_parameter;

extern cli_config configuration;
extern cli_parameter param;

#define CONF_SIZE sizeof(confparam) / sizeof(parameter_entry)

#endif