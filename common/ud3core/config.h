#if !defined(config_H)
#define config_H

//Timing constants
#define MIDI_ISR_Hz 8000
#define SG_CLOCK_Hz 320000


#define MIDI_ISR_US (1e6 / MIDI_ISR_Hz)
#define SG_PERIOD_NS (1e9 / SG_CLOCK_Hz)
#define SG_CLOCK_HALFCOUNT (SG_CLOCK_Hz/2)


//FreeRTOS
#define ACTIVATE_TASK_INFO 1
#define HEAP_SIZE 50    //kb
    
    
//MIN
#define NUM_MIN_CON 4
#define STREAMBUFFER_RX_SIZE    256     //bytes
#define STREAMBUFFER_TX_SIZE    1024    //bytes  
    
//Alarm-Event
#define AE_QUEUE_SIZE 50            //Length of the alarm and event queue
    
//Analog-Task
#define MAX_DC_MVOLT 558600
#define MAX_DC_MVOLT_10 (MAX_DC_VOLT/10)    
#define VOUT_MVOLT_CNT (8.52355957)
#define MAX_DAC_VAL 2047

    
#endif