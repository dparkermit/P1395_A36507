#ifndef __A36507_H
#define __A36507_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>

#include <timer.h>
#include "P1395_MODULE_CONFIG.h"

#include "ETM_ANALOG.h"
#include "DS3231.h"
#include "ETM_EEPROM.h"
#include "TCPmodbus.h"

//#include "ETM_CAN_PUBLIC.h"
//#include "ETM_CAN_P1395_MASTER.h"
#include "P1395_CAN_MASTER.h"





/*
  Hardware Module Resource Usage

  CAN1 - Can module
  CAN2 - Reserved in case we need CAN 2

  Timer1 - Used by Ehternet Module
  Timer2 - Used to time CAN transmits - This is configured by ETM CAN module
  Timer3 - Used as timeout on status update receives - This is configured by ETM CAN module

  Timer5 - Used for ethernet board timing

  UART1 - Reserved for TCU Communication
  UART2 - Reserved for Serial GUI


 */









// ----------------- IO PIN CONFIGURATION -------------------- //
/*
  All unused pins will be set to outputs and logic zero
  LAT values default to 0 at startup so they do not need to be manually set
*/



// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RA9  (Accidentally left grounded)
  RG0  (Unused Can Pin)
  RG1  (Unused Can Pin)
  RG14 (Reset Detect)
  RB14 (Analog Input)
  RB15 (Analog Input)
  
  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RB0 PROGRAM
  RB1 PROGRAM

  RF0 CAN 1
  RF1 CAN 1
  RF2 UART 1
  RF3 UART 1
  RF4 UART 2
  RF5 UART 2
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RA14 (Ethernet Module Interrupt Input)
  RA15 (Ethernet Module Reset Output)
  RD14 (Ethernet Module Clock Input)
  RD15 (Ethernet Module CS Output)
  

*/

#define A36507_TRISA_VALUE 0b1100001000000000 
#define A36507_TRISB_VALUE 0b0110000000000011 
#define A36507_TRISC_VALUE 0b0000000000000000 
#define A36507_TRISD_VALUE 0b1100000000000000 
#define A36507_TRISF_VALUE 0b0000000111111111 
#define A36507_TRISG_VALUE 0b0100000000001111


#define PIN_OUT_ETM_UART_1_DE                 _LATD7
#define PIN_OUT_ETM_UART_2_DE                 _LATD6
#define OLL_UART_TX_DRIVER_ENABLE             1

#define PIN_IN_ETM_RESET_DETECT               _RG14

#define PIN_OUT_ETM_LED_OPERATIONAL_GREEN     _LATA7
#define PIN_OUT_ETM_LED_TEST_POINT_A_RED      _LATG12
#define PIN_OUT_ETM_LED_TEST_POINT_B_GREEN    _LATG13
#define OLL_LED_ON                            0

#define PIN_OUT_TP_13                         _LATC15
#define PIN_OUT_TP_14                         _LATB7
#define PIN_OUT_TP_15                         _LATB8
#define PIN_OUT_TP_16                         _LATB9


// --------------- CONFIGURE TMR5 MODULE ----------------------- //
#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)



// ------------------------ CONFIGURE ADC MODULE ------------------- //
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_16 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)

#define ADPCFG_SETTING          (ENABLE_AN13_ANA & ENABLE_AN14_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 &  SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN15)

#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN13 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN14 & ADC_CH0_NEG_SAMPLEB_VREFN)








// ---------------------- FAULTS/WARNINGS ------------------------ //
#define FAULT_A36507_CAN_TIMEOUT              0b0000 0000 0000 0001
#define FAULT_A36507_CAN_ETHERNET_TIMEOUT     0b0000 0000 0000 0010
//#define FAULT_A36507_     




typedef struct {
  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_3v3_mon;                   // 1mV per LSB

  unsigned int control_state;
  unsigned int thyratron_warmup_counter_seconds;
  unsigned int magnetron_heater_warmup_counter_seconds;
  unsigned int gun_driver_heater_warmup_counter_seconds;

  unsigned int millisecond_counter;
  unsigned int warmup_timer_stage;
  unsigned int warmup_done;

  unsigned long magnetron_heater_last_warm_seconds;
  unsigned long thyratron_heater_last_warm_seconds;
  unsigned long gun_driver_heater_last_warm_seconds;


  unsigned long system_powered_seconds;
  unsigned long system_hv_on_seconds;
  unsigned long system_xray_on_seconds;

  RTC_TIME time_now;
  unsigned long time_seconds_now;
  

  //unsigned int send_pulse_sync_config;
  unsigned int drive_up_timer;

  unsigned int average_output_power_watts;
  unsigned int event_log_counter;
  
  unsigned int startup_counter;

} A36507GlobalVars;



extern A36507GlobalVars global_data_A36507;

#define _STATUS_X_RAY_DISABLED                          _STATUS_0
#define _STATUS_PERSONALITY_LOADED                      _STATUS_1

#define _FAULT_DRIVE_UP_TIMEOUT                         _FAULT_0
#define _FAULT_COOLING_NOT_CONNECTED                    _FAULT_1
#define _FAULT_COOLING_NOT_READY                        _FAULT_2

#define _FAULT_GUN_HEATER_OFF                           _FAULT_7
#define _FAULT_HV_LAMBDA_NOT_OPERATE                    _FAULT_8
#define _FAULT_ION_PUMP_NOT_OEPRATE                     _FAULT_9
#define _FAULT_AFC_NOT_OPERATE                          _FAULT_A
#define _FAULT_COOLING_NOT_OPERATE                      _FAULT_B
#define _FAULT_HTR_MAG_NOT_OPERATE                      _FAULT_C
#define _FAULT_GUN_DVR_NOT_OPERATE                      _FAULT_D
#define _FAULT_PULSE_CURRENT_MON_NOT_OPERATE            _FAULT_E
#define _FAULT_PULSE_SYNC_NOT_OPERATE                   _FAULT_F



// These are computed from Filament Lookup Table worksheet
// https://docs.google.com/spreadsheets/d/18de5OHQ0gJUx2U1b8VjYTvutx2ACGTG0XtN_QEIc9WI/

#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095 0xFFF,0xFD6,0xFD6,0xFAE,0xFAE,0xF85,0xF5C,0xF33,0xF0A,0xF0A,0xEE1,0xEB8,0xE8F,0xE66,0xE3D,0xE14,0xDEB,0xDC2,0xD70,0xD47,0xD1E,0xCF5,0xCA3,0xC7A,0xC51,0xBFF,0xBD6,0xB85,0xB5C,0xB0A,0xAB8,0xA8F,0xA3D,0x9EB,0x9C2,0x970,0x91E,0x8CC,0x87A,0x828,0x7D7,0x785,0x733,0x6E1,0x68F,0x63D,0x5EB,0x599,0x51E,0x4CC,0x47A,0x3FF,0x3AE,0x35C,0x2E1,0x28F,0x214,0x199,0x147,0xCC,0x7A,0x0,0x0,0x0 

#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193 0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xF85,0xF0A,0xE8F,0xE14,0xD70,0xCF5,0xC7A,0xBFF,0xB85,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0


#define EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC               0x000
#define EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA                 0x001
#define EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV                   0x002
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1          0x003
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2          0x004
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3          0x005
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4          0x006

#define EEPROM_PAGE_ON_TIME                                 0x007
#define EEPROM_PAGE_HEATER_TIMERS                           0x008
// EEPROM PAGES reserved for future use                     9->F





#define STATE_STARTUP                                0x10
#define STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC   0x12
#define STATE_WAITING_FOR_INITIALIZATION             0x15
#define STATE_WARMUP                                 0x20
#define STATE_STANDBY                                0x30
#define STATE_DRIVE_UP                               0x40
#define STATE_READY                                  0x50
#define STATE_XRAY_ON                                0x60


#define STATE_FAULT_HOLD                             0x80
#define STATE_FAULT_RESET                            0x90
#define STATE_FAULT_SYSTEM                           0xA0




#endif
