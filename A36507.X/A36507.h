#ifndef __A36507_H
#define __A36507_H

#include <p30f6014a.h>
#include <libpic30.h>




/*
  Hardware Module Resource Usage

  CAN1 - Can module
  CAN2 - Reserved in case we need CAN 2

  Timer2 - Used to time CAN transmits - This is configured by ETM CAN module
  Timer3 - Used as timeout on status update receives - This is configured by ETM CAN module

  UART1 - Reserved for TCU Communication
  UART2 - Reserved for Serial GUI

  
  


 */









// ----------------- IO PIN CONFIGURATION -------------------- //
/*
  All unused pins will be set to outputs and logic zero
  LAT values default to 0 at startup so they do not need to be manually set
*/


// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN13 - 5V    Voltage Monitor 
   AN14 - 3.3V  Voltage Monitor
   
*/
#define ADPCFG_VALUE       //0b1001 1111 1111 1111



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

#define A36507_TRISA_VALUE //0b1100 0010 0000 0000 
#define A36507_TRISB_VALUE //0b0110 0000 0000 0011 
#define A36507_TRISC_VALUE //0b0000 0000 0000 0000 
#define A36507_TRISD_VALUE //0b1100 0000 0000 0000 
#define A36507_TRISF_VALUE //0b0000 0001 1111 1111 
#define A36507_TRISG_VALUE //0b0100 0000 0000 1111


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



// ------------------------ CONFIGURE ADC MODULE ------------------- //




// ---------------------- FAULTS/WARNINGS ------------------------ //
#define FAULT_A36507_CAN_TIMEOUT              0b0000 0000 0000 0001
#define FAULT_A36507_CAN_ETHERNET_TIMEOUT     0b0000 0000 0000 0010
//#define FAULT_A36507_     


#endif
