#ifndef __ETM_CAN_USER_CONFIG_H
#define __ETM_CAN_USER_CONFIG_H


#define __A36507



#ifdef __A36224_500
#define __USE_CAN_2
#define FCY_CLK                             10000000      // 10 MHz
#define FCY_CLK_MHZ                         10.000        // 10 MHz
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_HEATER_MAGNET_BOARD
#define PIN_CAN_OPERATION_LED               _LATC4
#define ETM_CAN_INTERRUPT_PRIORITY          4
#endif



#ifdef __A36507
#define __USE_CAN_1
#define __ETM_CAN_MASTER_MODULE
#define FCY_CLK                             20000000      // 10 MHz
#define FCY_CLK_MHZ                         20.000        // 10 MHz
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_ETHERNET_BOARD
#define PIN_CAN_OPERATION_LED               _LATG13
#define ETM_CAN_INTERRUPT_PRIORITY          4
#endif


#endif



