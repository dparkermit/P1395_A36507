#ifndef __ETM_CAN_CONFIG_H
#define __ETM_CAN_CONFIG_H


#define __ETM_CAN_MASTER_MODULE


#define __USE_CAN_1

#ifdef __ETM_CAN_MASTER_MODULE
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_ETHERNET_BOARD
#else
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_HV_LAMBDA_BOARD
#endif



#define ETM_CAN_CXCFG1_VALUE                CXCFG1_10MHZ_FCY_VALUE

#define ETM_CAN_INTERRUPT_PRIORITY          4



#endif



