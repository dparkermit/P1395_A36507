#ifndef __ETM_CAN_USER_CONFIG_H
#define __ETM_CAN_USER_CONFIG_H

#define FCY_CLK                    10000000      // 10 MHz
#define FCY_CLK_MHZ                10.000        // 10 MHz

// This has been configured for A36224_500 (master mode)

//#define __ETM_CAN_MASTER_MODULE

#define __USE_CAN_2
#define __A36224_500
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_HEATER_MAGNET_BOARD


#define ETM_CAN_FAULT_MASK                 0b0001111111111111  // These are faults that are active on this board
#define ETM_CAN_INHIBIT_MASK               0b0000000100000100  // These are the inhibit bits that are active on this board



/*
  
#ifdef __ETM_CAN_MASTER_MODULE
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_ETHERNET_BOARD
#define ETM_CAN_STATUS_REGISTER_MASK        0x0000  // DPARKER CONFIGURE
#define ETM_CAN_FAULT_REGISTER_MASK         0x0000  // DPARKER CONFIGURE
#else
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_HV_LAMBDA_BOARD
#define ETM_CAN_STATUS_REGISTER_MASK        0x0000  // DPARKER CONFIGURE
#define ETM_CAN_FAULT_REGISTER_MASK         0x0000  // DPARKER CONFIGURE
#endif

*/


#define ETM_CAN_CXCFG1_VALUE                CXCFG1_10MHZ_FCY_VALUE

#define ETM_CAN_INTERRUPT_PRIORITY          4



#endif



