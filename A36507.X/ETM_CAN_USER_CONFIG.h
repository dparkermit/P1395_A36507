#ifndef __ETM_CAN_USER_CONFIG_H
#define __ETM_CAN_USER_CONFIG_H

#define FCY_CLK                    20000000      // 10 MHz
#define FCY_CLK_MHZ                20.000        // 10 MHz



#define __USE_CAN_1
#define __ETM_CAN_MASTER_MODULE

// This has been configured for A36507 (master mode)


// DPARKER move this to ETM_CAN.h based in FCY_CLK
#define ETM_CAN_CXCFG1_VALUE                CXCFG1_20MHZ_FCY_VALUE


#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_ETHERNET_BOARD
#define ETM_CAN_STATUS_REGISTER_MASK        0x0000  // DPARKER CONFIGURE
#define ETM_CAN_FAULT_REGISTER_MASK         0x0000  // DPARKER CONFIGURE

//#define ETM_CAN_FAULT_MASK                 0b0001111111111111  // These are faults that are active on this board
//#define ETM_CAN_INHIBIT_MASK               0b0000000100000100  // These are the inhibit bits that are active on this board


#define ETM_CAN_INTERRUPT_PRIORITY          4



#endif



