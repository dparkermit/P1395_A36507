
/*********************************************************************
 *
 *  Main Application Entry Point and TCP/IP Stack Demo
 *  Module for Microchip TCP/IP Stack
 *   -Demonstrates how to call and use the Microchip TCP/IP stack
 *   -Reference: Microchip TCP/IP Stack Help (TCPIPStack Help.chm)1150

 *
 *********************************************************************/
#ifndef __TCP_MODBUS_H
#define __TCP_MODBUS_H

#include "../ETM_CAN.h"

void TCPmodbus_init(void);

void TCPmodbus_task(void);




#define MAX_TX_SIZE    255
//#define MAX_DATA_SIZE  200	// leave some room for modbus header

#define MODBUS_COMMAND_TOTAL     10

enum
{
	MODBUS_WR_HVLAMBDA = 1, 	
	MODBUS_WR_ION_PUMP,
	MODBUS_WR_AFC,
	MODBUS_WR_COOLING,
	MODBUS_WR_HTR_MAGNET,
	MODBUS_WR_GUN_DRIVER,
	MODBUS_WR_MAGNETRON_CURRENT,
	MODBUS_WR_PULSE_SYNC,
	MODBUS_WR_ETHERNET,
	MODBUS_RD_COMMAND,


};



// STANDARD LOGGING DATA

typedef struct {
  ETMCanStatusRegister*  status_data;                  // This is 12 bytes of data
  ETMCanSystemDebugData* debug_data;                   // This is 48 bytes of data
  ETMCanCanStatus*       can_status;                   // This is 32 bytes of data
  ETMCanAgileConfig*     configuration;                // This is 16 bytes of data

  unsigned int*          custom_data;                  // This can be zero -> N bytes of Data
  unsigned char          custom_data_word_count;

  unsigned char          data_identification;          // This is a unique identifier for each data set
} ETMEthernetTXDataStructure;


// PUBLIC Variables
extern ETMEthernetTXDataStructure   eth_tx_hv_lamdba;
extern ETMEthernetTXDataStructure   eth_tx_ion_pump;
extern ETMEthernetTXDataStructure   eth_tx_afc;
extern ETMEthernetTXDataStructure   eth_tx_cooling;
extern ETMEthernetTXDataStructure   eth_tx_heater_magnet;
extern ETMEthernetTXDataStructure   eth_tx_gun_driver;
extern ETMEthernetTXDataStructure  	eth_tx_magnetron_current;
extern ETMEthernetTXDataStructure  	eth_tx_pulse_sync;
extern ETMEthernetTXDataStructure  	eth_tx_ethernet_board;



/*
Therefore our message in the "data_buffer" would look something like this
data_buffer[0] = data_identification;                   // Transaction ID High Byte
data_buffer[1] = (transaction identification)           // Transaction ID Low Byte
data_buffer[2] = 0;                                     // Protocal ID High Byte
data_buffer[3] = 0;                                     // Protocal ID Low Byte
data_buffer[4] = 0;                                     // Data Length High Byte = 0
data_buffer[5] = n;                                     // Data Length Low BYte =  2 + 108 + custom_data_element_count*2
data_buffer[6] = 0;                                     // Unit ID = 0
data_buffer[7] = 0x10;                                  // Function Code
data_buffer[8]
.
.
.
.
.
data_buffer[115] = Standard Data
data_buffer[116]
data_buffer[116+2*custom_data_element_count] = Board specific data

*/
typedef struct {
  unsigned int command_ready;
  unsigned int command_index;
  unsigned int command_data;
} DevelopmentRegister;

#if 0

// PULSE BY PULSE LOGGING DATA
typedef struct {
  // we need to store 36 bytes of data each pulse.
  // If we send out data once every 8 pulses then we will have 288 bytes of data + 8 bytes of header which means 296 bytes total.
  // This will be our largest transmit so the transmit buffer must be at leas this large
  // We can impliment this later but keep in mind that it will need to happen
  
} ETMETHERNETPULSEDATALOG;


unsigned int set_points[size_tbd];
// Element 0 and Element 1 would store the GUI set command authorization
// Element 0 would be the "technicial level" command authorization
// Element 1 would be the "engineer level" command authoriization
// Element 2 through N would store the configuration data 



unsigned int development_test_register[size_tbd];
// This would command

typedef struct {
  unsigned int command_ready;
  unsigned int command_index;
  unsigned int command_data;
} DevelopmentRegister;

#endif


#endif
