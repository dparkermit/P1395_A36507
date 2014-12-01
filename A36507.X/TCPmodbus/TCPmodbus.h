
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


void GenericTCPClient(void);

void TCPmodbus_init(void);

void TCPmodbus_task(void);




#define MAX_TX_SIZE    255
#define MAX_DATA_SIZE  200	// leave some room for modbus header
#define MODBUS_ARRAY_SIZE  15

typedef struct __attribute__((__packed__)) modbusStruct 
{
  unsigned int reference_num;          // reference #, or register address
  unsigned char length;                 // data length in word size
  unsigned char data[MAX_DATA_SIZE];      // data to be sent
  unsigned char is_write;				// write data to computer
  unsigned char poll_behind_pw;         // only send data out in super user mode

} MODBUS;



typedef struct {
  unsigned char* data_ptr;             // This points the the array that we want to transfer
  unsigned char  data_length;          // This is the length (in bytes) of the array that we want to transfer
  unsigned char  index;                // This is the index that the GUI is expecting
  unsigned int   reference_num;        // This is the register address for the data we are updating on the GUI
} ETMEthernetData;
  



//#ifdef IS_MODBUS_MAIN
//MODBUS modbus_array[MODBUS_ARRAY_SIZE]; 
//#else					 
extern MODBUS modbus_array[MODBUS_ARRAY_SIZE]; 
//#endif

extern void InitModbusArray(void);


#endif
