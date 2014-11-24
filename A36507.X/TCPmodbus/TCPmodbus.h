
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


#if defined(STACK_USE_UART)

	#define BAUD_RATE       (19200)		// bps

	void DoUARTConfig(void);

	#define SaveAppConfig(a)
// An actual function defined in MainDemo.c for displaying the current IP
// address on the UART and/or LCD.
void DisplayIPValue(IP_ADDR IPVal);

#endif

void GenericTCPClient(void);

void TCPmodbus_init(void);

void TCPmodbus_task(void);




#define MAX_TX_SIZE    125
#define MAX_DATA_SIZE  100	// leave some room for modbus header
#define MODBUS_ARRAY_SIZE  15

typedef struct __attribute__((__packed__)) modbusStruct 
{
	unsigned int reference_num;          // reference #, or register address
	unsigned char length;                 // data length in word size
	unsigned char data[MAX_DATA_SIZE];      // data to be sent
	unsigned char is_write;				// write data to computer
	unsigned char poll_behind_pw;         // only send data out in super user mode

} MODBUS;

#ifdef IS_MODBUS_MAIN
MODBUS modbus_array[MODBUS_ARRAY_SIZE]; 
#else					 
extern MODBUS modbus_array[MODBUS_ARRAY_SIZE]; 
#endif

extern void InitModbusArray(void);


#endif