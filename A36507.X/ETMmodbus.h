#ifndef __ETM_MODBUS_H
#define __ETM_MODBUS_H

#include <uart.h>
/* 
   The following must be global defines included in the project configuration
   ETM_FCY_CLK - defined as the processor FCY
   ETM_MODBUS_UART - defined as the UART for the modbus module to use
*/


/*
  When a command is called, the modbus module clears the "done" status for that response
  When the command completes (normally or with an error) the modbus module sets the "done" status.

  The ETM Modbus module must be able to buffer a number of modbus requests.
  The internal buffer should be 16 modbus "commands" deep.
  They should be executed in the order they were received.
  If the Modbus commond buffer is full, the response should be marked as "done" with a unique exeception code

  **** The user code is responsible for polling the done status and executing accordingly
  **** The user code is responsible for insuring that data in the response structure is not overwritten

*/
extern void ETMmodbus_init(void);

extern void ETMmodbus_task(void);

extern unsigned int ETMmodbus_timer_10ms;  // for timeout response

#define UART1_BAUDRATE             9600        // U1 Baud Rate

#define A36507_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36507_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A36507_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

#define UART1TX_ON_TRIS		(TRISDbits.TRISD7)
#define UART1TX_ON_IO		(PORTDbits.RD7)


// PLC slave address
#define PLC_SLAVE_ADDR       2

// PLC modbus functions
#define PLC_RD_RELAY         1
#define PLC_RD_REGISTER      3
#define PLC_WR_RELAY         5
#define PLC_WR_REGISTER      6
 


#define   ETMMODBUS_CMD_QUEUE_SIZE  16
#define   ETMMODBUS_TIMEOUT_SET     100  /* in 10ms, 1s timeout */


#define   ETMMODBUS_RESPONSE_SIZE_MIN      5  /* minimum bytes from PLC response */

#define   ETMMODBUS_ERROR_TIMEOUT       0xA0
#define   ETMMODBUS_ERROR_BUFFER_FULL	0xA1
#define   ETMMODBUS_ERROR_SLAVE_ADDR    0xA2
#define   ETMMODBUS_ERROR_FUNCTION      0xA3
#define   ETMMODBUS_ERROR_RESPONSE      0xA4   /* error response 0x80 | function code */
#define   ETMMODBUS_ERROR_CRC			0xA5

#define   ETMMODBUS_RESPONSE_OK		    1




typedef struct {
  unsigned char function_code;
  unsigned char data_length;
  unsigned char exception_code;
  unsigned char done;
  unsigned int  output_address;
  unsigned int  data;
} MODBUS_RESP_SMALL;


typedef struct {
  unsigned char function_code;
  unsigned char data_length;
  unsigned char exception_code;
  unsigned char done;
  unsigned int  output_address;
  unsigned int* data[125];
} MODBUS_RESP;



/*
  Calling this function starts (restarts) the modbus module at the specified baud rate using the global #defined ETM_MODBUS_UART
*/


void ETMModbusReadCoilsSmall(unsigned int starting_address, unsigned int quantity_coils, MODBUS_RESP_SMALL* response);
/* 
   starting_address must be in range 0x0000 -> 0xFFFF
   quantity_coils must be in range 0x0001 -> 0x0010
*/

void ETMModbusReadHoldingRegistersSmall(unsigned int starting_address, unsigned int quantity_registers, MODBUS_RESP_SMALL* response);
/*
  starting_address must be in range 0x0000 -> 0xFFFF
  quantity_coils must be equal to 0x0001
*/

void ETMModbusWriteSingleCoil(unsigned int output_address, unsigned int output_value, MODBUS_RESP_SMALL* response);
/*
  output address must be in range 0x0000 -> 0xFFFF
  output value is 0x0000 or (anything else)
  
*/




void ETMModbusReadCoils(unsigned int starting_address, unsigned int quantity_coils, MODBUS_RESP* response);
/* 
   starting_address must be in range 0x0000 -> 0xFFFF
   quantity_coils must be in range 0x0001 -> 0x07D0
*/

void ETMModbusReadHoldingRegisters(unsigned int starting_address, unsigned int quantity_registers, MODBUS_RESP* response);
/*
  starting_address must be in range 0x0000 -> 0xFFFF
  quantity_coils must be in range 0x0001 -> 0x007D
*/



#endif
