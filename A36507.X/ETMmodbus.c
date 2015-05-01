/*********************************************************************
   ETMModbus.c
 ********************************************************************/
#include <p30F6014a.h>
//#include "TCPmodbus.h"
#include "A36507.h"
#include "Buffern.h"


unsigned int ETMmodbus_timer_10ms;

static unsigned char  ETMmodbus_put_index;
static unsigned char  ETMmodbus_get_index;

static MODBUS_RESP_SMALL*  ETMmodbus_resp_ptr[ETMMODBUS_CMD_QUEUE_SIZE];

BUFFERnBYTE uart1_input_buffer;   
BUFFERnBYTE uart1_output_buffer;  

static enum _ETMmodbus_state
{
  STATE_IDLE = 0,        
  STATE_WAIT_RESPONSE,
  
} ETMmodbus_state = STATE_IDLE;
      

static unsigned char normal_reply_length;
static MODBUS_RESP_SMALL * current_response_ptr;



unsigned int checkCRC(unsigned char * ptr, unsigned int size);
/****************************************************************************
  Function:
    static BYTE ETM_queue_buffer_room()

  Input:
    index to a queue
  Description:
 	return buffer room left for the queue
  Remarks:
    None
***************************************************************************/
static unsigned char ETM_queue_buffer_room(void)
{
	unsigned char room = 0;
 
   	room  = (ETMmodbus_get_index - ETMmodbus_put_index - 1) & (ETMMODBUS_CMD_QUEUE_SIZE - 1);        
 
    return (room);
}
/****************************************************************************
  Function:
    static BYTE ETM_is_queue_empty()

  Input:
    index to a queue
  Description:
 	return length of the queue
  Remarks:
    None
***************************************************************************/
static unsigned char ETM_queue_is_empty(void)
{
	unsigned char is_empty = 0;

    if (ETMmodbus_put_index == ETMmodbus_get_index)
        	is_empty = 1;
        
    return (is_empty);
}
/****************************************************************************
  Function:
    static void ETM_queue_put_command(MODBUS_RESP_SMALL * buffer_ptr)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
static void ETM_queue_put_command(MODBUS_RESP_SMALL * buffer_ptr)
{
    if (ETM_queue_buffer_room() > 0)
    {
    	ETMmodbus_resp_ptr[ETMmodbus_put_index] = buffer_ptr;

        ETMmodbus_put_index++;
        ETMmodbus_put_index = ETMmodbus_put_index & (ETMMODBUS_CMD_QUEUE_SIZE - 1);
    
    }
    else
    	buffer_ptr->done = ETMMODBUS_ERROR_BUFFER_FULL;
        
}
/****************************************************************************
  Function:
    MODBUS_RESP_SMALL * GetNextResponsePointer(void)

  Input:
    
  Description:
  Remarks:
    None
***************************************************************************/
MODBUS_RESP_SMALL * GetNextResponsePointer(void)
{
    MODBUS_RESP_SMALL * ptr;
    
    if (ETM_queue_is_empty() == 0)
    {
    	ptr = ETMmodbus_resp_ptr[ETMmodbus_get_index]; 
        ETMmodbus_get_index++;
        ETMmodbus_get_index = ETMmodbus_get_index & (ETMMODBUS_CMD_QUEUE_SIZE - 1);
    
    }
    else
    	ptr = 0;
    
    return (ptr);    
        
}

/****************************************************************************
  Function:
		   ETM modbus interface functions
  Input:
     
  Description:
  Remarks:
    None
***************************************************************************/
void ETMModbusReadCoilsSmall(unsigned int starting_address, unsigned int quantity_coils, MODBUS_RESP_SMALL* response)
{
	response->function_code = PLC_RD_RELAY;
    response->output_address = starting_address;
    response->data_length = quantity_coils & 0xff;
    
    response->exception_code = 0;
    response->done = 0;

    ETM_queue_put_command(response);
    
}

void ETMModbusReadHoldingRegistersSmall(unsigned int starting_address, unsigned int quantity_registers, MODBUS_RESP_SMALL* response)
{
	response->function_code = PLC_RD_REGISTER;
    response->output_address = starting_address;
    response->data_length = quantity_registers;
    
    response->exception_code = 0;
    response->done = 0;

    ETM_queue_put_command(response);
    
}

void ETMModbusWriteSingleCoil(unsigned int output_address, unsigned int output_value, MODBUS_RESP_SMALL* response)
{
	response->function_code = PLC_WR_RELAY;
    response->output_address = output_address;

    response->data = output_value ? 0xff00 : 0;
    
    response->exception_code = 0;
    response->done = 0;

    ETM_queue_put_command(response);
    
}


/****************************************************************************
  Function:
    ETMmodbus_init()

  Input:
    
  Description:
  Remarks:
    None
***************************************************************************/
void ETMmodbus_init(void)
{
	  // Initialize application specific hardware
	  UART1TX_ON_TRIS = 0;
	  UART1TX_ON_IO = 1;    // always enable TX1


	    // Configure UART Interrupts
	  _U1RXIE = 0;
	  _U1RXIP = 3;
	  
	  _U1TXIE = 0;
	  _U1RXIP = 3;

	  // ----------------- UART #1 Setup and Data Buffer -------------------------//
	  // Setup the UART input and output buffers
	  uart1_input_buffer.write_location = 0;  
	  uart1_input_buffer.read_location = 0;
	  uart1_output_buffer.write_location = 0;
	  uart1_output_buffer.read_location = 0;
            
      ETMmodbus_put_index = 0;
	  ETMmodbus_get_index = 0;
	  
      U1MODE = A36507_U1MODE_VALUE;
	  U1BRG = A36507_U1BRG_VALUE;
	  U1STA = A36507_U1STA_VALUE;
	  
	  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
	  _U1TXIE = 1;	// Enable Transmit Interrupts
	  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
	  _U1RXIE = 1;	// Enable Recieve Interrupts
	  
	  U1MODEbits.UARTEN = 1;	// And turn the peripheral on


}
/****************************************************************************
  Function:
    SendCommand()

  Input:
    
  Description:
  Remarks: use uart1_input_buffer data field directly to avoid duplicate 
           data storage. 
***************************************************************************/
void LookForResponse(void) 
{
  unsigned char reply_length;
  unsigned int crc, crc_in;
  
  if (BuffernBytesInBuffer(&uart1_input_buffer) >= ETMMODBUS_RESPONSE_SIZE_MIN)
  {
  	  if (uart1_input_buffer.data[0] != PLC_SLAVE_ADDR) current_response_ptr->done = ETMMODBUS_ERROR_SLAVE_ADDR; 
  	  else 
      {
      	if ((uart1_input_buffer.data[1] & 0x7f) != current_response_ptr->function_code)
        		  current_response_ptr->done = ETMMODBUS_ERROR_FUNCTION;
        else
        {  
	      	if (uart1_input_buffer.data[1] & 0x80) 	reply_length = 5;
	        else 									reply_length = 5 + normal_reply_length;
	        
	        if (BuffernBytesInBuffer(&uart1_input_buffer) >= reply_length)  // else wait
	        { // check crc
	        	crc_in =  (uart1_input_buffer.data[reply_length - 1] << 8) + uart1_input_buffer.data[reply_length - 2];
	            crc = checkCRC(uart1_input_buffer.data, reply_length - 2); 
	            if (crc == crc_in)
	            {
                	if (reply_length == 5) 
                    {
                    	current_response_ptr->done = ETMMODBUS_ERROR_RESPONSE;
                        
                        current_response_ptr->exception_code = uart1_input_buffer.data[3];
                    }
                    else
                    {	
                    	current_response_ptr->done = ETMMODBUS_RESPONSE_OK;
                        
                        switch (current_response_ptr->function_code)
                        {
						  case PLC_RD_RELAY:
                           	 if (normal_reply_length == 1) 
                          	 	current_response_ptr->data = uart1_input_buffer.data[3];
                             else
                          	 	current_response_ptr->data = (uart1_input_buffer.data[4] << 8) + uart1_input_buffer.data[3];
                         	 break;
						  case PLC_RD_REGISTER: 
                          	 	current_response_ptr->data = (uart1_input_buffer.data[3] << 8) + uart1_input_buffer.data[4];
                             break;
                        
                          default:
                        	 break; 
                        
                        }
                    }                    
                    
	            }
                else
                	current_response_ptr->done = ETMMODBUS_ERROR_CRC;
	            
	        }
         }
  	  }
  }
  

}


/****************************************************************************
  Function:
    SendCommand()

  Input:
    
  Description:
  Remarks:
***************************************************************************/
void SendCommand(MODBUS_RESP_SMALL * ptr) 
{
  unsigned int crc;
  
  // clear input/output buffer first
  uart1_input_buffer.write_location = 0;  
  uart1_input_buffer.read_location = 0;
  uart1_output_buffer.write_location = 0;
  uart1_output_buffer.read_location = 0;
  
  switch (ptr->function_code)
  {
  case PLC_RD_RELAY:
  case PLC_RD_REGISTER:  
	  BuffernWriteByte(&uart1_output_buffer, PLC_SLAVE_ADDR);
	  BuffernWriteByte(&uart1_output_buffer, ptr->function_code);    
	  BuffernWriteByte(&uart1_output_buffer, (ptr->output_address >> 8) & 0xff);	// addr Hi
	  BuffernWriteByte(&uart1_output_buffer, ptr->output_address & 0xff);	// addr Lo
	  BuffernWriteByte(&uart1_output_buffer, (ptr->data_length>> 8) & 0xff);	// length Hi
	  BuffernWriteByte(&uart1_output_buffer, ptr->data_length & 0xff);	// length Lo
	  
      crc = checkCRC(uart1_output_buffer.data, 6);
	  BuffernWriteByte(&uart1_output_buffer, crc & 0xff);
	  BuffernWriteByte(&uart1_output_buffer, (crc >> 8) & 0xff);
	
      // calculate expected length for normal reply
      if (ptr->function_code == PLC_RD_RELAY)  normal_reply_length = 1 + ptr->data_length / 8;
      else									   normal_reply_length = ptr->data_length * 2;
      
      break;
      
  case PLC_WR_RELAY:
  case PLC_WR_REGISTER:
	  BuffernWriteByte(&uart1_output_buffer, PLC_SLAVE_ADDR);
	  BuffernWriteByte(&uart1_output_buffer, ptr->function_code);    
	  BuffernWriteByte(&uart1_output_buffer, (ptr->output_address >> 8) & 0xff);	// addr Hi
	  BuffernWriteByte(&uart1_output_buffer, ptr->output_address & 0xff);	// addr Lo
	  BuffernWriteByte(&uart1_output_buffer, (ptr->data >> 8) & 0xff);	// data Hi
	  BuffernWriteByte(&uart1_output_buffer, ptr->data & 0xff);	// data Lo
	  
      crc = checkCRC(uart1_output_buffer.data, 6);
	  BuffernWriteByte(&uart1_output_buffer, crc & 0xff);
	  BuffernWriteByte(&uart1_output_buffer, (crc >> 8) & 0xff);
      
      normal_reply_length = 8; // the reply is the same as the command if no error
      break;
      
  default:
  	  break;
  }
  
  while (U1STAbits.UTXBF);	// wait for output buffer empty, shouldn't be long
  
  // setup timer and flag
  ptr->done = 0;
  ETMmodbus_timer_10ms = 0;
  
  
  if ((!U1STAbits.UTXBF) && (BuffernIsNotEmpty(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U1TXREG = BuffernReadByte(&uart1_output_buffer);
  }
}

/****************************************************************************
  Function:
    ETMmodbus_task()

  Input:
    
  Description:
  Remarks:
    Need to call this task periodically
***************************************************************************/
void ETMmodbus_task(void)
{

	  switch (ETMmodbus_state)
      {
	  case STATE_IDLE:
      	 if (ETM_queue_is_empty() == 0)
         {
         	 current_response_ptr = GetNextResponsePointer();             
             if (current_response_ptr != 0)  
             {	// send command out
             	 SendCommand(current_response_ptr);
             	 ETMmodbus_state = STATE_WAIT_RESPONSE;
             }
         }
      	break;
	  case STATE_WAIT_RESPONSE:
      	LookForResponse();
		if (current_response_ptr->done)
        {
			  uart1_input_buffer.write_location = 0;  
			  uart1_input_buffer.read_location = 0;
        	  ETMmodbus_state = STATE_IDLE;        
		}
      	else if (ETMmodbus_timer_10ms > ETMMODBUS_TIMEOUT_SET)
        {
        	 current_response_ptr->done = ETMMODBUS_ERROR_TIMEOUT;
             ETMmodbus_state = STATE_IDLE;
        }
      	break;
	  default:
      	break;
        
      }

      
}

//-----------------------------------------------------------------------------
// CRC_Check
//-----------------------------------------------------------------------------
//
// Return Value : accum -- the end result of the CRC value.
// Parameter    : *ptr -- pointer to the array for CRC calculation.
//				: size -- size of the array
//
// This function calculates the 16-bit CRC value for the input array.
//
//-----------------------------------------------------------------------------
#define CRC_POLY  0xA001
unsigned int checkCRC(unsigned char * ptr, unsigned int size)
{
    unsigned int i, j;
    unsigned int accum, element;

    accum = 0xffff;

    for (j = 0; j < size; j++)
    {
        element = ptr[j];

        for (i = 8; i > 0; i--)		
        {
            if (((element ^ accum) & 0x0001) > 0)
                accum = (unsigned int)((accum >> 1) ^ ((unsigned int)CRC_POLY));
            else
                accum >>= 1;

            element >>= 1;
        }
    }

    return (accum);
}

//-----------------------------------------------------------------------------
//   UART Interrupts
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    BuffernWriteByte(&uart1_input_buffer, U1RXREG);
  }
}



void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  while ((!U1STAbits.UTXBF) && (BuffernBytesInBuffer(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = BuffernReadByte(&uart1_output_buffer);
  }
}

