/*********************************************************************
 *
 *  Generic TCP Client Example Application
 *  Module for Microchip TCP/IP Stack
 *   -Implements an example HTTP client and should be used as a basis 
 *	  for creating new TCP client applications
 *	 -Reference: None.  Hopefully AN833 in the future.
 *
 *********************************************************************
 * FileName:        GenericTCPClient.c
 * Dependencies:    TCP, DNS, ARP, Tick
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder     8/01/06	Original
 ********************************************************************/
#define __GENERICTCPCLIENT_C

#define THIS_IS_STACK_APPLICATION

#define IS_MODBUS_MAIN  1


#include "TCPIPStack/TCPIPStack/TCPIPConfig.h"
#include "TCPIPStack/TCPIPStack/TCPIP.h"

#include <p30F6014a.h>
#include "TCPmodbus.h"

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
static unsigned short wOriginalAppConfigChecksum;    // Checksum of the ROM defaults for AppConfig

#define LED_PUT(a)	do{unsigned char vTemp = (a); LEDOP_IO = vTemp&0x1; LEDA_IO = vTemp&0x4; LEDB_IO = vTemp&0x2;} while(0)

void GenericTCPClient(void);
void InitModbusData(void);


ETMEthernetMessageFromGUI    eth_message_from_GUI[ ETH_GUI_MESSAGE_BUFFER_SIZE ];
ETMEthernetCalToGUI          eth_cal_to_GUI[ ETH_CAL_TO_GUI_BUFFER_SIZE ];



static BYTE         data_buffer[MAX_TX_SIZE];
static BYTE         modbus_send_index = 0;

static BYTE         modbus_refresh_index = 0;
static BYTE         modbus_command_request = 0;  /* how many commands from GUI */

//static BYTE         super_user_mode = 0;
static WORD         transaction_number = 0;

static BYTE         modbus_cmd_need_repeat = 0;  


static BYTE         eth_message_from_GUI_put_index;
static BYTE         eth_message_from_GUI_get_index;

static BYTE         eth_cal_to_GUI_put_index;
static BYTE         eth_cal_to_GUI_get_index;

static BYTE         send_high_speed_data_buffer;  /* bit 0 high for buffer A, bit 1 high for buffer B */

#ifdef TEST_MODBUS
unsigned char event_data[ETH_EVENT_SIZE];
#endif

#define QUEUE_MESSAGE_FROM_GUI  1
#define QUEUE_CAL_TO_GUI        2
/****************************************************************************
  Function:
    static BYTE queue_buffer_room(q_index)

  Input:
    index to a queue
  Description:
 	return buffer room left for the queue
  Remarks:
    None
***************************************************************************/
static BYTE queue_buffer_room(BYTE q_index)
{
	BYTE room = 0;
    BYTE put_index;
    BYTE get_index;
    BYTE size;
    
	switch (q_index) {
	case QUEUE_MESSAGE_FROM_GUI: 
    	put_index = eth_message_from_GUI_put_index;
        get_index = eth_message_from_GUI_get_index;
        size = ETH_GUI_MESSAGE_BUFFER_SIZE;
    	break;
	case QUEUE_CAL_TO_GUI:
    	put_index = eth_cal_to_GUI_put_index;
        get_index = eth_cal_to_GUI_get_index;
        size = ETH_CAL_TO_GUI_BUFFER_SIZE;
    	break;
	default:
    	room = 0xff; // not defined
    	break;
    }
    
    if (room != 0xff)
    {
    	room  = (get_index - put_index - 1) & (size - 1);        
    }
    else
    	room = 0;
        
    return (room);
}
/****************************************************************************
  Function:
    static BYTE is_queue_empty(index)

  Input:
    index to a queue
  Description:
 	return length of the queue
  Remarks:
    None
***************************************************************************/
static BYTE queue_is_empty(BYTE q_index)
{
	BYTE is_empty = 0;
    BYTE put_index;
    BYTE get_index;
    BYTE size;
    
	switch (q_index) {
	case QUEUE_MESSAGE_FROM_GUI: 
    	put_index = eth_message_from_GUI_put_index;
        get_index = eth_message_from_GUI_get_index;
        size = ETH_GUI_MESSAGE_BUFFER_SIZE;
    	break;
	case QUEUE_CAL_TO_GUI:
    	put_index = eth_cal_to_GUI_put_index;
        get_index = eth_cal_to_GUI_get_index;
        size = ETH_CAL_TO_GUI_BUFFER_SIZE;
    	break;
	default:
    	is_empty = 0xff; // not defined
    	break;
    }
    
    if (is_empty != 0xff)
    {
    	if (put_index == get_index)
        	is_empty = 1;
        // else default is_empty = 0 	    
    }
    else
     	is_empty = 0; // not defined
        
    return (is_empty);
}
/****************************************************************************
  Function:
    static void queue_put_command(ETMEthernetMessageFromGUI command)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
static void queue_put_command(BYTE * buffer_ptr)
{
    if (queue_buffer_room(QUEUE_MESSAGE_FROM_GUI) > 0)
    {
    	eth_message_from_GUI[eth_message_from_GUI_put_index].index = (*buffer_ptr << 8) | *(buffer_ptr + 1);
    	eth_message_from_GUI[eth_message_from_GUI_put_index].data_2 = (*(buffer_ptr + 2) << 8) | *(buffer_ptr + 3);
    	eth_message_from_GUI[eth_message_from_GUI_put_index].data_1 = (*(buffer_ptr + 4) << 8) | *(buffer_ptr + 5);
    	eth_message_from_GUI[eth_message_from_GUI_put_index].data_0 = (*(buffer_ptr + 6) << 8) | *(buffer_ptr + 7);

        eth_message_from_GUI_put_index++;
        eth_message_from_GUI_put_index = eth_message_from_GUI_put_index & (ETH_GUI_MESSAGE_BUFFER_SIZE - 1);
    
    }
        
}
/****************************************************************************
  Function:
    ETMEthernetMessageFromGUI GetNextMessage(void)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
ETMEthernetMessageFromGUI GetNextMessage(void)
{
    ETMEthernetMessageFromGUI command;
    
    if (queue_is_empty(QUEUE_MESSAGE_FROM_GUI) == 0)
    {
    	command = eth_message_from_GUI[eth_message_from_GUI_get_index]; 
        eth_message_from_GUI_get_index++;
        eth_message_from_GUI_get_index = eth_message_from_GUI_get_index & (ETH_GUI_MESSAGE_BUFFER_SIZE - 1);
    
    }
    else
    	command.index = 0xffff;
    
    return (command);    
        
}
/****************************************************************************
  Function:
		unsigned int SendCalibrationData(unsigned int index, unsigned int scale, unsigned int offset)

  Input:
    pointer to data
    
  Description:
  Remarks:
	// This will add  a transmit message to the Send Calibration Data queue
	// It will return 0x0000 if the message was added to the queue or 0xFFFF if it was not (buffer full)
***************************************************************************/
unsigned int SendCalibrationData(unsigned int index, unsigned int scale, unsigned int offset)
{
    
    if (queue_buffer_room(QUEUE_CAL_TO_GUI) > 0)
    {
    	eth_cal_to_GUI[eth_cal_to_GUI_put_index].index  = index;
        eth_cal_to_GUI[eth_cal_to_GUI_put_index].scale = scale;
        eth_cal_to_GUI[eth_cal_to_GUI_put_index].offset = offset;
        
        eth_cal_to_GUI_put_index++;
        eth_cal_to_GUI_put_index = eth_cal_to_GUI_put_index & (ETH_CAL_TO_GUI_BUFFER_SIZE - 1);
    
    	return (0);
    }
    else
    	return (0xffff);
        
}
/****************************************************************************
  Function:
		unsigned int SendPulseData(unsigned char buffer_a)

  Input:
        0: buffer_b, non-0: buffer_a
    
  Description:
  Remarks:
	// This will change a flag to indicate pulse data ready
	// It will return 0x0000 if previous data was sent,  or 0xFFFF if it was not (buffer full)
***************************************************************************/
unsigned int SendPulseData(unsigned char is_buffer_a)
{
    
    if (is_buffer_a && (send_high_speed_data_buffer & 0x01) == 0)
    {
		send_high_speed_data_buffer |= 0x01;
    }
    else if (is_buffer_a == 0 && (send_high_speed_data_buffer & 0x02) == 0)
    {
		send_high_speed_data_buffer |= 0x02;
    }
    else
    	return (0xffff);

    return (0);
}
/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void InitializeBoard(void)
{    
  // LEDs
  LEDA_TRIS = 0;
  LEDB_TRIS = 0;
  LEDOP_TRIS = 0;
  LED_PUT(0x00);
    
#if defined(STACK_USE_UART)
  UART2TX_ON_TRIS = 0;
  UART2TX_ON_IO = 0;    // always disable TX1

  UART2TX_ON_TRIS = 0;
  UART2TX_ON_IO = 1;    // always enable TX2

#endif

  // UART
#if defined(STACK_USE_UART)

#if defined(__PIC24E__) || defined(__dsPIC33E__)
#if defined (ENC_CS_IO) || defined (WF_CS_IO)   // UART to be used in case of ENC28J60 or MRF24W
  __builtin_write_OSCCONL(OSCCON & 0xbf);
  RPOR9bits.RP101R = 3; //Map U2TX to RF5
  RPINR19bits.U2RXR = 0;
  RPINR19bits.U2RXR = 0x64; //Map U2RX to RF4
  __builtin_write_OSCCONL(OSCCON | 0x40);
#endif
#if(ENC100_INTERFACE_MODE == 0)                 // UART to be used only in case of SPI interface with ENC624Jxxx
  __builtin_write_OSCCONL(OSCCON & 0xbf);
  RPOR9bits.RP101R = 3; //Map U2TX to RF5
  RPINR19bits.U2RXR = 0;
  RPINR19bits.U2RXR = 0x64; //Map U2RX to RF4
  __builtin_write_OSCCONL(OSCCON | 0x40);

#endif
#endif

  UARTTX_TRIS = 0;
  UARTRX_TRIS = 1;
  UMODE = 0x8000;            // Set UARTEN.  Note: this must be done before setting UTXEN

#if defined(__C30__)
  USTA = 0x0400;        // UTXEN set
#define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
#define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
#else    //defined(__C32__)
  USTA = 0x00001400;        // RXEN set, TXEN set
#define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
#define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))
#endif
    
#define BAUD_ERROR ((BAUD_ACTUAL > BAUD_RATE) ? BAUD_ACTUAL-BAUD_RATE : BAUD_RATE-BAUD_ACTUAL)
#define BAUD_ERROR_PRECENT    ((BAUD_ERROR*100+BAUD_RATE/2)/BAUD_RATE)
#if (BAUD_ERROR_PRECENT > 3)
#warning UART frequency error is worse than 3%
#elif (BAUD_ERROR_PRECENT > 2)
#warning UART frequency error is worse than 2%
#endif
    
  UBRG = CLOSEST_UBRG_VALUE;
#endif


  // Deassert all chip select lines so there isn't any problem with 
  // initialization order.  Ex: When ENC28J60 is on SPI2 with Explorer 16, 
  // MAX3232 ROUT2 pin will drive RF12/U2CTS ENC28J60 CS line asserted, 
  // preventing proper 25LC256 EEPROM operation.
#if defined(ENC_CS_TRIS)
  ENC_CS_IO = 1;
  ENC_CS_TRIS = 0;
#endif
}

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
// MAC Address Serialization using a MPLAB PM3 Programmer and 
// Serialized Quick Turn Programming (SQTP). 
// The advantage of using SQTP for programming the MAC Address is it
// allows you to auto-increment the MAC address without recompiling 
// the code for each unit.  To use SQTP, the MAC address must be fixed
// at a specific location in program memory.  Uncomment these two pragmas
// that locate the MAC address at 0x1FFF0.  Syntax below is for MPLAB C 
// Compiler for PIC18 MCUs. Syntax will vary for other compilers.
//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
    
  // Start out zeroing all AppConfig bytes to ensure all fields are 
  // deterministic for checksum generation
  memset((void*)&AppConfig, 0x00, sizeof(AppConfig));
        
  AppConfig.Flags.bIsDHCPEnabled = 0;
  AppConfig.Flags.bInConfigMode = 0;
  memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
  //        {
  //            _prog_addressT MACAddressAddress;
  //            MACAddressAddress.next = 0x157F8;
  //            _memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
  //        }
  AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
  AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
  AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
  AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
  AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
  AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
  AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;
    
  AppConfig.MyRemIPAddr.Val = MY_DEFAULT_REM_IP_ADDR_BYTE1 | MY_DEFAULT_REM_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_REM_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_REM_IP_ADDR_BYTE4<<24ul;
    
    
  // Load the default NetBIOS Host Name
  memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
  FormatNetBIOSName(AppConfig.NetBIOSName);
    

  // Compute the checksum of the AppConfig defaults as loaded from ROM
  wOriginalAppConfigChecksum = CalcIPChecksum((BYTE*)&AppConfig, sizeof(AppConfig));


}
//
// called once for initilization.
//
void TCPmodbus_init(void)
{
  // Initialize application specific hardware
  InitializeBoard();

#if 1  //defined(STACK_USE_UART)
  // Initialize stack-related hardware components that may be 
  // required by the UART configuration routines
  TickInit();
#endif

  // Initialize Stack and application related NV variables into AppConfig.
  InitAppConfig();
    
  InitModbusData();

  // Initiates board setup process if button is depressed 
  // on startup
#if defined(STACK_USE_UART)
  DoUARTConfig();
#endif
    

  // Initialize core stack layers (MAC, ARP, TCP, UDP) and
  // application modules (HTTP, SNMP, etc.)
  StackInit();


}
//
// Need to call this task periodically
//
void TCPmodbus_task(void)
{
  static DWORD t = 0;
  _LATB8 = 1;



  // Blink LED0 (right most one) every second.
  if(TickGet() - t >= TICK_SECOND/2ul)
    {
      t = TickGet();
      LEDOP_IO ^= 1;
    }

  // This task performs normal stack task including checking
  // for incoming packet, type of packet and calling
  // appropriate stack entity to process it.
  StackTask();
        
  
  // This tasks invokes each of the core stack application tasks
 // StackApplications();  // don't need

 
  GenericTCPClient();

  _LATB8 = 0;

//  ExecuteCommands();

}



/****************************************************************************
  Function:
    static void InitModbusData(void)

  Description:
    This routine initializes modbus related data
 
  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
void InitModbusData(void)
{
 
 	eth_message_from_GUI_put_index = 0;
 	eth_message_from_GUI_get_index = 0;
 
 	eth_cal_to_GUI_put_index = 0;
 	eth_cal_to_GUI_get_index = 0;
 
  #if 1 
	/*
   // setup some fake data
   etm_can_hv_lambda_mirror.status_data.status_word_0	= 0x1111;
   etm_can_hv_lambda_mirror.status_data.status_word_1	= 0x3344;
   etm_can_hv_lambda_mirror.status_data.data_word_A	= 0x5566;
   etm_can_hv_lambda_mirror.status_data.data_word_B	= 0x7788;
   etm_can_hv_lambda_mirror.status_data.status_word_0_inhbit_mask = 0x9911;
   etm_can_hv_lambda_mirror.status_data.status_word_1_fault_mask = 0x1155;
   
   etm_can_hv_lambda_mirror.hvlambda_high_energy_set_point = 0x1122;
   etm_can_hv_lambda_mirror.hvlambda_readback_base_plate_temp = 0x5599;

   etm_can_ion_pump_mirror.status_data.status_word_0	= 0x2222;
   etm_can_ion_pump_mirror.status_data.status_word_1	= 0x3344;
   etm_can_ion_pump_mirror.status_data.data_word_A	= 0x5566;
   etm_can_ion_pump_mirror.status_data.data_word_B	= 0x7788;
   etm_can_ion_pump_mirror.status_data.status_word_0_inhbit_mask = 0x9911;
   etm_can_ion_pump_mirror.status_data.status_word_1_fault_mask = 0x2266;
   
    etm_can_afc_mirror.status_data.status_word_0 = 0x3333;
    etm_can_cooling_mirror.status_data.status_word_0 = 0x4444;
    etm_can_heater_magnet_mirror.status_data.status_word_0 = 0x5555;
    etm_can_gun_driver_mirror.status_data.status_word_0 = 0x6666;
    etm_can_magnetron_current_mirror.status_data.status_word_0 = 0x7777;
    etm_can_pulse_sync_mirror.status_data.status_word_0 = 0x8888;
    etm_can_ethernet_board_data.status_data->status_word_0 = 0x9999;
	*/

   #endif
   
   
}

/****************************************************************************
  Function:
    BuildModbusOutput_write_header(unsigned index)

  Description:
    Build the header for modbus command according to total bytes
 
	modbus header for write:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
	unit id (byte, 0xff), function code (byte, 0x10), reference number(word), data word count (word), 
	data byte count(byte), data bytes 
***************************************************************************/
void BuildModbusOutput_write_header(unsigned int total_bytes)
{
	    data_buffer[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
	    data_buffer[1] = transaction_number & 0xff;	 // transaction lo byte
	    data_buffer[2] = 0;	// protocol hi 
	    data_buffer[3] = 0;	// protocol lo 
	    // byte 4 and 5 for length
        data_buffer[4] = ((total_bytes + 7) >> 8) & 0xff;
        data_buffer[5] = (total_bytes + 7) & 0xff;
	    data_buffer[6] = modbus_send_index;	// unit Id 

	    data_buffer[7] = 0x10; // function code 
	    data_buffer[8] = 0;   // ref # hi
	    data_buffer[9] = 0;	  // ref # lo

	    data_buffer[10] = (total_bytes >> 9) & 0xff;  // data length in words hi, always 0, assume data length < 256
	    data_buffer[11] = total_bytes >> 1;     // data length in words lo
	    data_buffer[12] = total_bytes & 0xff;   // data length in bytes

}
/****************************************************************************
  Function:
    BuildModbusOutput_write_boards(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
WORD BuildModbusOutput_write_boards(unsigned char *tx_ptr)
{
	  WORD i; 
	  WORD total_bytes = 0;  // default: no cmd out 
    
      if (tx_ptr) // otherwise index is wrong, don't need send any cmd out
      {
        total_bytes = 108 + MAX_CUSTOM_DATA_LENGTH * 2; // bytes after length byte
        BuildModbusOutput_write_header(total_bytes);   

        // data starts at offset 13
	    for (i = 0; i < total_bytes; i++, tx_ptr++)
        {
        	data_buffer[i + 13] = *tx_ptr;
        }
        total_bytes = i + 13;
        		     
       }
       
       return (total_bytes);

}
/****************************************************************************
  Function:
    BuildModbusOutput_write_commands(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
WORD BuildModbusOutput_write_commands(unsigned char index)
{
	  WORD x; 
	  WORD total_bytes = 0;  // default: no cmd out 
      static unsigned pulse_index = 0;  // index for eash tracking
	  unsigned char *ptr;
    
      switch (index) // otherwise index is wrong, don't need send any cmd out
      {
      case MODBUS_WR_EVENTS: 
      	total_bytes = ETH_EVENT_SIZE;
        
        BuildModbusOutput_write_header(total_bytes);   

        // data starts at offset 13
        #ifdef TEST_MODBUS
      	for (x = 0; x < total_bytes; x++)
        	data_buffer[x + 13] = event_data[x];
        
        data_buffer[13 + 30] = queue_buffer_room(QUEUE_MESSAGE_FROM_GUI);    
        data_buffer[13 + 31] = queue_buffer_room(QUEUE_CAL_TO_GUI);    
        data_buffer[13 + 32] = 0x11;  
        data_buffer[13 + 33] = 0x66;
        #else
      	for (x = 0; x < total_bytes; x++)
        	data_buffer[x + 13] = x;
		#endif
        total_bytes += 13;
       
       break;
      case MODBUS_WR_ONE_CAL_ENTRY:
      	if (queue_is_empty(QUEUE_CAL_TO_GUI) == 0) 
        {
	      	total_bytes = 6;
	        
        	BuildModbusOutput_write_header(total_bytes);   

        	// data starts at offset 13
			x = eth_cal_to_GUI[eth_cal_to_GUI_get_index].index;
	        data_buffer[13] = (x >> 8) & 0xff;
	        data_buffer[14] = x & 0xff;
			x = eth_cal_to_GUI[eth_cal_to_GUI_get_index].scale;
	        data_buffer[15] = (x >> 8) & 0xff;
	        data_buffer[16] = x & 0xff;
			x = eth_cal_to_GUI[eth_cal_to_GUI_get_index].offset;
	        data_buffer[17] = (x >> 8) & 0xff;
	        data_buffer[18] = x & 0xff;
 
            eth_cal_to_GUI_get_index++;
            eth_cal_to_GUI_get_index = eth_cal_to_GUI_get_index & (ETH_CAL_TO_GUI_BUFFER_SIZE - 1);
           
	      	total_bytes += 13;
            
         }   
       break;
       
      case MODBUS_WR_PULSE_LOG:
      
      	if (send_high_speed_data_buffer) 
        {
	      	total_bytes = HIGH_SPEED_DATA_BUFFER_SIZE * sizeof(ETMCanHighSpeedData) + 2;
	        
        	BuildModbusOutput_write_header(total_bytes); 
            
            data_buffer[13] = (pulse_index >> 8) & 0xff;
            data_buffer[14] = pulse_index & 0xff;
            pulse_index++;  // overflows at 65535
            
            if (send_high_speed_data_buffer & 0x01) 
            {
            	ptr = (unsigned char *)&high_speed_data_buffer_a[0];
                send_high_speed_data_buffer &= 0xfe;
            } 
            else  
            {
            	ptr = (unsigned char *)&high_speed_data_buffer_b[0];
                send_high_speed_data_buffer = 0;
            } 

        	// data starts at offset 13
            total_bytes -= 2;
            for (x = 0; x < total_bytes; x++) {
	      data_buffer[15 + x] = *ptr;
	      *ptr = 0;
	      ptr++;
	    }
	      	total_bytes += 15;
            
         }   
       break;
      
      default:
       break;           
	     
       }
       
       return (total_bytes);

}
/****************************************************************************
  Function:
    BuildModbusOutput_read_command()

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
WORD BuildModbusOutput_read_command(BYTE index, BYTE byte_count)
{ 
      /* modbus header for read:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
	 unit id (byte, 0xff), function code (byte, 0x03), reference number(word), word count (byte) */

	  data_buffer[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
	  data_buffer[1] = transaction_number & 0xff;	 // transaction lo byte
	  data_buffer[2] = 0;	// protocol hi 
	  data_buffer[3] = 0;	// protocol lo 
      // fill the byte length    
      data_buffer[4] = 0;
      data_buffer[5] = 6;
	  data_buffer[6] = index;	// unit Id 

      data_buffer[7] = 0x3; // function code 
	  data_buffer[8] = 1;  // ref # hi
	  data_buffer[9] = index;  // ref # lo, redundant for now

      data_buffer[10] = 0;  // data length in words hi 
      data_buffer[11] = byte_count >> 1;  // data length in words lo
         
              
      return (12);	// always 12 bytes for read command

}
/****************************************************************************
  Function:
    BuildModbusOutput(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
WORD BuildModbusOutput(void)
{
  	static DWORD	Timer_write = 0;
    WORD total_bytes = 0;  // default: no cmd out
    unsigned char *tx_ptr = 0;
    
    
	  if((TickGet()-Timer_write) >= TICK_100MS) 
      {
 		  Timer_write = TickGet();
	      if (!modbus_cmd_need_repeat)
	      {
	      	   modbus_refresh_index++;
	           if (modbus_refresh_index > MODBUS_COMMAND_REFRESH_TOTAL) modbus_refresh_index = 1;	 // starts from 1
	      }
	      
          modbus_send_index = modbus_refresh_index;
          
	      if (modbus_send_index >= MODBUS_WR_HVLAMBDA && modbus_send_index <= MODBUS_WR_ETHERNET)
	      {  // write info to the GUI
		          switch (modbus_send_index)
		          {
				  case MODBUS_WR_HVLAMBDA:
		          	tx_ptr = (unsigned char *)&etm_can_hv_lambda_mirror;
		          	break;
				  case MODBUS_WR_ION_PUMP:
		          	tx_ptr = (unsigned char *)&etm_can_ion_pump_mirror;
		          	break;
				  case MODBUS_WR_AFC:
		          	tx_ptr = (unsigned char *)&etm_can_afc_mirror;
		          	break;
				  case MODBUS_WR_COOLING:
		          	tx_ptr = (unsigned char *)&etm_can_cooling_mirror;
		          	break;
				  case MODBUS_WR_HTR_MAGNET:
		          	tx_ptr = (unsigned char *)&etm_can_heater_magnet_mirror;
		          	break;
				  case MODBUS_WR_GUN_DRIVER:
		          	tx_ptr = (unsigned char *)&etm_can_gun_driver_mirror;
		          	break;
				  case MODBUS_WR_MAGNETRON_CURRENT:
		          	tx_ptr = (unsigned char *)&etm_can_magnetron_current_mirror;
		          	break;
				  case MODBUS_WR_PULSE_SYNC:
		          	tx_ptr = (unsigned char *)&etm_can_pulse_sync_mirror;
		          	break;
				  case MODBUS_WR_ETHERNET:
		         	tx_ptr = (unsigned char *)&etm_can_ethernet_board_data;
		          	break;
				  default: // move to the next for now, ignore some boards
		          	break;
		          } 
		      	  if (tx_ptr) // otherwise index is wrong, don't need send any cmd out
		          {
                                total_bytes = BuildModbusOutput_write_boards(tx_ptr);
		          }
	      }      
	      else
	      {	 // special command for rd or write info
		      switch (modbus_send_index)
		      {
		      	  case MODBUS_WR_EVENTS:  
                  	  total_bytes = BuildModbusOutput_write_commands(modbus_send_index);
              		break;
                  
			      default:
		          	break;
	      	  }
	          
	      }
	      
      }
      else {  // time to send queue commands
      	  modbus_send_index = 0;
          if (send_high_speed_data_buffer)
          	 modbus_send_index = MODBUS_WR_PULSE_LOG;
          else if (modbus_command_request) 
          {
          	 modbus_send_index = MODBUS_RD_COMMAND_DETAIL;
             modbus_command_request = 0; 
          }             
          else if (queue_is_empty(QUEUE_CAL_TO_GUI) == 0)
          	 modbus_send_index = MODBUS_WR_ONE_CAL_ENTRY;

		  switch (modbus_send_index)
		  {
		  	  case MODBUS_WR_ONE_CAL_ENTRY: 
		  	  case MODBUS_WR_PULSE_LOG: 
              	  total_bytes = BuildModbusOutput_write_commands(modbus_send_index);
          		break;
              
		      case MODBUS_RD_COMMAND_DETAIL:
	          	total_bytes = BuildModbusOutput_read_command(modbus_send_index, 8);
		      	break;
		      default:
		      	break;
	      }
      }

	  if (total_bytes) 
	  {
	  	 transaction_number++; // don't care about overflow
	      modbus_cmd_need_repeat = 1; // clear when there is response
	  }
	  return (total_bytes);
  	  
      
    
}
/*****************************************************************************
  Function:
	void GenericTCPClient(void)

  Summary:
	Implements a simple HTTP client (over TCP).

  Description:
	This function implements a simple HTTP client, which operates over TCP.  
	The function is called periodically by the stack, and waits for BUTTON1 
	to be pressed.  When the button is pressed, the application opens a TCP
	connection to an Internet search engine, performs a search for the word
	"Microchip" on "microchip.com", and prints the resulting HTML page to
	the UART.
	
	This example can be used as a model for many TCP and HTTP client
	applications.

  Precondition:
	TCP is initialized.

  Parameters:
	None

  Returns:
  	None
***************************************************************************/
void GenericTCPClient(void)
{

  WORD				w, len;

  //    char                sBuffer[250];
    
      
       
  static DWORD		Timer;
  static TCP_SOCKET	MySocket = INVALID_SOCKET;
  static enum _GenericTCPExampleState
  {
    SM_HOME = 0,
    SM_SOCKET_OBTAINED,
    SM_PROCESS_RESPONSE,
    SM_DISCONNECT,
    SM_DONE
  } GenericTCPExampleState = SM_DONE;

  switch(GenericTCPExampleState)
    {
    case SM_HOME:
      // Connect a socket to the remote TCP server, 192.168.66.15
      MySocket = TCPOpen(AppConfig.MyRemIPAddr.Val, TCP_OPEN_IP_ADDRESS, 502, TCP_PURPOSE_TCP_MODBUS_CLIENT);
      
      // Abort operation if no TCP socket of type TCP_PURPOSE_GENERIC_TCP_CLIENT is available
      // If this ever happens, you need to go add one to TCPIPConfig.h
      if(MySocket == INVALID_SOCKET)
	break;
      
      GenericTCPExampleState = SM_SOCKET_OBTAINED;
      Timer = TickGet();
      break;

    case SM_SOCKET_OBTAINED:
      // Wait for the remote server to accept our connection request
      if(!TCPIsConnected(MySocket)) {
		// Time out if too much time is spent in this state
		if((TickGet()-Timer) > 5*TICK_SECOND) {
		  // Close the socket so it can be used by other modules
		  TCPDisconnect(MySocket);
		  MySocket = INVALID_SOCKET;
		  GenericTCPExampleState = SM_HOME;
		}
		break;
      }
      
      Timer = TickGet();
      
      // Make certain the socket can be written to
      if (TCPIsPutReady(MySocket) < MAX_TX_SIZE) break;

      
      len = BuildModbusOutput();
      
      if (len == 0) break;  // don't want to send anything for now, stay in this state

      _LATB7 = 1;
      TCPPutArray(MySocket,  data_buffer, len);
      _LATB7 = 0;
      
      // Send the packet
      TCPFlush(MySocket);
      _LATB9 = 0;
      GenericTCPExampleState = SM_PROCESS_RESPONSE;
      break;

    case SM_PROCESS_RESPONSE:
      // Check to see if the remote node has disconnected from us or sent us any application data
      if(!TCPIsConnected(MySocket)) {
	      GenericTCPExampleState = SM_DISCONNECT;
	// Do not break;  We might still have data in the TCP RX FIFO waiting for us
      }
      
      // Get count of RX bytes waiting
      w = TCPIsGetReady(MySocket);	

     if (w)
           
     {
		if (w > (MAX_TX_SIZE-1)) {
		  w = (MAX_TX_SIZE-1);
		}
		
		len = TCPGetArray(MySocket, data_buffer, w);
		w -= len;
	
    	if (data_buffer[6] == modbus_send_index) {
        	if (modbus_send_index == MODBUS_RD_COMMAND_DETAIL)
            {
            	queue_put_command(&data_buffer[9]);
            }
            else /* write commands return command count in the reference field */
            {
            	modbus_command_request = (data_buffer[8] << 8) | data_buffer[9];
            }
    
	  		modbus_cmd_need_repeat = 0;

       //    GenericTCPExampleState = SM_SOCKET_OBTAINED; // repeat sending
		} // if (data_buffer[0] == (modbus_array_index + 1))
    

         GenericTCPExampleState = SM_SOCKET_OBTAINED; // repeat sending

    }  //  while(w)	
    else
    {
		// Time out if too much time is spent in this state
		if((TickGet()-Timer) > TICK_SECOND) {
		  // Close the socket so it can be used by other modules
		  TCPDisconnect(MySocket);
		  MySocket = INVALID_SOCKET;
		  GenericTCPExampleState = SM_HOME;
		}
    }

      break;
	
    case SM_DISCONNECT:
      // Close the socket so it can be used by other modules
      // For this application, we wish to stay connected, but this state will still get entered if the remote server decides to disconnect
      TCPDisconnect(MySocket);
      MySocket = INVALID_SOCKET;
      GenericTCPExampleState = SM_DONE;
      break;
	
    case SM_DONE:
      // Do nothing unless the user pushes BUTTON1 and wants to restart the whole connection/download process
      //if(BUTTON1_IO == 0u)
      GenericTCPExampleState = SM_HOME;
      break;
    }
}




