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


static BYTE         data_buffer[MAX_TX_SIZE];
static BYTE         modbus_array_index = 0;
static BYTE         super_user_mode = 0;
static WORD         transaction_number_lo = 0;

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
    
    InitModbusArray();

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
        StackApplications();

 
        GenericTCPClient();

}


#if defined(STACK_USE_UART)
// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
    BYTE IPDigit[4];
    BYTE i;

    for(i = 0; i < sizeof(IP_ADDR); i++)
    {
        uitoa((WORD)IPVal.v[i], IPDigit);

        putsUART((char *) IPDigit);

        if(i == sizeof(IP_ADDR)-1)
                break;

        while(BusyUART());
            WriteUART('.');
    }
}
#endif


/****************************************************************************
  Function:
    static void InitModbusArray(void)

  Description:
    This routine initializes modbus_array
  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void InitModbusArray(void)
{
	// write analog block
    modbus_array[0].reference_num = 11000;
    modbus_array[0].length = 29;
    modbus_array[0].is_write = 1;
    modbus_array[0].poll_behind_pw = 0;

	// write fault&event block
	modbus_array[1].reference_num = 12000;
    modbus_array[1].length = 5;
    modbus_array[1].is_write = 1;
    modbus_array[1].poll_behind_pw = 0;

	// write energy param
	modbus_array[2].reference_num = 11100;
    modbus_array[2].length = 23;
    modbus_array[2].is_write = 1;
    modbus_array[2].poll_behind_pw = 1;

	// rd pw
	modbus_array[3].reference_num = 11048;
    modbus_array[3].length = 2;
    modbus_array[3].is_write = 0;
    modbus_array[3].poll_behind_pw = 0;

	// rd Ekset
	modbus_array[4].reference_num = 11050;
    modbus_array[4].length = 1;
    modbus_array[4].is_write = 0;
    modbus_array[4].poll_behind_pw = 1;
    
	// rd fault mask, AFC pos
	modbus_array[5].reference_num = 11095;
    modbus_array[5].length = 2;
    modbus_array[5].is_write = 0;
    modbus_array[5].poll_behind_pw = 1;
    
 	// rd energy cmds
	modbus_array[6].reference_num = 11099;
    modbus_array[6].length = 1;
    modbus_array[6].is_write = 0;
    modbus_array[6].poll_behind_pw = 1;
    
 	// rd Ifset
	modbus_array[7].reference_num = 11051;
    modbus_array[7].length = 1;
    modbus_array[7].is_write = 0;
    modbus_array[7].poll_behind_pw = 1;

 	// rd AFC home
    modbus_array[8].reference_num = 11056;
    modbus_array[8].length = 1;
    modbus_array[8].is_write = 0;
    modbus_array[8].poll_behind_pw = 1;

 	// rd AFC param select
    modbus_array[9].reference_num = 11057;
    modbus_array[9].length = 1;
    modbus_array[9].is_write = 0;
    modbus_array[9].poll_behind_pw = 1;
    
 	// rd sleep delay
    modbus_array[10].reference_num = 11058;
    modbus_array[10].length = 1;
    modbus_array[10].is_write = 0;
    modbus_array[10].poll_behind_pw = 1;
    
 	// rd cabT limit
    modbus_array[11].reference_num = 11061;
    modbus_array[11].length = 1;
    modbus_array[11].is_write = 0;
    modbus_array[11].poll_behind_pw = 1;
    
 	// rd htd
    modbus_array[12].reference_num = 11062;
    modbus_array[12].length = 1;
    modbus_array[12].is_write = 0;
    modbus_array[12].poll_behind_pw = 1;
    
	// read energy param
	modbus_array[13].reference_num = 11100;
    modbus_array[13].length = 23;
    modbus_array[13].is_write = 0;
    modbus_array[13].poll_behind_pw = 1;

   
  	// rd digital cmds
    modbus_array[14].reference_num = 11047;
    modbus_array[14].length = 1;
    modbus_array[14].is_write = 0;
    modbus_array[14].poll_behind_pw = 0;
    


}
/*****************************************************************************
  Function: BuildModbusOutput(void)

  Description:
  		change global data_buffer, modbus_array_index

  Precondition:
	None
    
  Parameters:
	Index

  Returns:
  	total byte length
  ***************************************************************************/
WORD BuildModbusOutput(void)
{
	BYTE i;
    WORD total_bytes;
    
	if (super_user_mode == 0)
    {
		while (modbus_array[modbus_array_index].poll_behind_pw > 0)
        {
        	modbus_array_index++;
            if (modbus_array_index >= MODBUS_ARRAY_SIZE) modbus_array_index = 0;
        }
    }

    
    data_buffer[0] = modbus_array_index + 1;	 // transaction hi byte
    data_buffer[1] = transaction_number_lo;	 // transaction lo byte
    data_buffer[2] = 0;	// protocol hi 
    data_buffer[3] = 0;	// protocol lo 
	// byte 4 and 5 for length
    data_buffer[6] = 0;	// unit Id 

    data_buffer[8] = ((modbus_array[modbus_array_index].reference_num - 1) / 256) & 0xff;  // ref #
    data_buffer[9] = (modbus_array[modbus_array_index].reference_num - 1) % 256;

    if (modbus_array[modbus_array_index].is_write) 
    {  
	/* modbus header for write:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
       unit id (byte, 0xff), function code (byte, 0x10), reference number(word), data word count (word), 
       data byte count(byte), data bytes */

    	data_buffer[7] = 0x10; // function code 
        
        data_buffer[10] = 0;  // data length in words hi, always 0, assume data length < 256
        data_buffer[11] = modbus_array[modbus_array_index].length;  // data length in words lo
        data_buffer[12] = (modbus_array[modbus_array_index].length << 1);  // data length in bytes
        for (i = 0; i < data_buffer[12]; i++)
        	data_buffer[i + 13] = modbus_array[modbus_array_index].data[i];
        
        // fill the byte length    
        data_buffer[4] = ((data_buffer[12] + 7) / 256) & 0xff;
        data_buffer[5] = (data_buffer[12] + 7) % 256;
        total_bytes = i + 13; 
    }
    else
    {  
	/* modbus header for read:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
       unit id (byte, 0xff), function code (byte, 0x03), reference number(word), word count (byte) */

    	data_buffer[7] = 0x3; // function code 

        data_buffer[10] = 0;  // data length in words hi 
        data_buffer[11] = modbus_array[modbus_array_index].length;  // data length in words lo
        
        // fill the byte length    
        data_buffer[4] = 0;
        data_buffer[5] = 6;
        total_bytes = 12; 
        
    }
    
    transaction_number_lo++;
    transaction_number_lo = transaction_number_lo % 256;
    
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
	BYTE 				i;
	WORD				w, len;
    DWORD               pw;
//    char                sBuffer[250];
       
	static DWORD		Timer;
	static TCP_SOCKET	MySocket = INVALID_SOCKET;
	static enum _GenericTCPExampleState
	{
		SM_HOME = 0,
		SM_SOCKET_OBTAINED,
		#if defined(STACK_USE_SSL_CLIENT)
    	SM_START_SSL,
    	#endif
		SM_PROCESS_RESPONSE,
		SM_DISCONNECT,
		SM_DONE
	} GenericTCPExampleState = SM_DONE;

	switch(GenericTCPExampleState)
	{
		case SM_HOME:
			// Connect a socket to the remote TCP server, 192.168.66.15
  //			MySocket = TCPOpen(0x0F42A8C0, TCP_OPEN_IP_ADDRESS, 502, TCP_PURPOSE_TCP_MODBUS_CLIENT);
			MySocket = TCPOpen(AppConfig.MyRemIPAddr.Val, TCP_OPEN_IP_ADDRESS, 502, TCP_PURPOSE_TCP_MODBUS_CLIENT);
			
			// Abort operation if no TCP socket of type TCP_PURPOSE_GENERIC_TCP_CLIENT is available
			// If this ever happens, you need to go add one to TCPIPConfig.h
			if(MySocket == INVALID_SOCKET)
				break;

			#if defined(STACK_USE_UART)
			putrsUART((ROM char*)"\r\n\r\nConnecting using Microchip TCP API...\r\n");
			#endif

			GenericTCPExampleState++;
			Timer = TickGet();
			break;

		case SM_SOCKET_OBTAINED:
			// Wait for the remote server to accept our connection request
			if(!TCPIsConnected(MySocket))
			{
				// Time out if too much time is spent in this state
				if(TickGet()-Timer > 5*TICK_SECOND)
				{
					// Close the socket so it can be used by other modules
					TCPDisconnect(MySocket);
					MySocket = INVALID_SOCKET;
					GenericTCPExampleState--;
				}
				break;
			}

			Timer = TickGet();
			
			// Make certain the socket can be written to
			if(TCPIsPutReady(MySocket) < 125u)
				break;
			
            #if 0
			// Place the application protocol data into the transmit buffer.  For this example, we are connected to an HTTP server, so we'll send an HTTP GET request.
			TCPPutROMString(MySocket, (ROM BYTE*)"GET ");
			TCPPutROMString(MySocket, RemoteURL);
			TCPPutROMString(MySocket, (ROM BYTE*)" HTTP/1.0\r\nHost: ");
			TCPPutString(MySocket, ServerName);
			TCPPutROMString(MySocket, (ROM BYTE*)"\r\nConnection: close\r\n\r\n");
			#else
           // TCPPutArray(MySocket,  (BYTE *)"\xa\x9d\x0\x0\x0\x6\xff\x3\x2b\x27\x00\x02", 12);
            len = BuildModbusOutput();
            TCPPutArray(MySocket,  data_buffer, len);
           
            #endif
			// Send the packet
			TCPFlush(MySocket);
			GenericTCPExampleState++;
			break;

		case SM_PROCESS_RESPONSE:
			// Check to see if the remote node has disconnected from us or sent us any application data
			// If application data is available, write it to the UART
			if(!TCPIsConnected(MySocket))
			{
				GenericTCPExampleState = SM_DISCONNECT;
				// Do not break;  We might still have data in the TCP RX FIFO waiting for us
			}
	
			// Get count of RX bytes waiting
			w = TCPIsGetReady(MySocket);	
	
			// Obtian and print the server reply
			i = sizeof(data_buffer)-1;
			data_buffer[i] = '\0';
		
        	while(w)	// ignore if incoming data is larger than the data_buffer
			{
				if(w < i)
				{
					i = w;
					data_buffer[i] = '\0';
				}
			 //	w -= TCPGetArray(MySocket, vBuffer, i);
                len = TCPGetArray(MySocket, data_buffer, i);
                w -= len;
				#if 0 //defined(STACK_USE_UART)
                for (k = 0; k < len; k++)
                	sprintf(sBuffer + 3 * k, "%02x ", data_buffer[k]);
                sBuffer[3 * k] = 0;    
				putsUART((char*)sBuffer);
				#endif
				
				// putsUART is a blocking call which will slow down the rest of the stack 
				// if we shovel the whole TCP RX FIFO into the serial port all at once.  
				// Therefore, let's break out after only one chunk most of the time.  The 
				// only exception is when the remote node disconncets from us and we need to 
				// use up all the data before changing states.
			
            //	if(GenericTCPExampleState == SM_PROCESS_RESPONSE)  break;
            /* put into modbus_array */
	            if (data_buffer[0] == (modbus_array_index + 1))
	            {
		            if (data_buffer[7] == 0x03 && data_buffer[8] == (modbus_array[modbus_array_index].length * 2))
	                { // read
	                	for (i = 0; i < data_buffer[8]; i++)
	                		modbus_array[modbus_array_index].data[i] = data_buffer[i + 9];

	                    if (modbus_array_index == 3) // pw
	                    {
	                    	pw = (modbus_array[modbus_array_index].data[2] << 8) | modbus_array[modbus_array_index].data[3];
	                        pw <<= 16;
	                        pw |= ((modbus_array[modbus_array_index].data[0] << 8) | modbus_array[modbus_array_index].data[1]) & 0xffff;
                            if (pw == 452873) 
	                    		super_user_mode = 1;
                            else
	                    		super_user_mode = 0;
                                
	                    }
	            	}
	            }
            
			    modbus_array_index++;
			    if (modbus_array_index >= MODBUS_ARRAY_SIZE) modbus_array_index = 0;
			}
    
            GenericTCPExampleState = SM_SOCKET_OBTAINED; // repeat sending
	
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

