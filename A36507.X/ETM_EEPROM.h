#ifndef __ETM_EEPROM_H
#define __ETM_EEPROM_H
#include "ETM_I2C.h"

/*
  This module is designed to inteface a 16 bit microchip processor with the 8 bit flash EEPROM.
  This module supports word mode with ETMEEPromWriteWord and ETMEEPromReadWord.
  This module supports page mode (Array of 16 Integers) with ETMEEPromWritePage and ETMEEPromReadPage
  
  This module has been developed to work with Flash EEPROM Microchip 24LC64F, or FRAM CYPRESS FM24C64B, but should compatible with any device with up to a word of address space (64K x 8)
  

  DPARKER Need to add functionality to check if a FLASH based device is busy and wait for it to be not busy.

*/

// ---------- Configure For the EEPROM I2C Addresses  ----------------- //
#define EEPROM_I2C_ADDRESS_0                 0b10100000
#define EEPROM_I2C_ADDRESS_1                 0b10100010
#define EEPROM_I2C_ADDRESS_2                 0b10100100
#define EEPROM_I2C_ADDRESS_3                 0b10100110
#define EEPROM_I2C_ADDRESS_4                 0b10101000
#define EEPROM_I2C_ADDRESS_5                 0b10101010
#define EEPROM_I2C_ADDRESS_6                 0b10101100
#define EEPROM_I2C_ADDRESS_7                 0b10101110


#define EEPROM_SIZE_8K_BYTES                0x2000
#define EEPROM_SIZE_64K_BITS                0x2000

typedef struct {
  unsigned char address;
  unsigned char i2c_port;
  unsigned int  size_bytes;
} ETMEEProm;


void ETMEEPromConfigureDevice(ETMEEProm* ptr_eeprom, unsigned char i2c_address, unsigned char i2c_port, unsigned int size_bytes, unsigned long fcy_clk, unsigned long i2c_baud_rate);
/*
  This configures the EEPROM and initializes the I2C port
*/

void ETMEEPromWriteWord(ETMEEProm* ptr_eeprom, unsigned int register_location, unsigned int data);
/*
  This writes a single word to the EEprom
  register_location is NOT the internal address. It is the position in words (instead of bytes)
*/

unsigned int ETMEEPromReadWord(ETMEEProm* ptr_eeprom, unsigned int register_location);
/*
  This reads a single word from the EEprom
  register_location is NOT the internal address. It is the position in words (instead of bytes)
*/

void ETMEEPromWritePage(ETMEEProm* ptr_eeprom, unsigned int page_number, unsigned int words_to_write, unsigned int *data);
/*
  This writes data from the data array into the eeprom.

  These are page aligned instructions (16 words per page) even on devices that do not have pages.
  If words_to_write > 16 it will be truncated to 16.
*/

void ETMEEPromReadPage(ETMEEProm* ptr_eeprom, unsigned int page_number, unsigned int words_to_read, unsigned int *data);
/*
  This reads data from the eeprom into the data array

  These are page aligned instructions (16 words per page) even on devices that do not have pages.
  If words_to_read > 16 it will be truncated to 16.
  
  NOTE!!!!
  DO NOT READY MORE WORDS THAN YOU HAVE ALLOCATED TO YOUR DATA ARRAY
  IF YOU DO, THE VALUES IN RAM AT ADDRESSES PROCEEDING YOUR ARRAY WILL GET CLOBBERED 
*/

#endif
