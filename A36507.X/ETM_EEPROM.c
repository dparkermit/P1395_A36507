#include "ETM_EEPROM.h"


void ETMEEPromConfigureDevice(ETMEEProm* ptr_eeprom, unsigned char i2c_address, unsigned char i2c_port, unsigned int size_bytes, unsigned long fcy_clk, unsigned long i2c_baud_rate) {
  ptr_eeprom->address = i2c_address;
  ptr_eeprom->i2c_port = i2c_port;
  ptr_eeprom->size_bytes = size_bytes;
  ConfigureI2C(ptr_eeprom->i2c_port, I2CCON_DEFAULT_SETUP_PIC30F, i2c_baud_rate, fcy_clk, 0);
}

void ETMEEPromWriteWord(ETMEEProm* ptr_eeprom, unsigned int register_location, unsigned int data) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  unsigned int error_check;
   
   
  if (register_location < (ptr_eeprom->size_bytes >> 1) ) {
    // The requeted register address is within the boundry of this device.
    
    data_high_byte = (data >> 8);
    data_low_byte = (data & 0x00FF);
    
    temp = (register_location << 1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
    
    error_check = WaitForI2CBusIdle(ptr_eeprom->i2c_port);
    error_check |= GenerateI2CStart(ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(ptr_eeprom->address | I2C_WRITE_CONTROL_BIT, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(adr_high_byte, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(adr_low_byte, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(data_low_byte, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(data_high_byte, ptr_eeprom->i2c_port);
    error_check |= GenerateI2CStop(ptr_eeprom->i2c_port);
  } else {
    // The requested address to write is outside the boundry of this device
    error_check = 0xFFFF;
  }

  // DPARKER - what to do if error check != 0?
}





unsigned int ETMEEPromReadWord(ETMEEProm* ptr_eeprom, unsigned int register_location) {
  unsigned int error_check;
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  
  if (register_location < (ptr_eeprom->size_bytes >> 1) ) {
    // The requeted register address is within the boundry of this device.
    
    temp = (register_location << 1);
    adr_high_byte = (temp >> 8);
    adr_low_byte = (temp & 0x00FF);
 
    error_check = WaitForI2CBusIdle(ptr_eeprom->i2c_port);
    error_check |= GenerateI2CStart(ptr_eeprom->i2c_port);
    
    error_check |= WriteByteI2C(ptr_eeprom->address | I2C_WRITE_CONTROL_BIT, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(adr_high_byte, ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(adr_low_byte, ptr_eeprom->i2c_port);
    
    error_check |= GenerateI2CRestart(ptr_eeprom->i2c_port);
    error_check |= WriteByteI2C(ptr_eeprom->address | I2C_READ_CONTROL_BIT, ptr_eeprom->i2c_port);
    data_low_byte = ReadByteI2C(ptr_eeprom->i2c_port);
    error_check |= GenerateI2CAck(ptr_eeprom->i2c_port);
    data_high_byte = ReadByteI2C(ptr_eeprom->i2c_port);
    error_check |= GenerateI2CNack(ptr_eeprom->i2c_port);

    error_check |= GenerateI2CStop(ptr_eeprom->i2c_port);
  
  } else {
    // The requested address to write is outside the boundry of this device
    error_check = 0xFFFF;
    data_low_byte = 0;
    data_high_byte = 0;
  }

  // DPARKER - what to do if error check != 0?  Probably return zero and increment error counter

  temp = data_high_byte;
  temp <<= 8; 
  temp += data_low_byte;
  return temp;

}


void ETMEEPromWritePage(ETMEEProm* ptr_eeprom, unsigned int page_number, unsigned int words_to_write, unsigned int *data) {
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  unsigned int error_check;
  unsigned int n;
  if (words_to_write > 0) {
    if (words_to_write > 16) {
      words_to_write = 16;
    }
    if (page_number < (ptr_eeprom->size_bytes >> 5)) {
      // The requeted page address is within the boundry of this device.
      
      temp = (page_number << 5);
      adr_high_byte = (temp >> 8);
      adr_low_byte = (temp & 0x00FF);
      
      
      error_check = WaitForI2CBusIdle(ptr_eeprom->i2c_port);
      error_check |= GenerateI2CStart(ptr_eeprom->i2c_port);
      error_check |= WriteByteI2C(ptr_eeprom->address | I2C_WRITE_CONTROL_BIT, ptr_eeprom->i2c_port);  
      error_check |= WriteByteI2C(adr_high_byte, ptr_eeprom->i2c_port);
      error_check |= WriteByteI2C(adr_low_byte, ptr_eeprom->i2c_port);
      
      for (n = 0; n < words_to_write ; n++) {
	data_high_byte = (data[n] >> 8);
	data_low_byte = (data[n] & 0x00FF);
	error_check |= WriteByteI2C(data_low_byte, ptr_eeprom->i2c_port);
	error_check |= WriteByteI2C(data_high_byte, ptr_eeprom->i2c_port);    
      }
      
      
      error_check |= GenerateI2CStop(ptr_eeprom->i2c_port);
      
    } else {
      // The requested page to write is outside the boundry of this device
      error_check = 0xFFFF;
    }
    // DPARKER - what to do if error check != 0?
  }

}


void ETMEEPromReadPage(ETMEEProm* ptr_eeprom, unsigned int page_number, unsigned int words_to_read, unsigned int *data) {
  unsigned int error_check;
  unsigned int temp;
  unsigned char adr_high_byte;
  unsigned char adr_low_byte;
  unsigned char data_low_byte;
  unsigned char data_high_byte;
  unsigned int n;

  if (words_to_read > 0) {
    if (words_to_read > 16) {
      words_to_read = 16;
    }
    if (page_number < (ptr_eeprom->size_bytes >> 5) ) {
      // The requeted page address is within the boundry of this device.
      
      temp = (page_number << 5);
      adr_high_byte = (temp >> 8);
      adr_low_byte = (temp & 0x00FF);

      error_check = WaitForI2CBusIdle(ptr_eeprom->i2c_port);
      error_check |= GenerateI2CStart(ptr_eeprom->i2c_port);
      
      error_check |= WriteByteI2C(ptr_eeprom->address | I2C_WRITE_CONTROL_BIT, ptr_eeprom->i2c_port);
      error_check |= WriteByteI2C(adr_high_byte, ptr_eeprom->i2c_port);
      error_check |= WriteByteI2C(adr_low_byte, ptr_eeprom->i2c_port);
      
      error_check |= GenerateI2CRestart(ptr_eeprom->i2c_port);
      error_check |= WriteByteI2C(ptr_eeprom->address | I2C_READ_CONTROL_BIT, ptr_eeprom->i2c_port);
      
      for (n = 0; n < words_to_read-1 ; n++) {
	data_low_byte = ReadByteI2C(ptr_eeprom->i2c_port);
	error_check |= GenerateI2CAck(ptr_eeprom->i2c_port);
	data_high_byte = ReadByteI2C(ptr_eeprom->i2c_port);
	error_check |= GenerateI2CAck(ptr_eeprom->i2c_port);
	data[n] = data_high_byte;
	data[n] <<= 8;
	data[n] += data_low_byte; 
      }
      
      data_low_byte = ReadByteI2C(ptr_eeprom->i2c_port);
      error_check |= GenerateI2CAck(ptr_eeprom->i2c_port);
      data_high_byte = ReadByteI2C(ptr_eeprom->i2c_port);
      error_check |= GenerateI2CNack(ptr_eeprom->i2c_port);
      data[n] = data_high_byte;
      data[n] <<= 8;
      data[n] += data_low_byte; 

      
      error_check |= GenerateI2CStop(ptr_eeprom->i2c_port);
      
    } else {
      // The requested page to read is outside the boundry of this device
      error_check = 0xFFFF;
      data_low_byte = 0;
      data_high_byte = 0;
    }
   // DPARKER - what to do if error check != 0?  Probably return zero and increment error counter
  }
}
