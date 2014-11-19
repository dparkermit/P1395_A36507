#include "A36507.h"
#include "ETM_CAN_PUBLIC.h"

#include "ETM_CAN.h"

#include "ETM_EEPROM.h"

/*
  Modules to be created

  Ethernet Interface Module - Dongying
  Higher Level communication based on Ethernet 
  Real Time Clock 
  
  
  
*/
_FOSC(ECIO_PLL16 & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


ETMEEProm U5_FM24C64B;

unsigned int dan_test_0;
unsigned int dan_test_1;
unsigned int dan_test_2;
unsigned int dan_test_3;
unsigned int dan_test_4;
unsigned int dan_test_5;
unsigned int dan_test_6;
unsigned int dan_test_7;
unsigned int dan_test_8;
unsigned int dan_test_9;
unsigned int dan_test_10;
unsigned int dan_test_11;
unsigned int dan_test_12;
unsigned int dan_test_13;
unsigned int dan_test_14;
unsigned int dan_test_15;


unsigned int dan_test_array[16];
unsigned int dan_test_read_array[16];

void InitializeA36507(void);


int main(void) {
  
  ETMCanInitialize();

  InitializeA36507();

  dan_test_array[0]  = 0x50;
  dan_test_array[1]  = 0x51;
  dan_test_array[2]  = 0x52;
  dan_test_array[3]  = 0x53;
  dan_test_array[4]  = 0x54;
  dan_test_array[5]  = 0x55;
  dan_test_array[6]  = 0x56;
  dan_test_array[7]  = 0x57;
  dan_test_array[8]  = 0x58;
  dan_test_array[9]  = 0x59;
  dan_test_array[10] = 0x5A;
  dan_test_array[11] = 0x5B;
  dan_test_array[12] = 0x5C;
  dan_test_array[13] = 0x5D;
  dan_test_array[14] = 0x5E;
  dan_test_array[15] = 0x5F;


  ETMEEPromWritePage(&U5_FM24C64B,5,16,dan_test_array);
  
  
  

  dan_test_0  = ETMEEPromReadWord(&U5_FM24C64B, 0x50);
  dan_test_1  = ETMEEPromReadWord(&U5_FM24C64B, 0x51);
  dan_test_2  = ETMEEPromReadWord(&U5_FM24C64B, 0x52);
  dan_test_3  = ETMEEPromReadWord(&U5_FM24C64B, 0x53);
  dan_test_4  = ETMEEPromReadWord(&U5_FM24C64B, 0x54);
  dan_test_5  = ETMEEPromReadWord(&U5_FM24C64B, 0x55);
  dan_test_6  = ETMEEPromReadWord(&U5_FM24C64B, 0x56);
  dan_test_7  = ETMEEPromReadWord(&U5_FM24C64B, 0x57);
  dan_test_8  = ETMEEPromReadWord(&U5_FM24C64B, 0x58);
  dan_test_9  = ETMEEPromReadWord(&U5_FM24C64B, 0x59);
  dan_test_10 = ETMEEPromReadWord(&U5_FM24C64B, 0x5A);
  dan_test_11 = ETMEEPromReadWord(&U5_FM24C64B, 0x5B);
  dan_test_12 = ETMEEPromReadWord(&U5_FM24C64B, 0x5C);
  dan_test_13 = ETMEEPromReadWord(&U5_FM24C64B, 0x5D);
  dan_test_14 = ETMEEPromReadWord(&U5_FM24C64B, 0x5E);
  dan_test_15 = ETMEEPromReadWord(&U5_FM24C64B, 0x5F);


  ETMEEPromReadPage(&U5_FM24C64B,5,11,dan_test_read_array);
  
  //#define ETM_CAN_REGISTER_CALIBRATION_TEST          0x0400
  //#define ETM_CAN_REGISTER_LOCAL_TEST                0x0100
  

  
  etm_can_heater_magnet_mirror.htrmag_heater_current_set_point = 5000;
  etm_can_heater_magnet_mirror.htrmag_magnet_current_set_point = 10000;
  
  while (1) {
    ETMCanDoCan();
    
  }
}



void InitializeA36507(void) {
  _TRISA7 = 0;
  _TRISG13 = 0;

  ETMEEPromConfigureDevice(&U5_FM24C64B, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);
  
}
