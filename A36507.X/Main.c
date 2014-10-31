#include <p30F6014a.h>
#include <libpic30.h>
//#include "Main.h"
#include "ETM_CAN.h"


/*
  Modules to be created

  Ethernet Interface Module - Dongying
  Higher Level communication based on Ethernet 
  Real Time Clock 
  
  
  
*/



_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


unsigned int unit_counter;





ETMCanMessage test_read_message;

unsigned int global_0 = 0;
unsigned int global_1 = 0;
unsigned int global_2 = 0;

unsigned int slots_available = 0;
unsigned int rows_available = 0;
unsigned int slots_full = 0;


int main(void) {
  global_0 = 0;
  global_1 = 0;

  while (global_0 <= 0xFF00) {
    global_0++;
    ClrWdt();
    global_1 = 0;
    while (global_1 <= 100) {
      global_1++;
    }
  }
  
  ETMCanInitialize();

  
#define ETM_CAN_REGISTER_CALIBRATION_TEST          0x0400
#define ETM_CAN_REGISTER_LOCAL_TEST                0x0100
  
  
  
  
#ifdef __ETM_CAN_MASTER_MODULE
  // DPARKER read configuration from EEPROM
  etm_can_my_configuration.agile_number_high_word = 0;
  etm_can_my_configuration.agile_number_low_word  = 34760;
  etm_can_my_configuration.agile_dash             = 000;
  etm_can_my_configuration.agile_rev_ascii        = 'G';

  etm_can_my_configuration.serial_number          = 107;
  etm_can_my_configuration.firmware_branch        = 0;
  etm_can_my_configuration.firmware_major_rev     = 10;
  etm_can_my_configuration.firmware_minor_rev     = 2;
#else
  etm_can_my_configuration.agile_number_high_word = 0;
  etm_can_my_configuration.agile_number_low_word  = 36224;
  etm_can_my_configuration.agile_dash             = 000;
  etm_can_my_configuration.agile_rev_ascii        = 'B';

  etm_can_my_configuration.serial_number          = 102;
  etm_can_my_configuration.firmware_branch        = 0;
  etm_can_my_configuration.firmware_major_rev     = 1;
  etm_can_my_configuration.firmware_minor_rev     = 5;
#endif
  

  


  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 
  
  _LATG14 = 0;
  _TRISG14 = 0;
  
  while (1) {
    ETMCanProcessMessage();
    
#ifdef __ETM_CAN_MASTER_MODULE
    ETMCanProcessLogData();
    ETMCanMaster100msCommunication();
#else
    etm_can_system_debug_data.debug_0 = etm_can_tx_message_buffer.message_write_count;
    etm_can_system_debug_data.debug_1 = etm_can_rx_message_buffer.message_write_count;
    etm_can_status_register.status_word_0 = 0xF0F0;
    ETMCanSlaveLog100ms();
#endif
  }
}



