#include "A36507.h"
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


void InitializeA36507(void);


int main(void) {
  
  ETMCanInitialize();

  InitializeA36507();

  
  //#define ETM_CAN_REGISTER_CALIBRATION_TEST          0x0400
  //#define ETM_CAN_REGISTER_LOCAL_TEST                0x0100
  

  
  while (1) {
    ETMCanProcessMessage();

    if (_T3IF) {
      _T3IF = 0;
      etm_can_can_status.can_status_timeout++;
      // DPARKER Execute error code
      // For the master this means to tell the pulse sync to stop pulsing.
      // For other modules this means to enter the hot fault state
    }
  

    
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



void InitializeA36507(void) {
  // DPARKER read configuration from EEPROM
  etm_can_my_configuration.agile_number_high_word = 0;
  etm_can_my_configuration.agile_number_low_word  = 34760;
  etm_can_my_configuration.agile_dash             = 000;
  etm_can_my_configuration.agile_rev_ascii        = 'G';
  etm_can_my_configuration.serial_number          = 107;


  // Firmware data should be stored in H File
  etm_can_my_configuration.firmware_branch        = FIRMWARE_BRANCH_A36507;
  etm_can_my_configuration.firmware_major_rev     = FIRMWARE_MAJOR_REV_A36507;
  etm_can_my_configuration.firmware_minor_rev     = FIRMWARE_MINOR_REV_A36507;


  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 
  
  _LATG14 = 0;
  _TRISG14 = 0;
  
  


}
