#include "A36507.h"
#include "ETM_CAN_PUBLIC.h"

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


void InitializeA36507(void);


int main(void) {
  
  ETMCanInitialize();

  InitializeA36507();

  
  //#define ETM_CAN_REGISTER_CALIBRATION_TEST          0x0400
  //#define ETM_CAN_REGISTER_LOCAL_TEST                0x0100
  

  
  while (1) {
    ETMCanDoCan();
    
  }
}



void InitializeA36507(void) {
  _TRISA7 = 0;
}
