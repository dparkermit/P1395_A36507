#include "A36507.h"
#include "ETM_CAN_PUBLIC.h"
#include "ETM_CAN.h"
#include "ETM_EEPROM.h"
#include "TCPmodbus.h"
#include "FIRMWARE_VERSION.h"

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

void InitializeA36507(void);



typedef struct {
  unsigned int control_state;
  unsigned int thyratron_warmup_counter_seconds;
  unsigned int magnetron_heater_warmup_counter_seconds;
  unsigned int gun_driver_heater_warmup_counter_seconds;
} A36507GlobalVars;

A36507GlobalVars global_data_A36507;


#define STATE_STARTUP            0x10
#define STATE_WARMUP             0x20
#define STATE_STANDBY            0x30
#define STATE_DRIVE_UP           0x40
#define STATE_READY              0x50
#define STATE_XRAY_ON            0x60


#define STATE_COLD_FAULT         0x80
#define STATE_WARM_FAULT         0x90
#define STATE_HEATER_FAULT       0xA0


void DoStateMachine(void);

int main(void) {
  
  global_data_A36507.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}


#define THYRATRON_WARMUP_SECONDS          120
#define MAGNETRON_WARMUP_SECONDS          120
#define GUN_DRIVER_WARMUP_SECONDS         120

void UpdateWarmupTimers(void);

void DoStateMachine(void) {
  unsigned int millisecond_counter;
  
  switch (global_data_A36507.control_state) {
    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.control_state = STATE_WARMUP;

    // DPARKER calculate all of the warmup counters based on previous warmup completed
    // Why do we need a gun driver warmup timer.  It is ALWAYS going to be less than the thyratron warmup counter
    break;


  case STATE_WARMUP:
    global_data_A36507.thyratron_warmup_counter_seconds = 0;
    global_data_A36507.magnetron_heater_warmup_counter_seconds = 0;
    global_data_A36507.gun_driver_heater_warmup_counter_seconds = 0;

    
    while (global_data_A36507.control_state == STATE_WARMUP) {
      etm_can_system_debug_data.debug_0 = global_data_A36507.thyratron_warmup_counter_seconds;
      etm_can_system_debug_data.debug_1 = global_data_A36507.magnetron_heater_warmup_counter_seconds;
      etm_can_system_debug_data.debug_2 = global_data_A36507.gun_driver_heater_warmup_counter_seconds;

      ETMCanDoCan();
      TCPmodbus_task();
      if (_T5IF) {
	_T5IF = 0;
	millisecond_counter += 10;
	if (millisecond_counter >= 1000) {
	  UpdateWarmupTimers();
	  millisecond_counter = 0;
	}
      }
    
      if ((global_data_A36507.thyratron_warmup_counter_seconds >= THYRATRON_WARMUP_SECONDS) && (global_data_A36507.magnetron_heater_warmup_counter_seconds >= MAGNETRON_WARMUP_SECONDS) && (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= GUN_DRIVER_WARMUP_SECONDS)) {
	global_data_A36507.control_state = STATE_STANDBY;
      }

      if (CheckFault()) {
	global_data_A36507.control_state = STATE_COLD_FAULT;
      }
      
    }
    break;
    
        
  case STATE_STANDBY:
    while (global_data_A36507.control_state == STATE_STANDBY) {
      ETMCanDoCan();
      TCPmodbus_task();
      // Write Heater Times
      if (etm_can_sync_message.sync_0 == 0) {
	// DO NOTHING
      }
    }
    break;


  case STATE_WARM_FAULT:
    while (global_data_A36507.control_state == STATE_WARM_FAULT) {
      ETMCanDoCan();
      TCPmodbus_task();
      // Write Heater Times


      // IF Customer HV is OFF, attempt to reset all faults  
      
      if (!CheckFault()) {
	global_data_A36507.control_state == STATE_STANDBY;
      }
      
      if (CheckHeaterFault()) {
	global_data_A36507.control_state == STATE_HEATER_FAULT;
      }
    }
    break;


    /*
  case STATE_HEATER_FAULT:
    // Disable HV Lambda
    // Disable Heater Magnet
    // Disable HV ON Command to Pulse Sync Board
    // Disable Gun Driver 
    while (global_data_A36507.control_state == STATE_HEATER_FAULT) {
      ETMCanDoCan();
      TCPmodbus_task();
      // Write Tyratron Heater Timer
      // If Gun Driver Heater Enabled, write gun driver heater time
      // If Heater/Magnet is Operational, write heater/Magnet time

      
      // IF Customer HV is OFF, attempt to reset all faults

      if (!CheckFault()) {
	global_data_A36507.control_state == STATE_WARMUP;
      }
    }
    
    break;

    */

  case STATE_COLD_FAULT:
    // Disable HV Lambda
    // Disable HV ON Command to Pulse Sync Board
    // Disable Gun Driver High Voltage
    while (global_data_A36507.control_state == STATE_COLD_FAULT) {
      ETMCanDoCan();
      TCPmodbus_task();
      // Thyratron Heater -- Increment Thyratron Heater Counter, If greater than warmup time, write the time
      // Gun Driver Heater -- If Gun Driver Heater Enabled, Increment the Thyratro Heater Counter (else decrement by 2), If greater than warmup time, write the time
      // Magnetron Heater -- If the Magnetron/Heater is not Faulted, Increment the Magnetron Heater Counter, If greater than warmup, write the time
      
      // IF Customer HV is OFF, attempt to reset all faults

      if (!CheckFault()) {
	global_data_A36507.control_state == STATE_WARMUP;
      }
    }
    
    break;



  default:
    global_data_A36507.control_state = STATE_COLD_FAULT;
    break;
  }
  
}



void UpdateWarmupTimers(void) {
  // The thyratron is warming up if the power is on, so increment the thyratron warmup counter
  global_data_A36507.thyratron_warmup_counter_seconds++;	  
  if (global_data_A36507.thyratron_warmup_counter_seconds >= 0xFF00) {
    global_data_A36507.thyratron_warmup_counter_seconds = 0xFF00;
  }
  
  // If the magnetron heater is on, increment it's heater counter otherwise set it to zero
  if ((ETMCanCheckBit(etm_can_heater_magnet_mirror.status_data.status_word_0, STATUS_BIT_SUM_FAULT) == 0) && (ETMCanCheckBit(etm_can_heater_magnet_mirror.status_data.status_word_0, STATUS_BIT_PULSE_INHIBITED) == 0)) {
    global_data_A36507.magnetron_heater_warmup_counter_seconds++;
    if (global_data_A36507.magnetron_heater_warmup_counter_seconds >= 0xFF00) {
      global_data_A36507.magnetron_heater_warmup_counter_seconds = 0xFF00;
    }
  } else {
    if (global_data_A36507.magnetron_heater_warmup_counter_seconds >= 2) {
      global_data_A36507.magnetron_heater_warmup_counter_seconds -= 2;
    }
  }
  
  // If the gun driver heater is on, increment it's heater counter otherwise decrement it by 2
  if (ETMCanCheckBit(etm_can_gun_driver_mirror.status_data.status_word_0, STATUS_BIT_USER_DEFINED_8) == 0) {
    global_data_A36507.gun_driver_heater_warmup_counter_seconds++;
    if (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= 0xFF00) {
      global_data_A36507.gun_driver_heater_warmup_counter_seconds = 0xFF00;
    }
  } else {
    if (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= 2) {
      global_data_A36507.gun_driver_heater_warmup_counter_seconds -= 2;
    }
  } 
}


void InitializeA36507(void) {
  unsigned int startup_counter;


  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  //etm_can_status_register.status_word_0_inhbit_mask = A36444_INHIBIT_MASK;
  //etm_can_status_register.status_word_1_fault_mask  = A36444_FAULT_MASK;


  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;


  // Initialize all I/O Registers
  TRISA = A36507_TRISA_VALUE;
  TRISB = A36507_TRISB_VALUE;
  TRISC = A36507_TRISC_VALUE;
  TRISD = A36507_TRISD_VALUE;
  TRISF = A36507_TRISF_VALUE;
  TRISG = A36507_TRISG_VALUE;


#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)

  // Initialize TMR5
  PR5   = PR5_VALUE_10_MILLISECONDS;
  TMR5  = 0;
  _T5IF = 0;
  T5CON = T5CON_VALUE;


  ETMEEPromConfigureDevice(&U5_FM24C64B, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);

  // Initialize the Can module
  ETMCanInitialize();
  
  // Initialize TCPmodbus Module
  TCPmodbus_init();



  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  /*
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING_STARTUP;

  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  */


  // Flash LEDs at Startup
  startup_counter = 0;
  while (startup_counter <= 400) {  // 4 Seconds total
    ETMCanDoCan();
    if (_T5IF) {
      _T5IF =0;
      startup_counter++;
    } 
    switch (((startup_counter >> 4) & 0b11)) {
      
    case 0:
      PIN_OUT_ETM_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
      break;
      
    case 1:
      PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
      break;
      
    case 2:
      PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
      break;

    case 3:
      PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
      PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = OLL_LED_ON;
      break;
    }
  }

}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
