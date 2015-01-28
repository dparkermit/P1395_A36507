#include "A36507.h"
#include "FIRMWARE_VERSION.h"



unsigned long dan_test_long = 0x12345678;

unsigned int dan_test_high_word;
unsigned int dan_test_low_word;

unsigned int dan_test[10] = {0x0102,0x0304,0x0405,0x0607,0x0809,0x0A0B,0x0C0D,0x0E0F,0x1020,0x3040};

unsigned int dan_test2[10];

unsigned char dan_test_char;

RTC_TIME test_time;



void DoA36507(void);

/*
  Modules to be created

  Ethernet Interface Module - Dongying
  Higher Level communication based on Ethernet 
  Real Time Clock 
  
  
  
*/
_FOSC(ECIO_PLL16 & CSW_FSCM_OFF); 
//_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


ETMEEProm U5_FM24C64B;
RTC_DS3231 U6_DS3231;
A36507GlobalVars global_data_A36507;

unsigned int CheckSystemFault(void);
unsigned int CheckHVOffFault(void);
unsigned int CheckFault(void);
unsigned int CheckAllModulesConfigured(void);
void CalculateHeaterWarmupTimers(void);
void InitializeA36507(void);
void DoStateMachine(void);
void SendToEventLog(ETMCanStatusRegister* ptr_status);



#define STATE_STARTUP                     0x10
#define STATE_WAITING_FOR_INITIALIZATION  0x15
#define STATE_WARMUP                      0x20
#define STATE_STANDBY                     0x30
#define STATE_DRIVE_UP                    0x40
#define STATE_READY                       0x50
#define STATE_XRAY_ON                     0x60


#define STATE_FAULT_HOLD                  0x80
#define STATE_FAULT_RESET                 0x90
#define STATE_FAULT_SYSTEM                0xA0


int main(void) {
  
  global_data_A36507.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {

    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
    break;


  case STATE_WAITING_FOR_INITIALIZATION:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    // Calculate all of the warmup counters based on previous warmup completed
    CalculateHeaterWarmupTimers();
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      if (CheckAllModulesConfigured()) {
      	global_data_A36507.control_state = STATE_WARMUP;
      }
    }
    break;
    

  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();
      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (CheckSystemFault()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;
    
        
  case STATE_STANDBY:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
     while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (!_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckHVOffFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
     }
    break;


#define DRIVE_UP_TIMEOUT            1000  // 10 Seconds
  case STATE_DRIVE_UP:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    global_data_A36507.drive_up_timer = 0;
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      // Check to see if the HV Lambda is ready, if it is check all faults and move to ready or fault hold
      if (!_HV_LAMBDA_NOT_READY) {
	if (CheckFault()) {
	  global_data_A36507.control_state = STATE_FAULT_HOLD;
	} else {
	  global_data_A36507.control_state = STATE_READY;
	}
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (global_data_A36507.drive_up_timer >= DRIVE_UP_TIMEOUT) {
	_FAULT_DRIVE_UP_TIMEOUT = 1;
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
      if (CheckHVOffFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_READY:
    // Enable XRAYs to Pulse Sync Board
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_XRAY_ON:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
    }
    break;


  case STATE_FAULT_HOLD:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
      }
    }
    break;


  case STATE_FAULT_RESET:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    while (global_data_A36507.control_state == STATE_FAULT_RESET) {
      DoA36507();
      if (CheckHVOffFault() == 0) {
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
      }
      if (CheckSystemFault()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;

    
  case STATE_FAULT_SYSTEM:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    while (1) {
      DoA36507();
    }
    break;
    
    
  default:
    global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    break;
  }
}


unsigned int CheckHVOffFault(void) {
  unsigned int fault = 0;
  
  if (_HEATER_MAGNET_OFF) {
    if (!_FAULT_HTR_MAG_NOT_OPERATE) {
      // There is a new Heater Magnet fault
      SendToEventLog(&etm_can_heater_magnet_mirror.status_data);
    }
#ifndef __IGNORE_HEATER_MAGNET_MODULE
    fault = 1;
#endif
  }
  _FAULT_HTR_MAG_NOT_OPERATE = _HEATER_MAGNET_OFF;
  
  if (!_GUN_HEATER_ON) {
    if (!_FAULT_GUN_HEATER_OFF) {
      // The gun heater has just turned off
      _FAULT_GUN_DVR_NOT_OPERATE = 1;
      SendToEventLog(&etm_can_gun_driver_mirror.status_data);
    }
#ifndef __IGNORE_GUN_DRIVER_MODULE
    fault = 1;
#endif
  }
  _FAULT_GUN_HEATER_OFF = !_GUN_HEATER_ON;

  return fault;
}

unsigned int CheckFault(void) {
  unsigned int fault = 0;
  
  // Update the fault status of each of the boards.
  if (_HV_LAMBDA_NOT_READY) {
    if (!_FAULT_HV_LAMBDA_NOT_OPERATE) {
      // There is a NEW Lambda fault.
      SendToEventLog(&etm_can_hv_lamdba_mirror.status_data);
    } 
#ifndef __IGNORE_HV_LAMBDA_MODULE
    fault = 1;
#endif
  }
  _FAULT_HV_LAMBDA_NOT_OPERATE = _HV_LAMBDA_NOT_READY;
  
  
  if (_ION_PUMP_NOT_READY) {
    if (!_FAULT_ION_PUMP_NOT_OEPRATE) {
      // There is a NEW Ion Pump Fault
      SendToEventLog(&etm_can_ion_pump_mirror.status_data);
    }
#ifndef __IGNORE_ION_PUMP_MODULE
    fault = 1;
#endif
  }
  _FAULT_ION_PUMP_NOT_OEPRATE = _ION_PUMP_NOT_READY;
  
  if (_AFC_NOT_READY) {
    if (!_FAULT_AFC_NOT_OPERATE) {
      // There is a NEW AFC Fault
      SendToEventLog(&etm_can_afc_mirror.status_data);
    }
#ifndef __IGNORE_AFC_MODULE
    fault = 1;
#endif
  }
  _FAULT_AFC_NOT_OPERATE = _AFC_NOT_READY;

  if (_COOLING_NOT_READY) {
    if (!_FAULT_COOLING_NOT_OPERATE) {
      // There is a NEW Cooling Fault
      SendToEventLog(&etm_can_cooling_mirror.status_data);
    }
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
    fault = 1;
#endif  
  }
  _FAULT_COOLING_NOT_OPERATE = _COOLING_NOT_READY;
  
  if (_GUN_DRIVER_NOT_READY) {
    if ((!_FAULT_GUN_DVR_NOT_OPERATE) && (_GUN_HEATER_ON)) {
      // There is a NEW Gun Driver Fault (if it was the gun heater turning off, it gets logged in CheckHVOffFault()
      SendToEventLog(&etm_can_gun_driver_mirror.status_data);
    }
#ifndef __IGNORE_GUN_DRIVER_MODULE
    fault = 1;
#endif
  }
  _FAULT_GUN_DVR_NOT_OPERATE = _GUN_DRIVER_NOT_READY;
  
  if (_PULSE_CURRENT_NOT_READY) {
    if (!_FAULT_PULSE_CURRENT_MON_NOT_OPERATE) {
      // There is a new pulse current monitor fault
      SendToEventLog(&etm_can_magnetron_current_mirror.status_data);
    }
#ifndef __IGNORE_PULSE_CURRENT_MODULE
    fault = 1;
#endif
  }
  _FAULT_PULSE_CURRENT_MON_NOT_OPERATE = _PULSE_CURRENT_NOT_READY;
  
  if (_PULSE_SYNC_NOT_READY) {    
    if (!_FAULT_PULSE_SYNC_NOT_OPERATE) {
      // There is a new pulse sync fault
      SendToEventLog(&etm_can_pulse_sync_mirror.status_data);
    }
#ifndef __IGNORE_PULSE_SYNC_MODULE
    fault = 1;
#endif
  }
  _FAULT_PULSE_SYNC_NOT_OPERATE = _PULSE_SYNC_NOT_READY;
  
  return fault;
}

void SendToEventLog(ETMCanStatusRegister* ptr_status) {
  
}


unsigned int CheckSystemFault(void) {
  return 0;
}







#define EEPROM_PAGE_AFC_HEATER_MAGNET           0
#define EEPROM_PAGE_HV_LAMBDA                   1
#define EEPROM_PAGE_GUN_DRIVER                  2
#define EEPROM_PAGE_PULSE_SYNC_PERSONALITY_1    3
#define EEPROM_PAGE_PULSE_SYNC_PERSONALITY_2    4
#define EEPROM_PAGE_PULSE_SYNC_PERSONALITY_3    5
#define EEPROM_PAGE_PULSE_SYNC_PERSONALITY_4    6
#define EEPROM_PAGE_ON_TIME                     7
#define EEPROM_PAGE_HEATER_TIMERS               8
// EEPROM PAGES reserved for future use         9->15







//global_data_A36507.magnetron_heater_last_warm_seconds  = 0x00010002;
//global_data_A36507.thyratron_heater_last_warm_seconds  = 0x00030004;
//global_data_A36507.gun_driver_heater_last_warm_seconds = 0x00050006;


//ETMEEPromWritePage(&U5_FM24C64B, EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);

//ETMEEPromReadPage(&U5_FM24C64B, EEPROM_PAGE_HEATER_TIMERS, 8, &dan_test2[0]);


#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
//#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
#define THYRATRON_WARM_UP_TIME               15   // 15 seconds
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes  



void CalculateHeaterWarmupTimers(void) {
  unsigned long seconds_now;
  unsigned long difference;
  // Read the warmup timers stored in EEPROM
  ETMEEPromReadPage(&U5_FM24C64B, EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
  dan_test_char = ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);
  
  // Calculate new magnetron heater warm up time remaining
  difference = seconds_now - global_data_A36507.magnetron_heater_last_warm_seconds;
  if (difference > (MAGNETRON_HEATER_WARM_UP_TIME >> 1)) {
    global_data_A36507.magnetron_heater_warmup_counter_seconds = MAGNETRON_HEATER_WARM_UP_TIME;    
  } else {
    global_data_A36507.magnetron_heater_warmup_counter_seconds = (difference << 1);
  }

  // Calculate new thyratron warm up time remaining
  difference = seconds_now - global_data_A36507.thyratron_heater_last_warm_seconds;
  if (difference > (THYRATRON_WARM_UP_TIME >> 1)) {
    global_data_A36507.thyratron_warmup_counter_seconds = THYRATRON_WARM_UP_TIME;    
  } else {
    global_data_A36507.thyratron_warmup_counter_seconds = (difference << 1);
  }
  
  // Calculate new gun driver heater warm up time remaining
  difference = seconds_now - global_data_A36507.gun_driver_heater_last_warm_seconds;
  if (difference > (GUN_DRIVER_HEATER_WARM_UP_TIME >> 1)) {
    global_data_A36507.gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_HEATER_WARM_UP_TIME;
  } else {
    global_data_A36507.gun_driver_heater_warmup_counter_seconds = (difference << 1);
  }

}


void DoA36507(void) {
  unsigned long seconds_now;
  ETMCanDoCan();
  TCPmodbus_task();

  



  etm_can_ethernet_board_data.control_state_mirror = global_data_A36507.control_state;

  local_debug_data.debug_0 = global_data_A36507.thyratron_warmup_counter_seconds;
  local_debug_data.debug_1 = global_data_A36507.magnetron_heater_warmup_counter_seconds;
  local_debug_data.debug_2 = global_data_A36507.gun_driver_heater_warmup_counter_seconds;
  local_debug_data.debug_3 = global_data_A36507.control_state;
  
  local_debug_data.debug_D = global_data_A36507.drive_up_timer;
  local_debug_data.debug_E = global_data_A36507.control_state;
  local_debug_data.debug_F = *(unsigned int*)&etm_can_sync_message.sync_0_control_word;


  if (_T5IF) {
    // 10ms Timer has expired
    _T5IF = 0;
    
    if (global_data_A36507.control_state == STATE_DRIVE_UP) {
      global_data_A36507.drive_up_timer++;
    }

    // DPARKER Check for cooling fault, and set the sync bit message as appropriate


    // Run at 1 second interval
    global_data_A36507.millisecond_counter += 10;
    if (global_data_A36507.millisecond_counter >= 1000) {
      global_data_A36507.millisecond_counter = 0;
    }

    // Run once a second at 0 milliseconds
    if (global_data_A36507.millisecond_counter == 0) {
      // Read Date/Time from RTC and update the warmup up counters
      ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
      seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

      // Update the warmup counters
      if (global_data_A36507.thyratron_warmup_counter_seconds > 0) {
	global_data_A36507.thyratron_warmup_counter_seconds--;
      } else {
	global_data_A36507.thyratron_heater_last_warm_seconds = seconds_now;
      }
      
      if ((!_HEATER_MAGNET_NOT_CONNECTED) && (!_HEATER_MAGNET_OFF)) {
	// The Magnetron heater is on
	if (global_data_A36507.magnetron_heater_warmup_counter_seconds > 0) {
	  global_data_A36507.magnetron_heater_warmup_counter_seconds--;
	} else {
	  global_data_A36507.magnetron_heater_last_warm_seconds = seconds_now;
	}
      } else {
	global_data_A36507.magnetron_heater_warmup_counter_seconds += 2;
	if (global_data_A36507.magnetron_heater_warmup_counter_seconds >= MAGNETRON_HEATER_WARM_UP_TIME) {
	  global_data_A36507.magnetron_heater_warmup_counter_seconds = MAGNETRON_HEATER_WARM_UP_TIME;
	}
      }
	
      if (!_GUN_DRIVER_NOT_CONNECTED && _GUN_HEATER_ON) {
	// The gun heater is on
	if (global_data_A36507.gun_driver_heater_warmup_counter_seconds > 0) {
	  global_data_A36507.gun_driver_heater_warmup_counter_seconds--;
	} else {
	  global_data_A36507.gun_driver_heater_last_warm_seconds = seconds_now;
	}
      } else {
	global_data_A36507.gun_driver_heater_warmup_counter_seconds += 2;
	if (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= GUN_DRIVER_HEATER_WARM_UP_TIME) {
	  global_data_A36507.gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_HEATER_WARM_UP_TIME;
	}
      }
      // Check for warmup done
      
#ifdef __IGNORE_HEATER_MAGNET_MODULE
      global_data_A36507.magnetron_heater_warmup_counter_seconds = 0;
#endif
      
#ifdef __IGNORE_GUN_DRIVER_MODULE
      global_data_A36507.gun_driver_heater_warmup_counter_seconds = 0;
#endif
      
      if ((global_data_A36507.thyratron_warmup_counter_seconds) || (global_data_A36507.magnetron_heater_warmup_counter_seconds) || (global_data_A36507.gun_driver_heater_warmup_counter_seconds)) {
	global_data_A36507.warmup_done = 0;
      } else {
	global_data_A36507.warmup_done = 1;
      }

      // Check if we need to send configuration to the pulse sync board.
      if (_PULSE_SYNC_NOT_CONFIGURED) {
	global_data_A36507.send_pulse_sync_config = 1;
      } else {
	global_data_A36507.send_pulse_sync_config = 0;
      }


    } //     if (global_data_A36507.millisecond_counter == 0) {

    // Run once a second at 250 milliseconds
    if (global_data_A36507.millisecond_counter == 250) {
      // Write Warmup Done Timers to EEPROM
      ETMEEPromWritePage(&U5_FM24C64B, EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
    }


    // Run once a second at 500 milliseconds
    if (global_data_A36507.millisecond_counter == 500) {
      // Write Seconds on Counters to EEPROM
      ETMEEPromWritePage(&U5_FM24C64B, EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);
    }
  }
}


unsigned int CheckAllModulesConfigured(void) {
  unsigned int system_configured;
  
  system_configured = 1;

#ifndef __IGNORE_HV_LAMBDA_MODULE
  if ((_HV_LAMBDA_NOT_CONNECTED) || (_HV_LAMBDA_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_ION_PUMP_MODULE
  if ((_ION_PUMP_NOT_CONNECTED) || (_ION_PUMP_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_AFC_MODULE
  if ((_AFC_NOT_CONNECTED) || (_AFC_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif  

#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if ((_COOLING_NOT_CONNECTED) || (_COOLING_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if ((_HEATER_MAGNET_NOT_CONNECTED) || (_HEATER_MAGNET_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_GUN_DRIVER_MODULE
  if ((_GUN_DRIVER_NOT_CONNECTED) || (_GUN_DRIVER_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if ((_PULSE_CURRENT_NOT_CONNECTED) || (_PULSE_CURRENT_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if ((_PULSE_SYNC_NOT_CONNECTED) || (_PULSE_SYNC_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

  return system_configured;

}

void InitializeA36507(void) {
  unsigned int startup_counter;

  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;

  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;


  // Set the not connected bits for all boards
  _HV_LAMBDA_NOT_CONNECTED     = 1;
  _ION_PUMP_NOT_CONNECTED      = 1;
  _AFC_NOT_CONNECTED           = 1;
  _COOLING_NOT_CONNECTED       = 1;
  _HEATER_MAGNET_NOT_CONNECTED = 1;
  _GUN_DRIVER_NOT_CONNECTED    = 1;
  _PULSE_CURRENT_NOT_CONNECTED = 1;
  _PULSE_SYNC_NOT_CONNECTED    = 1;


  // Initialize all I/O Registers
  TRISA = A36507_TRISA_VALUE;
  TRISB = A36507_TRISB_VALUE;
  TRISC = A36507_TRISC_VALUE;
  TRISD = A36507_TRISD_VALUE;
  TRISF = A36507_TRISF_VALUE;
  TRISG = A36507_TRISG_VALUE;

  // Initialize TMR5
  PR5   = PR5_VALUE_10_MILLISECONDS;
  TMR5  = 0;
  _T5IF = 0;
  T5CON = T5CON_VALUE;

  ETMEEPromConfigureDevice(&U5_FM24C64B, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);

  ConfigureDS3231(&U6_DS3231, I2C_PORT, RTC_DEFAULT_CONFIG, FCY_CLK, ETM_I2C_400K_BAUD);
  
  // Initialize the Can module
  ETMCanInitialize();
  
  // Initialize TCPmodbus Module
  TCPmodbus_init();

  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;

  _ADIF = 0;
  _ADON = 1;

  // Wait for data to be read
  while (_ADIF == 0);

  global_data_A36507.analog_input_5v_mon.filtered_adc_reading  = ADCBUF0 + ADCBUF2 + ADCBUF4 + ADCBUF6 + ADCBUF8 + ADCBUFA + ADCBUFC + ADCBUFE;
  global_data_A36507.analog_input_3v3_mon.filtered_adc_reading = ADCBUF1 + ADCBUF3 + ADCBUF5 + ADCBUF7 + ADCBUF9 + ADCBUFB + ADCBUFD + ADCBUFF;

  global_data_A36507.analog_input_5v_mon.filtered_adc_reading  <<= 1;
  global_data_A36507.analog_input_3v3_mon.filtered_adc_reading <<= 1;  

  ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_5v_mon);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_3v3_mon);

  
  local_debug_data.debug_8 = global_data_A36507.analog_input_5v_mon.reading_scaled_and_calibrated;
  local_debug_data.debug_9 = global_data_A36507.analog_input_3v3_mon.reading_scaled_and_calibrated;

  _ADON = 0;
  
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

  dan_test_low_word = *(unsigned int*)&dan_test_long;
  dan_test_high_word = *((unsigned int*)&dan_test_long + 1); 

}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
