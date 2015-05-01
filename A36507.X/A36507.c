#include "A36507.h"
#include "FIRMWARE_VERSION.h"

#define etm_can_status_register   etm_can_ethernet_board_data.status_data
#define local_debug_data          etm_can_ethernet_board_data.debug_data
#define local_can_errors          etm_can_ethernet_board_data.can_status
#define etm_can_my_configuration  etm_can_ethernet_board_data.configuration


unsigned int dan_temp_counter = 0;

const unsigned int FilamentLookUpTable[64] = {FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193};

void ZeroSystemPoweredTime(void);
void LoadDefaultSystemCalibrationToEEProm(void);
void ReadSystemConfigurationFromEEProm(unsigned int personality);
void FlashLeds(void);

//unsigned int i2c_error_check = 0;  // DPARKER - ALL I2C functions should check for bus error so that we can reset if needed

void UpdateHeaterScale(void);
unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage);


void ExecuteEthernetCommand(unsigned int personality);

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
_FBORPOR(PWRT_64 & NONE & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



RTC_DS3231 U6_DS3231;
A36507GlobalVars global_data_A36507;

unsigned int CheckSystemFault(void);
unsigned int CheckHVOffFault(void);
unsigned int CheckFault(void);
unsigned int CheckAllModulesConfigured(void);
void CalculateHeaterWarmupTimers(void);
void InitializeA36507(void);
void DoStateMachine(void);


int main(void) {
  
  global_data_A36507.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}

#define __SYSTEM_CONFIGURATION_2_5_MEV

void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {

    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.control_state = STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP, 0);
    break;

  case STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _STATUS_PERSONALITY_LOADED = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC, 0);
    while (global_data_A36507.control_state == STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC) {
      DoA36507();
      FlashLeds();

#ifdef __IGNORE_PULSE_SYNC_MODULE
      etm_can_pulse_sync_mirror.status_data.data_word_B = 255;
#endif
      if (etm_can_pulse_sync_mirror.status_data.data_word_B != 0) {
	// a personality has been received from pulse sync board
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
	SendToEventLog(LOG_ID_PERSONALITY_RECEIVED, 0);

#ifndef __SYSTEM_CONFIGURATION_2_5_MEV
	if (etm_can_pulse_sync_mirror.status_data.data_word_B >= 5) {
	  global_data_A36507.control_state = STATE_FAULT_SYSTEM;
	  SendToEventLog(LOG_ID_PERSONALITY_ERROR_6_4, 0);
	}
#endif
	
#ifndef __SYSTEM_CONFIGURATION_6_4_MEV
	if (etm_can_pulse_sync_mirror.status_data.data_word_B != 255) {
	  global_data_A36507.control_state = STATE_FAULT_SYSTEM;
	  SendToEventLog(LOG_ID_PERSONALITY_ERROR_2_5, 0);
	}
#endif
      }
    }

  case STATE_WAITING_FOR_INITIALIZATION:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _STATUS_PERSONALITY_LOADED = 1;
    ReadSystemConfigurationFromEEProm(etm_can_pulse_sync_mirror.status_data.data_word_B);
    // Calculate all of the warmup counters based on previous warmup completed
    CalculateHeaterWarmupTimers();
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION, 0);
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      FlashLeds();
      if (CheckAllModulesConfigured() && (global_data_A36507.startup_counter >= 300)) {
      	global_data_A36507.control_state = STATE_WARMUP;
	SendToEventLog(LOG_ID_ALL_MODULES_CONFIGURED, 0);
      }
    }
    break;
    

  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_WARMUP, 0);
    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();
      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_WARMUP_DONE, 0);
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
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY, 0);
     while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (!_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
	SendToEventLog(LOG_ID_CUSTOMER_HV_ON, 0);
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
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP, 0);
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      // Check to see if the HV Lambda is ready, if it is check all faults and move to ready or fault hold
      if (!_HV_LAMBDA_NOT_READY) {
	if (CheckFault()) {
	  global_data_A36507.control_state = STATE_FAULT_HOLD;
	} else {
	  global_data_A36507.control_state = STATE_READY;
	  SendToEventLog(LOG_ID_DRIVEUP_COMPLETE, 0);
	}
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF, 0);
      }
      if (global_data_A36507.drive_up_timer >= DRIVE_UP_TIMEOUT) {
	_FAULT_DRIVE_UP_TIMEOUT = 1;
	global_data_A36507.control_state = STATE_FAULT_HOLD;
	SendToEventLog(LOG_ID_DRIVE_UP_TIMEOUT, 0);
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
    SendToEventLog(LOG_ID_ENTERED_STATE_READY, 0);
    while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF == 0) {
	global_data_A36507.control_state = STATE_XRAY_ON;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_ON, 0);
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF, 0);
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
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON, 0);
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
	global_data_A36507.control_state = STATE_READY;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_OFF, 0);
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF, 0);
      }
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_FAULT_HOLD:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD, 0);
    while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_OFF, 0);
      }
    }
    break;


  case STATE_FAULT_RESET:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET, 0);
    while (global_data_A36507.control_state == STATE_FAULT_RESET) {
      DoA36507();
      _FAULT_REGISTER = 0; // DPARKER IS THIS RIGHT????


      if (CheckHVOffFault() == 0) {
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
	SendToEventLog(LOG_ID_HV_OFF_FAULTS_CLEAR, 0);
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
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM, 0);
    while (1) {
      DoA36507();
    }
    break;
    
    
  default:
    global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    break;
  }
}


void FlashLeds(void) {
  switch (((global_data_A36507.startup_counter >> 4) & 0b11)) {
    
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

unsigned int CheckHVOffFault(void) {
  unsigned int fault = 0;
  
  if (!CheckAllModulesConfigured()) {
    fault = 1;
    SendToEventLog(LOG_ID_FAULT_MODULE_NOT_CONFIGURED, 0);
    if ((global_data_A36507.control_state == STATE_XRAY_ON) || (global_data_A36507.control_state == STATE_READY) || (global_data_A36507.control_state == STATE_DRIVE_UP) || (global_data_A36507.control_state == STATE_STANDBY)) {
   
    }
  }

  if (_HEATER_MAGNET_OFF) {
    if (!_FAULT_HTR_MAG_NOT_OPERATE) {
      // There is a new Heater Magnet fault

      SendToEventLog(LOG_ID_FAULT_HTR_MAG_BOARD,0);

      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_0) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_OVER_CURRENT_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_1) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_UNDER_CURRENT_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_2) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_OVER_CURRENT_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_3) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_UNDER_CURRENT_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_4) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_OVER_VOLTAGE_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_5) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HEATER_UNDER_VOTLAGE_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_6) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_OVER_CURRENT_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_7) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_UNDER_CURRENT_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_8) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_OVER_CURRENT_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_9) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_UNDER_CURRENT_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_A) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_OVER_VOLTAGE_ABSOLUTE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_B) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_MAGNET_UNDER_VOTLAGE_RELATIVE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_C) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HW_HEATER_OVER_VOLTAGE,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_D) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_HW_TEMPERATURE_SWITCH,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_E) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_COOLANT_FAULT,0);
      }
      if (etm_can_heater_magnet_mirror.status_data.fault_bits.fault_F) {
	SendToEventLog(LOG_ID_FAULT_HTR_MAG_CAN_COMMUNICATION_LATCHED,0);
      }
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
      SendToEventLog(LOG_ID_FAULT_GUN_DRIVER_BOARD_HV_OFF,0);
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
  
  if (CheckHVOffFault()) {
    fault = 1;
  }


  // Update the fault status of each of the boards.
  if (_HV_LAMBDA_NOT_READY) {
    if (!_FAULT_HV_LAMBDA_NOT_OPERATE) {
      // There is a NEW Lambda fault.
      SendToEventLog(LOG_ID_FAULT_HV_LAMBDA_BOARD,0);
    } 
#ifndef __IGNORE_HV_LAMBDA_MODULE
    fault = 1;
#endif
  }
  _FAULT_HV_LAMBDA_NOT_OPERATE = _HV_LAMBDA_NOT_READY;
  
  
  if (_ION_PUMP_NOT_READY) {
    if (!_FAULT_ION_PUMP_NOT_OEPRATE) {
      // There is a NEW Ion Pump Fault
      SendToEventLog(LOG_ID_FAULT_ION_PUMP_BOARD,0);
    }
#ifndef __IGNORE_ION_PUMP_MODULE
    fault = 1;
#endif
  }
  _FAULT_ION_PUMP_NOT_OEPRATE = _ION_PUMP_NOT_READY;

  
  if (_AFC_NOT_READY) {
    if (!_FAULT_AFC_NOT_OPERATE) {
      // There is a NEW AFC Fault
      SendToEventLog(LOG_ID_FAULT_AFC_BOARD,0);
    }
#ifndef __IGNORE_AFC_MODULE
    fault = 1;
#endif
  }
  _FAULT_AFC_NOT_OPERATE = _AFC_NOT_READY;


  if (_COOLING_NOT_READY) {
    if (!_FAULT_COOLING_NOT_OPERATE) {
      // There is a NEW Cooling Fault
      SendToEventLog(LOG_ID_FAULT_COOING_INTERFACE_BOARD,0);
    }
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
    fault = 1;
#endif  
  }
  _FAULT_COOLING_NOT_OPERATE = _COOLING_NOT_READY;
  

  if (_GUN_DRIVER_NOT_READY) {
    if ((!_FAULT_GUN_DVR_NOT_OPERATE) && (_GUN_HEATER_ON)) {
      // There is a NEW Gun Driver Fault (if it was the gun heater turning off, it gets logged in CheckHVOffFault()
      SendToEventLog(LOG_ID_FAULT_GUN_DRIVER_BOARD_GENERAL,0);
    }
#ifndef __IGNORE_GUN_DRIVER_MODULE
    fault = 1;
#endif
  }
  _FAULT_GUN_DVR_NOT_OPERATE = _GUN_DRIVER_NOT_READY;
  
  if (_PULSE_CURRENT_NOT_READY) {
    if (!_FAULT_PULSE_CURRENT_MON_NOT_OPERATE) {
      // There is a new pulse current monitor fault
      SendToEventLog(LOG_ID_FAULT_PULSE_MONITOR_BOARD,0);
    }
#ifndef __IGNORE_PULSE_CURRENT_MODULE
    fault = 1;
#endif
  }
  _FAULT_PULSE_CURRENT_MON_NOT_OPERATE = _PULSE_CURRENT_NOT_READY;
  
  if (_PULSE_SYNC_NOT_READY) {    
    if (!_FAULT_PULSE_SYNC_NOT_OPERATE) {
      // There is a new pulse sync fault
      SendToEventLog(LOG_ID_FAULT_PULSE_SYNC_BOARD,0);
    }
#ifndef __IGNORE_PULSE_SYNC_MODULE
    fault = 1;
#endif
  }
  _FAULT_PULSE_SYNC_NOT_OPERATE = _PULSE_SYNC_NOT_READY;
  
  return fault;
}


unsigned int CheckSystemFault(void) {
  return 0;
}





#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes



void CalculateHeaterWarmupTimers(void) {
  unsigned long difference;
  // Read the warmup timers stored in EEPROM
  ETMEEPromReadPage(EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
  ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  global_data_A36507.time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

  // Calculate new magnetron heater warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.magnetron_heater_last_warm_seconds;
  if (difference > (MAGNETRON_HEATER_WARM_UP_TIME >> 1)) {
    global_data_A36507.magnetron_heater_warmup_counter_seconds = MAGNETRON_HEATER_WARM_UP_TIME;    
  } else {
    global_data_A36507.magnetron_heater_warmup_counter_seconds = (difference << 1);
  }

  // Calculate new thyratron warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.thyratron_heater_last_warm_seconds;
  if (difference > (THYRATRON_WARM_UP_TIME >> 1)) {
    global_data_A36507.thyratron_warmup_counter_seconds = THYRATRON_WARM_UP_TIME;    
  } else {
    global_data_A36507.thyratron_warmup_counter_seconds = (difference << 1);
  }
  
  // Calculate new gun driver heater warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.gun_driver_heater_last_warm_seconds;
  if (difference > (GUN_DRIVER_HEATER_WARM_UP_TIME >> 1)) {
    global_data_A36507.gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_HEATER_WARM_UP_TIME;
  } else {
    global_data_A36507.gun_driver_heater_warmup_counter_seconds = (difference << 1);
  }

}

#define SEND_BUFFER_A            1
#define SEND_BUFFER_B            0


void DoA36507(void) {
  ETMCanMasterDoCan();
  TCPmodbus_task();
  ExecuteEthernetCommand(1);  // DPARKER This is using personality 1, should read from pulse sync

  if ((global_data_A36507.buffer_a_ready_to_send) & (!global_data_A36507.buffer_a_sent)) {
    SendPulseData(SEND_BUFFER_A);
    global_data_A36507.buffer_a_sent = 1;
  }

  if ((global_data_A36507.buffer_b_ready_to_send) & (!global_data_A36507.buffer_b_sent)) {
    SendPulseData(SEND_BUFFER_B);
    global_data_A36507.buffer_b_sent = 1;
  }


  // Check to see if cooling is present
  _SYNC_CONTROL_COOLING_FAULT = 0;
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (_COOLING_NOT_CONNECTED) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
    _FAULT_COOLING_NOT_CONNECTED = 1;
  }
  if (_COOLING_NOT_READY) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
    _FAULT_COOLING_NOT_CONNECTED = 1;
  }
#endif
  
  local_debug_data.debug_0 = global_data_A36507.control_state;
  local_debug_data.debug_1 = global_data_A36507.event_log_counter;
  local_debug_data.debug_2 = (unsigned int)(global_data_A36507.time_seconds_now >> 16);
  local_debug_data.debug_3 = (unsigned int)(global_data_A36507.time_seconds_now & 0x0000FFFF);
  


  local_debug_data.debug_4 = global_data_A36507.no_connect_count_ion_pump_board;
  local_debug_data.debug_5 = global_data_A36507.no_connect_count_magnetron_current_board;
  local_debug_data.debug_6 = global_data_A36507.no_connect_count_pulse_sync_board;
  local_debug_data.debug_7 = global_data_A36507.no_connect_count_hv_lambda_board;
  local_debug_data.debug_8 = global_data_A36507.no_connect_count_afc_board;
  local_debug_data.debug_9 = global_data_A36507.no_connect_count_cooling_interface_board;
  local_debug_data.debug_A = global_data_A36507.no_connect_count_heater_magnet_board;
  local_debug_data.debug_B = global_data_A36507.no_connect_count_gun_driver_board;
  local_debug_data.debug_C = etm_can_next_pulse_level;
  local_debug_data.debug_D = etm_can_next_pulse_count;

  etm_can_ethernet_board_data.mirror_sync_0_control_word = *(unsigned int*)&etm_can_sync_message.sync_0_control_word;
  


  if (_T5IF) {
    // 10ms Timer has expired
    _T5IF = 0;
    global_data_A36507.millisecond_counter += 10;
    
    // Copy data from global variable strucutre to strucutre that gets sent to GUI
    // DPARKER do something better here
    etm_can_ethernet_board_data.mirror_control_state = global_data_A36507.control_state;
    etm_can_ethernet_board_data.mirror_system_powered_seconds = global_data_A36507.system_powered_seconds;
    etm_can_ethernet_board_data.mirror_system_hv_on_seconds = global_data_A36507.system_hv_on_seconds;
    etm_can_ethernet_board_data.mirror_system_xray_on_seconds = global_data_A36507.system_xray_on_seconds;
    etm_can_ethernet_board_data.mirror_time_seconds_now = global_data_A36507.time_seconds_now;
    etm_can_ethernet_board_data.mirror_average_output_power_watts = global_data_A36507.average_output_power_watts;
    etm_can_ethernet_board_data.mirror_thyratron_warmup_counter_seconds = global_data_A36507.thyratron_warmup_counter_seconds;
    etm_can_ethernet_board_data.mirror_magnetron_heater_warmup_counter_seconds = global_data_A36507.magnetron_heater_warmup_counter_seconds;
    etm_can_ethernet_board_data.mirror_gun_driver_heater_warmup_counter_seconds = global_data_A36507.gun_driver_heater_warmup_counter_seconds;

    if (global_data_A36507.control_state == STATE_DRIVE_UP) {
      global_data_A36507.drive_up_timer++;
    }

    if ((global_data_A36507.control_state == STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC) || (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION)) {
      global_data_A36507.startup_counter++;
    }

    // DPARKER Check for cooling fault, and set the sync bit message as appropriate


    // Run at 1 second interval
    if (global_data_A36507.millisecond_counter >= 1000) {
      global_data_A36507.millisecond_counter = 0;
      
    
      /*
      // DPARKER TESTING LOG FUNCTION
      high_speed_data_buffer_a[0].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[1].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[2].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[3].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[4].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[5].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[6].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[7].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[8].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[9].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[10].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[11].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[12].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[13].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[14].pulse_count = dan_temp_counter++;
      high_speed_data_buffer_a[15].pulse_count = dan_temp_counter++;


      high_speed_data_buffer_a[0].x_ray_on_seconds_lsw = global_data_A36507.time_seconds_now;
      high_speed_data_buffer_a[0].x_ray_on_milliseconds = global_data_A36507.millisecond_counter;
      high_speed_data_buffer_a[0].x_ray_on_milliseconds += TMR5>>10;  // Need to divide by 1250 to get milliseconds
      high_speed_data_buffer_a[0].hvlambda_readback_high_energy_lambda_program_voltage = 0;
      high_speed_data_buffer_a[0].hvlambda_readback_low_energy_lambda_program_voltage = 1;
      high_speed_data_buffer_a[0].hvlambda_readback_peak_lambda_voltage = 2;
      high_speed_data_buffer_a[0].afc_readback_current_position = 3;
      high_speed_data_buffer_a[0].afc_readback_target_position = 4;
      high_speed_data_buffer_a[0].afc_readback_a_input = 5;
      high_speed_data_buffer_a[0].afc_readback_b_input = 6;
      high_speed_data_buffer_a[0].afc_readback_filtered_error_reading = 7;
      high_speed_data_buffer_a[0].ionpump_readback_high_energy_target_current_reading = 8;
      high_speed_data_buffer_a[0].ionpump_readback_low_energy_target_current_reading = 9;
      high_speed_data_buffer_a[0].magmon_readback_magnetron_high_energy_current = 10;
      high_speed_data_buffer_a[0].magmon_readback_magnetron_low_energy_current = 11;
      high_speed_data_buffer_a[0].psync_readback_trigger_width_and_filtered_trigger_width = 12;
      high_speed_data_buffer_a[0].psync_readback_high_energy_grid_width_and_delay = 13;
      high_speed_data_buffer_a[0].psync_readback_low_energy_grid_width_and_delay = 14;
      


      
      SendPulseData(1);
      */
    
    }

    // Run once a second at 0 milliseconds
    if (global_data_A36507.millisecond_counter == 0) {
      // Read Date/Time from RTC and update the warmup up counters
      ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
      global_data_A36507.time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

      // Update the warmup counters
      if (global_data_A36507.thyratron_warmup_counter_seconds > 0) {
	global_data_A36507.thyratron_warmup_counter_seconds--;
      } else {
	global_data_A36507.thyratron_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
      }
      
      if ((!_HEATER_MAGNET_NOT_CONNECTED) && (!_HEATER_MAGNET_OFF)) {
	// The Magnetron heater is on
	if (global_data_A36507.magnetron_heater_warmup_counter_seconds > 0) {
	  global_data_A36507.magnetron_heater_warmup_counter_seconds--;
	} else {
	  global_data_A36507.magnetron_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
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
	  global_data_A36507.gun_driver_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
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


    } //     if (global_data_A36507.millisecond_counter == 0) {

    // Run once a second at 250 milliseconds
    if (global_data_A36507.millisecond_counter == 250) {
      // Write Warmup Done Timers to EEPROM
      ETMEEPromWritePage(EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
    }


    // Run once a second at 500 milliseconds
    if (global_data_A36507.millisecond_counter == 500) {
      // Write Seconds on Counters to EEPROM
      global_data_A36507.system_powered_seconds += 1;
      
      if (global_data_A36507.control_state == STATE_READY) {
	global_data_A36507.system_hv_on_seconds++;
      }
      
      if (global_data_A36507.control_state == STATE_XRAY_ON) {
	global_data_A36507.system_hv_on_seconds++;
	global_data_A36507.system_xray_on_seconds++;
      }
      ETMEEPromWritePage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);
    }
    
    // Update the heater current based on Output Power
    UpdateHeaterScale();
  }
}


// DPARKER - This will change if we use a new PFN with a different capacitance
unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage) {
  unsigned long power_milli_joule;
  unsigned int return_data;

  /*
    The Pulse Energy is Calculated for Each Pulse
    The Pulse Energy is then multiplied by the PRF to generate the power.
    The filament heater voltage is generated from the power.

    Power = 1/2 * C * V^2
    C = 90nF
    In Floating Point Math
    power(milli_joule) = .5 * 90e-9 * V^2 * 1000

    power_milli_joule = .5 * 90e-9 * V^2 * 1000
                      = v^2/22222.22
		      = v*v / 2^6 / 347.22
		      = v*v / 2^6 * 47 / 2^14 (.4% fixed point error)
		      
  */
  power_milli_joule = lambda_voltage;
  power_milli_joule *= lambda_voltage;
  power_milli_joule >>= 6;
  power_milli_joule *= 47;
  power_milli_joule >>= 14;

  if (power_milli_joule >= 0xFFFF) {
    power_milli_joule = 0xFFFF;
  }
  power_milli_joule &= 0xFFFF;

  return_data = power_milli_joule;

  return return_data;
}


void UpdateHeaterScale() {
  unsigned long temp32;
  unsigned int temp16;

  // Load the energy per pulse into temp32
  // Use the higher of High/Low Energy set point
  // DPARKER  - WHAT TO DO ON THE 2.5???
  if (etm_can_hv_lambda_mirror.ecb_high_set_point > etm_can_hv_lambda_mirror.ecb_low_set_point) {
    temp32 = CalculatePulseEnergyMilliJoules(etm_can_hv_lambda_mirror.ecb_low_set_point);
  } else {
    temp32 = CalculatePulseEnergyMilliJoules(etm_can_hv_lambda_mirror.ecb_low_set_point);
  }
  
  // Multiply the Energy per Pulse times the PRF (in deci-Hz)
  temp32 *= etm_can_pulse_sync_mirror.status_data.data_word_A; // This is the pulse frequency
  temp32 >>= 6;
  temp32 *= 13;
  temp32 >>= 11;  // Temp32 is now Magnetron Power (in Watts)
  
  global_data_A36507.average_output_power_watts = temp32;
  temp16 = global_data_A36507.average_output_power_watts;

  temp16 >>= 7; // Convert to index for our rolloff table
  if (temp16 >= 0x3F) {
    // Prevent Rollover of the index
    // This is a maximum magnitron power of 8064 Watts
    // If the Magnritron power is greater thatn 8064 it will rolloff as if the power was 8064 watts
    // This would happen at a lambda voltage of 21.2 KV which is well above the maximum voltage of the Lambda
    temp16 = 0x3F;
  }
  
  
  etm_can_heater_magnet_mirror.htrmag_heater_current_set_point_scaled = ETMScaleFactor16(etm_can_heater_magnet_mirror.htrmag_heater_current_set_point,
											 FilamentLookUpTable[temp16],
											 0);

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
  unsigned int loop_counter;

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

  // manually clock out I2C CLK
  // DPARKER make this a generic reset I2C Function
  
  _TRISG2 = 0; // g2 is output
  for (loop_counter = 0; loop_counter <= 100; loop_counter++) {
    _LATG2 = 0;
    __delay32(25);
    _LATG2 = 1;
    __delay32(25);
  }

  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);
  ConfigureDS3231(&U6_DS3231, I2C_PORT, RTC_DEFAULT_CONFIG, FCY_CLK, ETM_I2C_400K_BAUD);

  // Initialize the Can module
  ETMCanMasterInitialize();
  
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

  
  _ADON = 0;

  // Load System powered time from EEPROM
  ETMEEPromReadPage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);

}

#define DEFAULT_UNUSED_EEPROM                     0


#define DEFAULT_MAGNITRON_HEATER_CURRENT       9000
#define DEFAULT_MAGNET_CURRENT_PER_1          16000
#define DEFAULT_MAGNET_CURRENT_PER_2          16000
#define DEFAULT_MAGNET_CURRENT_PER_3          16000
#define DEFAULT_MAGNET_CURRENT_PER_4          16000
#define DEFAULT_AFC_HOME_PER_1                19200
#define DEFAULT_AFC_HOME_PER_2                19200
#define DEFAULT_AFC_HOME_PER_3                19200
#define DEFAULT_AFC_HOME_PER_4                19200


#define DEFAULT_HV_LAMBDA_HIGH_PER_1          15000
#define DEFAULT_HV_LAMBDA_LOW_PER_1           15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_2          15000
#define DEFAULT_HV_LAMBDA_LOW_PER_2           15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_3          15000
#define DEFAULT_HV_LAMBDA_LOW_PER_3           15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_4          15000
#define DEFAULT_HV_LAMBDA_LOW_PER_4           15000


#define DEFAULT_GUN_DRV_HEATER_VOLT            6300
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_1   1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_1    1800
#define DEFAULT_GUN_DRV_CATHODE_PER_1         20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_2   1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_2    1800
#define DEFAULT_GUN_DRV_CATHODE_PER_2         20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_3   1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_3    1800
#define DEFAULT_GUN_DRV_CATHODE_PER_3         20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_4   1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_4    1800
#define DEFAULT_GUN_DRV_CATHODE_PER_4         20000


#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_A      0x06
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_B      0x05
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_C      0x04
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_D      0x03
#define DEFAULT_P_SYNC_HIGH_THYRATRON_DELAY_PER_1        0x02
#define DEFAULT_P_SYNC_HIGH_DOSE_SAMPLE_DELAY_PER_1               0x01


#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_A      0x16
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_B      0x15
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_C      0x14
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_D      0x13
#define DEFAULT_P_SYNC_HIGH_AFC_SAMPLE_DELAY_PER_1       0x12
#define DEFAULT_P_SYNC_HIGH_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1            0x11


#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_A       0x26
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_B       0x25
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_C       0x24
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_D       0x23
#define DEFAULT_P_SYNC_LOW_THYRATRON_DELAY_PER_1         0x22
#define DEFAULT_P_SYNC_LOW_DOSE_SAMPLE_DELAY_PER_1                0x21

#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_A       0x36
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_B       0x35
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_C       0x34
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_D       0x33
#define DEFAULT_P_SYNC_LOW_AFC_SAMPLE_DELAY_PER_1        0x32
#define DEFAULT_P_SYNC_LOW_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1             0x31




const unsigned int eeprom_default_values_htr_mag_afc[16] = {DEFAULT_MAGNITRON_HEATER_CURRENT,
							    DEFAULT_MAGNET_CURRENT_PER_1,
							    DEFAULT_MAGNET_CURRENT_PER_2,
							    DEFAULT_MAGNET_CURRENT_PER_3,
							    DEFAULT_MAGNET_CURRENT_PER_4,
							    DEFAULT_AFC_HOME_PER_1,
							    DEFAULT_AFC_HOME_PER_2,
							    DEFAULT_AFC_HOME_PER_3,
							    DEFAULT_AFC_HOME_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};

const unsigned int eeprom_default_values_hv_lambda[16]   = {DEFAULT_HV_LAMBDA_HIGH_PER_1,
							    DEFAULT_HV_LAMBDA_LOW_PER_1,
							    DEFAULT_HV_LAMBDA_HIGH_PER_2,
							    DEFAULT_HV_LAMBDA_LOW_PER_2,
							    DEFAULT_HV_LAMBDA_HIGH_PER_3,
							    DEFAULT_HV_LAMBDA_LOW_PER_3,
							    DEFAULT_HV_LAMBDA_HIGH_PER_4,
							    DEFAULT_HV_LAMBDA_LOW_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};



const unsigned int eeprom_default_values_gun_driver[16]  = {DEFAULT_GUN_DRV_HEATER_VOLT,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_1,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_1,
							    DEFAULT_GUN_DRV_CATHODE_PER_1,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_2,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_2,
							    DEFAULT_GUN_DRV_CATHODE_PER_2,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_3,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_3,
							    DEFAULT_GUN_DRV_CATHODE_PER_3,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_4,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_4,
							    DEFAULT_GUN_DRV_CATHODE_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};



const unsigned int eeprom_default_values_p_sync_per_1[16]= {((DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_HIGH_DOSE_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_HIGH_THYRATRON_DELAY_PER_1),
							    ((DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_HIGH_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_HIGH_AFC_SAMPLE_DELAY_PER_1),
							    ((DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_LOW_DOSE_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_LOW_THYRATRON_DELAY_PER_1),	
							    ((DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_LOW_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_LOW_AFC_SAMPLE_DELAY_PER_1),
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};



#define EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT                      0x0000
#define EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT                      0x0001
#define EEPROM_REGISTER_AFC_HOME_POSITION                           0x0005
#define EEPROM_REGISTER_AFC_OFFSET                                  0x0009
#define EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE                     0x000A

#define EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT                0x0010
#define EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT                 0x0011

#define EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE                         0x0020
#define EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP                      0x0021
#define EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP                       0x0022
#define EEPROM_REGISTER_GUN_DRV_CATHODE                             0x0023





void ReadSystemConfigurationFromEEProm(unsigned int personality) {
  if (personality >= 5) {
    personality = 1;
  }
  if (personality) {
    personality--;  // Personality is now a register offset
  }
  
  // Load data for HV Lambda
  etm_can_hv_lambda_mirror.ecb_low_set_point = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT + (2*personality)));
  etm_can_hv_lambda_mirror.ecb_high_set_point = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT + (2*personality)));

  // Load data for AFC
  etm_can_afc_mirror.afc_home_position = ETMEEPromReadWord((EEPROM_REGISTER_AFC_HOME_POSITION + personality));
  etm_can_afc_mirror.afc_offset = ETMEEPromReadWord(EEPROM_REGISTER_AFC_OFFSET);
  etm_can_afc_mirror.aft_control_voltage = ETMEEPromReadWord(EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE);
  
  // Load Data for Heater/Magnet Supply
  etm_can_heater_magnet_mirror.htrmag_heater_current_set_point = ETMEEPromReadWord(EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT);
  etm_can_heater_magnet_mirror.htrmag_magnet_current_set_point = ETMEEPromReadWord((EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT + personality));
  
  // Load data for Gun Driver
  etm_can_gun_driver_mirror.gun_heater_voltage_set_point = ETMEEPromReadWord(EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE);
  etm_can_gun_driver_mirror.gun_high_energy_pulse_top_voltage_set_point = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP + (3*personality)));
  etm_can_gun_driver_mirror.gun_low_energy_pulse_top_voltage_set_point = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP + (3*personality)));
  etm_can_gun_driver_mirror.gun_cathode_voltage_set_point = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_CATHODE + (3*personality)));

  // Load data for Pulse Sync
  ETMEEPromReadPage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1 + personality), 12, (unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_3);
}


void ZeroSystemPoweredTime(void) {
  global_data_A36507.system_powered_seconds = 0;
  global_data_A36507.system_hv_on_seconds = 0;
  global_data_A36507.system_xray_on_seconds = 0;
}


void LoadDefaultSystemCalibrationToEEProm(void) {
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC, 16, (unsigned int*)&eeprom_default_values_htr_mag_afc);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA, 16, (unsigned int*)&eeprom_default_values_hv_lambda);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV, 16, (unsigned int*)&eeprom_default_values_gun_driver);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
}



#define REGISTER_HEATER_CURRENT_AT_STANDBY                                                 0x0000
#define REGISTER_ELECTROMAGNET_CURRENT                                                     0x0001
#define REGISTER_HOME_POSITION                                                             0x0005
#define REGISTER_AFC_OFFSET                                                                0x0009
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE                                                   0x000A
#define REGISTER_HIGH_ENERGY_SET_POINT                                                     0x0010
#define REGISTER_LOW_ENERGY_SET_POINT                                                      0x0011
#define REGISTER_GUN_DRIVER_HEATER_VOLTAGE                                                 0x0020
#define REGISTER_GUN_DRIVER_HIGH_ENERGY_PULSE_TOP_VOLTAGE                                  0x0021
#define REGISTER_GUN_DRIVER_LOW_ENERGY_PULSE_TOP_VOLTAGE                                   0x0022
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE                                                0x0023

#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_A_B                               0x0030
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_C_D                               0x0031
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_HIGH_ENERGY               0x0032

#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_A_B                               0x0033
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_C_D                               0x0034
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_HIGH_ENERGY                          0x0035

#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_A_B                                0x0036
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_C_D                                0x0037
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_LOW_ENERGY                0x0038

#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_A_B                                0x0039
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_C_D                                0x003A
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_LOW_ENERGY                           0x003B

#define REGISTER_CMD_AFC_SELECT_AFC_MODE                                                   0x5081
#define REGISTER_CMD_AFC_SELECT_MANUAL_MODE                                                0x5082
#define REGISTER_CMD_AFC_MANUAL_TARGET_POSITION                                            0x5083
#define REGISTER_CMD_AFC_MANUAL_MOVE                                                       0x5084

#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES                            0x6082
#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT          0x6083
#define REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE                            0x6084
#define REGISTER_SPECIAL_ECB_LOAD_DEFAULT_SETTINGS_TO_EEPROM_AND_REBOOT                    0xE080
#define REGISTER_SPECIAL_ECB_RESET_ARC_AND_PULSE_COUNT                                     0xE081
#define REGISTER_SPECIAL_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON                           0xE082

#define REGISTER_SPECIAL_ECB_RESET_SLAVE                                                   0xE083

#define REGISTER_DEBUG_TOGGLE_RESET                                                        0xEF00
#define REGISTER_DEBUG_TOGGLE_HIGH_SPEED_LOGGING                                           0xEF01
#define REGISTER_DEBUG_TOGGLE_HV_ENABLE                                                    0xEF02
#define REGISTER_DEBUG_TOGGLE_XRAY_ENABLE                                                  0xEF03
#define REGISTER_DEBUG_TOGGLE_COOLING_FAULT                                                0xEF04
#define REGISTER_DEBUG_TOGGLE_RESET_DEBUG                                                  0xEF05
#define REGISTER_DEBUG_ENABLE_HIGH_SPEED_LOGGING                                           0xEF06
#define REGISTER_DEBUG_DISABLE_HIGH_SPEED_LOGGING                                          0xEF07

#define REGISTER_SPECIAL_SET_TIME                                                          0xEF08


void ExecuteEthernetCommand(unsigned int personality) {
  ETMEthernetMessageFromGUI next_message;
  unsigned int eeprom_register;

  ETMCanMessage can_message;
  unsigned long temp_long;
  RTC_TIME set_time;


  // DPARKER what happens if this is called before personality has been read??? 
  // Easy to solve in the state machine, just don't call until state when the personality is known

  if (personality >= 5) {
    personality = 1;
  }
  if (personality) {
    personality--;  // Personality is now a register offset
  }
  next_message = GetNextMessage();
  if (next_message.index == 0xFFFF) {
    // there was no message
    return;
  }
  
  if ((next_message.index & 0x0F00) == 0x0100) {
    // this is a calibration set message, route to appropriate board
    // DPARKER only allow when customer has not commanded high voltage on
    SendCalibrationSetPointToSlave(next_message.index, next_message.data_1, next_message.data_0);
  } else if ((next_message.index & 0x0F00) == 0x0900) {
    // this is a calibration requestion message, route to appropriate board
    // When the response is received, the data will be transfered to the GUI
    // DPARKER only allow when customer has not commanded high voltage on
    ReadCalibrationSetPointFromSlave(next_message.index);
  } else {
    // This message needs to be processsed by the ethernet control board
    switch (next_message.index) {
    case REGISTER_HEATER_CURRENT_AT_STANDBY:
      etm_can_heater_magnet_mirror.htrmag_heater_current_set_point = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_ELECTROMAGNET_CURRENT:
      etm_can_heater_magnet_mirror.htrmag_magnet_current_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_HOME_POSITION:
      etm_can_afc_mirror.afc_home_position = next_message.data_2;
      eeprom_register = next_message.index + personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_AFC_OFFSET:
      etm_can_afc_mirror.afc_offset = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;
      
    case REGISTER_AFC_AFT_CONTROL_VOLTAGE:
      etm_can_afc_mirror.aft_control_voltage = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);

    case REGISTER_HIGH_ENERGY_SET_POINT:
      etm_can_hv_lambda_mirror.ecb_high_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      // DPARKER figure out how this is going to work - voltage or current programming
      break;

    case REGISTER_LOW_ENERGY_SET_POINT:
      etm_can_hv_lambda_mirror.ecb_low_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      // DPARKER figure out how this is going to work - voltage or current programming
      break;

    case REGISTER_GUN_DRIVER_HEATER_VOLTAGE:
      etm_can_gun_driver_mirror.gun_heater_voltage_set_point = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_HIGH_ENERGY_PULSE_TOP_VOLTAGE:
      etm_can_gun_driver_mirror.gun_high_energy_pulse_top_voltage_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_LOW_ENERGY_PULSE_TOP_VOLTAGE:
      etm_can_gun_driver_mirror.gun_low_energy_pulse_top_voltage_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE:
      etm_can_gun_driver_mirror.gun_cathode_voltage_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      




    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_A_B:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_3 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_C_D:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_HIGH_ENERGY:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_pfn_delay_high = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_A_B:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_3 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_C_D:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_HIGH_ENERGY:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_high = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_A_B:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_3 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_C_D:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_LOW_ENERGY:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_pfn_delay_low = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_A_B:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_3 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_C_D:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_LOW_ENERGY:
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_low = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

      


    case REGISTER_CMD_AFC_MANUAL_TARGET_POSITION:
      can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 3));
      can_message.word3 = ETM_CAN_REGISTER_AFC_CMD_SELECT_MANUAL_MODE;
      can_message.word2 = 0;
      can_message.word1 = 0;
      can_message.word0 = 0;
      ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &can_message);
      MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  

      can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 3));
      can_message.word3 = ETM_CAN_REGISTER_AFC_CMD_SET_MANUAL_TARGET_POSITION;
      can_message.word2 = 0;
      can_message.word1 = 0;
      can_message.word0 = next_message.data_2;
      ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &can_message);
      MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  

      break;

      // DPARKER ADD IN THE PULSE SYNC SETTINGS
      
    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES:
      can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 3));
      can_message.word3 = ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE;
      can_message.word2 = 0;
      can_message.word1 = 0;
      can_message.word0 = 0;
      ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &can_message);
      MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT:
      can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 3));
      can_message.word3 = ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE;
      can_message.word2 = 0;
      can_message.word1 = 0;
      can_message.word0 = next_message.data_2;
      ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &can_message);
      MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE:
      can_message.identifier = (ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 3));
      can_message.word3 = ETM_CAN_REGISTER_COOLING_CMD_RESET_BOTTLE_COUNT;
      can_message.word2 = 0;
      can_message.word1 = 0;
      can_message.word0 = next_message.data_2;
      ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &can_message);
      MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()  
      break;

    case REGISTER_SPECIAL_SET_TIME:
      temp_long = next_message.data_2;
      temp_long <<= 16;
      temp_long += next_message.data_1;
      RTCSecondsToDate(temp_long, &set_time);
      SetDateAndTime(&U6_DS3231, &set_time);
      break;

    case REGISTER_SPECIAL_ECB_RESET_ARC_AND_PULSE_COUNT:
      break;

    case REGISTER_SPECIAL_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON:
      // Sets all to the value in data_1:data_0
      break;

    case REGISTER_SPECIAL_ECB_LOAD_DEFAULT_SETTINGS_TO_EEPROM_AND_REBOOT:
      // DPARKER only allow when customer has not commanded high voltage on
      SendSlaveLoadDefaultEEpromData(next_message.data_2);
      break;
    
    case REGISTER_SPECIAL_ECB_RESET_SLAVE:
      SendSlaveReset(next_message.data_2);
      break;
      

    /*
    case REGISTER_SPECIAL_SEND_ALL_CAL_DATA_TO_GUI:
      // DPARKER only allow when customer has not commanded high voltage on
      SendSlaveUploadAllCalData(next_message.data_2);
      break;
    */

    case REGISTER_DEBUG_TOGGLE_RESET:
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_SYNC_CONTROL_RESET_ENABLE = 0;
      } else {
	_SYNC_CONTROL_RESET_ENABLE = 1;
      }
      break;

    case REGISTER_DEBUG_ENABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      break;

    case REGISTER_DEBUG_DISABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      break;
      
    case REGISTER_DEBUG_TOGGLE_HIGH_SPEED_LOGGING:
      if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
	_SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      } else {
	_SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_HV_ENABLE:
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
      } else {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_XRAY_ENABLE:
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY) {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
      } else {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_COOLING_FAULT:
      if (_SYNC_CONTROL_COOLING_FAULT) {
	_SYNC_CONTROL_COOLING_FAULT = 0;
      } else {
	_SYNC_CONTROL_COOLING_FAULT = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_RESET_DEBUG:
      if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
      } else {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
      }
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
