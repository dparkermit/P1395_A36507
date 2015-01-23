#include "A36507.h"
#include "FIRMWARE_VERSION.h"



unsigned long dan_test_long = 0x12345678;

unsigned int dan_test_high_word;
unsigned int dan_test_low_word;

unsigned int dan_test[10] = {0x0102,0x0304,0x0405,0x0607,0x0809,0x0A0B,0x0C0D,0x0E0F,0x1020,0x3040};

unsigned int dan_test2[10];

unsigned char dan_test_char;

RTC_TIME test_time;

void CalculateHeaterWarmupTimers(void);


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

void InitializeA36507(void);


unsigned int CheckReadyForOperation(void) {
  return 1;
}


unsigned int CheckCustomerHVOn(void) {
  return 1;
}



#define STATE_STARTUP            0x10
#define STATE_WAITING_FOR_INITIALIZATION  0x15
#define STATE_WARMUP             0x20
#define STATE_STANDBY            0x30
#define STATE_DRIVE_UP           0x40
#define STATE_READY              0x50
#define STATE_XRAY_ON            0x60


#define STATE_COLD_FAULT         0x80
#define STATE_WARM_FAULT         0x90


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


unsigned int CheckFault(void);
unsigned int CheckHeaterFault(void);


void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {
    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;

    break;

  case STATE_WAITING_FOR_INITIALIZATION:
    CalculateHeaterWarmupTimers();
    
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      // DPARKER wait for all boards to report that they have been initialized
      global_data_A36507.control_state = STATE_WARMUP;
    }
    break;
    

  case STATE_WARMUP:
    // Calculate all of the warmup counters based on previous warmup completed

    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();

      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_COLD_FAULT;
      }
      
    }
    break;
    
        
  case STATE_STANDBY:
    while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      
      if (CheckCustomerHVOn()) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_WARM_FAULT;
      }
      
    }
    break;


  case STATE_DRIVE_UP:
    // Enable HV ON Command to Pulse Sync Board
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      
      DoA36507();
      
      if (CheckReadyForOperation()) {
	global_data_A36507.control_state = STATE_READY;
      }

      if (!CheckCustomerHVOn()) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_WARM_FAULT;
      }
      
    }
    break;



  case STATE_READY:
    // Enable HV ON Command to Pulse Sync Board
    while (global_data_A36507.control_state == STATE_READY) {
      
      DoA36507();
      
      if (!CheckCustomerHVOn()) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_WARM_FAULT;
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
	global_data_A36507.control_state = STATE_STANDBY;
      }
      
      if (CheckHeaterFault()) {
	global_data_A36507.control_state = STATE_COLD_FAULT;
      }
    }
    break;


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
	global_data_A36507.control_state = STATE_WARMUP;
      }
    }
    
    break;
    
    
    
  default:
    global_data_A36507.control_state = STATE_COLD_FAULT;
    break;
  }
  
}

unsigned int CheckHeaterFault(void) {
  return 0;
}

unsigned int CheckFault(void) {
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
#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
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


  local_debug_data.debug_0 = global_data_A36507.thyratron_warmup_counter_seconds;
  local_debug_data.debug_1 = global_data_A36507.magnetron_heater_warmup_counter_seconds;
  local_debug_data.debug_2 = global_data_A36507.gun_driver_heater_warmup_counter_seconds;
  local_debug_data.debug_3 = global_data_A36507.control_state;
  
  local_debug_data.debug_F = *(unsigned int*)&etm_can_sync_message.sync_0_control_word;


  if (_T5IF) {
    // 10ms Timer has expired
    _T5IF = 0;
    
    // DPARKER Check for cooling fault, and set the sync bit message as appropriate


    // Run at 1 second interval
    global_data_A36507.millisecond_counter += 10;

    if (global_data_A36507.millisecond_counter >= 1000) {
      global_data_A36507.millisecond_counter = 0;
    }


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
      
      if (_HEATER_MAGNET_CONNECTED && _HEATER_MAGNET_ON) {
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
	
      if (_GUN_DRIVER_CONNECTED && _GUN_HEATER_ON) {
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
    }

    if (global_data_A36507.millisecond_counter == 250) {
      // Write Warmup Done Timers to EEPROM
      ETMEEPromWritePage(&U5_FM24C64B, EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
    }
    
    if (global_data_A36507.millisecond_counter == 500) {
      // Write Seconds on Counters to EEPROM
      ETMEEPromWritePage(&U5_FM24C64B, EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);
    }
  }
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
