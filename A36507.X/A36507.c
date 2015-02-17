#include "A36507.h"
#include "ETM_CAN_PUBLIC.h"
#include "ETM_CAN.h"
#include "ETM_EEPROM.h"
#include "TCPmodbus.h"
#include "FIRMWARE_VERSION.h"

unsigned int CheckReadyForOperation(void) {
  return 1;
}


unsigned int CheckCustomerHVOn(void) {
  return 1;
}

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

  unsigned int millisecond_counter;
  unsigned int warmup_timer_stage;
  unsigned int warmup_done;
  //REAL_TIME_CLOCK time_now;
  
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


void DoWarmupTimers(void);
unsigned int CheckFault(void);
unsigned int CheckHeaterFault(void);


void DoStateMachine(void) {
  
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
      local_debug_data.debug_0 = global_data_A36507.thyratron_warmup_counter_seconds;
      local_debug_data.debug_1 = global_data_A36507.magnetron_heater_warmup_counter_seconds;
      local_debug_data.debug_2 = global_data_A36507.gun_driver_heater_warmup_counter_seconds;


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




void DoA36507(void) {
#ifdef TEST_MODBUS
	ETMEthernetMessageFromGUI command;
    static unsigned int ms250_count = 0;
    static unsigned char msg_index = 0;
    static unsigned cal_index = 0;
    static unsigned pulse_index = 0;
    static unsigned pulse_enabled = 0;
#endif
  ETMCanDoCan();
  TCPmodbus_task();


  _STATUS_0 = _SYNC_CONTROL_RESET_ENABLE;
  _STATUS_1 = _SYNC_CONTROL_HIGH_SPEED_LOGGING;
  _STATUS_2 = _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV;
  _STATUS_3 = _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY;
  _STATUS_4 = _SYNC_CONTROL_COOLING_FAULT;

  if (_T5IF) {
    // 10ms Timer has expired
    _T5IF = 0;
    
    global_data_A36507.millisecond_counter += 10;
    if (global_data_A36507.millisecond_counter >= 250) {
      global_data_A36507.millisecond_counter = 0;
#ifdef TEST_MODBUS
	  ms250_count++;
      if (ms250_count > 8) ms250_count = 0;  // 2s repeat
#endif
      DoWarmupTimers();
    }
#ifdef TEST_MODBUS
	if (pulse_enabled && (global_data_A36507.millisecond_counter % 20) == 0)
    {	
    	event_data[0] = (pulse_index >> 8) & 0xff;
    	event_data[1] = pulse_index & 0xff; 
        event_data[2] = global_data_A36507.millisecond_counter;
    	SendPulseData((unsigned char *)event_data);
        pulse_index++;
    }
	else if (global_data_A36507.millisecond_counter == 22)
    {	
        SendCalibrationData(cal_index, cal_index + 64, cal_index + 16);
        cal_index++;
        if (cal_index > 200) cal_index = 0;
    }
    else if (ms250_count == 5)
    {
    	command = GetNextMessage();	   // put command msg to event log
        if (command.index != 0xffff) {
        	pulse_enabled = 1;	// enable pulsing until Ethernet connected and received a message
	        event_data[0 + msg_index * 8] = (command.index >> 8) & 0xff;
	        event_data[1 + msg_index * 8] = command.index & 0xff;
	        event_data[2 + msg_index * 8] = (command.data_2 >> 8) & 0xff;
	        event_data[3 + msg_index * 8] = command.data_2 & 0xff;
	        event_data[4 + msg_index * 8] = (command.data_1 >> 8) & 0xff;
	        event_data[5 + msg_index * 8] = command.data_1 & 0xff;
	        event_data[6 + msg_index * 8] = (command.data_0 >> 8) & 0xff;
	        event_data[7 + msg_index * 8] = command.data_0 & 0xff;
            msg_index ++;
            if (msg_index > 10) msg_index = 0;
       }
                
    }
#endif    
    
  }
  
}

#define WARMUP_TIMER_STAGE_READ_TIME        0
#define WARMUP_TIMER_STAGE_THYRATRON        1
#define WARMUP_TIMER_STAGE_MAGNETRON        2
#define WARMUP_TIMER_STAGE_GUN_DRIVER       3



void DoWarmupTimers(void) {

  switch (global_data_A36507.warmup_timer_stage) 
    {
    case WARMUP_TIMER_STAGE_READ_TIME:
      //ReadDateAndTime(&global_data_A36507.time_now);
      global_data_A36507.warmup_timer_stage = WARMUP_TIMER_STAGE_THYRATRON;
      break;

    case WARMUP_TIMER_STAGE_THYRATRON:
      global_data_A36507.thyratron_warmup_counter_seconds++;	  
      if (global_data_A36507.thyratron_warmup_counter_seconds >= (THYRATRON_WARMUP_SECONDS + 3)) {
	global_data_A36507.thyratron_warmup_counter_seconds = THYRATRON_WARMUP_SECONDS + 3;
	// DPARKER WRITE CURRENT TIME TO FRAM THYRATRON PAGE 
      }
      global_data_A36507.warmup_timer_stage = WARMUP_TIMER_STAGE_MAGNETRON;
      break;

    case WARMUP_TIMER_STAGE_MAGNETRON:
      // If the magnetron heater is on, increment it's heater counter otherwise set it to zero
      //if ((ETMCanCheckBit(etm_can_heater_magnet_mirror.status_data.status_word_0, STATUS_BIT_SUM_FAULT) == 0) && (ETMCanCheckBit(etm_can_heater_magnet_mirror.status_data.status_word_0, STATUS_BIT_PULSE_INHIBITED) == 0)) {
      if (1) {
	global_data_A36507.magnetron_heater_warmup_counter_seconds++;
	if (global_data_A36507.magnetron_heater_warmup_counter_seconds >= (MAGNETRON_WARMUP_SECONDS + 3)) {
	  global_data_A36507.magnetron_heater_warmup_counter_seconds = MAGNETRON_WARMUP_SECONDS + 3;
	  // DPARKER WRITE CURRENT TIME TO FRAM MAGNETRON PAGE 
	}
      } else {
	if (global_data_A36507.magnetron_heater_warmup_counter_seconds >= 2) {
	  global_data_A36507.magnetron_heater_warmup_counter_seconds -= 2;
	}
      }
      global_data_A36507.warmup_timer_stage = WARMUP_TIMER_STAGE_GUN_DRIVER;
      break;
      
    case WARMUP_TIMER_STAGE_GUN_DRIVER:
      // If the gun driver heater is on, increment it's heater counter otherwise decrement it by 2
      
      //if (ETMCanCheckBit(etm_can_gun_driver_mirror.status_data.status_word_0, STATUS_BIT_USER_DEFINED_8) == 0) {
      if (1) {
	global_data_A36507.gun_driver_heater_warmup_counter_seconds++;
	if (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= (GUN_DRIVER_WARMUP_SECONDS + 3)) {
	  global_data_A36507.gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_WARMUP_SECONDS + 3;
	  // DPARKER WRITE CURRENT TIME TO FRAM GUN DRIVER PAGE 
	}
      } else {
	if (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= 2) {
	  global_data_A36507.gun_driver_heater_warmup_counter_seconds -= 2;
	}
      }
      global_data_A36507.warmup_timer_stage = WARMUP_TIMER_STAGE_READ_TIME;
      break;
    }
  
  if ((global_data_A36507.thyratron_warmup_counter_seconds >= THYRATRON_WARMUP_SECONDS) && (global_data_A36507.magnetron_heater_warmup_counter_seconds >= MAGNETRON_WARMUP_SECONDS) && (global_data_A36507.gun_driver_heater_warmup_counter_seconds >= GUN_DRIVER_WARMUP_SECONDS)) {
    global_data_A36507.warmup_done = 1;
  } else {
    global_data_A36507.warmup_done = 0;
  }
}


void InitializeA36507(void) {
  unsigned int startup_counter;


  //etm_can_status_register.status_word_0 = 0x0000;
  //etm_can_status_register.status_word_1 = 0x0000;
  //etm_can_status_register.data_word_A = 0x0000;
  //etm_can_status_register.data_word_B = 0x0000;
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

  //ConfigureDS3231(&global_data_A36507.time_now, I2C_PORT, RTC_DEFAULT_CONFIG);
  
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
