#include "ETM_CAN.h"

void ETMCanSetValueBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  switch (index_word) {
    /*
      Place all board specific set values here
    */
  case ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT:
    etm_can_status_register.data_word_A = message_ptr->word2;  // Low energy Program Voltage
    etm_can_status_register.data_word_B = message_ptr->word1;  // High energy Program Voltage
    break;

  default:
    etm_can_can_status.can_status_invalid_index++;
    break;
  }
}



#ifdef __ETM_CAN_MASTER_MODULE
ETMCanRamMirrorHVLambda          etm_can_hv_lamdba_mirror;
ETMCanRamMirrorIonPump           etm_can_ion_pump_mirror;
ETMCanRamMirrorAFC               etm_can_afc_mirror;
ETMCanRamMirrorCooling           etm_can_cooling_mirror;
ETMCanRamMirrorHeaterMagnet      etm_can_heater_magnet_mirror;
ETMCanRamMirrorGunDriver         etm_can_gun_driver_mirror;
ETMCanRamMirrorMagnetronCurrent  etm_can_magnetron_current_mirror;
ETMCanRamMirrorPulseSync         etm_can_pulse_sync_mirror;

ETMCanHighSpeedData etm_can_high_speed_data_test;




unsigned int etm_can_status_received_register;





void ETMCanUpdateStatusBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int source_board;
  source_board = (message_ptr->identifier >> 3);
  source_board &= 0x000F;
  switch (source_board) {
    /*
      Place all board specific status updates here
    */

  case ETM_CAN_ADDR_ION_PUMP_BOARD:
    etm_can_ion_pump_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_ion_pump_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_ion_pump_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_ion_pump_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_ION_PUMP_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD:
    etm_can_magnetron_current_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_magnetron_current_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_magnetron_current_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_magnetron_current_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_MAGNETRON_CURRENT_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_PULSE_SYNC_BOARD:
    etm_can_pulse_sync_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_pulse_sync_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_pulse_sync_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_pulse_sync_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_PULSE_SYNC_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_HV_LAMBDA_BOARD:
    etm_can_hv_lamdba_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_hv_lamdba_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_hv_lamdba_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_hv_lamdba_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_HV_LAMBDA_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_AFC_CONTROL_BOARD:
    etm_can_afc_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_afc_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_afc_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_afc_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_AFC_CONTROL_BOARD;
    ClrWdt();
    break;
    
  case ETM_CAN_ADDR_COOLING_INTERFACE_BOARD:
    etm_can_cooling_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_cooling_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_cooling_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_cooling_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_COOLING_INTERFACE_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_HEATER_MAGNET_BOARD:
    etm_can_heater_magnet_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_heater_magnet_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_heater_magnet_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_heater_magnet_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_HEATER_MAGNET_BOARD;
    ClrWdt();
    break;

  case ETM_CAN_ADDR_GUN_DRIVER_BOARD:
    etm_can_gun_driver_mirror.status_data.status_word_0 = message_ptr->word0;
    etm_can_gun_driver_mirror.status_data.status_word_1 = message_ptr->word1;
    etm_can_gun_driver_mirror.status_data.data_word_A   = message_ptr->word2;
    etm_can_gun_driver_mirror.status_data.data_word_B   = message_ptr->word3;
    etm_can_status_received_register |= ETM_CAN_BIT_GUN_DRIVER_BOARD;
    ClrWdt();
    break;


    if ((etm_can_status_received_register & ETM_CAN_BIT_ALL_SLAVES) == ETM_CAN_BIT_ALL_SLAVES) {
      // A status update has been received from all boards
      TMR3 = 0;
      etm_can_status_received_register = 0x0000;
    } 
    
  default:
    etm_can_can_status.can_status_address_error++;
    break;

  }
}


void ETMCanProcessLogData(void) {
  ETMCanMessage          next_message;
  unsigned int           data_log_index;
  ETMCanSystemDebugData* debug_data_ptr;
  ETMCanAgileConfig*     config_data_ptr;
  ETMCanCanStatus*       can_status_ptr;

  while (ETMCanBufferNotEmpty(&etm_can_rx_data_log_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_rx_data_log_buffer, &next_message);
    data_log_index = next_message.identifier;
    data_log_index >>= 3;
    data_log_index &= 0x00FF;
    
    if ((data_log_index & 0x000F) <= 0x000B) {
      // It is a common data logging buffer, load into the appropraite ram location
      
      switch (data_log_index >> 4) 
	{
	case ETM_CAN_ADDR_ION_PUMP_BOARD:
	  debug_data_ptr  = &etm_can_ion_pump_mirror.debug_data;
	  config_data_ptr = &etm_can_ion_pump_mirror.configuration;
	  can_status_ptr  = &etm_can_ion_pump_mirror.can_status;
	  break;
	  
	case ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD:
	  debug_data_ptr  = &etm_can_magnetron_current_mirror.debug_data;
	  config_data_ptr = &etm_can_magnetron_current_mirror.configuration;
	  can_status_ptr  = &etm_can_magnetron_current_mirror.can_status;
	  break;

	case ETM_CAN_ADDR_PULSE_SYNC_BOARD:
	  debug_data_ptr  = &etm_can_pulse_sync_mirror.debug_data;
	  config_data_ptr = &etm_can_pulse_sync_mirror.configuration;
	  can_status_ptr  = &etm_can_pulse_sync_mirror.can_status;
	  break;
	  
	case ETM_CAN_ADDR_HV_LAMBDA_BOARD:
	  debug_data_ptr  = &etm_can_hv_lamdba_mirror.debug_data;
	  config_data_ptr = &etm_can_hv_lamdba_mirror.configuration;
	  can_status_ptr  = &etm_can_hv_lamdba_mirror.can_status;
	  break;
	  
	case ETM_CAN_ADDR_AFC_CONTROL_BOARD:
	  debug_data_ptr  = &etm_can_afc_mirror.debug_data;
	  config_data_ptr = &etm_can_afc_mirror.configuration;
	  can_status_ptr  = &etm_can_afc_mirror.can_status;
	  break;

	case ETM_CAN_ADDR_COOLING_INTERFACE_BOARD:
	  debug_data_ptr  = &etm_can_cooling_mirror.debug_data;
	  config_data_ptr = &etm_can_cooling_mirror.configuration;
	  can_status_ptr  = &etm_can_cooling_mirror.can_status;
	  break;
	  
	case ETM_CAN_ADDR_HEATER_MAGNET_BOARD:
	  debug_data_ptr  = &etm_can_heater_magnet_mirror.debug_data;
	  config_data_ptr = &etm_can_heater_magnet_mirror.configuration;
	  can_status_ptr  = &etm_can_heater_magnet_mirror.can_status;
	  break;

	case ETM_CAN_ADDR_GUN_DRIVER_BOARD:
	  debug_data_ptr  = &etm_can_gun_driver_mirror.debug_data;
	  config_data_ptr = &etm_can_gun_driver_mirror.configuration;
	  can_status_ptr  = &etm_can_gun_driver_mirror.can_status;
	  break;

	default:
	  //DPARKER add in some error counter
	  break;
	  
	}

      
      switch (data_log_index & 0x000F)
	{
	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_0:
	  debug_data_ptr->i2c_bus_error_count        = next_message.word3;
	  debug_data_ptr->spi_bus_error_count        = next_message.word2;
	  debug_data_ptr->can_bus_error_count        = next_message.word1;
	  debug_data_ptr->scale_error_count          = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_1:
	  debug_data_ptr->reset_count                = next_message.word3;
	  debug_data_ptr->self_test_result_register  = next_message.word2;
	  debug_data_ptr->reserved_0                 = next_message.word1;
	  debug_data_ptr->reserved_1                 = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0:
	  debug_data_ptr->debug_0                    = next_message.word3;
	  debug_data_ptr->debug_1                    = next_message.word2;
	  debug_data_ptr->debug_2                    = next_message.word1;
	  debug_data_ptr->debug_3                    = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1:
	  debug_data_ptr->debug_4                    = next_message.word3;
	  debug_data_ptr->debug_5                    = next_message.word2;
	  debug_data_ptr->debug_6                    = next_message.word1;
	  debug_data_ptr->debug_7                    = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2:
	  debug_data_ptr->debug_8                    = next_message.word3;
	  debug_data_ptr->debug_9                    = next_message.word2;
	  debug_data_ptr->debug_A                    = next_message.word1;
	  debug_data_ptr->debug_B                    = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3:
	  debug_data_ptr->debug_C                    = next_message.word3;
	  debug_data_ptr->debug_D                    = next_message.word2;
	  debug_data_ptr->debug_E                    = next_message.word1;
	  debug_data_ptr->debug_F                    = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_0:
	  config_data_ptr->agile_number_high_word     = next_message.word3;
	  config_data_ptr->agile_number_low_word      = next_message.word2;
	  config_data_ptr->agile_dash                 = next_message.word1;
	  config_data_ptr->agile_rev_ascii            = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_1:
	  config_data_ptr->serial_number              = next_message.word3;
	  config_data_ptr->firmware_branch            = next_message.word2;
	  config_data_ptr->firmware_major_rev         = next_message.word1;
	  config_data_ptr->firmware_minor_rev         = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_0:
	  can_status_ptr->can_status_CXEC_reg        = next_message.word3;
	  can_status_ptr->can_status_error_flag      = next_message.word2;
	  can_status_ptr->can_status_tx_1            = next_message.word1;
	  can_status_ptr->can_status_tx_2            = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_1:
	  can_status_ptr->can_status_rx_0_filt_0     = next_message.word3;
	  can_status_ptr->can_status_rx_0_filt_1     = next_message.word2;
	  can_status_ptr->can_status_rx_1_filt_2     = next_message.word1;
	  can_status_ptr->can_status_isr_entered     = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_2:
	  can_status_ptr->can_status_unknown_message_identifier     = next_message.word3;
	  can_status_ptr->can_status_invalid_index                  = next_message.word2;
	  can_status_ptr->can_status_address_error                  = next_message.word1;
	  can_status_ptr->can_status_tx_0                           = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_3:
	  can_status_ptr->can_status_message_tx_buffer_overflow     = next_message.word3;
	  can_status_ptr->can_status_message_rx_buffer_overflow     = next_message.word2;
	  can_status_ptr->can_status_data_log_rx_buffer_overflow    = next_message.word1;
	  can_status_ptr->can_status_timeout                        = next_message.word0;
	  break;
	  
	} 

    } else {
      // It is board specific logging data
      switch (data_log_index) 
	{
	case ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_PROGRAM_VOLTAGE:
	  etm_can_high_speed_data_test.pulse_count = next_message.word3;
	  etm_can_high_speed_data_test.hvlambda_readback_high_energy_lambda_program_voltage = next_message.word2;
	  etm_can_high_speed_data_test.hvlambda_readback_low_energy_lambda_program_voltage = next_message.word1;
	  etm_can_high_speed_data_test.hvlambda_readback_peak_lambda_voltage = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_SLOW_SET_POINT:
	  etm_can_hv_lamdba_mirror.hvlambda_readback_high_energy_set_point = next_message.word2;
	  etm_can_hv_lamdba_mirror.hvlambda_readback_low_energy_set_point = next_message.word1;
	  etm_can_hv_lamdba_mirror.hvlambda_readback_base_plate_temp = next_message.word0;
	  break;

	  
	case ETM_CAN_DATA_LOG_REGISTER_ION_PUMP_SLOW_MONITORS:
	  etm_can_ion_pump_mirror.ionpump_readback_ion_pump_volage_monitor = next_message.word3;
	  etm_can_ion_pump_mirror.ionpump_readback_ion_pump_current_monitor = next_message.word2;
	  etm_can_ion_pump_mirror.ionpump_readback_filtered_high_energy_target_current = next_message.word1;
	  etm_can_ion_pump_mirror.ionpump_readback_filtered_low_energy_target_current = next_message.word0;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_POSITION:
	  etm_can_high_speed_data_test.pulse_count = next_message.word3;
	  etm_can_high_speed_data_test.afc_readback_current_position = next_message.word2;
	  etm_can_high_speed_data_test.afc_readback_target_position = next_message.word1;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_READINGS:
	  etm_can_high_speed_data_test.pulse_count = next_message.word3;
	  etm_can_high_speed_data_test.afc_readback_a_input = next_message.word2;
	  etm_can_high_speed_data_test.afc_readback_b_input = next_message.word1;
	  etm_can_high_speed_data_test.afc_readback_filtered_error_reading = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_AFC_SLOW_SETTINGS:
	  etm_can_afc_mirror.afc_readback_home_position = next_message.word3;
	  etm_can_afc_mirror.afc_readback_offset = next_message.word2;
	  etm_can_afc_mirror.afc_readback_current_position = next_message.word1;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_PREVIOUS_PULSE:
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_FILTERED_PULSE:
	  etm_can_magnetron_current_mirror.magmon_readback_spare = next_message.word3;
	  etm_can_magnetron_current_mirror.magmon_readback_arcs_this_hv_on = next_message.word2;
	  etm_can_magnetron_current_mirror.magmon_filtered_low_energy_pulse_current = next_message.word1;
	  etm_can_magnetron_current_mirror.magmon_filtered_high_energy_pulse_current = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_ARCS:
	  etm_can_magnetron_current_mirror.magmon_arcs_lifetime = next_message.word3;
	  etm_can_magnetron_current_mirror.magmon_arcs_lifetime <<= 16;
	  etm_can_magnetron_current_mirror.magmon_arcs_lifetime += next_message.word2;
	  etm_can_magnetron_current_mirror.magmon_pulses_this_hv_on = next_message.word1;
	  etm_can_magnetron_current_mirror.magmon_pulses_this_hv_on <<= 16;
	  etm_can_magnetron_current_mirror.magmon_pulses_this_hv_on += next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_PULSE_COUNT:
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime = next_message.word3;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime <<= 16;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime += next_message.word2;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime <<= 16;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime += next_message.word1;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime <<= 16;
	  etm_can_magnetron_current_mirror.magmon_pulses_lifetime += next_message.word0;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_0:
	  etm_can_cooling_mirror.cool_readback_hvps_coolant_flow = next_message.word3;
	  etm_can_cooling_mirror.cool_readback_magnetron_coolant_flow = next_message.word2;
	  etm_can_cooling_mirror.cool_readback_linac_coolant_flow = next_message.word1;
	  etm_can_cooling_mirror.cool_readback_circulator_coolant_flow = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_1:
	  etm_can_cooling_mirror.cool_readback_spare_word_1 = next_message.word3;
	  etm_can_cooling_mirror.cool_readback_spare_word_0 = next_message.word2;
	  etm_can_cooling_mirror.cool_readback_hx_coolant_flow = next_message.word1;
	  etm_can_cooling_mirror.cool_readback_spare_coolant_flow = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_ANALOG_READINGS:
	  etm_can_cooling_mirror.cool_readback_coolant_temperature = next_message.word3;
	  etm_can_cooling_mirror.cool_readback_sf6_pressure = next_message.word2;
	  etm_can_cooling_mirror.cool_readback_cabinet_temperature = next_message.word1;
	  etm_can_cooling_mirror.cool_readback_linac_temperature = next_message.word0;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_READINGS:
	  etm_can_heater_magnet_mirror.htrmag_readback_heater_current = next_message.word3;
	  etm_can_heater_magnet_mirror.htrmag_readback_heater_voltage = next_message.word2;
	  etm_can_heater_magnet_mirror.htrmag_readback_magnet_current = next_message.word1;
	  etm_can_heater_magnet_mirror.htrmag_readback_magnet_voltage = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_SET_POINTS:
	  etm_can_heater_magnet_mirror.htrmag_readback_heater_current_set_point = next_message.word3;
	  etm_can_heater_magnet_mirror.htrmag_readback_heater_voltage_set_point = next_message.word2;
	  etm_can_heater_magnet_mirror.htrmag_readback_magnet_current_set_point = next_message.word1;
	  etm_can_heater_magnet_mirror.htrmag_readback_magnet_voltage_set_point = next_message.word0;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_PULSE_TOP_MON:
	  etm_can_gun_driver_mirror.gun_readback_high_energy_pulse_top_voltage_monitor = next_message.word3;
	  etm_can_gun_driver_mirror.gun_readback_low_energy_pulse_top_voltage_monitor = next_message.word2;
	  etm_can_gun_driver_mirror.gun_readback_cathode_voltage_monitor = next_message.word1;
	  etm_can_gun_driver_mirror.gun_readback_peak_beam_current = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_HEATER_MON:
	  etm_can_gun_driver_mirror.gun_readback_heater_voltage_monitor = next_message.word3;
	  etm_can_gun_driver_mirror.gun_readback_heater_current_monitor = next_message.word2;
	  etm_can_gun_driver_mirror.gun_readback_heater_time_delay_remaining = next_message.word1;
	  etm_can_gun_driver_mirror.gun_readback_driver_temperature = next_message.word0;
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_SET_POINTS:
	  etm_can_gun_driver_mirror.gun_readback_high_energy_pulse_top_set_point = next_message.word3;
	  etm_can_gun_driver_mirror.gun_readback_low_energy_pulse_top_set_point = next_message.word2;
	  etm_can_gun_driver_mirror.gun_readback_heater_voltage_set_point = next_message.word1;
	  etm_can_gun_driver_mirror.gun_readback_cathode_voltage_set_point = next_message.word0;
	  break;


	case ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_TRIGGER_DATA:
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_0:
	  // DPARKER these are special because they need to be broken down into byte
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_1:
	  // DPARKER these are special because they need to be broken down into byte
	  break;

	case ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_2:
	  // DPARKER these are special because they need to be broken down into byte
	  break;

	default:
	  //DPARKER add some error
	  break;
	}
    }
  }
}

#else //#ifdef __ETM_CAN_MASTER_MODULE

void ETMCanExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  switch (index_word) {
    /*
      Place all board specific commands here
    */

    
    
  case ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_ON:
    etm_can_status_register.status_word_1 = 0x0000;
    break;

  case ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_OFF:
    etm_can_status_register.status_word_1 = 0xFFFF;
    break;
      







  default:
    etm_can_can_status.can_status_invalid_index++;
    break;
  }
}


void ETMCanReturnValueBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  index_word &= 0x0FFF;
  switch (index_word) {
    
    /*
      Place all board specific return value commands here
    */
  default:
    etm_can_can_status.can_status_invalid_index++;
    break;
  }
}


void ETMCanResetFaults(void) {
  // Reset faults associated with this board
}


void ETMCanLogCustomPacketC(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanLogData(ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_PROGRAM_VOLTAGE, 0, 18000, 14000, 17950);
}

void ETMCanLogCustomPacketD(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanLogData(ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_SLOW_SET_POINT, 0, 18000, 14000, 17950);
}

void ETMCanLogCustomPacketE(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

  // There is no E packet for HV Lamdba, leave blank

}

void ETMCanLogCustomPacketF(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

  // There is no F packet for HV Lamdba, leave blank

}


  
#endif  //#ifdef __ETM_CAN_MASTER_MODULE


