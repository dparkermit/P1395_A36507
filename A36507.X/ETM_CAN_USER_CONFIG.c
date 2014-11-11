#include "ETM_CAN.h"

#ifdef __A36224_500
#include "A36224_500.h"
#endif



void ETMCanSetValueBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  switch (index_word) {
    /*
      Place all board specific set values here
    */

#ifdef __A36224_500
  case ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT:
    global_data_A36224_500.heater_current_set_point = message_ptr->word1;
    global_data_A36224_500.magnet_current_set_point = message_ptr->word1;
    break;

#endif


#ifdef __A_WHATEVER_HV_LAMBDA_IS
  case ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT:
    etm_can_status_register.data_word_A = message_ptr->word2;  // Low energy Program Voltage
    etm_can_status_register.data_word_B = message_ptr->word1;  // High energy Program Voltage
    break;
#endif


  default:
    etm_can_can_status.can_status_invalid_index++;
    break;
  }
}




#ifndef __ETM_CAN_MASTER_MODULE

void ETMCanExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
#ifdef __A36224_500
    case ETM_CAN_REGISTER_HEATER_MAGNET_CMD_OUTPUT_ENABLE:
      etm_can_status_register.status_word_0 &= !STATUS_SOFTWARE_DISABLE;  // Clear the software disable bit 
    break;
    
    case ETM_CAN_REGISTER_HEATER_MAGNET_CMD_OUTPUT_DISABLE:
      etm_can_status_register.status_word_0 |= STATUS_SOFTWARE_DISABLE; // Set the software disable bit
    break;
#endif
    
    
    
#ifdef __A_WHATEVER_HV_LAMBDA_IS
    case ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_ON:
      etm_can_status_register.status_word_1 = 0x0000;
      break;
      
    case ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_OFF:
      etm_can_status_register.status_word_1 = 0xFFFF;
      break;
#endif
      
      
      
      
      
      
      
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


  
#endif  //#ifndef __ETM_CAN_MASTER_MODULE




