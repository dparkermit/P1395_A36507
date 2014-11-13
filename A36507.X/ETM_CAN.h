#ifndef __ETM_CAN_H
#define __ETM_CAN_H
#include <p30Fxxxx.h>
#include "ETM_CAN_USER_CONFIG.h"
#include "ETM_CAN_PUBLIC.h"
#include "timer.h"  // DPARKER remove the requirement for this


/*
  Can Resources
  SEE : https://docs.google.com/document/d/1KNxxG_HYJT6JKNi3hfjoc4e7S5AWfiagMMd_ywYbgFg/edit
*/



/*
  X = Not Implimented (Don't Care)
  C = Command
  A = Address

  SID REGISTER

  RECEIVE MODE                                  0bXXXCCCCCCAAAAMX0
  TRANSMIT MODE                                 0bCCCCCXXXCAAAAMX0

*/

typedef struct {
  unsigned int identifier;
  unsigned int word0;
  unsigned int word1;
  unsigned int word2;
  unsigned int word3;
} ETMCanMessage;


typedef struct {
  unsigned int message_write_index;
  unsigned int message_read_index;
  unsigned int message_write_count;
  unsigned int message_overwrite_count;
  ETMCanMessage message_data[16];
} ETMCanMessageBuffer;




void ETMCanRXMessage(ETMCanMessage* message_ptr, volatile unsigned int* rx_register_address);
/*
  This stores the data selected by data_ptr (C1RX0CON,C1RX1CON,C2RX0CON,C2RX1CON) into the message
  If there is no data RX Buffer then error information is placed into the message
  This clears the RXFUL bit so that the buffer is available to receive another message
*/


void ETMCanRXMessageBuffer(ETMCanMessageBuffer* buffer_ptr, volatile unsigned int* rx_data_address);
/*
  This stores the data selected by data_ptr (C1RX0CON,C1RX1CON,C2RX0CON,C2RX1CON) into the next available slot in the selected buffer.
  If the message buffer is full the data in the RX buffer is discarded.
  If the RX buffer is empty, nothing is added to the message buffer
  This clears the RXFUL bit so that the buffer is available to receive another message
*/


void ETMCanTXMessage(ETMCanMessage* message_ptr, volatile unsigned int* tx_register_address);
/*
  This moves the message data to the TX register indicated by tx_register_address (C1TX0CON, C1TX1CON, C1TX2CON)
  If the TX register is not empty, the data will be overwritten.
  Also sets the transmit bit to queue transmission
*/


void ETMCanTXMessageBuffer(ETMCanMessageBuffer* buffer_ptr, volatile unsigned int* tx_register_address);
/*
  This moves the oldest message in the buffer to to the TX register indicated by tx_register_address (C1TX0CON, C1TX1CON, C1TX2CON)
  If the TX register is not empty, no data will be transfered and the message buffer state will remain unchanged
  If the message buffer is empty, no data will be transmited and no error will be generated
  Also sets the transmit bit to queue transmission
*/


void ETMCanAddMessageToBuffer(ETMCanMessageBuffer* buffer_ptr, ETMCanMessage* message_ptr);
/*
  This adds a message to the buffer
  If the buffer is full the data is discarded.
*/


void ETMCanReadMessageFromBuffer(ETMCanMessageBuffer* buffer_ptr, ETMCanMessage* message_ptr);
/*
  This moves the oldest message in the buffer to the message_ptr
  If the buffer is empty it returns the error identifier (0b0000111000000000) and fills the data with Zeros.
*/


void ETMCanBufferInitialize(ETMCanMessageBuffer* buffer_ptr);
/*
  This initializes a can message buffer.
*/


unsigned int ETMCanBufferRowsAvailable(ETMCanMessageBuffer* buffer_ptr);
/*
  This returns 0 if the buffer is full, otherwise returns the number of available rows
*/


unsigned int ETMCanBufferNotEmpty(ETMCanMessageBuffer* buffer_ptr);
/*
  Returns 0 if the buffer is Empty, otherwise returns the number messages in the buffer
*/


// ----------- Can Timers T2 & T3 Configuration ----------- //
#ifdef __ETM_CAN_MASTER_MODULE
#define T2_FREQUENCY_HZ          40  // This is 25mS rate
#else
#define T2_FREQUENCY_HZ          10  // This is 100mS rate
#endif

#define T3_FREQUENCY_HZ          4   // This is 250ms rate

// DPARKER remove the need for timers.h here
#define T2CON_VALUE              (T2_OFF & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define PR2_VALUE                (FCY_CLK/256/T2_FREQUENCY_HZ)

#define T3CON_VALUE              (T3_OFF & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_256 & T3_SOURCE_INT)
#define PR3_VALUE                (FCY_CLK/256/T3_FREQUENCY_HZ)





// Define RX SID Masks
// RECEIVE MODE                                  0bXXXCCCCCCAAAAMX0
#define ETM_CAN_MASTER_RX0_MASK                  0b0001100000000001
#define ETM_CAN_MASTER_RX1_MASK                  0b0001100000000101

#define ETM_CAN_SLAVE_RX0_MASK                   0b0001100000000001
#define ETM_CAN_SLAVE_RX1_MASK                   0b0001100001111101

// Define RX SID Filters
#define ETM_CAN_MSG_LVL_FILTER                   0b0000000000000000
#define ETM_CAN_MSG_SYNC_FILTER                  0b0000100000000000
#define ETM_CAN_MSG_SLAVE_FILTER                 0b0001000000000000
#define ETM_CAN_MSG_MASTER_FILTER                0b0001000000000100
#define ETM_CAN_MSG_DATA_LOG_FILTER              0b0001100000000000
#define ETM_CAN_MSG_FILTER_OFF                   0b0001011111111100


// Define RX SID VALUES
#define ETM_CAN_MSG_MASTER_ADDR_MASK             0b0001111110000100
#define ETM_CAN_MSG_SLAVE_ADDR_MASK              0b0001110001111100

#define ETM_CAN_MSG_SET_3_RX                     0b0001000000000000
#define ETM_CAN_MSG_SET_2_RX                     0b0001010000000100
#define ETM_CAN_MSG_CMD_RX                       0b0001010010000000
#define ETM_CAN_MSG_STATUS_RX                    0b0001010100000100
#define ETM_CAN_MSG_SET_1_RX                     0b0001010110000000
#define ETM_CAN_MSG_REQUEST_RX                   0b0001011000000000


// Define TX SID VALUES
#define ETM_CAN_MSG_LVL_TX                       0b0000000000000000
#define ETM_CAN_MSG_SYNC_TX                      0b0100000000000000
#define ETM_CAN_MSG_SET_3_TX                     0b1000000000000000
#define ETM_CAN_MSG_SET_2_TX                     0b1010000000000100
#define ETM_CAN_MSG_CMD_TX                       0b1010000010000000
#define ETM_CAN_MSG_STATUS_TX                    0b1010100000000100
#define ETM_CAN_MSG_SET_1_TX                     0b1010100010000000
#define ETM_CAN_MSG_REQUEST_TX                   0b1011000000000000
#define ETM_CAN_MSG_DATA_LOG_TX                  0b1100000000000000


// Can configuration parameters
#define CXCTRL_CONFIG_MODE_VALUE                 0b0000010000000000      // This sets Fcan to 4xFcy
#define CXCTRL_OPERATE_MODE_VALUE                0b0000000000000000      // This sets Fcan to 4xFcy
#define CXCTRL_LOOPBACK_MODE_VALUE               0b0000001000000000      // This sets Fcan to 4xFcy

#define CXCFG1_10MHZ_FCY_VALUE                   0b0000000000000001      // This sets TQ to 4/Fcan
#define CXCFG1_20MHZ_FCY_VALUE                   0b0000000000000011      // This sets TQ to 8/Fcan
#define CXCFG1_25MHZ_FCY_VALUE                   0b0000000000000100      // This sets TQ to 10/Fcan


#if FCY_CLK == 25000000
#define ETM_CAN_CXCFG1_VALUE                CXCFG1_25MHZ_FCY_VALUE
#elif FCY_CLK == 20000000
#define ETM_CAN_CXCFG1_VALUE                CXCFG1_20MHZ_FCY_VALUE
#elif FCY_CLK == 10000000
#define ETM_CAN_CXCFG1_VALUE                CXCFG1_10MHZ_FCY_VALUE
#else
#define ETM_CAN_CXCFG1_VALUE                COMPILE_ERROR_YOU_SHOULD_FIX_THIS
#endif




#define CXCFG2_VALUE                             0b0000001110010001      // This will created a bit timing of 10x TQ

#define CXTXXCON_VALUE_HIGH_PRIORITY             0b0000000000000011
#define CXTXXCON_VALUE_MEDIUM_PRIORITY           0b0000000000000010
#define CXTXXCON_VALUE_LOW_PRIORITY              0b0000000000000001
#define CXRXXCON_VALUE                           0b0000000000000000

#define CXTXXDLC_VALUE                           0b0000000011000000





typedef struct {
  // Configuration 0
  unsigned int  agile_number_high_word;
  unsigned int  agile_number_low_word;
  unsigned int  agile_dash;
  unsigned int  agile_rev_ascii;

  // Configuarion 1
  unsigned int  serial_number;
  unsigned int  firmware_branch;
  unsigned int  firmware_major_rev;
  unsigned int  firmware_minor_rev;

} ETMCanAgileConfig;




typedef struct {
  // Can data log 0
  unsigned int can_status_CXEC_reg;
  unsigned int can_status_error_flag;
  unsigned int can_status_tx_1;
  unsigned int can_status_tx_2;
  
  // Can data log 1
  unsigned int can_status_rx_0_filt_0;
  unsigned int can_status_rx_0_filt_1;
  unsigned int can_status_rx_1_filt_2;
  unsigned int can_status_isr_entered;

  // Can data log 2
  unsigned int can_status_unknown_message_identifier;
  unsigned int can_status_invalid_index; 
  unsigned int can_status_address_error;
  unsigned int can_status_tx_0;
  
  // Can data log 3
  unsigned int can_status_message_tx_buffer_overflow;
  unsigned int can_status_message_rx_buffer_overflow;
  unsigned int can_status_data_log_rx_buffer_overflow;
  unsigned int can_status_timeout;

} ETMCanCanStatus;



// Can Module Buffers
extern ETMCanMessageBuffer etm_can_rx_message_buffer;
extern ETMCanMessageBuffer etm_can_tx_message_buffer;


// Can Module Debug and Status registers
extern ETMCanAgileConfig     etm_can_my_configuration;
extern ETMCanCanStatus       etm_can_can_status;

// Can Module Functions
void ETMCanSetValueBoardSpecific(ETMCanMessage* message_ptr);


#ifndef __ETM_CAN_MASTER_MODULE
void ETMCanResetFaults(void);
void ETMCanLogData(unsigned int packet_id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0);
void ETMCanExecuteCMDBoardSpecific(ETMCanMessage* message_ptr);
void ETMCanReturnValueBoardSpecific(ETMCanMessage* message_ptr);
#endif

//------------------------------- Specific Board and Command Defines -------------------------- // 

#define ETM_CAN_ADDR_ETHERNET_BOARD                                     14
#define ETM_CAN_ADDR_ION_PUMP_BOARD                                     1
#define ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD                            2
#define ETM_CAN_ADDR_PULSE_SYNC_BOARD                                   3
#define ETM_CAN_ADDR_HV_LAMBDA_BOARD                                    4
#define ETM_CAN_ADDR_AFC_CONTROL_BOARD                                  5
#define ETM_CAN_ADDR_COOLING_INTERFACE_BOARD                            6
#define ETM_CAN_ADDR_HEATER_MAGNET_BOARD                                7
#define ETM_CAN_ADDR_GUN_DRIVER_BOARD                                   8


#define ETM_CAN_BIT_ETHERNET_BOARD                                      0b0100000000000000
#define ETM_CAN_BIT_ION_PUMP_BOARD                                      0b0000000000000010
#define ETM_CAN_BIT_MAGNETRON_CURRENT_BOARD                             0b0000000000000100
#define ETM_CAN_BIT_PULSE_SYNC_BOARD                                    0b0000000000001000
#define ETM_CAN_BIT_HV_LAMBDA_BOARD                                     0b0000000000010000
#define ETM_CAN_BIT_AFC_CONTROL_BOARD                                   0b0000000000100000
#define ETM_CAN_BIT_COOLING_INTERFACE_BOARD                             0b0000000001000000
#define ETM_CAN_BIT_HEATER_MAGNET_BOARD                                 0b0000000010000000
#define ETM_CAN_BIT_GUN_DRIVER_BOARD                                    0b0000000100000000
#define ETM_CAN_BIT_ALL_ACTIVE_BOARDS                                   0b0100000111111110
#define ETM_CAN_BIT_ALL_ACTIVE_SLAVE_BOARDS                             0b0000000111111110


// Default Register Locations
#define ETM_CAN_REGISTER_DEFAULT_CMD_RESET_FAULTS                       0x000
#define ETM_CAN_REGISTER_DEFAULT_CMD_RESET_MCU                          0x001
#define ETM_CAN_REGISTER_DEFAULT_CMD_RESEND_CONFIG                      0x002
#define ETM_CAN_REGISTER_DEFAULT_CMD_WRITE_EEPROM_PAGE                  0x003
#define ETM_CAN_REGISTER_DEFAULT_CMD_DISABLE_HIGH_SPEED_DATA_LOGGING    0x004
#define ETM_CAN_REGISTER_DEFAULT_CMD_ENABLE_HIGH_SPEED_DATA_LOGGING     0x005

// Default Calibration Locations
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN0                       0x400 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN1                       0x402 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN2                       0x404 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN3                       0x406 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN4                       0x408 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN5                       0x40A 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN6                       0x40C 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN7                       0x40E 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN8                       0x410 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN9                       0x412 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN10                      0x414 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN11                      0x416 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN12                      0x418 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN13                      0x41A 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN14                      0x41C 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_AN15                      0x41E 

#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN0                       0x420 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN1                       0x422 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN2                       0x424 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN3                       0x426 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN4                       0x428 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN5                       0x42A 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN6                       0x42C 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN7                       0x42E 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN8                       0x430 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN9                       0x432 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN10                      0x434 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN11                      0x436 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN12                      0x438 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN13                      0x43A 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN14                      0x43C 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_AN15                      0x43E 

#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC0                      0x440 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC1                      0x442 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC2                      0x444 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC3                      0x446 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC4                      0x448
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC5                      0x44A 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC6                      0x44C 
#define ETM_CAN_CALIBRATION_REGISTER_INTERNAL_DAC7                      0x44E 

#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC0                      0x460 
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC1                      0x462
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC2                      0x464
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC3                      0x466
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC4                      0x468
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC5                      0x46A
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC6                      0x46C
#define ETM_CAN_CALIBRATION_REGISTER_EXTERNAL_DAC7                      0x46E

#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_0_1                      0x480
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_2_3                      0x482
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_4_5                      0x484
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_6_7                      0x486
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_8_9                      0x488
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_10_11                    0x48A
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_12_13                    0x48C
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_14_15                    0x48E
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_16_17                    0x490
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_18_19                    0x492
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_20_21                    0x494
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_22_23                    0x496
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_24_25                    0x498
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_26_27                    0x49A
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_28_29                    0x49C
#define ETM_CAN_CALIBRATION_REGISTER_PARAMETER_30_31                    0x49E


// Board Specific Register Locations
#define ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT               0x4100
#define ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_ON                            0x4080
#define ETM_CAN_REGISTER_HV_LAMBDA_CMD_HV_OFF                           0x4081

#define ETM_CAN_REGISTER_AFC_SET_1_HOME_POSITION                        0x5100
#define ETM_CAN_REGISTER_AFC_SET_1_AFC_OFFSET                           0x5101
#define ETM_CAN_REGISTER_AFC_CMD_DO_AUTO_ZERO                           0x5080
#define ETM_CAN_REGISTER_AFC_CMD_ENTER_AFC_MODE                         0x5081
#define ETM_CAN_REGISTER_AFC_CMD_ENTER_MANUAL_MODE                      0x5082
#define ETM_CAN_REGISTER_AFC_CMD_SET_TARGET_POSITION                    0x5083
#define ETM_CAN_REGISTER_AFC_CMD_DO_RELATIVE_MOVE                       0x5084


#define ETM_CAN_REGISTER_COOLING_CMD_OPEN_SF6_SOLENOID_RELAY            0x6080
#define ETM_CAN_REGISTER_COOLING_CMD_CLOSE_SF6_SOLENOID_RELAY           0x6081


#define ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT          0x7100
#define ETM_CAN_REGISTER_HEATER_MAGNET_CMD_OUTPUT_ENABLE                0x7080
#define ETM_CAN_REGISTER_HEATER_MAGNET_CMD_OUTPUT_DISABLE               0x7081


#define ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT            0x8100
#define ETM_CAN_REGISTER_GUN_DRIVER_SET_1_HEATER_CATHODE_SET_POINT      0x8101
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_ENABLE_HEATER                   0x8080
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_DISABLE_HEATER                  0x8081
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_ENABLE_TRIGGER                  0x8082
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_DISABLE_TRIGGER                 0x8083
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_ENABLE_PULSE_TOP                0x8084
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_DISABLE_PULSE_TOP               0x8085
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_ENABLE_HV                       0x8086
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_DISABLE_HV                      0x8087
#define ETM_CAN_REGISTER_GUN_DRIVER_CMD_ZERO_HEATER_TIME_DELAY          0x8088

#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_0      0x3100
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_1      0x3101
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_0       0x3102
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_1       0x3103
#define ETM_CAN_REGISTER_PULSE_SYNC_SET_1_CUSTOMER_LED_OUTPUT           0x3110
#define ETM_CAN_REGISTER_PULSE_SYNC_REQUEST_PERSONALITY_MODULE          0x3120
#define ETM_CAN_REGISTER_PULSE_SYNC_CMD_ENABLE_PULSES                   0x3080
#define ETM_CAN_REGISTER_PULSE_SYNC_CMD_DISABLE_PULSES                  0x3081


#define ETM_CAN_REGISTER_ECB_SET_2_HIGH_ENERGY_TARGET_CURRENT_MON       0xE100
#define ETM_CAN_REGISTER_ECB_SET_2_LOW_ENERGY_TARGET_CURRENT_MON        0xE101
#define ETM_CAN_REGISTER_ECB_SET_2_PERSONAILITY_MODULE                  ETM_CAN_REGISTER_PULSE_SYNC_REQUEST_PERSONALITY_MODULE


//------------------ DATA LOGGING REGISTERS --------------------------//

// Default data logging registers
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_0                       0x0
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_1                       0x1
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0                       0x2
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1                       0x3
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2                       0x4
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3                       0x5
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_0                      0x6
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_1                      0x7
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_0                   0x8
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_1                   0x9
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_2                   0xA
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_3                   0xB


#define ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_PROGRAM_VOLTAGE        0x4C
#define ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_SLOW_SET_POINT              0x4D

#define ETM_CAN_DATA_LOG_REGISTER_ION_PUMP_SLOW_MONITORS                0x1C

#define ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_POSITION                     0x5C
#define ETM_CAN_DATA_LOG_REGISTER_AFC_FAST_READINGS                     0x5D
#define ETM_CAN_DATA_LOG_REGISTER_AFC_SLOW_SETTINGS                     0x5E

#define ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_PREVIOUS_PULSE     0x2C
#define ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_FILTERED_PULSE     0x2D
#define ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_ARCS               0x2E
#define ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_PULSE_COUNT        0x2F

#define ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_0                   0x6C
#define ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_1                   0x6D
#define ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_ANALOG_READINGS          0x6E

#define ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_READINGS           0x7C
#define ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_SET_POINTS         0x7D

#define ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_PULSE_TOP_MON         0x8C
#define ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_HEATER_MON            0x8D
#define ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_SET_POINTS            0x8E

#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_TRIGGER_DATA          0x3C
#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_0         0x3D
#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_1         0x3E
#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_2         0x3F







// This allows the module to be configured to use CAN1 or CAN2
#ifdef __USE_CAN_1

#define _CXIE                       _C1IE
#define _CXIF                       _C1IF
#define _CXIP                       _C1IP
#define CXTX0CON                    C1TX0CON
#define CXTX1CON                    C1TX1CON
#define CXINTF                      C1INTF
#define CXCTRL                      C1CTRL
#define CXCFG1                      C1CFG1
#define CXCFG2                      C1CFG2
#define CXRXM0SID                   C1RXM0SID
#define CXRXM1SID                   C1RXM1SID
#define CXRXF0SID                   C1RXF0SID
#define CXRXF1SID                   C1RXF1SID
#define CXRXF2SID                   C1RXF2SID
#define CXRXF3SID                   C1RXF3SID
#define CXRXF4SID                   C1RXF4SID
#define CXRXF5SID                   C1RXF5SID
#define CXTX2CON                    C1TX2CON
#define CXTX0DLC                    C1TX0DLC
#define CXTX1DLC                    C1TX1DLC
#define CXTX2DLC                    C1TX2DLC
#define CXRX0CON                    C1RX0CON
#define CXRX1CON                    C1RX1CON
#define CXRX1SID                    C1RX1SID
#define CXRX1B1                     C1RX1B1
#define CXEC                        C1EC
#define CXRX1CONbits                C1RX1CONbits
#define CXCTRLbits                  C1CTRLbits
#define CXTX0CONbits                C1TX0CONbits
#define CXTX1CONbits                C1TX1CONbits
#define CXINTFbits                  C1INTFbits
#define CXRX0CONbits                C1RX0CONbits
#define CXINTEbits                  C1INTEbits
#define _CXInterrupt                _C1Interrupt

#else

#define _CXIE                       _C2IE
#define _CXIF                       _C2IF
#define _CXIP                       _C2IP
#define CXTX0CON                    C2TX0CON
#define CXTX1CON                    C2TX1CON
#define CXINTF                      C2INTF
#define CXCTRL                      C2CTRL
#define CXCFG1                      C2CFG1
#define CXCFG2                      C2CFG2
#define CXRXM0SID                   C2RXM0SID
#define CXRXM1SID                   C2RXM1SID
#define CXRXF0SID                   C2RXF0SID
#define CXRXF1SID                   C2RXF1SID
#define CXRXF2SID                   C2RXF2SID
#define CXRXF3SID                   C2RXF3SID
#define CXRXF4SID                   C2RXF4SID
#define CXRXF5SID                   C2RXF5SID
#define CXTX2CON                    C2TX2CON
#define CXTX0DLC                    C2TX0DLC
#define CXTX1DLC                    C2TX1DLC
#define CXTX2DLC                    C2TX2DLC
#define CXRX0CON                    C2RX0CON
#define CXRX1CON                    C2RX1CON
#define CXRX1SID                    C2RX1SID
#define CXRX1B1                     C2RX1B1
#define CXEC                        C2EC
#define CXRX1CONbits                C2RX1CONbits
#define CXCTRLbits                  C2CTRLbits
#define CXTX0CONbits                C2TX0CONbits
#define CXTX1CONbits                C2TX1CONbits
#define CXINTFbits                  C2INTFbits
#define CXRX0CONbits                C2RX0CONbits
#define CXINTEbits                  C2INTEbits
#define _CXInterrupt                _C2Interrupt

#endif



#define MacroETMCanCheckTXBuffer() if (!CXTX0CONbits.TXREQ) { _CXIF = 1; }










// ---------------- MASTER MODULE ONLY ------------------------------

#ifdef __ETM_CAN_MASTER_MODULE

typedef struct {
  unsigned int pulse_count;
  unsigned int high_low_energy;
  
  unsigned int hvlambda_readback_high_energy_lambda_program_voltage;
  unsigned int hvlambda_readback_low_energy_lambda_program_voltage;
  unsigned int hvlambda_readback_peak_lambda_voltage;

  unsigned int afc_readback_current_position;
  unsigned int afc_readback_target_position;
  unsigned int afc_readback_a_input;
  unsigned int afc_readback_b_input;
  unsigned int afc_readback_filtered_error_reading;

  unsigned int ionpump_readback_high_energy_target_current_reading;
  unsigned int ionpump_readback_low_energy_target_current_reading;

  unsigned int magmon_readback_magnetron_high_energy_current;
  unsigned int magmon_readback_magnetron_low_energy_current;
  unsigned int magmon_readback_arc_detected;

  unsigned char psync_readback_customer_trigger_width;
  unsigned char psync_readback_filtered_customer_trigger_width;
  unsigned char psync_readback_high_energy_grid_width;
  unsigned char psync_readback_high_energy_grid_delay;
  unsigned char psync_readback_low_energy_grid_width;
  unsigned char psync_readback_low_energy_grid_delay;
  
} ETMCanHighSpeedData;



typedef struct {
  // ------------------- ETHERNET CONTROL BOARD --------------------- //
  // Standard Registers for all Boards
  ETMCanStatusRegister*  status_data; 
  ETMCanSystemDebugData* debug_data;
  ETMCanCanStatus*       can_status;
  ETMCanAgileConfig*     configuration;

  unsigned int           fault_status_bits;
  unsigned int           pulse_inhibit_status_bits;
  unsigned int           software_pulse_enable;
  unsigned int           pulse_sync_disable_requested;

  unsigned int           status_received_register;

} ETMCanRamMirrorEthernetBoard;


typedef struct {
  // ------------------- HV LAMBDA BOARD --------------------- //
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets on HV Lambda
  unsigned int hvlambda_high_energy_set_point;
  unsigned int hvlambda_low_energy_set_point;

  // "SLOW" Data that the Ethernet control board reads back from HV Lambda
  unsigned int hvlambda_readback_high_energy_set_point;
  unsigned int hvlambda_readback_low_energy_set_point;
  unsigned int hvlambda_readback_base_plate_temp;

} ETMCanRamMirrorHVLambda;


typedef struct {
  // ------------------- ION PUMP BOARD --------------------- //
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  // NONE!!!!
  
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int ionpump_readback_ion_pump_volage_monitor;
  unsigned int ionpump_readback_ion_pump_current_monitor;
  unsigned int ionpump_readback_filtered_high_energy_target_current;
  unsigned int ionpump_readback_filtered_low_energy_target_current;

} ETMCanRamMirrorIonPump;

typedef struct {
  // -------------------- AFC CONTROL BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  unsigned int afc_home_position;
  int          afc_offset;
    
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int afc_readback_home_position;
  unsigned int afc_readback_offset;
  unsigned int afc_readback_current_position;

} ETMCanRamMirrorAFC;




typedef struct {
  // -------------------- COOLING INTERFACE BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  // NONE!!!!
    
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int cool_readback_hvps_coolant_flow;
  unsigned int cool_readback_magnetron_coolant_flow;
  unsigned int cool_readback_linac_coolant_flow;
  unsigned int cool_readback_circulator_coolant_flow;
  unsigned int cool_readback_spare_word_0;
  unsigned int cool_readback_spare_word_1;
  unsigned int cool_readback_hx_coolant_flow;
  unsigned int cool_readback_spare_coolant_flow;
  unsigned int cool_readback_coolant_temperature;
  unsigned int cool_readback_sf6_pressure;
  unsigned int cool_readback_cabinet_temperature;
  unsigned int cool_readback_linac_temperature;

} ETMCanRamMirrorCooling;



typedef struct {
  // -------------------- HEATER/MAGNET INTERFACE BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  unsigned int htrmag_magnet_current_set_point;
  unsigned int htrmag_heater_current_set_point;
      
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int htrmag_readback_heater_current;
  unsigned int htrmag_readback_heater_voltage;
  unsigned int htrmag_readback_magnet_current;
  unsigned int htrmag_readback_magnet_voltage;
  unsigned int htrmag_readback_heater_current_set_point;
  unsigned int htrmag_readback_heater_voltage_set_point;
  unsigned int htrmag_readback_magnet_current_set_point;
  unsigned int htrmag_readback_magnet_voltage_set_point;

} ETMCanRamMirrorHeaterMagnet;



typedef struct {
  // -------------------- GUN DRIVER INTERFACE BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  unsigned int gun_high_energy_pulse_top_voltage_set_point;
  unsigned int gun_low_energy_pulse_top_voltage_set_point;
  unsigned int gun_heater_voltage_set_point;
  unsigned int gun_cathode_voltage_set_point;
  
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int gun_readback_high_energy_pulse_top_voltage_monitor;
  unsigned int gun_readback_low_energy_pulse_top_voltage_monitor;
  unsigned int gun_readback_cathode_voltage_monitor;
  unsigned int gun_readback_peak_beam_current;
  unsigned int gun_readback_heater_voltage_monitor;
  unsigned int gun_readback_heater_current_monitor;
  unsigned int gun_readback_heater_time_delay_remaining;
  unsigned int gun_readback_driver_temperature;
  unsigned int gun_readback_high_energy_pulse_top_set_point;
  unsigned int gun_readback_low_energy_pulse_top_set_point;
  unsigned int gun_readback_heater_voltage_set_point;
  unsigned int gun_readback_cathode_voltage_set_point;

} ETMCanRamMirrorGunDriver;



typedef struct {
  // -------------------- MAGNETRON CURRENT MONITOR BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  // NONE!!!!
  
  // "SLOW" Data that the Ethernet control board reads back
  unsigned int magmon_readback_spare;
  unsigned int magmon_readback_arcs_this_hv_on;
  unsigned int magmon_filtered_high_energy_pulse_current;
  unsigned int magmon_filtered_low_energy_pulse_current;
  unsigned long magmon_arcs_lifetime;
  unsigned long magmon_pulses_this_hv_on;
  unsigned long long magmon_pulses_lifetime;

} ETMCanRamMirrorMagnetronCurrent;


typedef struct {
  // -------------------- PULSE SYNC BOARD ---------------//
  // Standard Registers for all Boards
  ETMCanStatusRegister  status_data; 
  ETMCanSystemDebugData debug_data;
  ETMCanCanStatus       can_status;
  ETMCanAgileConfig     configuration;
  
  // Values that the Ethernet control board sets 
  unsigned char psync_grid_delay_high_intensity_3;
  unsigned char psync_grid_delay_high_intensity_2;
  unsigned char psync_grid_delay_high_intensity_1;
  unsigned char psync_grid_delay_high_intensity_0;
  unsigned char psync_pfn_delay_high;
  unsigned char psync_rf_delay_high;

  unsigned char psync_grid_width_high_intensity_3;
  unsigned char psync_grid_width_high_intensity_2;
  unsigned char psync_grid_width_high_intensity_1;
  unsigned char psync_grid_width_high_intensity_0;
  unsigned char psync_afc_delay_high;
  unsigned char psync_spare_delay_high;

  unsigned char psync_grid_delay_low_intensity_3;
  unsigned char psync_grid_delay_low_intensity_2;
  unsigned char psync_grid_delay_low_intensity_1;
  unsigned char psync_grid_delay_low_intensity_0;
  unsigned char psync_pfn_delay_low;
  unsigned char psync_rf_delay_low;
 
  unsigned char psync_grid_width_low_intensity_3;
  unsigned char psync_grid_width_low_intensity_2;
  unsigned char psync_grid_width_low_intensity_1;
  unsigned char psync_grid_width_low_intensity_0;
  unsigned char psync_afc_delay_low;
  unsigned char psync_spare_delay_low;

  unsigned int  psync_customer_led;

  // "SLOW" Data that the Ethernet control board reads back
  // NONE!!!!!!
} ETMCanRamMirrorPulseSync;


// PUBLIC Variables
extern ETMCanRamMirrorHVLambda          etm_can_hv_lamdba_mirror;
extern ETMCanRamMirrorIonPump           etm_can_ion_pump_mirror;
extern ETMCanRamMirrorAFC               etm_can_afc_mirror;
extern ETMCanRamMirrorCooling           etm_can_cooling_mirror;
extern ETMCanRamMirrorHeaterMagnet      etm_can_heater_magnet_mirror;
extern ETMCanRamMirrorGunDriver         etm_can_gun_driver_mirror;
extern ETMCanRamMirrorMagnetronCurrent  etm_can_magnetron_current_mirror;
extern ETMCanRamMirrorPulseSync         etm_can_pulse_sync_mirror;
extern ETMCanHighSpeedData              etm_can_high_speed_data_test;
extern ETMCanRamMirrorEthernetBoard     etm_can_ethernet_board_data;



// Can Module Buffers
extern ETMCanMessageBuffer etm_can_rx_data_log_buffer;

// Can Module Funtions
void ETMCanUpdateStatusBoardSpecific(ETMCanMessage* message_ptr);
void ETMCanSendSync(unsigned int sync_3, unsigned int sync_2, unsigned int sync_1, unsigned int sync_0);
void ETMCanMasterPulseSyncDisable(void);
void ETMCanMasterHVLambdaUpdateOutput(void);
void ETMCanMasterGunDriverUpdatePulseTop(void);
unsigned int ETMCanMasterReadyToPulse(void);



#endif  // #ifdef __ETM_CAN_MASTER_MODULE









#endif
