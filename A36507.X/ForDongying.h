

// STANDARD LOGGING DATA

typedef struct {
  ETMCanStatusRegister*  status_data;                  // This is 12 bytes of data
  ETMCanSystemDebugData* debug_data;                   // This is 48 bytes of data
  ETMCanCanStatus*       can_status;                   // This is 32 bytes of data
  ETMCanAgileConfig*     configuration;                // This is 16 bytes of data

  unsigned int*          custom_data;                  // This can be zero -> N bytes of Data
  unsigned char          custom_data_element_count;

  unsigned char          data_identification;          // This is a unique identifier for each data set
} ETMEthernetTXDataStructure;



/*
Therefore our message in the "data_buffer" would look something like this
data_buffer[0] = data_identification;                   // Transaction ID High Byte
data_buffer[1] = (transaction identification)           // Transaction ID Low Byte
data_buffer[2] = 0;                                     // Protocal ID High Byte
data_buffer[3] = 0;                                     // Protocal ID Low Byte
data_buffer[4] = 0;                                     // Data Length High Byte = 0
data_buffer[5] = n;                                     // Data Length Low BYte =  2 + 108 + custom_data_element_count*2
data_buffer[6] = 0;                                     // Unit ID = 0
data_buffer[7] = 0x10;                                  // Function Code
data_buffer[8]
.
.
.
.
.
data_buffer[115] = Standard Data
data_buffer[116]
data_buffer[116+2*custom_data_element_count] = Board specific data

*/



// PULSE BY PULSE LOGGING DATA
typedef struct {
  // we need to store 36 bytes of data each pulse.
  // If we send out data once every 8 pulses then we will have 288 bytes of data + 8 bytes of header which means 296 bytes total.
  // This will be our largest transmit so the transmit buffer must be at leas this large
  // We can impliment this later but keep in mind that it will need to happen
  
} ETMETHERNETPULSEDATALOG;


unsigned int set_points[size_tbd];
// Element 0 and Element 1 would store the GUI set command authorization
// Element 0 would be the "technicial level" command authorization
// Element 1 would be the "engineer level" command authoriization
// Element 2 through N would store the configuration data 



unsigned int development_test_register[size_tbd];
// This would command

typedef struct {
  unsigned int command_ready;
  unsigned int command_index;
  unsigned int command_data;
} DevelopmentRegister
