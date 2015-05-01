#include "Buffern.h"

void BuffernWriteByte(BUFFERnBYTE* ptr, unsigned char value) {
  ptr->data[ptr->write_location] = value;
  ptr->write_location += 1;
  ptr->write_location &= BuffernMask;
  if (ptr->write_location == ptr->read_location) {
    ptr->read_location += 1;
    ptr->read_location &= BuffernMask;
  }
}

unsigned char BuffernReadByte(BUFFERnBYTE* ptr) {
  unsigned char local_read_location;
  unsigned char return_data;
						
  local_read_location = ptr->read_location;
  if (local_read_location != ptr->write_location) {
    // the buffer is not empty
    return_data = ptr->data[local_read_location];
    local_read_location += 1;
    local_read_location &= BuffernMask; 
    ptr->read_location = local_read_location;
  } else {
    // the buffer was empty
    // return zero and do not increment the read_location
    return_data = 0;
  }
  return return_data;
}

unsigned char BuffernBytesInBuffer(BUFFERnBYTE* ptr) {
  return ((ptr->write_location - ptr->read_location) & BuffernMask);
}

unsigned char BuffernIsNotEmpty(BUFFERnBYTE* ptr) {
  if (ptr->write_location == ptr->read_location) {
    return 0;
  } else {
    return 1;
  }
}



