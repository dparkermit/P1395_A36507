#ifndef __BUFFERN
#define __BUFFERN

typedef struct {
  unsigned char data[16];
  unsigned char write_location;
  unsigned char read_location;
} BUFFERnBYTE;

#define BuffernMask 0b00001111	/* must equal (data size - 1) */



  
void BuffernWriteByte(BUFFERnBYTE* ptr, unsigned char value);
/*
  Writes a byte to the buffer
  If the buffer is full the oldest byte will be overwritten
*/

unsigned char BuffernReadByte(BUFFERnBYTE* ptr);
/*
  Reads a single byte from the buffer.
  If the buffer is empty zero will be returned and the write/read location will not be changed
  Before calling BuffernReadByte the buffer should be checked with BuffernBytesInBuffer or BuffernIsNotEmpty
*/

unsigned char BuffernBytesInBuffer(BUFFERnBYTE* ptr);
/*
  Returns the number of bytes stored in the buffer
*/

unsigned char BuffernIsNotEmpty(BUFFERnBYTE* ptr);
/*
  Returns zero if the buffer is Empty
  Returns one if the buffer is not empty
*/






#endif
