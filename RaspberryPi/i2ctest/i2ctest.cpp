// compile: g++ i2ctest.cpp -o i2ctest -lwiringPi -lstdc++

#include <iostream>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <fcntl.h>				//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

using namespace std;

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned long  uint32;
typedef char  sint8;
typedef short sint16;
typedef long  sint32;

const uint8 SER_START = 0xA0;
const uint8 SER_END = 0xA1;

const uint8 LOC_START = 0;
const uint8 LOC_PI_STATUS = 1;
const uint8 LOC_LED_STATUS = 2;
const uint8 LOC_DATA_START = 3;
const uint8 LOC_END = 1;
const uint8 LOC_CHECK_BYTE = 2;
const uint8 LOC_DATA_END = 3;

const uint8  MAX_TEAM_COLOR = 34;
const uint8  MAX_SEND = 22;
const uint8  MAX_RECEIVE = 30;
const int  MAX_PACKET = 2000;

void receiveEvent(int howMany);
bool DoWeHaveAGoodMessage();
uint8 CalcCheckByte(uint8* data, uint8 start, uint8 number);
uint8 FindNextMessage();
void RemoveDataForNextMessage(uint8 offset, bool isBad);
void sendMessage();
void putU16IntoU8Array(uint8* data, uint8 location, int value);
int GetU16FrombyteArray(uint8* data, int location);

const int waitTime = 1000;  //us

int mFileI2c;
uint8 mBuffer[MAX_PACKET];
uint8 mBufferLoc = 0;
uint8 mSend[MAX_SEND];
bool mGoodPacket = false;
uint16 mCommCount = 0;

void iniI2C(int addr)
{
  //----- OPEN THE I2C BUS -----
  char *filename = (char*)"/dev/i2c-1";
  if ((mFileI2c = open(filename, O_RDWR)) < 0)
  {
    //ERROR HANDLING: you can check errno to see what went wrong
    printf("Failed to open the i2c bus");
    return;
  }

  if (ioctl(mFileI2c, I2C_SLAVE, addr) < 0)
  {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    //ERROR HANDLING; you can check errno to see what went wrong
    return;
  }
}


int main(void)
{
  int comCount = 0;
  int otherCommCount = 0;
  int oldOtherCommCount = 0;
  iniI2C(43);


  while (true)
  {
    sendMessage();
    usleep(waitTime);
    receiveEvent(1);

    if (true == mGoodPacket)
    {
      comCount++;
      putU16IntoU8Array(mSend, LOC_DATA_START, comCount);
      otherCommCount = GetU16FrombyteArray(mBuffer, LOC_DATA_START);
      if ((0 != otherCommCount)&&(0xFFFF != otherCommCount))
      {
        printf("Good packet:%d\n", otherCommCount);
      }
      oldOtherCommCount = otherCommCount;
    }
  }
}

///--------------------------------------------------------------------
/// Purpose:
/// <summary>
///      Pull a short out of a data buffer
/// </summary>
/// 
/// Returns:
/// <returns>
///     The UInt16 from the data
/// </returns>
/// 
/// Notes:
/// <remarks>
///     None.
/// </remarks>
///--------------------------------------------------------------------
int GetU16FrombyteArray(uint8* data, int location)
{
  return (int)((data[location+1] << 8) +
    data[location]);
}



//----------------------------------------------------------------------------
//  Purpose:
//      Chop a int up and put it into a buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void putU16IntoU8Array(uint8* data, uint8 location, int value)
{
  data[location + 1] = (char)((value >> 8) & 0xFF);
  data[location + 0] = (char)(value & 0xFF);
}


int mainx(void)
{
    int file_i2c;
    int length;
    unsigned char buffer[60] = { 0 };

    //----- OPEN THE I2C BUS -----
    char *filename = (char*)"/dev/i2c-1";
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("Failed to open the i2c bus");
        return 0;
    }

    int addr = 0x40;          //<<<<<The I2C address of the slave
    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        return 0;
    }

    uint8 data[1];

    data[0] = 'B';

    //----- WRITE BYTES -----
    length = 1;			//<<< Number of bytes to write
    if (write(file_i2c, data, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
    }

    usleep(waitTime);

    //----- READ BYTES -----
    length = 1;			//<<< Number of bytes to read
    if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("Failed to read from the i2c bus.\n");
    }
    else
    {
        printf("Data read: %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    }

    return 0;
}

void writeByte(uint8 ch)
{
  uint8 data[1];

  data[0] = ch;

  //----- WRITE BYTES -----
  uint8 length = 1;			//<<< Number of bytes to write
  if (write(mFileI2c, data, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
  {
  }
}

uint8 mReadBuffer[60];
int ReadBytes()
{
  //----- READ BYTES -----
  uint8 length = 60;			//<<< Number of bytes to read
  length = read(mFileI2c, mReadBuffer, length);
  return length;
}


void sendMessage()
{
  mSend[LOC_START] = SER_START;
  mSend[MAX_SEND - LOC_CHECK_BYTE] = CalcCheckByte(mSend, LOC_PI_STATUS, MAX_SEND - LOC_DATA_END);
  mSend[MAX_SEND - LOC_END] = SER_END;

  writeByte(mSend[0]);

  for (uint8 index = 1; index < MAX_SEND - 1; index++)
  {
    writeByte(mSend[index]);
  }
  writeByte(mSend[MAX_SEND - 1]);
}

//----------------------------------------------------------------------------
//  Purpose:
//      Get bytes from the I2C
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void receiveEvent(int howMany)
{
  int theByte = 0;
  uint8 count = 0;
  int readCount = ReadBytes();
  int curReadCount = 0;

  while((curReadCount<readCount)&&(mBufferLoc < MAX_PACKET))
  {
    theByte = mReadBuffer[curReadCount++];
    // Add the byte to the buffer
    mBuffer[mBufferLoc] = theByte;
    mBufferLoc++;
  }

  // if we over read the clear everything
  if (mBufferLoc >= MAX_PACKET)
  {
    mBufferLoc = 0;
  }

  count = 0;
  // Trim the garbage from the start
  while ((SER_START != mBuffer[0]) && (count < mBufferLoc))
  {
    count++;
  }

  if (count > 0)
  {
    RemoveDataForNextMessage(count, true);
  }

  // find if we are good or have garbage
  int nextMessage = FindNextMessage();

  if (true == DoWeHaveAGoodMessage())
  {
    mGoodPacket = true;
  }
  else
  {
    // Trim garbage if there is any
    if (nextMessage > 0)
    {
      RemoveDataForNextMessage(nextMessage, true);
    }
  }
}

//----------------------------------------------------------------------------
//  Purpose:
//      Return if the packet is well formed
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
bool DoWeHaveAGoodMessage()
{
  bool returnValue = false;
  uint8 theLength = MAX_RECEIVE;

  if ((mBufferLoc >= theLength) && (mBufferLoc != 0) && (theLength != 0))
  {
    //Is the preamble where it should be
    if ((mBuffer[LOC_START] == SER_START) && (mBuffer[theLength - LOC_END] == SER_END))
    {
      uint8 checkByte = CalcCheckByte(mBuffer, LOC_PI_STATUS, theLength - LOC_DATA_END);

      if (checkByte == mBuffer[theLength - LOC_CHECK_BYTE])
      {
        returnValue = true;
      }
      else
      {
        RemoveDataForNextMessage(theLength, true);
      }
    }
    else
    {
      RemoveDataForNextMessage(theLength, true);
    }
  }
  return returnValue;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Calc a check byte from the data
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8 CalcCheckByte(uint8* data, uint8 start, uint8 number)
{
  uint8 checkByte = 0xFF;

  for (uint8 index = 0; index < number; index++)
  {
    checkByte ^= data[start + index];
  }
  return checkByte;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Find the next message after the first one
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
uint8 FindNextMessage()
{
  uint8 nextMessIndex = 0;
  bool found = false;
  uint8 theLength = MAX_RECEIVE;

  //Is the preamble where it should be
  if ((mBuffer[LOC_START] == SER_START) && (mBufferLoc>2))
  {
    theLength = MAX_RECEIVE;
    //From the end of the message search the rest of what we have gotten
    //for another preamble.
    for (nextMessIndex = 1; nextMessIndex < mBufferLoc; nextMessIndex++)
    {
      if (mBuffer[nextMessIndex] == SER_START)
      {
        //If we found one stop
        found = true;
        break;
      }
    }
  }

  //If we found one the nextMessIndex should be good.
  //If not then set it to 0 and return 0.
  if (found == false)
  {
    nextMessIndex = 0;
  }
  return nextMessIndex;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Trim the front of the buffer
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void RemoveDataForNextMessage(uint8 offset, bool isBad)
{
  int index;

  //Move the first 'offset' number of bytes forward.
  for (index = 0; index < mBufferLoc - offset; index++)  // JSF162 JSF213 Exception
  {
    mBuffer[index] = mBuffer[offset + index];
  }

  //if we have been asked to remove more bytes than we have set the number
  //of bytes to 0.
  if (offset > mBufferLoc)
  {
    mBufferLoc = 0;
  }
  else
  {
    //If not then reduce the number of bytes we have by the offset.
    mBufferLoc -= offset;
  }

  //Move the rest of the message down to right after the 'offset' bytes.
  // Process the rest of the buffer
  for (; index < MAX_PACKET; index++)   // JSF200 JSF162 Exception
  {
    //If we are under the MAX packet size then move the data.
    if ((offset + index) < (mBufferLoc))
    {
      mBuffer[index] = mBuffer[offset + index];
    }
    else
    {
      //If we are over the MAX packet size then clear out the bytes.
      mBuffer[index] = 0;
    }
  }
}



