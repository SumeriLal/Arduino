#include "MasterPZEM.h"
MasterPZEM::MasterPZEM(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

void MasterPZEM::begin(uint8_t slave, Stream &serial)
{
  _u8MBSlave = slave;
  _serial = &serial;
}

uint16_t MasterPZEM::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u16ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}
void MasterPZEM::clearResponseBuffer()
{
  uint8_t x;
  for (x = 0; x < ku8MaxBufferSize; x++)
  {
    _u16ResponseBuffer[x] = 0;
  }
}

uint8_t MasterPZEM::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
{
  if (u8Index < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[u8Index] = u16Value;
    return ku8MBSuccess;
  }
}

void MasterPZEM::clearTransmitBuffer()
{
  uint8_t x;

  for (x = 0; x < ku8MaxBufferSize; x++)
  {
    _u16TransmitBuffer[x] = 0;
  }
}
uint8_t MasterPZEM::readInputRegisters(uint16_t u16ReadAddress, uint8_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  return MasterPZEMTransaction(ku8MBReadInputRegisters);
}


/* ---------------------------------------------------PRIVATE FUNCTIONS---------------------------------------------------- */

uint8_t MasterPZEM::MasterPZEMTransaction(uint8_t u8MBFunction)
{
  uint8_t pzemADU[256];
  uint8_t pzemADUSize = 0;
  uint8_t x ;
  uint16_t u16CRC;
  uint32_t u32StartTime;
  uint8_t u8BytesLeft = 8;
  uint8_t u8MBStatus = ku8MBSuccess;

  // assemble pzemMaster data Request Application Data Unit_serial
  pzemADU[pzemADUSize++] = _u8MBSlave;
  pzemADU[pzemADUSize++] = u8MBFunction;

  switch(u8MBFunction)
  {
    case ku8MBReadInputRegisters:
      pzemADU[pzemADUSize++] = highByte(_u16ReadAddress);
      pzemADU[pzemADUSize++] = lowByte(_u16ReadAddress);
      pzemADU[pzemADUSize++] = highByte(_u16ReadQty);
      pzemADU[pzemADUSize++] = lowByte(_u16ReadQty);
      break;
  }

  // append CRC or calculation
  u16CRC = 0xFFFF;
  for (x = 0; x < pzemADUSize; x++)
  {
    u16CRC = crc16_update(u16CRC, pzemADU[x]);
  }
  pzemADU[pzemADUSize++] = lowByte(u16CRC);
  pzemADU[pzemADUSize++] = highByte(u16CRC);
  pzemADU[pzemADUSize] = 0;
  while (_serial->read() != -1);

  // transmit request from the modbus

  if (_preTransmission)
  {
    _preTransmission();
  }
  for (x = 0; x < pzemADUSize; x++)
  {
    _serial->write(pzemADU[x]);
  }

  pzemADUSize = 0;
  _serial->flush();
  if (_postTransmission)
  {
    _postTransmission();
  }

  // loop until we run out of time or bytes, or an error occurs
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
#if __MasterPZEM_DEBUG__
      digitalWrite(__MasterPZEM_DEBUG_PIN_A__, true);
#endif
      pzemADU[pzemADUSize++] = _serial->read();
      u8BytesLeft--;
#if __MasterPZEM_DEBUG__
      digitalWrite(__MasterPZEM_DEBUG_PIN_A__, false);
#endif
    }
    else
    {
#if __MasterPZEM_DEBUG__
      digitalWrite(__MasterPZEM_DEBUG_PIN_B__, true);
#endif
      if (_idle)
      {
        _idle();
      }
#if __MasterPZEM_DEBUG__
      digitalWrite(__MasterPZEM_DEBUG_PIN_B__, false);
#endif
    }

    // evaluate slave ID, function code once enough bytes have been read
    if (pzemADUSize == 5)
    {
      if (bitRead(pzemADU[1], 7))
      {
        u8MBStatus = pzemADU[2];
        break;
      }

      // evaluate returned pzem modbus function code
      switch(pzemADU[1])
      {
        case ku8MBReadInputRegisters:
          u8BytesLeft = pzemADU[2];
          break;
      }
    }
    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  };


  // verify response is large enough to inspect further
  if (!u8MBStatus && pzemADUSize >= 5)
  {
    // calculate CRC
    u16CRC = 0xFFFF;
    for (x = 0; x < (pzemADUSize - 2); x++)
    {
      u16CRC = crc16_update(u16CRC, pzemADU[x]);
    }
  }
  if (!u8MBStatus)
  {
    switch(pzemADU[1])
    {
      case ku8MBReadInputRegisters:
        for (x = 0; x < (pzemADU[2] >> 1); x++)
        {
          if (x < ku8MaxBufferSize)
          {
            _u16ResponseBuffer[x] = word(pzemADU[2 * x + 3], pzemADU[2 * x + 4]);
          }
          _u8ResponseBufferLength = x;
        }
        break;
    }
  }
  return u8MBStatus;
}