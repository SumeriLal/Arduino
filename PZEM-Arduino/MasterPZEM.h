#ifndef MasterPZEM_h
#define MasterPZEM_h
#include "Arduino.h"
#include "util/crc16.h"
#include "util/word.h"
#define __MasterPZEM_DEBUG__ (0)
#define __MasterPZEM_DEBUG_PIN_A__ 4
#define __MasterPZEM_DEBUG_PIN_B__ 5


class MasterPZEM
{
  public:
    MasterPZEM();
    void begin(uint8_t, Stream &serial);
    static const uint8_t ku8MBSuccess                    = 0x00;
    static const uint8_t ku8MBResponseTimedOut           = 0xE2;
    uint16_t getResponseBuffer(uint8_t);
    void     clearResponseBuffer();
    uint8_t  setTransmitBuffer(uint8_t, uint16_t);
    void     clearTransmitBuffer();
    uint8_t available(void);
    uint8_t  readInputRegisters(uint16_t, uint8_t);
    
  private:
    Stream* _serial;                                             ///< reference to serial port object
    uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
    static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers    
    uint16_t _u16ReadAddress;                                    ///< slave register from which to read
    uint16_t _u16ReadQty;                                        ///< quantity of words to read
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
    uint8_t _u8ResponseBufferLength;
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    
    // Modbus timeout [milliseconds]
    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]
    
    // master function that conducts Modbus transactions
    uint8_t MasterPZEMTransaction(uint8_t u8MBFunction);
    
    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
    // preTransmission callback function; gets called before writing a Modbus message
    void (*_preTransmission)();
    // postTransmission callback function; gets called after a Modbus message has been sent
    void (*_postTransmission)();
};
#endif
