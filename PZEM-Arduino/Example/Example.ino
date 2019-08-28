/* For the power calculation Pzem 0004t module v3.0, Library link : https://github.com/4-20ma/ModbusMaster  */
#include <MasterPZEM.h>
#include <SoftwareSerial.h> 
 
SoftwareSerial pzemSerial(10,11); //rx, tx
MasterPZEM node;
static uint8_t pzemSlaveAddr = 0x01;
#define LEDPIN 13 
 
void setup() {
  pzemSerial.begin(9600); 
  Serial.begin(9600);
  //resetEnergy(pzemSlaveAddr);
  node.begin(pzemSlaveAddr, pzemSerial);
  pinMode(13, OUTPUT);
  digitalWrite(LEDPIN,0);
}
 
/* 
RegAddr Description                 Resolution
0x0000  Voltage value               1LSB correspond to 0.1V       
0x0001  Current value low 16 bits   1LSB correspond to 0.001A
0x0002  Current value high 16 bits  
0x0003  Power value low 16 bits     1LSB correspond to 0.1W
0x0004  Power value high 16 bits  
0x0005  Energy value low 16 bits    1LSB correspond to 1Wh
0x0006  Energy value high 16 bits 
0x0007  Frequency value             1LSB correspond to 0.1Hz
0x0008  Power factor value          1LSB correspond to 0.01
0x0009  Alarm status  0xFFFF is alarm，0x0000is not alarm
*/
 
void loop() {
  uint8_t result;
  digitalWrite(LEDPIN,1);
  result = node.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
  digitalWrite(LEDPIN,0);
  if (result == node.ku8MBSuccess)
  {
    float voltage = node.getResponseBuffer(0x0000) / 10.0;
 
    uint16_t tempWord;
 
    float power;
    tempWord = 0x0000;
    tempWord |= node.getResponseBuffer(0x0003);       //LowByte
    tempWord |= node.getResponseBuffer(0x0004) << 8;  //highByte
    power = tempWord / 10.0;
 
 
    float current;
    tempWord = 0x0000;
    tempWord |= node.getResponseBuffer(0x0001);       //LowByte
    tempWord |= node.getResponseBuffer(0x0002) << 8;  //highByte
    current = tempWord / 1000.0;
    
    uint16_t energy;
    tempWord = 0x0000;
    tempWord |= node.getResponseBuffer(0x0005);       //LowByte
    tempWord |= node.getResponseBuffer(0x0006) << 8;  //highByte
    energy = tempWord;
 
    Serial.print(voltage);
    Serial.print("V   ");
 
    Serial.print(current);
    Serial.print("A   ");
   
    Serial.print(power);    
    Serial.print("W  ");

    int frequency = node.getResponseBuffer(0x0007);
    Serial.print(frequency/10);
    Serial.print("Hz   ");
    
    
    Serial.print(node.getResponseBuffer(0x0008));
    Serial.print("pf   ");
 
    Serial.print(energy);
    Serial.print("Wh  ");


    int alarmStatus = node.getResponseBuffer(0x0009);
    Serial.print(alarmStatus);
    Serial.print(" Value  ");
    Serial.println();
 return current, voltage, power, frequency;
  } 
  else 
  {
    Serial.println("Failed to read modbus");  
  }
  delay(2000);
}
 
void resetEnergy(uint8_t slaveAddr){
  //The command to reset the slave's energy is (total 4 bytes):
  //Slave address + 0x42 + CRC check high byte + CRC check low byte.
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  pzemSerial.write(slaveAddr);
  pzemSerial.write(resetCommand);
  pzemSerial.write(lowByte(u16CRC));
  pzemSerial.write(highByte(u16CRC));
  delay(100);
}
