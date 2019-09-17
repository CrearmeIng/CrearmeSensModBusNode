#include <Arduino.h>
#include <ModbusSlave.h>
#include <MeanFilterLib.h>

#define SLAVE_ID    0x01
#define CTRL_PIN    2
#define BAUDRATE    115200
#define SENS1_PIN   A6
#define SENS2_PIN   A5

//#define DEBUG

#define SMPL_TM      50   //ms

#define WINDOW_SIZE  10

Modbus slave(SLAVE_ID,CTRL_PIN);
MeanFilter<float> meanFilter1(WINDOW_SIZE);
MeanFilter<float> meanFilter2(WINDOW_SIZE);
float pressure1 = 0;
float pressure2 = 0;
long lstTm = 0;
long currTm = 0;

uint8_t writeDigitlOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint16_t val = slave.readCoilFromBuffer(0);
  digitalWrite(LED_BUILTIN, val);
  #ifdef DEBUG
    delay(50);
    Serial.print(" fc: "); Serial.print(fc);
    Serial.print(" address: "); Serial.print(address);
    Serial.print(" length: "); Serial.print(length);
    Serial.print(" value: "); Serial.println(val);
  #endif
  return STATUS_OK;
}

union unFlt {
  float flt;
  int16_t int16[sizeof(flt)/2];
} ufl;

uint8_t ReadInputReg(uint8_t fc, uint16_t address, uint16_t length) {
  if ((length == 2) && (fc == 0x04)){
    if (address == 0x00DA){                                     // -------------------------- Pressure Sensor No. 1 --------------------------
      ufl.flt = pressure1;                                      // Convert float to dual uint16 registers
      slave.writeRegisterToBuffer(0,ufl.int16[0]);
      slave.writeRegisterToBuffer(1, ufl.int16[1]);
      return STATUS_OK;
    } else if (address == 0x00DC){                              // -------------------------- Pressure Sensor No. 2 --------------------------
      ufl.flt = pressure2; 
      slave.writeRegisterToBuffer(0,ufl.int16[0]);
      slave.writeRegisterToBuffer(1, ufl.int16[1]);
      return STATUS_OK;
    } else {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }
  } else {
    return STATUS_ILLEGAL_FUNCTION;
  }
  
}

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  slave.cbVector[CB_WRITE_COILS] = writeDigitlOut;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = ReadInputReg;
  Serial.begin( BAUDRATE );
  slave.begin( BAUDRATE );
}

void loop() {
  slave.poll();
  currTm = millis();
  uint16_t sample;
  float rawPressure;
  if (abs(currTm - lstTm) >= SMPL_TM){
    // --------------------- Pressure Sensor No. 1  - Sampling ---------------------
    sample = (uint16_t)analogRead(SENS1_PIN);        // Analog sampling
    rawPressure = ((float)sample - 217.0f)/204.6f;   // Sample to kPa conversion : 217.0 = Tare Value | 1023 / 5 = 204.6
    pressure1 = meanFilter1.AddValue(rawPressure);
    // --------------------- Pressure Sensor No. 2  - Sampling ---------------------
    sample = (uint16_t)analogRead(SENS2_PIN);        // Analog sampling
    rawPressure = ((float)sample - 217.0f)/204.6f;   // Sample to kPa conversion : 217.0 = Tare Value | 1023 / 5 = 204.6
    pressure2 = meanFilter2.AddValue(rawPressure);
    lstTm = currTm;
  }
}