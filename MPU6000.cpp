#include "MPU6000.h"
#include "Arduino.h"
#include <SPI.h>
#include <math.h>

int _ChipSelPin;

MPU6000::MPU6000(int ChipSelPin)
{
  _ChipSelPin = ChipSelPin;
  pinMode(ChipSelPin, OUTPUT);
}

void MPU6000::SPIwrite(byte reg, byte data) {
  uint8_t dump;
  digitalWrite(_ChipSelPin,LOW);
  dump=SPI.transfer(reg);
  dump=SPI.transfer(data);
  digitalWrite(_ChipSelPin,HIGH);
}


uint8_t MPU6000::SPIread(byte reg) {
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr=reg|0x80;
  digitalWrite(_ChipSelPin,LOW);
  dump=SPI.transfer(addr);
  return_value=SPI.transfer(0x00);
  digitalWrite(_ChipSelPin,HIGH);
  return(return_value);
}

int MPU6000::AcceX() {
  uint8_t AcceX_H=SPIread(0x3B);
  uint8_t AcceX_L=SPIread(0x3C);
  int16_t AcceX=AcceX_H<<8|AcceX_L;
  return(AcceX);
}

int MPU6000::AcceY() {
  uint8_t AcceY_H=SPIread(0x3D);
  uint8_t AcceY_L=SPIread(0x3E);
  int16_t AcceY=AcceY_H<<8|AcceY_L;
  return(AcceY);
}

int MPU6000::AcceZ() {
  uint8_t AcceZ_H=SPIread(0x3F);
  uint8_t AcceZ_L=SPIread(0x40);
  int16_t AcceZ=AcceZ_H<<8|AcceZ_L;
  return(AcceZ);
}

float MPU6000::AcceDeg(int AxisSelect) {
  float Ax=ToG(AcceX());
  float Ay=ToG(AcceY());
  float Az=ToG(AcceZ());
  float ADegX=((atan(Ax/(sqrt((Ay*Ay)+(Az*Az)))))/PI)*180;
  float ADegY=((atan(Ay/(sqrt((Ax*Ax)+(Az*Az)))))/PI)*180;
  float ADegZ=((atan((sqrt((Ax*Ax)+(Ay*Ay)))/Az))/PI)*180;
  switch (AxisSelect)
  {
    case 0:
    return ADegX;
    break;
    case 1:
    return ADegY;
    break;
    case 2:
    return ADegZ;
    break;
  }
}

void MPU6000::Configure()
{
  SPI.begin();  
  SPI.setClockDivider(SPI_CLOCK_DIV16); 


  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0);
  // DEVICE_RESET @ PWR_MGMT_1, reset device
  SPIwrite(0x6B,0x80);
  delay(150);

  // TEMP_DIS @ PWR_MGMT_1, wake device and select GyroZ clock
  SPIwrite(0x6B,0x03);
  delay(150);

  // I2C_IF_DIS @ USER_CTRL, disable I2C interface
  SPIwrite(0x6A,0x10);
  delay(150);

  // SMPRT_DIV @ SMPRT_DIV, sample rate at 1000Hz
  SPIwrite(0x19,0x00);
  delay(150);

  // DLPF_CFG @ CONFIG, digital low pass filter at 42Hz
  SPIwrite(0x1A,0x03);
  delay(150);

  // FS_SEL @ GYRO_CONFIG, gyro scale at 250dps
  SPIwrite(0x1B,0x00);
  delay(150);

  // AFS_SEL @ ACCEL_CONFIG, accel scale at 2g (1g=8192)
  SPIwrite(0x1C,0x00);
  delay(150);
}
