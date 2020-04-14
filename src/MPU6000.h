#include "Arduino.h"

#define ToD(x) (x/131)
#define ToG(x) (x*9.80665/16384)
#define xAxis 0
#define yAxis 1
#define zAxis 2
#define Aoffset 0.8

class MPU6000
{
  public:
    MPU6000(int ChipSelPin);
    void Configure();
    float AcceDeg(int AxisSelect);
    int AcceX();
    int AcceY();
    int AcceZ();
  private:
    int _ChipSelPin;
    void SPIwrite(byte reg, byte data);
    uint8_t SPIread(byte reg);
};
