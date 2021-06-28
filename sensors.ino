float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
float getVoltage(){
  float volt = (analogRead(VOLTAGE_IN) * 4.86) / 1024.0/ (R2/(R1+R2));
  return volt; 
  //return mapfloat(volt, 0, 25.5, 0, 255);
}
