void armed(){
  #ifndef NEEDGPSTOARM
    gpsfix = true;
  #endif
  if(arming == 2 && gpsfix){
    arm = true;
  }else{
    arm = false;
    motor1CH = 1500;
  }
}