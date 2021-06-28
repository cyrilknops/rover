static void GPSloop()
{
  bool light;
  if(gps.available( gpsPort )) {
    fix = gps.read();
    if(fix.valid.location){
      gpsfix = true;
      digitalWrite(A_LED_PIN, LED_ON);
      if(second.checkT()){
        light = !light;
      }

    }else{
      gpsfix = false;
      digitalWrite(A_LED_PIN,LED_OFF);
    }
  }
}