static void debug(){
  if(debugT.checkT()){
      DEBUG_PORT.print(" Channels: ");
      DEBUG_PORT.print(motor1CH);
      DEBUG_PORT.print(", ");
      DEBUG_PORT.print(steeringCH);
      DEBUG_PORT.print(", ");
      DEBUG_PORT.print(armedCH);
      DEBUG_PORT.print(", ");
      DEBUG_PORT.print(modeCH);
      DEBUG_PORT.print(", ");
      DEBUG_PORT.print(auxCH);
      DEBUG_PORT.print(" Armed: ");
      DEBUG_PORT.print(arm);
      DEBUG_PORT.print(" Mode: ");
      DEBUG_PORT.print(modes[mode]);
      DEBUG_PORT.print(" Voltage: ");
      DEBUG_PORT.print(vbat);
      //DEBUG_PORT.print(" AngleToHome: ");
      //DEBUG_PORT.print(fix.location.BearingToDegrees( base )*57.2957795);
      //DEBUG_PORT.print(" ");
      //DEBUG_PORT.print(fix.location.BearingToDegrees( base ));
      DEBUG_PORT.println("");
  }
}
