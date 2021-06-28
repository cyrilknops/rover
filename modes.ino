void cruisecontrol(float speed){
    DEBUG_PORT.print("CH-IN:");
    DEBUG_PORT.print(motor1CH);
    DEBUG_PORT.print("PID-In:");
    DEBUG_PORT.print(speedKm);
    DEBUG_PORT.print(" PID-Out:");
    Output = pid.calculate(speed, speedKm);
    DEBUG_PORT.print(Output);
    if(speedKm > speed){
        motor1CH = map(motor1CH, 1000, 2000, 1000, 2000-Output*100);
    }else{
        
    }
    DEBUG_PORT.print(" CH-Out:");
    DEBUG_PORT.println(motor1CH);
}