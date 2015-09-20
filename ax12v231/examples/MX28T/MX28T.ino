// connect motors to Serial2 
// comment out ISR (USART2_RX_vect) in HardwareSerial.cpp

#include <ax12.h>

// motor declaration
AX12 pan_motor, tilt_motor;

void setup(void) {
       
    AX12::init (57142);                                 // Dynamixel MX-28 init
    byte detect[2];                                     // array ID auto-detection
    byte num = AX12::autoDetect (detect, 2);             
    Serial.print ("Motor detection: ");
    Serial.println (num, DEC);
    pan_motor = AX12(detect[0]);      // "pan" is the lowest ID detected
    tilt_motor = AX12(detect[1]);     // "tilt" is the highest ID detected
    motor_init (&pan_motor);
    motor_init (&tilt_motor);

}

void loop() {
  
  pan_motor.setVel (random (100, 300));
  pan_motor.setPos (random (200, 800));
    
     delay (100);
    
    int pos = pan_motor.getPos(); 
    int vel = pan_motor.getSpeed(); 
    int load = pan_motor.getLoad();
    
    Serial.println (" ");
    Serial.print (" posicion: ");
    Serial.println (pos, DEC);
    Serial.print (" velocidad: ");
    Serial.println (vel, DEC);
    Serial.print (" carga: ");
    Serial.println (load, DEC);
        
    delay (1100);
  
}

void motor_init (AX12 *motor) {
    motor->setSRL (RETURN_READ);
    motor->torqueOn;                                     // habilita el torque
    motor->setEndlessTurnMode(false);                    // lo pone en modo servo
    motor->setVel(1023);
    motor->writeInfo (LIMIT_TEMPERATURE, 80);
    delay(1);
    motor->writeInfo (DOWN_LIMIT_VOLTAGE, 60);
    delay (5);
    motor->writeInfo (UP_LIMIT_VOLTAGE, 160);
    delay (5);
    motor->writeInfo (RETURN_DELAY_TIME, 50);
    delay (10);
}
