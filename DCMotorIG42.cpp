#include "DCMotorIG42.h"
#include <math.h>
#include <Arduino.h>

// Constructor
DCMotorIG42::DCMotorIG42 (int pin1, int pin2, int id)
{
    // Hook the encoder
    encPin1 = pin1;
    encPin2 = pin2;
    rotaryEncoder.InterruptHook(encPin1, encPin2);
    // set the motor id
    _motorId = id;
    // Motor Driver initialization
    motorDriver.init();
    // Variabls initialization 
    oldCount =0; newCount = 0;
    prevTime = 0; curTime = millis();
}

void DCMotorIG42::setSpeed(int speed)
{
    // PWM Duty: 0~255 -> 0~400
    switch(_motorId) {
        // Should not protect pwm value
        case 1:
            motorDriver.setM1Speed(speed);
            break;
        case 2:
            motorDriver.setM2Speed(speed);
            break;
    }
}

/*
 * Unit Conversion needed?
 */
void DCMotorIG42::setSpeedRPS(double rps)
{
    /* Via System identification of specific motor, we fit
     * the curve that 
     * pwm = 95.76 * rps^2 -185.6 * rps + 129.7
     * that cool
     */
    double cf = 95.76 * rps*rps -185.6 *rps +129.7;
    int speed = floor(cf);

    Serial.print("speed: ");
    Serial.println(speed);
    // PWM Duty: 0~255 -> 0~400
    switch(_motorId) {
        // Should not protect pwm value
        case 1:
            motorDriver.setM1Speed(speed);
            break;
        case 2:
            motorDriver.setM2Speed(speed);
            break;
    }
}

/*
 * Unit Conversion?
 */

double DCMotorIG42::getSpeed()
{
    // Do unit conversion here, i think
        return rotVel;
}

double DCMotorIG42::getSpeedRPS()
{
    /* The Encoder send ~600Hz pulses @ 12 volt
     * which working 2 RPS @ 12V. 
     * Each Pulse can be divided into 4 phases,
     * that 1 rev/sec = 1200 counts/sec.
     * So, the actual rotational speed is 
     *   movement/msec * counts/msec * Rev/counts
     * = RotVel * 1/1200 * 1000/1
     * = 1.2
     * So we have this gain 1.2.
     */
    return 1.2 * rotVel;
}

void DCMotorIG42::stopIfFault()
{
    switch(_motorId) {
        case 1:
            if(motorDriver.getM1Fault()) {
                Serial.println("M1 Fault");
                while(1);
            }
            break;
        case 2:
            if(motorDriver.getM2Fault()) {
                Serial.println("M2 Fault");
                while(1);
            }
            break;
    }
}

void DCMotorIG42::RunTest()
{
    // Run different speed section
    for (int v = 0; v <= 400; v += 20) {
        // Roughly 5 secs each time
        setSpeed(v);
        Serial.print("/* Speed: ");
        Serial.print(v);
        Serial.println(" */");
        for(int i=0; i < 5; ++i) {
            Tick();
            delay(1000);
            Tock();
            // Show data
            Serial.print("Mov: ");
            Serial.println(getMov());
            Serial.print("Intv: ");
            Serial.println(getTimeInv());
            Serial.print("RotVel: ");
            Serial.println(getSpeed());
            Serial.print("RPS: ");
            Serial.println(getSpeedRPS());
            Serial.print("Magicfunc: ");
            Serial.println(invmagicfunction(getSpeedRPS()));
        }
    }

    /* RPS */
    for (double w=0; w < 3; w +=.5) {
        Serial.print("Set RPS = ");
        Serial.println(w);
        setSpeedRPS(w);
        Serial.print("Magicfunc: ");
        Serial.println(magicfunction(getSpeedRPS()));
        delay(1000);
    }
    stop();
}

long DCMotorIG42::readEncoder()
{
    return rotaryEncoder.read();
}

void DCMotorIG42::Tick()
{
    prevTime = millis();
    oldCount = rotaryEncoder.read();
}

void DCMotorIG42::Tock()
{
    curTime = millis();
    newCount = rotaryEncoder.read();
    movement = newCount - oldCount;
    interval = curTime - prevTime;
    rotVel = (double) (newCount-oldCount) / (curTime-prevTime); 
}

long DCMotorIG42::getMov() { return movement;}
double DCMotorIG42::getTimeInv() { return interval;}
