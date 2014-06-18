#ifndef __DCMOTORIG42_H_
#define __DCMOTORIG42_H_
#include <Arduino.h>
#include "DualVNH2SH30MotorShield.h"
#include "Encoder.h"

class DCMotorIG42
{
    public:
        DCMotorIG42(int EncoderPin1, int EncoderPin2, int MotorIndex);
        // Check current limit
        void stopIfFault();
        // PWM Duty Adjustment, speed range:-400~400

        void setSpeed(int pwm);
        double getSpeed();

        /* TODO: Change unit into RPS( revolution per sec) */
        void setSpeedRPS(double rps);
        double getSpeedRPS();
        // Just set speed to zero
        inline void stop() {setSpeed(0);};

        /* Tick and Tock to record duration time in millisce
         * and how 'long' the encoder counts, which called movement.
         * Read Encoder return count directly
         */
        void Tick();
        void Tock();
        long readEncoder();
        /* Get the duration of count between tick and tock.
         * Show time interval between tick and tock.
         */
        long getMov();
        double getTimeInv();

        // Unit Test code
        void RunTest();

    private:
        // Variables:
        uint8_t _motorId; // choose controlling motor 1 or 2, in our application 1 always
        uint8_t encPin1, encPin2; // pins of gpio interrupt to count encoder
        double curTime, prevTime, interval;
        unsigned long oldCount, newCount, movement;
        double rotVel; // Rotary Velocity, TODO:RPS

        // curve fitting get the result, rps = mag(pwm)
        inline double magicfunction(int pwm) { return -2.6e-5*pwm*pwm + .0169*pwm +.009467;};
        // curve fitting get the result, pwm = mag(rps)
        inline double invmagicfunction(int rps) { return 95.76 * rps*rps -185.6 *rps +129.7;};
       
        // Modules:
        DualVNH2SH30MotorShield motorDriver; // Motor driver 
        Encoder rotaryEncoder; // Encoder
};

#endif
