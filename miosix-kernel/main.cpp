/* YBG for robot motor */
/*
 * YBG backwards?
 * YGB no
 * BGY no
 * BYG no
 * GYB no
 * GBY no
 */

#include <stdio.h>
#include <iostream>
#include <string.h>
#include "drivers/pmac_driver_stm32.h"
#include <math.h>

#define MONITOR_LABVIEW

#ifdef MOTOR_HUBMOTOR
#define DEFAULT_KP 0.001
#define DEFAULT_KI 0.001
#define DEFAULT_KD 0.0
#define MAX_SPEED_DPS 2700
#define MIN_SPEED_DPS 250
#else
#define DEFAULT_KP 0.01
#define DEFAULT_KI 0.05
#define DEFAULT_KD 0.0
#define MAX_SPEED_DPS 35000
#define MIN_SPEED_DPS 2000
#endif


using namespace std;
using namespace miosix;

Mutex mutex;
ConditionVariable cond, ack;
char c = 0;
float speed = 2500;
float dutyCycle = .1;
float kp = DEFAULT_KP; // proportional constant
float ki = DEFAULT_KI; // integral constant
float kd = DEFAULT_KD; // derivative constant

void thread_SerialControl(void *argv) {

    PMACdriver::instance();
    PMACdriver::setDrivingFrequency(PMAC_PWM_FREQUENCY);
    PMACdriver::enable();
    PMACdriver::start();

    while (1) {
#ifdef MONITOR_LABVIEW
        string instruction;
        cin >> instruction;
        if (instruction == "start") {
            cout << "ok" << endl;
            PMACdriver::enableDriver();
            dutyCycle = .1;
            speed = MIN_SPEED_DPS;
            PMACdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "stop") {
            cout << "ok" << endl;
            PMACdriver::disableDriver();
            dutyCycle = 0;
            speed = 0;
            PMACdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "gs") {
            cout << PMACdriver::getSpeed(0) << endl;
        } /*else if (instruction == "gos") {
            cout << PMACdriver::getOrbisSensorBits() << endl;
        }*/ else if (instruction == "geap") {
            cout << PMACdriver::getElectricalAngularPosition() << endl;
        } else if (instruction == "gc1") {
            cout << PMACdriver::getShuntCurrent(1) * 1000 << endl;
        } else if (instruction == "gc3") {
            cout << PMACdriver::getShuntCurrent(3) * 1000 << endl;
        } else if (instruction == "ss") {
            float inSpeed;
            cin >> inSpeed;
            if ((inSpeed <= MAX_SPEED_DPS) && (inSpeed >= MIN_SPEED_DPS)) {
                speed = inSpeed;
            }
        } else if (instruction == "gk") {
            cout << kp * 1000 << endl;
            cout << ki * 1000 << endl;
            cout << kd * 1000 << endl;
        } else if (instruction == "skp") {
            float kpIn;
            cin >> kpIn;
            kp = kpIn / 1000;
        } else if (instruction == "ski") {
            float kiIn;
            cin >> kiIn;
            ki = kiIn / 1000;
        } else if (instruction == "skd") {
            float kdIn;
            cin >> kdIn;
            kd = kdIn / 1000;
        }
        
#ifdef SINUSOIDAL_DRIVE
        /*New instructions*/
        else if (instruction == "focv0") {
            /* Theta */
            cout << (PMACdriver::getFOCvariables(0))*1000 << endl;
        }
        
        else if (instruction == "focv1") {
            /* i_qs_ref */
            cout << (PMACdriver::getFOCvariables(1))*1000 << endl;
        }
        
        else if (instruction == "focv2") {
            /* i_ds_ref */
            cout << (PMACdriver::getFOCvariables(2))*1000 << endl;
        }
        
        else if (instruction == "focv3") {
            /*Current 1*/
            cout << (PMACdriver::getFOCvariables(3))*1000 << endl;
        }
        
        else if (instruction == "focv4") {
            /*Current 2*/
            cout << (PMACdriver::getFOCvariables(4))*1000 << endl;
        }
        
        else if (instruction == "focv5") {
            /*Current 3*/
            cout << (PMACdriver::getFOCvariables(5))*1000 << endl;
        }
        
        else if (instruction == "focv6") {
            /*i_alpha*/
            cout << (PMACdriver::getFOCvariables(6))*1000 << endl;
        }
        
        else if (instruction == "focv7") {
            /*i_beta*/
            cout << (PMACdriver::getFOCvariables(7))*1000 << endl;
        }
        
        else if (instruction == "focv8") {
            /*i_ds*/
            cout << (PMACdriver::getFOCvariables(8))*1000 << endl;
        }
        
        else if (instruction == "focv9") {
            /*i_qs*/
            cout << (PMACdriver::getFOCvariables(9))*1000 << endl;
        }
        
        else if (instruction == "focv10") {
            /*quadratureCurrent_error_0*/
            cout << (PMACdriver::getFOCvariables(10))*1000 << endl;
        }
        
        else if (instruction == "focv11") {
            /*directCurrent_error_0*/
            cout << (PMACdriver::getFOCvariables(11))*1000 << endl;
        }
        
        else if (instruction == "focv12") {
            /*quadratureCurrent_integralError_0*/
            cout << (PMACdriver::getFOCvariables(12))*1000 << endl;
        }
        
        else if (instruction == "focv30") {
            /*directCurrent_integralError_0*/
            cout << PMACdriver::getFOCvariables(30) << endl;
        }
        
        else if (instruction == "focv31") {
            /*dutyCycle_A*/
            cout << PMACdriver::getFOCvariables(31)*1000 << endl;
        }
        else if (instruction == "focv32") {
            /*dutyCycle_B*/
            cout << PMACdriver::getFOCvariables(32)*1000 << endl;
        }
        else if (instruction == "focv33") {
            /*dutyCycle_C*/
            cout << PMACdriver::getFOCvariables(33)*1000 << endl;
        }
        
        
        
        
        
        else if (instruction == "sfocv1"){
            /* Set quadrature current */
            float iq_in;
            cin >> iq_in;
            if ((iq_in <= 3) && (iq_in >= 0)) {
                PMACdriver::setFOCvariables(1,iq_in);
            }
        }
        
        else if (instruction == "sfocv2"){
            /* Set direct current */
            float id_in;
            cin >> id_in;
            if ((id_in <= 3) && (id_in >= 0)) {
                PMACdriver::setFOCvariables(2,id_in);
            }
        }
        
        
#endif // SINUSOIDAL_DRIVE

#else
        // cout << "test" << ctr++ <<endl;
        cout << "Input instruction: ";
        string instruction;
        cin >> instruction;
        cout << instruction << endl;
        if (instruction == "help") {
            cout << "[help]" << endl;
            cout << " Available instructions: " << endl;
            cout << " -Start: start " << endl;
            cout << " -Stop: stop " << endl;
            cout << " -Set Speed: ss " << endl;
            cout << " -Get Speed: gs " << endl;
            cout << " -Set Kp: kp " << endl;
            cout << " -Set Ki: ki " << endl;
        } else if (instruction == "start") {
            cout << "[start]" << endl;
            PMACdriver::enableDriver();
            dutyCycle = .1;
            speed = MIN_SPEED_DPS;
            PMACdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "stop") {
            cout << "[stop]" << endl;
            PMACdriver::disableDriver();
            dutyCycle = 0;
            speed = 0;
            PMACdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "ss") {
            cout << "[SetSpeed]" << endl;
            //cout << "Speed must be a float value between 0 and " << TOP_SPEED_DPS << endl;
            float inSpeed;
            cin >> inSpeed;
            if ((inSpeed > MAX_SPEED_DPS) || (inSpeed < MIN_SPEED_DPS)) {
                cout << "Speed must be a float value between " << MIN_SPEED_DPS <<  " and " << MAX_SPEED_DPS << endl;
            } else {
                speed = inSpeed;
                cout << "Speed = " << speed << endl;
                // This value will be taken by the other thread
                // maybe I can implement another mechanism
            }
        } else if (instruction == "sdc") {
            cout << "[Set Duty Cycle]" << endl;
            //cout << "Speed must be a float value between 0 and " << TOP_SPEED_DPS << endl;
            float inDutyCycle;
            cin >> inDutyCycle;
            if ((inDutyCycle < .1) || (inDutyCycle > 1)) {
                cout << "Duty cycle must be a float value between 0.1 and 1" << endl;
            } else {
                dutyCycle = inDutyCycle;
                PMACdriver::changeDutyCycle(dutyCycle);
                cout << "Duty Cycle = " << dutyCycle << endl;
                // This value will be taken by the other thread
                // maybe I can implement another mechanism

            }
        } else if (instruction == "gs") {
            cout << "[GetSpeed]" << endl;
            cout << "hall_effect_sensors_frequency="        << PMACdriver::getSpeed(1) << endl
                    << "speed_radiansPerSecond_electric="   << PMACdriver::getSpeed(2) << endl
                    << "speed_degreesPerSecond_electric="   << PMACdriver::getSpeed(3) << endl
                    << "speed_RPM_electric="                << PMACdriver::getSpeed(4) << endl
                    << "speed_radiansPerSecond_mechanic="   << PMACdriver::getSpeed(5) << endl
                    << "speed_degreesPerSecond_mechanic="   << PMACdriver::getSpeed(6) << endl
                    << "speed_RPM_mechanic="                << PMACdriver::getSpeed(7) << endl;
        } else if (instruction == "kp") {
            cout << "[Set Kp]" << endl;
            cout << "Kp = " << kp << " write new Kp" << endl;
            cin >> kp;
            cout << "New Kp = " << kp << endl;
        } else if (instruction == "ki") {
            cout << "[Set Ki]" << endl;
            cout << "Ki = " << ki << " write new Ki" << endl;
            cin >> ki;
            cout << "New Ki = " << ki << endl;
        } else if (instruction == "kd") {
            cout << "[Set Kd]" << endl;
            cout << "Kd = " << kd << " write new Kd" << endl;
            cin >> kd;
            cout << "New Kd = " << kd << endl;
        } else if (instruction == "gv") {
            cout << "[Get ADC Value]" << endl;
            cout << "value = " << PMACdriver::getADCvalue() << endl;
        } else if (instruction == "gbt") {
            cout << "[Get Board Temperature]" << endl;
            //float adcBitsValue = (float) PMACdriver::getSimpleADCvalue();
            //float boardTemperature = THERMISTOR_B_VALUE / (log((4096 / adcBitsValue) - 1) + (THERMISTOR_B_VALUE / 25));
            //cout << "value = " << boardTemperature << endl;
        } else if (instruction == "gc1") {
            cout << "[Get Current 1]" << endl;
            float adcCurrent1BitsValue = (float) PMACdriver::getADCvalue();
            float current1 = ( ( adcCurrent1BitsValue * 3.3 / 4096 ) - (3.3 / 2) ) / ( 10 * 0.001 );
            cout << "value = " << current1 << endl;
        } else if (instruction == "gos") {
            cout << "[Get Orbis Sensor Bits]" << endl;
            cout << "Angular Position = " << PMACdriver::getOrbisSensorBits() << endl;
        } else if (instruction == "ctp") {
            cout << "[Change Trigger Point]" << endl;
            float inTriggerPoint;
            cin >> inTriggerPoint;
            if ((inTriggerPoint < .1) || (inTriggerPoint > 1)) {
                cout << "Trigger point must be a float value between 0.1 and 1" << endl;
            } else {
                float triggerPoint = inTriggerPoint;
                PMACdriver::changeTriggerPoint(triggerPoint);
                cout << "Trigger Point @ " << triggerPoint << endl;

            }
        }
#endif 
    }
}

void thread_1s_tick(void *argv) {
    //Run every second
    const int period = static_cast<int> (TICK_FREQ * 1);
    long long tick = getTick();
    while (1) {
        tick += period;
        Thread::sleepUntil(tick);
        greenLED_on();
        tick += period;
        Thread::sleepUntil(tick);
        greenLED_off();
        // Why can't I use toggle() as high() or low()?
#if 0
#ifndef MONITOR_LABVIEW
        if (ADC1->SR & ADC_SR_OVR){
            cout << "adc1 overrun" << endl;
            
        }
            cout << THERMISTOR_B_VALUE / (log((4096 / ((float)(ADC1->DR))) - 1) + (THERMISTOR_B_VALUE / 25)) << endl;
            //redLED_on();
#endif
#endif
        
    }
}

void thread_PMAC_PI(void *argv) {
    //Run every second
    const int period = static_cast<int> (TICK_FREQ * 0.001);
    long long tick = getTick();

    float err_0 = 0; // expected output - actual output ie. error;
    float err_1 = 0; // error from previous loop
    float intErr_0 = 0; // intErr from previous loop + err; ( i.e. integral error )
    float intErr_1 = 0; // intErr from previous loop
    float dt = 0.001; // execution time of loop
    float q = 0;
    float pTerm = 0;
    float iTerm = 0;

    while (1) {
        /*if (PMACdriver::getMotorStatus()) {
            err_1 = err_0;                                                      // store previous error value
            intErr_1 = intErr_0;                                                // store previous integral error value
            err_0 = (speed - PMACdriver::getSpeed(0)) / MAX_SPEED_DPS;          // get new error value
            //err_0 = (speed - PMACdriver::getSpeed(5)) / MAX_SPEED_DPS;          // get new error value
            intErr_0 = intErr_1 + err_0;                                        // calculate integral error
            if (intErr_0 > 1) intErr_0 = 1;                                           // upper-side saturation of the integral term
            if (intErr_0 < -1) intErr_0 = -1;                                         // lower-side saturation of the integral term
            pTerm = kp * err_0;                                                 // calculate the proportional term
            iTerm = ki * intErr_0 * dt;                                         // calculate the integral term
            if (iTerm > 1) iTerm = 1;                                           // upper-side saturation of the integral term
            if (iTerm < -1) iTerm = -1;                                         // lower-side saturation of the integral term
            q = pTerm + iTerm;                                                  // output signal value
            //dutyCycle = .5 + ( q / 2 );                                         // calculation of the output 
            dutyCycle += q;
            //dutyCycle = q;
            if (dutyCycle < .1) dutyCycle = .1;                                 // to avoid stopping the motor
            PMACdriver::changeDutyCycle(dutyCycle);                             // setup the duty cycle
        }*/
        if (PMACdriver::getMotorStatus()) PMACdriver::sinusoidalDrive();
        /************************/
        tick += period;
        Thread::sleepUntil(tick);
    }
}

int main() {

    Thread *thread1;
    Thread *thread2;
    Thread *thread3;
    thread1 = Thread::create(thread_SerialControl, 2048, 2, NULL, Thread::DEFAULT);
    thread2 = Thread::create(thread_1s_tick, 2048, 3, NULL, Thread::DEFAULT);
#ifdef SINUSOIDAL_DRIVE
    /* TODO: Create a new thread with speed control loop using FOC */
    //thread3 = Thread::create(thread_PMAC_PI, 2048, 1, NULL, Thread::DEFAULT);
#else
    thread3 = Thread::create(thread_PMAC_PI, 2048, 1, NULL, Thread::DEFAULT);
#endif
    

    /* TODO */
    // Dead times!!
    // Review FOC algorithm
    // FOC implementation
    // Change "magic numbers" for defines
    
    cout << "Permanent Magnet Alternating Current Motor Driver" << endl;

    while (1) {
    }

    return 0;
}
