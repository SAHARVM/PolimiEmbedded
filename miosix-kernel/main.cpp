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
float speed = 0;
float dutyCycle = .1;
float kp = DEFAULT_KP; // proportional constant
float ki = DEFAULT_KI; // integral constant
float kd = DEFAULT_KD; // derivative constant
int OrbisSensorValue_bits_0 = 0;
int OrbisSensorValue_bits_1 = 0;
int OrbisSensorDelta_bits = 0;
float ElectricDelta_rads = 0;
char HallEffectSensors_0 = 0;
char HallEffectSensors_1 = 0;

float Electric_rads = 0;
int theta_char_0 = 0;
char theta_char_1 = 0;
char delta_theta = 0;

float err_0 = 0; // expected output - actual output ie. error;
float err_1 = 0; // error from previous loop
float intErr = 0; // integral error
const float dt = 0.001; // execution time of loop
float q = 0;
float pTerm = 0;
float iTerm = 0;

float u = 0;
float e = 0;
float i_q = 0;

bool flag = 0;

float I_reference_trapezoidal_A = 0;
float I_measured_trapezoidal_A = 0;

void thread_SerialControl(void *argv) {

    PMACdriver::instance();
    PMACdriver::setDrivingFrequency(PMAC_PWM_FREQUENCY);
    PMACdriver::enable();
    PMACdriver::start();
    PMACdriver::currentReadingCalibration();

    while (1) {
#ifdef MONITOR_LABVIEW
        string instruction;
        cin >> instruction;
        if (instruction == "start") {
            cout << "ok" << endl;
            PMACdriver::updateHallEffectSensorsValue();
            PMACdriver::calculateSpeed();
            PMACdriver::enableDriver();
        } else if (instruction == "stop") {
            cout << "ok" << endl;
            PMACdriver::disableDriver();
            dutyCycle = 0;
            PMACdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "gs") {
            cout << PMACdriver::getSpeed(5) * 1000 << endl;
        } else if (instruction == "gdc") {
            cout << u*1000 << endl;
        } 
        
        else if (instruction == "geap") {
            cout << PMACdriver::getElectricalAngularPosition() << endl;
            //cout << theta_rads << endl;
        } else if (instruction == "gc1") {
            PMACdriver::calculateShuntCurrent();
            cout << PMACdriver::getFOCvariables(HMI_VAR_I_A)*1000 << endl;
        } else if (instruction == "gc3") {
            PMACdriver::calculateShuntCurrent();
            cout << PMACdriver::getFOCvariables(HMI_VAR_I_C)*1000 << endl;
        } else if (instruction == "ss") {
            float inSpeed;
            cin >> inSpeed;
            speed = inSpeed;
        }else if (instruction == "sdc") {
            float inDutyCycle;
            cin >> inDutyCycle;
            dutyCycle = inDutyCycle/100;
            PMACdriver::changeDutyCycle(dutyCycle);
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
        else if (instruction == "HMI_VAR_THETA") cout << PMACdriver::getFOCvariables(HMI_VAR_THETA)*1000 << endl;
        else if (instruction == "HMI_VAR_I_Q_REF") cout << PMACdriver::getFOCvariables(HMI_VAR_I_Q_REF)*1000 << endl;
        else if (instruction == "HMI_VAR_I_D_REF") cout << PMACdriver::getFOCvariables(HMI_VAR_I_D_REF)*1000 << endl;
        else if (instruction == "HMI_VAR_I_A") cout << PMACdriver::getFOCvariables(HMI_VAR_I_A)*1000 << endl;
        else if (instruction == "HMI_VAR_I_B") cout << PMACdriver::getFOCvariables(HMI_VAR_I_B)*1000 << endl;
        else if (instruction == "HMI_VAR_I_C") cout << PMACdriver::getFOCvariables(HMI_VAR_I_C)*1000 << endl;
        else if (instruction == "HMI_VAR_I_ALPHA") cout << PMACdriver::getFOCvariables(HMI_VAR_I_ALPHA)*1000 << endl;
        else if (instruction == "HMI_VAR_I_BETA") cout << PMACdriver::getFOCvariables(HMI_VAR_I_BETA)*1000 << endl;
        else if (instruction == "HMI_VAR_I_Q") cout << PMACdriver::getFOCvariables(HMI_VAR_I_Q)*1000 << endl;
        else if (instruction == "HMI_VAR_I_D") cout << PMACdriver::getFOCvariables(HMI_VAR_I_D)*1000 << endl;
        else if (instruction == "HMI_VAR_I_Q_ERROR") cout << PMACdriver::getFOCvariables(HMI_VAR_I_Q_ERROR)*1000 << endl;
        else if (instruction == "HMI_VAR_I_D_ERROR") cout << PMACdriver::getFOCvariables(HMI_VAR_I_D_ERROR)*1000 << endl;
        else if (instruction == "HMI_VAR_I_Q_INTEGRAL_ERROR") cout << PMACdriver::getFOCvariables(HMI_VAR_I_Q_INTEGRAL_ERROR)*1000 << endl;
        else if (instruction == "HMI_VAR_I_D_INTEGRAL_ERROR") cout << PMACdriver::getFOCvariables(HMI_VAR_I_D_INTEGRAL_ERROR) << endl;
        else if (instruction == "HMI_VAR_PROPORTIONAL_Q") cout << PMACdriver::getFOCvariables(HMI_VAR_PROPORTIONAL_Q)*1000 << endl;
        else if (instruction == "HMI_VAR_PROPORTIONAL_D") cout << PMACdriver::getFOCvariables(HMI_VAR_PROPORTIONAL_D)*1000 << endl;
        else if (instruction == "HMI_VAR_INTEGRAL_Q") cout << PMACdriver::getFOCvariables(HMI_VAR_INTEGRAL_Q)*1000 << endl;
        else if (instruction == "HMI_VAR_INTEGRAL_D") cout << PMACdriver::getFOCvariables(HMI_VAR_INTEGRAL_D)*1000 << endl;
        else if (instruction == "HMI_VAR_V_Q") cout << PMACdriver::getFOCvariables(HMI_VAR_V_Q)*1000 << endl;
        else if (instruction == "HMI_VAR_V_D") cout << PMACdriver::getFOCvariables(HMI_VAR_V_D)*1000 << endl;
        else if (instruction == "HMI_VAR_V_ALPHA") cout << PMACdriver::getFOCvariables(HMI_VAR_V_ALPHA)*1000 << endl;
        else if (instruction == "HMI_VAR_V_BETA") cout << PMACdriver::getFOCvariables(HMI_VAR_V_BETA)*1000 << endl;
        else if (instruction == "HMI_VAR_U_ALPHA") cout << PMACdriver::getFOCvariables(HMI_VAR_U_ALPHA)*1000 << endl;
        else if (instruction == "HMI_VAR_U_BETA") cout << PMACdriver::getFOCvariables(HMI_VAR_U_BETA)*1000 << endl;
        else if (instruction == "HMI_VAR_X") cout << PMACdriver::getFOCvariables(HMI_VAR_X)*1000 << endl;
        else if (instruction == "HMI_VAR_Y") cout << PMACdriver::getFOCvariables(HMI_VAR_Y)*1000 << endl;
        else if (instruction == "HMI_VAR_Z") cout << PMACdriver::getFOCvariables(HMI_VAR_Z)*1000 << endl;
        else if (instruction == "HMI_VAR_TIME_PHASE_A") cout << PMACdriver::getFOCvariables(HMI_VAR_TIME_PHASE_A)*1000000000 << endl;
        else if (instruction == "HMI_VAR_TIME_PHASE_B") cout << PMACdriver::getFOCvariables(HMI_VAR_TIME_PHASE_B)*1000000000 << endl;
        else if (instruction == "HMI_VAR_TIME_PHASE_C") cout << PMACdriver::getFOCvariables(HMI_VAR_TIME_PHASE_C)*1000000000 << endl;
        else if (instruction == "HMI_VAR_DUTY_CYCLE_A") cout << PMACdriver::getFOCvariables(HMI_VAR_DUTY_CYCLE_A)*1000 << endl;
        else if (instruction == "HMI_VAR_DUTY_CYCLE_B") cout << PMACdriver::getFOCvariables(HMI_VAR_DUTY_CYCLE_B)*1000 << endl;
        else if (instruction == "HMI_VAR_DUTY_CYCLE_C") cout << PMACdriver::getFOCvariables(HMI_VAR_DUTY_CYCLE_C)*1000 << endl;
        else if (instruction == "HMI_VAR_MOTOR_STATUS") cout << PMACdriver::getFOCvariables(HMI_VAR_MOTOR_STATUS)*1000 << endl;
        else if (instruction == "HMI_VAR_ANGULAR_SLIP") cout << PMACdriver::getFOCvariables(HMI_VAR_ANGULAR_SLIP)*1000 << endl;
        else if (instruction == "HMI_VAR_KP_FOC_CURRENT") cout << PMACdriver::getFOCvariables(HMI_VAR_KP_FOC_CURRENT)*1000 << endl;
        else if (instruction == "HMI_VAR_KI_FOC_CURRENT") cout << PMACdriver::getFOCvariables(HMI_VAR_KI_FOC_CURRENT)*1000 << endl;
        else if (instruction == "HMI_VAR_KP_FOC_SPEED") cout << PMACdriver::getFOCvariables(HMI_VAR_KP_FOC_SPEED)*1000 << endl;
        else if (instruction == "HMI_VAR_KI_FOC_SPEED") cout << PMACdriver::getFOCvariables(HMI_VAR_KI_FOC_SPEED)*1000 << endl;
        else if (instruction == "HMI_VAR_DRV8302_FAULT_FLAG") cout << PMACdriver::getFOCvariables(HMI_VAR_DRV8302_FAULT_FLAG)*1000 << endl;
        else if (instruction == "HMI_VAR_SPEED") cout << PMACdriver::getFOCvariables(HMI_VAR_SPEED)*1000 << endl;
        else if (instruction == "HMI_VAR_TRIGGER_POSITION") cout << PMACdriver::getFOCvariables(HMI_VAR_TRIGGER_POSITION)*1000 << endl;  
        else if (instruction == "HMI_VAR_VOLTAGE_BUS") cout << PMACdriver::getFOCvariables(HMI_VAR_VOLTAGE_BUS)*1000 << endl;
        

        else if (instruction == "HMI_SET_VAR_I_Q_REF") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_I_Q_REF, x);
        } else if (instruction == "HMI_SET_VAR_I_D_REF") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_I_D_REF, x);
        } else if (instruction == "HMI_SET_VAR_ANGULAR_SLIP") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_ANGULAR_SLIP, x);
        }
        else if (instruction == "HMI_SET_VAR_KP_FOC_CURRENT") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_KP_FOC_CURRENT, x);
        } else if (instruction == "HMI_SET_VAR_KI_FOC_CURRENT") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_KI_FOC_CURRENT, x);
        } else if (instruction == "HMI_SET_VAR_KP_FOC_SPEED") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_KP_FOC_SPEED, x);
        } else if (instruction == "HMI_SET_VAR_KI_FOC_SPEED") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_KI_FOC_SPEED, x);
        } else if (instruction == "HMI_SET_VAR_TRIGGER_POSITION") {
            float x;
            cin >> x;
            PMACdriver::setFOCvariables(HMI_SET_VAR_TRIGGER_POSITION, x);
        } else if (instruction == "HMI_SET_VAR_SPEED") {
            float x;
            cin >> x;
            speed = x/1000;
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
            //dutyCycle = .1;
            //speed = MIN_SPEED_DPS;
            //PMACdriver::changeDutyCycle(dutyCycle);
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
            //if ((inSpeed > MAX_SPEED_DPS) || (inSpeed < MIN_SPEED_DPS)) {
            //    cout << "Speed must be a float value between " << MIN_SPEED_DPS << " and " << MAX_SPEED_DPS << endl;
            //} else {
            speed = inSpeed;
            cout << "Speed = " << speed << endl;
            // This value will be taken by the other thread
            // maybe I can implement another mechanism
            //}
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
            cout << "hall_effect_sensors_frequency=" << PMACdriver::getSpeed(1) << endl
                    << "speed_radiansPerSecond_electric=" << PMACdriver::getSpeed(2) << endl
                    << "speed_degreesPerSecond_electric=" << PMACdriver::getSpeed(3) << endl
                    << "speed_RPM_electric=" << PMACdriver::getSpeed(4) << endl
                    << "speed_radiansPerSecond_mechanic=" << PMACdriver::getSpeed(5) << endl
                    << "speed_degreesPerSecond_mechanic=" << PMACdriver::getSpeed(6) << endl
                    << "speed_RPM_mechanic=" << PMACdriver::getSpeed(7) << endl;
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
            //cout << "value = " << PMACdriver::getADCvalue() << endl;
        } else if (instruction == "gbt") {
            cout << "[Get Board Temperature]" << endl;
            //float adcBitsValue = (float) PMACdriver::getSimpleADCvalue();
            //float boardTemperature = THERMISTOR_B_VALUE / (log((4096 / adcBitsValue) - 1) + (THERMISTOR_B_VALUE / 25));
            //cout << "value = " << boardTemperature << endl;
        } else if (instruction == "gc1") {
            cout << "[Get Current 1]" << endl;
            //float adcCurrent1BitsValue = (float) PMACdriver::getADCvalue();
            //float current1 = ((adcCurrent1BitsValue * 3.3 / 4096) - (3.3 / 2)) / (10 * 0.001);
            //cout << "value = " << current1 << endl;
        } else if (instruction == "gos") {
            cout << "[Get Orbis Sensor Bits]" << endl;
            //cout << "Angular Position = " << PMACdriver::getOrbisSensorBits() << endl;
        } else if (instruction == "ctp") {
            cout << "[Change Trigger Point]" << endl;
            float inTriggerPoint;
            cin >> inTriggerPoint;
            if ((inTriggerPoint < .1) || (inTriggerPoint > 1)) {
                cout << "Trigger point must be a float value between 0.1 and 1" << endl;
            } else {
                //float triggerPoint = inTriggerPoint;
                //PMACdriver::changeTriggerPoint(triggerPoint);
                //cout << "Trigger Point @ " << triggerPoint << endl;

            }
        }
#endif 
    }
}

void thread_1s_tick(void *argv) {
    /* Run every second */
    const int period = static_cast<int> (TICK_FREQ * 1);
    long long tick = getTick();
    while (1) {
        tick += period;
        Thread::sleepUntil(tick);
        greenLED_on();
        tick += period;
        Thread::sleepUntil(tick);
        greenLED_off();
    }
}

void thread_PMAC_PI(void *argv) {
    //Run every millisecond
    const int period = static_cast<int> (TICK_FREQ * 0.001);
    long long tick = getTick();
#ifdef SINUSOIDAL_DRIVE
    while (1) {
        if (PMACdriver::getMotorStatus()) {
            e = (speed - PMACdriver::getSpeed(5));
            u = (e * PMACdriver::getFOCvariables(HMI_VAR_KP_FOC_SPEED)) + intErr;          // Torque
            i_q = u / MOTOR_TORQUE_CONSTANT;  // Quadrature Current
            if (i_q > I_MAX) i_q = I_MAX;
            else if (i_q < -I_MAX) i_q = -I_MAX;
            else intErr = intErr + ((PMACdriver::getFOCvariables(HMI_VAR_KI_FOC_SPEED) * e) * dt);
            PMACdriver::setFOCvariables(HMI_SET_VAR_I_Q_REF,i_q*1000);
            PMACdriver::setFOCvariables(HMI_SET_VAR_I_D_REF,0);
        }
        else {
            e = 0;
            u = 0;
            intErr = 0;
        }
        tick += period;
        Thread::sleepUntil(tick);
    }
#else
    while (1) {
        if (PMACdriver::getMotorStatus()) {
            e = (speed - PMACdriver::getSpeed(5));
            u = (e * kp) + intErr;  // best kp = 0.0016
            if (u > 1) u = 1;
            else if (u < 0) u = 0;
            else intErr = intErr + ((ki * e) * dt);     // best ki = 0.2048
            PMACdriver::changeDutyCycle(u);
        } else {
            e = 0;
            u = 0;
            intErr = 0;
        }
        
        tick += period;
        Thread::sleepUntil(tick);
    }
#endif
}

int main() {

    Thread *thread1;
    Thread *thread2;
    Thread *thread3;
    thread1 = Thread::create(thread_SerialControl, 2048, 2, NULL, Thread::DEFAULT);
    thread2 = Thread::create(thread_1s_tick, 2048, 3, NULL, Thread::DEFAULT);
#ifdef SPEED_CONTROL
    thread3 = Thread::create(thread_PMAC_PI, 2048, 1, NULL, Thread::DEFAULT);
#endif
    /* TODO */
    // PMACdriver::currentReadingCalibration();
    // Modify ADC for current reading in trapezoidal
    // Create Speed loop enabler/disabler
    // Setup DMA

    cout << "Permanent Magnet Alternating Current Motor Driver" << endl;

    while (1) {
    }

    return 0;
}
