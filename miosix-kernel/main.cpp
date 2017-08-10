

#include <stdio.h>
#include <iostream>
#include <string.h>
#include "drivers/pmsm_drive_stm32.h"
#include "drivers/serial_stm32.h"
#include "kernel/logging.h"
#include "miosix/arch/common/drivers/pmsm_drive_stm32.h"

#define PMSM_PWM_FREQUENCY 15000
#define TOP_SPEED_DPS 30000

#define DEFAULT_KP 0.01
#define DEFAULT_KI 0.05
#define DEFAULT_KD 0.0

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

    const int period = static_cast<int> (TICK_FREQ * 1);
    long long tick = getTick();

    PMSMdriver::instance();
    PMSMdriver::setFrequency(PMSM_PWM_FREQUENCY);
    PMSMdriver::enable();
    PMSMdriver::start();

    //Run every second
    while (1) {
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
            cout << " -Set Kd: kd " << endl;
        } else if (instruction == "start") {
            cout << "[start]" << endl;
            PMSMdriver::enableDriver();
            dutyCycle = .1;
            speed = 2500;
            PMSMdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "stop") {
            cout << "[stop]" << endl;
            PMSMdriver::disableDriver();
            dutyCycle = 0;
            speed = 0;
            PMSMdriver::changeDutyCycle(dutyCycle);
        } else if (instruction == "ss") {
            cout << "[SetSpeed]" << endl;
            //cout << "Speed must be a float value between 0 and " << TOP_SPEED_DPS << endl;
            float inSpeed;
            cin >> inSpeed;
            if ((inSpeed > TOP_SPEED_DPS) || (inSpeed < 2000)) {
                cout << "Speed must be a float value between 2000 and " << TOP_SPEED_DPS << endl;
            } else {
                speed = inSpeed;
                cout << "Speed = " << speed << endl; 
                // This value will be taken by the other thread
                // maybe I can implement another mechanism
            }
        } else if (instruction == "gs") {
            cout << "[GetSpeed]" << endl;
            cout << "Speed = " << PMSMdriver::getSpeed(0) << " DPS"
                    << " = " << PMSMdriver::getSpeed(1) << " RPM" << endl;
        }else if (instruction == "kp"){
            cout << "[Set Kp]" << endl;
            cout << "Kp = " << kp << " write new Kp" << endl;
            cin >> kp;
            cout << "New Kp = " << kp << endl;
        } else if (instruction == "ki"){
            cout << "[Set Ki]" << endl;
            cout << "Ki = " << ki << " write new Ki" << endl;
            cin >> ki;
            cout << "New Ki = " << ki << endl;
        } else if (instruction == "kd"){
            cout << "[Set Kd]" << endl;
            cout << "Kd = " << kd << " write new Kd" << endl;
            cin >> kd;
            cout << "New Kd = " << kd << endl;
        }
        tick += period;
        Thread::sleepUntil(tick);
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
    }
}

void thread_PMSM_PID(void *argv) {
    //Run every second
    const int period = static_cast<int> (TICK_FREQ * 0.001);
    long long tick = getTick();

    float err_0 = 0; // expected output - actual output ie. error;
    float err_1 = 0; // error from previous loop
    float intErr_0 = 0; // intErr from previous loop + err; ( i.e. integral error )
    float intErr_1 = 0; // intErr from previous loop
    float derErr = 0; // err - err from previous loop; ( i.e. differential error)
    float dt = 0.001; // execution time of loop
    float q = 0;
    float pTerm = 0;
    float iTerm = 0;
    float dTerm = 0;

    while (1) {
        if (PMSMdriver::getMotorStatus()) {
                err_1 = err_0; // store previous error value
                intErr_1 = intErr_0; // store previous integral error value
                err_0 = (speed - PMSMdriver::getSpeed(0)) / TOP_SPEED_DPS; // get new error value
                intErr_0 = intErr_1 + err_0;
                derErr = err_0 - err_1;
                pTerm = kp * err_0;
                iTerm = ki * intErr_0 * dt;
                dTerm = kd * derErr / dt;
                //if (iTerm >= 1) iTerm = 1;
                //if (iTerm <= -1) iTerm = -1;
                q = pTerm + iTerm + dTerm;
                dutyCycle += q;
                if (dutyCycle < .1) dutyCycle = .1;
                PMSMdriver::changeDutyCycle(dutyCycle);
            }
        /*********************************************************************************/
        tick += period;
        Thread::sleepUntil(tick);
    }
}

int main() {

    sleep(1); // Don't delete this yet, it's useful to avoid current peaks when we need to reprogram the system

    Thread *thread1;
    Thread *thread2;
    Thread *thread3;
    thread1 = Thread::create(thread_SerialControl, 2048, 2, NULL, Thread::DEFAULT);
    thread2 = Thread::create(thread_1s_tick, 2048, 3, NULL, Thread::DEFAULT);
    thread3 = Thread::create(thread_PMSM_PID, 2048, 1, NULL, Thread::DEFAULT);

    cout << "Permanent Magnet Synchronous Motor Driver" << endl;

    while (1) {
    }

    return 0;
}
