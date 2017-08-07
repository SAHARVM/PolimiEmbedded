
#define VESC412

#include <stdio.h>
#include <iostream>
#include <string.h>
#include "drivers/pmsm_drive_stm32.h"
#include "drivers/serial_stm32.h"
#include "kernel/logging.h"
#include "miosix/arch/common/drivers/pmsm_drive_stm32.h"

using namespace std;
using namespace miosix;

Mutex mutex;
ConditionVariable cond, ack;
char c = 0;

void th_PMSM_PWM(void *argv) {
    //Run every 10 milliseconds
    const int period = static_cast<int> (TICK_FREQ * 0.001);
    long long tick = getTick();
    
    cout << "VESC Board Initialization" << endl;

    PMSMdriver& driveSignals = PMSMdriver::instance();
    driveSignals.setFrequency(1000);
    driveSignals.enable();
    driveSignals.start();
    driveSignals.setWidth(0);

    cout << "PWM signals initialized" << endl;
    
    while(1) {
        for (float i = 0; i <= 1; i += .001) {
            driveSignals.setWidth(i);
            tick += period;
            Thread::sleepUntil(tick);
        }
        for (float i = 1; i >= 0; i -= .001) {
            driveSignals.setWidth(i);
            tick += period;
            Thread::sleepUntil(tick);
        }
        cout << driveSignals.getHallEffectSensorsValue() << endl;
    }
}

void th_1s_tick(void *argv) {
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
    }
}

int main() {

    Thread *thread1;
    Thread *thread2;
    thread1 = Thread::create(th_PMSM_PWM, 2048, 1, NULL, Thread::DEFAULT);
    thread2 = Thread::create(th_1s_tick, 2048, 2, NULL, Thread::DEFAULT);


    

    
    while (1) {

    }

    return 0;
}
