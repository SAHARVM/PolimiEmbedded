

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

void thread_PMSM_PWM(void *argv) {
    //Run every 10 milliseconds
    const int period = static_cast<int> (TICK_FREQ * 0.001);
    long long tick = getTick();

    PMSMdriver& driveSignals = PMSMdriver::instance();
    driveSignals.setFrequency(10000);
    driveSignals.enable();
    driveSignals.start();
    driveSignals.enableDriver();

    float dutyCycle = .1;
    
    while (1) {
        
        driveSignals.trapezoidalDrive(dutyCycle, CW);
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
    }
}

int main() {

    sleep(5);   // Don't delete this, it's useful to avoid current peaks when we need to reprogram the system
    
    Thread *thread1;
    Thread *thread2;
    thread1 = Thread::create(thread_PMSM_PWM, 2048, 1, NULL, Thread::DEFAULT);
    thread2 = Thread::create(thread_1s_tick, 2048, 2, NULL, Thread::DEFAULT);
    
    while (1) {
    }

    return 0;
}
