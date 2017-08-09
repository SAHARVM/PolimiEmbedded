

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
    //Run every 100 milliseconds
    const int period = static_cast<int> (TICK_FREQ * 0.1);
    long long tick = getTick();
    float dutyCycle = .1;
    char ramp = 0; 
    
    PMSMdriver::instance();
    PMSMdriver::setFrequency(20000);
    PMSMdriver::enable();
    PMSMdriver::start();
    PMSMdriver::enableDriver();

    while (1) {
        if (ramp == 0){
            dutyCycle += .01;
            if (dutyCycle >= 1){
                sleep(5);
                ramp = 1;
            }
        } else if (ramp == 1){
            dutyCycle -= .01;
            if (dutyCycle <= .1){
                ramp = 0;
            }
        }
        PMSMdriver::changeDutyCycle(dutyCycle);
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

    sleep(5);   // Don't delete this yet, it's useful to avoid current peaks when we need to reprogram the system
    
    Thread *thread1;
    Thread *thread2;
    thread1 = Thread::create(thread_PMSM_PWM, 2048, 1, NULL, Thread::DEFAULT);
    thread2 = Thread::create(thread_1s_tick, 2048, 2, NULL, Thread::DEFAULT);
    
    while (1) {
    }

    return 0;
}
