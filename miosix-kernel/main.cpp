
#define VESC412

#include <stdio.h>
#include "drivers/pmsm_drive_stm32.h"

using namespace std;
using namespace miosix;

int main() {


    // TODO: Find out how to write in the terminal
    
    pmsmPWMsignal& driveSignals = pmsmPWMsignal::instance();
    driveSignals.setFrequency(1000);
    driveSignals.enable();
    driveSignals.start();
    driveSignals.setWidth(0);

    while (1) {

        for (float i = 0; i <= 1; i += .1) {
            greenLED_on();
            redLED_off();
            sleep(1);
            driveSignals.waitForCycleBegin();
            driveSignals.setWidth(i);
            greenLED_off();
            redLED_on();
            sleep(1);
        }
        for (float i = 1; i >= 0; i -= .1) {
            greenLED_on();
            redLED_off();
            sleep(1);
            driveSignals.waitForCycleBegin();
            driveSignals.setWidth(i);
            greenLED_off();
            redLED_on();
            sleep(1);
        }
    }
    return 0;
}
