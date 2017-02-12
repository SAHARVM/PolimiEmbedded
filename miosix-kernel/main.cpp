
#define VESC412

//#include <cstdio>
#include <stdio.h>
//#include "miosix.h"
#include "drivers/servo_stm32.h"



//using namespace std;
using namespace miosix;

int main()
{
    SynchronizedServo& servo=SynchronizedServo::instance();
    servo.enable(0); //Enable channel 0 (connect servo to PB6)
    servo.start();
       
    while(1)
    {
        for(float f=0.0f;;f+=0.01f)
        {
            servo.waitForCycleBegin();
            greenLED_on();
            servo.setPosition(0,f);
            if(f>=1.0f) f=0.0f;
            greenLED_off();
        }
    }
  return 0;
}
