
#define VESC412

//#include <cstdio>
#include <stdio.h>
//#include "miosix.h"
#include "drivers/servo_stm32.h"

using namespace std;
using namespace miosix;

int main()
{
#if 0
    while(1)
    {
        greenLED_on();
        sleep(1);
        greenLED_off();
        sleep(1);
    }
#endif
    
#if 1
    
    SynchronizedServo& servo=SynchronizedServo::instance();
    servo.enable(4); //Enable channel 4 (connect servo to PB5)
    servo.start();
    
    for(int i=0;i<4;i++) servo.enable(i);
    servo.start();
    for(;;)
    {
        printf("id (0-3), val (0.0-1.0)?\n");
        int id;
        float val;
        scanf("%d %f",&id,&val);
        servo.setPosition(id,val);
    }
#endif
    
  return 0;
}
