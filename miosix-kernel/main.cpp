
#include <cstdio>
#include "miosix.h"

using namespace std;
using namespace miosix;

typedef Gpio<GPIOC_BASE,4>  greenLED;

int main()
{
    greenLED::mode(Mode::OUTPUT);
        
    while(1) 
    {
        greenLED::high();
        sleep(1);
        greenLED::low();
        sleep(1);
    }
  
  return 0;
}
