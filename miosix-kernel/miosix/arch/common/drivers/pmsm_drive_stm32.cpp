
/* 
 * File:   pmsm_drive_stm32.cpp
 * Author: Arturo Montufar Arreola, based on servo_stm32.cpp by Federico Terraneo
 * 
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#include "pmsm_drive_stm32.h"
#include "kernel/scheduler/scheduler.h"
#include <algorithm>
#include <cstdio>
#include <cmath>

#define TRAPEZOIDAL_DRIVE
// #define SINUSOIDAL_DRIVE

using namespace std;
using namespace miosix;

typedef Gpio<GPIOB_BASE, 5> pwmSignal0; // TIM3_CH2, this output can drive a servomotor, must now be allocated in another file maybe

typedef Gpio<GPIOA_BASE, 8> pwmSignalH3; // TIM1_CH1
typedef Gpio<GPIOA_BASE, 9> pwmSignalH2; // TIM1_CH2
typedef Gpio<GPIOA_BASE, 10> pwmSignalH1; // TIM1_CH3
#ifdef TRAPEZOIDAL_DRIVE
typedef Gpio<GPIOB_BASE, 13> outSignalL3;
typedef Gpio<GPIOB_BASE, 14> outSignalL2;
typedef Gpio<GPIOB_BASE, 15> outSignalL1;
#elif SINUSOIDAL_DRIVE
// TODO: Find out how to use complementary TIM1 signals as constant output...
typedef Gpio<GPIOB_BASE, 13> pwmSignalL3; // TIM1_CH1N
typedef Gpio<GPIOB_BASE, 14> pwmSignalL2; // TIM1_CH2N
typedef Gpio<GPIOB_BASE, 15> pwmSignalL1; // TIM1_CH3N
#endif
typedef Gpio<GPIOB_BASE, 6> hallSensor1;
typedef Gpio<GPIOB_BASE, 7> hallSensor2;
typedef Gpio<GPIOC_BASE, 11> hallSensor3;

typedef Gpio<GPIOC_BASE, 10> enableGate;

typedef Gpio<GPIOC_BASE, 12> faultPin; // TODO: implement monitor and handler for this signal

static Thread *waiting = 0;

char hallEffectSensors_newPosition = 0;
char hallEffectSensors_oldPosition = 0;

float vDutyCycle = 0;
bool vDirection = 0; // 0 is CW, 1 is CCW  

/**
 * Timer 1 interrupt handler actual implementation
 */
void __attribute__ ((used)) tim1impl() {
    /* The code of the interrupt handler goes from here...*/
    TIM1->SR = 0; // Clear interrupt flags
    /*... to here*/

    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * Timer 1 interrupt handler
 */
void __attribute__ ((naked)) TIM1_CC_IRQHandler() {
    saveContext();
    asm volatile("bl _Z8tim1implv");
    restoreContext();
}

/**
 * Timer 2 interrupt handler actual implementation
 */

void __attribute__ ((used)) tim2impl() {
    /* The code of the interrupt handler goes from here...*/
    
    TIM2->SR = 0; // Clear interrupt flags
    PMSMdriver::trapezoidalDrive();
   
    /*... to here*/

    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * Timer 2 interrupt handler
 */
void __attribute__ ((naked)) TIM2_IRQHandler() {
    saveContext();
    asm volatile("bl _Z8tim2implv");
    restoreContext();
}

/**
 * Timer 3 interrupt handler actual implementation
 */
void __attribute__ ((used)) tim3impl() {
    TIM3->SR = 0; // Clear interrupt flags
    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * Timer 3 interrupt handler
 */
void __attribute__ ((naked)) TIM3_IRQHandler() {
    saveContext();
    asm volatile("bl _Z8tim3implv");
    restoreContext();
}


namespace miosix {

    //
    // class PMSMdriver
    //

    PMSMdriver& PMSMdriver::instance() {
        static PMSMdriver singleton;
        return singleton;
    }

    //PMSMdriver::PMSMdriver()/* : status(STOPPED) */{
    PMSMdriver::PMSMdriver(){
        {
            FastInterruptDisableLock dLock;
            // The RCC register should be written with interrupts disabled to
            // prevent race conditions with other threads.
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Servo motor drive
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // PMSM drive
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Control PMSM
            RCC_SYNC();
        }

        // Configure timer 3
        TIM3->CR1 = 0;
        TIM3->CCR2 = 0;

        // Configure timer 1
        TIM1->CR1 = 0; // Control Register 1 = 0, reset everything related to this timer
        TIM1->CCR1 = 0; // Capture Compare register = 0, 
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;

        // Configure interrupt on timer 3 overflow
        TIM3->DIER = TIM_DIER_UIE;
        NVIC_SetPriority(TIM3_IRQn, 16); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM3_IRQn);

        // Configure interrupt on timer 1 overflow
        TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE;
        NVIC_SetPriority(TIM1_CC_IRQn, 14); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM1_CC_IRQn);

        // Initialize motor driver specifics
        setupHallSensors();
        enableGate::mode(Mode::OUTPUT);
        faultPin::mode(Mode::INPUT); // Doesn't have a handler yet
        disableDriver(); // Must be enabled later to drive the power MOS gates
        setupControlTimer(15000);
        
    }

    void PMSMdriver::setFrequency(unsigned int PWM_frequency) {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call

        uint32_t TIMER_Frequency = SystemCoreClock / 2; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PWM_frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to

        TIM3->PSC = PSC_Value;
        TIM3->ARR = ARR_Value;

        /*In case that timers have different clock sources, use different configurations*/
        TIMER_Frequency = SystemCoreClock; // From the data sheet
        COUNTER_Frequency = PWM_RESOLUTION * PWM_frequency; // How many steps inside a Period
        PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        ARR_Value = PWM_RESOLUTION - 1; // Top value to count to

        TIM1->PSC = PSC_Value;
        TIM1->ARR = ARR_Value;
    }

    void PMSMdriver::enable() {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call
        {
            FastInterruptDisableLock dLock;
            // Calling the mode() function on a GPIO is subject to race conditions
            // between threads on the STM32, so we disable interrupts

            /*
             *  PWM configuration:
             *      CCMRn - capture/compare mode register 1 (for channels 1 and 2) and 2 (for channel 3):
             *          - Output Compare 1 mode 110: PWM mode 1, start up and at CC go down
             *              TIM_CCMR1_OCxM_2 | TIM_CCMR1_OCxM_1
             *          - Output Compare 2 pre-load enable
             *              TIM_CCMR1_OCxPE
             *      CCER - Capture/compare enable register:
             *          - Capture/Compare 1 output enable
             *              TIM_CCER_CCxE
             */

            TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
            TIM3->CCER |= TIM_CCER_CC2E;
            pwmSignal0::alternateFunction(2);
            pwmSignal0::mode(Mode::ALTERNATE);

            TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
            TIM1->CCER |= TIM_CCER_CC1E;
            pwmSignalH3::alternateFunction(1);
            pwmSignalH3::mode(Mode::ALTERNATE);

            TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
            TIM1->CCER |= TIM_CCER_CC2E;
            pwmSignalH2::alternateFunction(1);
            pwmSignalH2::mode(Mode::ALTERNATE);

            TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
            TIM1->CCER |= TIM_CCER_CC3E;
            pwmSignalH1::alternateFunction(1);
            pwmSignalH1::mode(Mode::ALTERNATE);

#ifdef TRAPEZOIDAL_DRIVE
            outSignalL3::mode(Mode::OUTPUT);
            outSignalL3::low();
            outSignalL2::mode(Mode::OUTPUT);
            outSignalL2::low();
            outSignalL1::mode(Mode::OUTPUT);
            outSignalL1::low();
#elif SINUSOIDAL_DRIVE
            /*TODO*/
#endif
        }
    }

    void PMSMdriver::disable() {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call
        {
            FastInterruptDisableLock dLock;
            // Calling the mode() function on a GPIO is subject to race conditions
            // between threads on the STM32, so we disable interrupts

            disableDriver();

            pwmSignal0::mode(Mode::INPUT);
            TIM3->CCER &= ~TIM_CCER_CC2E;
            pwmSignalH1::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC1E;
            pwmSignalH2::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC2E;
            pwmSignalH3::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC3E;

#ifdef TRAPEZOIDAL_DRIVE
            outSignalL3::mode(Mode::INPUT);
            outSignalL2::mode(Mode::INPUT);
            outSignalL1::mode(Mode::INPUT);
#elif SINUSOIDAL_DRIVE
            /*TODO*/
#endif
        }
    }

    void PMSMdriver::start() {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call

        // While status is starting neither member function callable with timer
        // started nor stopped are allowed
        //status = STARTED;
        TIM3->CNT = 0;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->CR1 = TIM_CR1_CEN;

        TIM1->CNT = 0; // Start counter value from 0
        TIM1->EGR = TIM_EGR_UG; // Event Generation Register = Update generation
        TIM1->BDTR = TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM1->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable
    }

    void PMSMdriver::stop() {
        //Lock<FastMutex> l(mutex);
        //if (status != STARTED) return; // If timer disabled ignore the call
        //status = STOPPED;
        // Erase the value in the capture/compare registers
        TIM3->CCR2 = 0;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        {
            FastInterruptDisableLock dLock;
            // Wakeup an eventual thread waiting on waitForCycleBegin()
            if (waiting) waiting->IRQwakeup();
            IRQwaitForTimerOverflow(dLock);
        }
        // Disable the timers completely
        TIM3->CR1 = 0;
        TIM1->CR1 = 0;
    }

    void PMSMdriver::setHighSideWidth(char channel, float pulseWidth) {
        //Lock<FastMutex> l(mutex);
        //if (status != STARTED) return; // If timer disabled ignore the call
        switch (channel) {
            case 0:
                TIM3->CCR2 = pulseWidth * PWM_RESOLUTION;
                break;
            case 1:
                TIM1->CCR3 = pulseWidth * PWM_RESOLUTION;
                break;
            case 2:
                TIM1->CCR2 = pulseWidth * PWM_RESOLUTION;
                break;
            case 3:
                TIM1->CCR1 = pulseWidth * PWM_RESOLUTION;
                break;
            default:
                break;
        }
    }

    void PMSMdriver::setLowSide(char channel, bool value) {
        //Lock<FastMutex> l(mutex);
        //if (status != STARTED) return; // If timer disabled ignore the call
        switch (channel) {
            case 1:
                value == 1 ? outSignalL1::high() : outSignalL1::low();
                break;
            case 2:
                value == 1 ? outSignalL2::high() : outSignalL2::low();
                break;
            case 3:
                value == 1 ? outSignalL3::high() : outSignalL3::low();
                break;
            default:
                break;
        }
    }

    
    /*bool PMSMdriver::waitForCycleBegin() {
        // No need to lock the mutex because disabling interrupts is enough to avoid
        // race conditions. Also, locking the mutex here would prevent other threads
        // from calling other member functions of this class
        FastInterruptDisableLock dLock;
        if (status != STARTED) return true;
        IRQwaitForTimerOverflow(dLock);
        return status != STARTED;
    }*/

    void PMSMdriver::setupHallSensors() {
        hallSensor1::mode(Mode::INPUT);
        hallSensor2::mode(Mode::INPUT);
        hallSensor3::mode(Mode::INPUT);
    }

    void PMSMdriver::updateHallEffectSensorsValue() {
        hallEffectSensors_oldPosition = hallEffectSensors_newPosition;
        hallEffectSensors_newPosition = (hallSensor1::value() << 0) | (hallSensor2::value() << 1) | (hallSensor3::value() << 2);
    }

    char PMSMdriver::getHallEffectSensorsValue() {
        updateHallEffectSensorsValue();
        return hallEffectSensors_newPosition;
    }

    void PMSMdriver::enableDriver() {
        enableGate::high();
    }

    void PMSMdriver::disableDriver() {
        enableGate::low();
    }

    void PMSMdriver::updateFaultFlag(){
        faultFlag = faultPin::value();
    }

    bool PMSMdriver::getFaultFlag() {
        updateFaultFlag();
        return faultFlag;
    }

    int PMSMdriver::trapezoidalDrive() {
        updateHallEffectSensorsValue();
        if (hallEffectSensors_oldPosition != hallEffectSensors_newPosition) {
            if (vDirection == CW) {
                if (hallEffectSensors_newPosition == 0b001) {
                    setLowSide(2, 0);
                    setLowSide(3, 0);
                    setHighSideWidth(1, 0);
                    setHighSideWidth(3, 0);

                    setLowSide(1, 1);
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b101) {
                    setLowSide(2, 0);
                    setLowSide(3, 0);
                    setHighSideWidth(1, 0);
                    setHighSideWidth(2, 0);

                    setLowSide(1, 1);
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b100) {
                    setLowSide(1, 0);
                    setLowSide(3, 0);
                    setHighSideWidth(1, 0);
                    setHighSideWidth(2, 0);

                    setLowSide(2, 1);
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b110) {
                    setLowSide(1, 0);
                    setLowSide(3, 0);
                    setHighSideWidth(2, 0);
                    setHighSideWidth(3, 0);

                    setLowSide(2, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b010) {
                    setLowSide(1, 0);
                    setLowSide(2, 0);
                    setHighSideWidth(2, 0);
                    setHighSideWidth(3, 0);

                    setLowSide(3, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b011) {
                    setLowSide(1, 0);
                    setLowSide(2, 0);
                    setHighSideWidth(1, 0);
                    setHighSideWidth(3, 0);

                    setLowSide(3, 1);
                    setHighSideWidth(2, vDutyCycle);
                }
            } else if (vDirection == CCW) {}
            return 1;
        }
        else {
            return 0;
        }
    }
    
    void PMSMdriver::setupControlTimer(unsigned int frequency){
         // Initialize Timer 2 to apply the trapezoidal drive @ 80kHz
        TIM2->CR1 = 0;
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->EGR = TIM_EGR_UG;
        NVIC_SetPriority(TIM2_IRQn, 15); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM2_IRQn);
        //TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
        //TIM2->CCER |= TIM_CCER_CC2E;
        uint32_t TIMER_Frequency = SystemCoreClock / 2; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to
        TIM2->PSC = PSC_Value;  //  Prescaler
        TIM2->ARR = ARR_Value;
        TIM2->CNT = 0;
        TIM2->CR1 = TIM_CR1_CEN;
    }
    
    void PMSMdriver::changeDutyCycle (float dutyCycle){
        vDutyCycle = dutyCycle;
    }
    
    void PMSMdriver::changeDirection (float direction){
        vDutyCycle = direction;
    }

    void PMSMdriver::IRQwaitForTimerOverflow(FastInterruptDisableLock& dLock) {
        waiting = Thread::IRQgetCurrentThread();
        do {
            Thread::IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        } while (waiting);
    }
}