
/* 
 * File:   pmac_driver_stm32.cpp
 * Author: Arturo Montufar Arreola, based on servo_stm32.cpp by Federico Terraneo
 * 
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#include "pmac_driver_stm32.h"
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

typedef Gpio<GPIOB_BASE, 13> outSignalL3;
typedef Gpio<GPIOB_BASE, 14> outSignalL2;
typedef Gpio<GPIOB_BASE, 15> outSignalL1;

// In output mode (forced, output compare or PWM), OCxREF can be re-directed to the OCx
// output or to OCxN output by configuring the CCxE and CCxNE bits in the TIMx_CCER register.
// This allows the user to send a specific waveform (such as PWM or static active level) on
// one output while the complementary remains at its inactive level.

//typedef Gpio<GPIOB_BASE, 13> pwmSignalL3; // TIM1_CH1N
//typedef Gpio<GPIOB_BASE, 14> pwmSignalL2; // TIM1_CH2N
//typedef Gpio<GPIOB_BASE, 15> pwmSignalL1; // TIM1_CH3N

typedef Gpio<GPIOB_BASE, 6> hallSensor1;
typedef Gpio<GPIOB_BASE, 7> hallSensor2;
typedef Gpio<GPIOC_BASE, 11> hallSensor3;
typedef Gpio<GPIOC_BASE, 10> enableGate;
typedef Gpio<GPIOC_BASE, 12> faultPin; // TODO: implement monitor and handler for this signal

typedef Gpio<GPIOA_BASE, 0> phaseVoltage3_ADC; // ADC channel 0
typedef Gpio<GPIOA_BASE, 1> phaseVoltage2_ADC; // ADC channel 1
typedef Gpio<GPIOA_BASE, 2> phaseVoltage1_ADC; // ADC channel 2
typedef Gpio<GPIOA_BASE, 3> boardTemperature_ADC; // ADC channel 3
typedef Gpio<GPIOB_BASE, 0> currentSense2_ADC; // ADC channel 8
typedef Gpio<GPIOB_BASE, 1> currentSense1_ADC; // ADC channel 9
typedef Gpio<GPIOC_BASE, 0> motorTemperature_ADC; // ADC channel 10
// typedef Gpio<GPIOC_BASE, 1> externalAnalogSignal_ADC;   // ADC channel 12

static Thread *waiting = 0;

char hallEffectSensors_newPosition = 0;
char hallEffectSensors_oldPosition = 0;

float vDutyCycle = 0;
bool vDirection = 0; // 0 is CW, 1 is CCW  
int speedCounter = 0;
float speedDPS = 0;
float speedRPM = 0;
bool motorRunning = 0;

bool LED_flag = 0;

/**
 * Timer 1 interrupt handler actual implementation
 */
void __attribute__ ((used)) tim1impl() {
    TIM1->SR = 0; // Clear interrupt flags

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
    TIM2->SR = 0; // Clear interrupt flags
    if (PMACdriver::getMotorStatus()) PMACdriver::trapezoidalDrive();

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

/**
 * Timer 8 interrupt handler actual implementation
 */
void __attribute__ ((used)) tim8impl() {
    TIM8->SR = 0; // Clear interrupt flags
        if (LED_flag == 0) {
            pwmSignal0::high();
            LED_flag = 1;
        } else {
            pwmSignal0::low();
            LED_flag = 0;
        }
    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * Timer 8 interrupt handler
 */
void __attribute__ ((naked)) TIM8_CC_IRQHandler() {
    saveContext();
    asm volatile("bl _Z8tim8implv");
    restoreContext();
}

/**
 * ADC 1 interrupt handler actual implementation
 */
void __attribute__ ((used)) adcimpl() {
    //ADC1->SR = 0; // Clear interrupt flags
    if (ADC1->SR & ADC_SR_EOC) {
        if (LED_flag == 0) {
            redLED_on();
            LED_flag = 1;
        } else {
            redLED_off();
            LED_flag = 0;
        }
    }

    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * ADC 1 interrupt handler
 */
void __attribute__ ((naked)) ADC_IRQHandler() {
    saveContext();
    asm volatile("bl _Z7adcimplv");
    restoreContext();
}

namespace miosix {

    //
    // class PMACdriver
    //

    PMACdriver& PMACdriver::instance() {
        static PMACdriver singleton;
        return singleton;
    }

    //PMACdriver::PMACdriver()/* : status(STOPPED) */{

    PMACdriver::PMACdriver() {
        {
            FastInterruptDisableLock dLock;
            // The RCC register should be written with interrupts disabled to
            // prevent race conditions with other threads.
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Servo motor drive
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // PMAC drive
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Control PMAC
            RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Timer for ADC Sampling
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC Clock
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // ADC Clock
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // ADC Clock
            //RCC->AHBENR |= RCC_AHBENR_DMA1EN;   // DMA Clock
            RCC_SYNC();
        }

        // Configure timer 1
        TIM1->CR1 = 0; // Control Register 1 = 0, reset everything related to this timer
        TIM1->CCR1 = 0; // Capture Compare register = 0, 
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;

        // Configure interrupt on timer 1 overflow
        TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE;
        NVIC_SetPriority(TIM1_CC_IRQn, 14); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM1_CC_IRQn);

        // Initialize motor driver specifics
        setupHallSensors();
        enableGate::mode(Mode::OUTPUT);
        faultPin::mode(Mode::INPUT); // Doesn't have a handler yet
        disableDriver(); // Must be enabled later to drive the power MOS gates
        setupControlTimer(CONTROL_TIMER_FREQUENCY);
        setupADC();
        setupADCTimer();    // Timer 8
        //setupTimer3(1000);
        //dutyCycleTimer3(.1);
        pwmSignal0::mode(Mode::OUTPUT);
    }

    void PMACdriver::setDrivingFrequency(unsigned int PWM_frequency) {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call

        uint32_t TIMER_Frequency = SystemCoreClock; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PWM_frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to

        TIM1->PSC = PSC_Value;
        TIM1->ARR = ARR_Value;
    }

    void PMACdriver::enable() {
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

            TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 
                    | TIM_CCMR1_OC1M_1 
                    | TIM_CCMR1_OC1PE;
            TIM1->CCER |= TIM_CCER_CC1E;
            pwmSignalH3::alternateFunction(1);
            pwmSignalH3::mode(Mode::ALTERNATE);

            TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 
                    | TIM_CCMR1_OC2M_1 
                    | TIM_CCMR1_OC2PE;
            TIM1->CCER |= TIM_CCER_CC2E;
            pwmSignalH2::alternateFunction(1);
            pwmSignalH2::mode(Mode::ALTERNATE);

            TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 
                    | TIM_CCMR2_OC3M_1 
                    | TIM_CCMR2_OC3PE;
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

    void PMACdriver::disable() {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call
        {
            FastInterruptDisableLock dLock;
            // Calling the mode() function on a GPIO is subject to race conditions
            // between threads on the STM32, so we disable interrupts

            disableDriver();
            
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

    void PMACdriver::start() {
        //Lock<FastMutex> l(mutex);
        //if (status != STOPPED) return; // If timer enabled ignore the call

        // While status is starting neither member function callable with timer
        // started nor stopped are allowed
        //status = STARTED;

        TIM1->CNT = 0; // Start counter value from 0
        TIM1->EGR = TIM_EGR_UG; // Event Generation Register = Update generation
        TIM1->BDTR = TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM1->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable
    }

    void PMACdriver::stop() {
        //Lock<FastMutex> l(mutex);
        //if (status != STARTED) return; // If timer disabled ignore the call
        //status = STOPPED;
        // Erase the value in the capture/compare registers
        //TIM3->CCR2 = 0;
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
        //TIM3->CR1 = 0;
        TIM1->CR1 = 0;
    }

    void PMACdriver::setHighSideWidth(char channel, float pulseWidth) {
        //Lock<FastMutex> l(mutex);
        //if (status != STARTED) return; // If timer disabled ignore the call
        switch (channel) {
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

    void PMACdriver::setLowSide(char channel, bool value) {
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

    /*bool PMACdriver::waitForCycleBegin() {
        // No need to lock the mutex because disabling interrupts is enough to avoid
        // race conditions. Also, locking the mutex here would prevent other threads
        // from calling other member functions of this class
        FastInterruptDisableLock dLock;
        if (status != STARTED) return true;
        IRQwaitForTimerOverflow(dLock);
        return status != STARTED;
    }*/

    void PMACdriver::setupHallSensors() {
        hallSensor1::mode(Mode::INPUT);
        hallSensor2::mode(Mode::INPUT);
        hallSensor3::mode(Mode::INPUT);
    }

    void PMACdriver::updateHallEffectSensorsValue() {
        hallEffectSensors_oldPosition = hallEffectSensors_newPosition;
        hallEffectSensors_newPosition = (hallSensor1::value() << 0) | (hallSensor2::value() << 1) | (hallSensor3::value() << 2);
    }

    char PMACdriver::getHallEffectSensorsValue() {
        updateHallEffectSensorsValue();
        return hallEffectSensors_newPosition;
    }

    void PMACdriver::enableDriver() {
        motorRunning = 1;
        enableGate::high();
    }

    void PMACdriver::disableDriver() {
        motorRunning = 0;
        enableGate::low();
    }

    void PMACdriver::updateFaultFlag() {
        faultFlag = faultPin::value();
    }

    bool PMACdriver::getFaultFlag() {
        updateFaultFlag();
        return faultFlag;
    }

    int PMACdriver::trapezoidalDrive() {
        updateHallEffectSensorsValue();
        calculateSpeed();
        if (hallEffectSensors_oldPosition != hallEffectSensors_newPosition) {
            allGatesLow();
            if (vDirection == CW) {
                if (hallEffectSensors_newPosition == 0b001) {
                    setLowSide(1, 1);
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b101) {
                    setLowSide(1, 1);
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b100) {
                    setLowSide(2, 1);
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b110) {
                    setLowSide(2, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b010) {
                    setLowSide(3, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b011) {
                    setLowSide(3, 1);
                    setHighSideWidth(2, vDutyCycle);
                }
            } else if (vDirection == CCW) {
                if (hallEffectSensors_newPosition == 0b001) {
                    setLowSide(2, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b101) {
                    setLowSide(3, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b100) {
                    setLowSide(3, 1);
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b110) {
                    setLowSide(1, 1);
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b010) {
                    setLowSide(1, 1);
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b011) {
                    setLowSide(2, 1);
                    setHighSideWidth(3, vDutyCycle);
                }
            }
            return 1;
        } else {
            return 0;
        }
    }

    void PMACdriver::allGatesLow() {
        highSideGatesLow();
        lowSideGatesLow();
    }

    void PMACdriver::highSideGatesLow() {
        setHighSideWidth(1, 0);
        setHighSideWidth(2, 0);
        setHighSideWidth(3, 0);
    }

    void PMACdriver::lowSideGatesLow() {
        setLowSide(1, 0);
        setLowSide(2, 0);
        setLowSide(3, 0);
    }

    void PMACdriver::setupControlTimer(unsigned int frequency) {
        // Initialize Timer 2 to apply the trapezoidal drive @ 50kHz
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
        TIM2->PSC = PSC_Value; //  Prescaler
        TIM2->ARR = ARR_Value;
        TIM2->CNT = 0;
        TIM2->CR1 = TIM_CR1_CEN;
    }

    void PMACdriver::setupADCTimer() {
        TIM8->CR1 = 0; // Erase Control Register 1 to set it up
        TIM8->DIER = TIM_DIER_UIE; // DMA/interrupt enable register: Update interrupt enable
        TIM8->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
        NVIC_SetPriority(TIM8_CC_IRQn, 17); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM8_CC_IRQn);
        // capture/compare register 1:         
        TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
        TIM8->CCER |= TIM_CCER_CC1E;

        TIM8->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
        TIM8->CCER |= TIM_CCER_CC2E;

        uint32_t TIMER_Frequency = SystemCoreClock; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PMAC_PWM_FREQUENCY/10; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to
        TIM8->PSC = PSC_Value;
        TIM8->ARR = ARR_Value;

        // TIM1 Master and TIM8 slave
        TIM1->CR2 |= TIM_CR2_MMS_1;
        TIM1->SMCR |= TIM_SMCR_MSM;
        // TIM8->SMCR |= 0; //TIM_TS_ITR0;
        TIM8->SMCR |= TIM_SMCR_SMS_2;

        TIM8->CNT = 0; // Start counter value from 0
        TIM8->EGR = TIM_EGR_UG; // Event Generation Register = Update generation
        TIM8->BDTR = TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM8->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable
        
        TIM8->CCR2 = .1 * PWM_RESOLUTION;

    }

    void PMACdriver::setupADC() {
        // http://en.radzio.dxp.pl/stm32vldiscovery/lesson9,adc,with,direct,memory,access,dma.html

        // uint16_t AIN[4]; // 4 locations

        phaseVoltage3_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 0
        phaseVoltage2_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 1
        phaseVoltage1_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 2
        boardTemperature_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 3
        currentSense2_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 8
        currentSense1_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 9
        motorTemperature_ADC::mode(Mode::INPUT_ANALOG); // ADC channel 10
        // externalAnalogSignal_ADC::mode(Mode::INPUT_ANALOG);   // ADC channel 12


        ADC1->CR2 = ADC_CR2_ADON | // turn on ADC 
                ADC_CR2_CONT | // enable continuos mode
                ADC_CR2_DMA; // enable DMA mode
        ADC1->CR1 = ADC_CR1_SCAN; // enable scan mode

        //ADC1->SMPR2 = ADC_SMPR2_SMP3_0; //Sample for 15 cycles channel 3 (temperature)
        //ADC1->SQR1 = 0; //Do only one conversion
        //ADC1->SQR2 = 0;
        //ADC1->SQR3 = ADC_SQR3_SQ1_3; //Convert channel 2 (battery voltage)

        ADC1->SQR1 = ADC_SEQUENCE_LENGTH(3); // four channels in sequence
        ADC1->SQR3 = ADC_SEQ1(0) | // channel 0 is first in sequence
                ADC_SEQ2(1) | // channel 1 is second in sequence
                ADC_SEQ3(2) | // channel 2 is third in sequence
                ADC_SEQ4(3); // channel 3 is fourth in sequence
        ADC1->SMPR2 = ADC_SAMPLE_TIME0(SAMPLE_TIME_239_5) | // sample time for first channel in sequence
                ADC_SAMPLE_TIME1(SAMPLE_TIME_239_5) | // sample time for second channel in sequence
                ADC_SAMPLE_TIME2(SAMPLE_TIME_239_5) | // sample time for third channel in sequence
                ADC_SAMPLE_TIME3(SAMPLE_TIME_239_5); // sample time for fourth channel in sequence

        // DMA setup

        /*DMA1_Channel1->CPAR  = (uint32_t)(&(ADC1->DR));	// peripheral (source) address
        DMA1_Channel1->CMAR  = (uint32_t)AIN;	          // memory (desination) address
        DMA1_Channel1->CNDTR = 4;	                      // 4 transfers

        DMA1_Channel1->CCR |= DMA_CCR1_CIRC |    // circular mode enable
                              DMA_CCR1_MINC |    // memory increment mode enable
                              DMA_CCR1_MSIZE_0 | // memory size 16 bits
                              DMA_CCR1_PSIZE_0;  // peripheral size 16 bits

        DMA1_Channel1->CCR |= DMA_CCR1_EN ;	// Enable channel*/
    }

    int PMACdriver::getBatteryVoltage() {
        if (LED_flag == 0) {
            redLED_on();
            LED_flag = 1;
        } else {
            redLED_off();
            LED_flag = 0;
        }
        ADC1->CR2 = ADC_CR2_ADON; //Turn ADC ON
        Thread::sleep(5); //Wait for voltage to stabilize
        ADC1->CR2 |= ADC_CR2_SWSTART; //Start conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0); //Wait for conversion
        int result = ADC1->DR; //Read result
        ADC1->CR2 = 0; //Turn ADC OFF
        return result;
    }

    void PMACdriver::changeDutyCycle(float dutyCycle) {
        vDutyCycle = dutyCycle;
    }

    void PMACdriver::changeDirection(float direction) {
        vDutyCycle = direction;
    }

    void PMACdriver::calculateSpeed() {
        if (hallEffectSensors_oldPosition == hallEffectSensors_newPosition) {
            speedCounter++; // this will sum every 1/CONTROL_TIMER_FREQUENCY = 20us
            // therefore, 1 rev every speedCounter*/CONTROL_TIMER_FREQUENCY
        }
        /*if (direction == CW...)*/
        if ((hallEffectSensors_newPosition == 0b001)&&(hallEffectSensors_oldPosition == 0b101)) {
            // lets do this calculations in a thread...
            speedDPS = 360 * CONTROL_TIMER_FREQUENCY / (((float) speedCounter)*(MOTOR_POLE_PAIRS / 2));
            speedRPM = speedDPS * 60 / 360;
            speedCounter = 0;
        }
    }

    float PMACdriver::getSpeed(char type) {
        if (type == 0) return speedDPS;
        else if (type == 1) return speedRPM;
        else return 0;
    }

    char PMACdriver::getMotorStatus() {
        return motorRunning;
    }

    void PMACdriver::setupTimer3(int frequency) {
        // Configure timer 3
        TIM3->CR1 = 0;
        TIM3->CCR2 = 0;
        // Configure interrupt on timer 3 overflow
        TIM3->DIER = TIM_DIER_UIE;
        NVIC_SetPriority(TIM3_IRQn, 16); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM3_IRQn);
        
        uint32_t TIMER_Frequency = SystemCoreClock / 2; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to

        TIM3->PSC = PSC_Value;
        TIM3->ARR = ARR_Value;
        
        
        TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
        TIM3->CCER |= TIM_CCER_CC2E;
        pwmSignal0::alternateFunction(2);
        pwmSignal0::mode(Mode::ALTERNATE);
        
        // Start Timer 3 PWM
        TIM3->CNT = 0;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->CR1 = TIM_CR1_CEN;
    }

    void PMACdriver::dutyCycleTimer3(float dutyCycle) {
        TIM3->CCR2 = dutyCycle * PWM_RESOLUTION;
    }

    void PMACdriver::IRQwaitForTimerOverflow(FastInterruptDisableLock& dLock) {
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