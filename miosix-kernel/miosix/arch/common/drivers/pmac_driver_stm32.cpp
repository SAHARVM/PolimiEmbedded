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
typedef Gpio<GPIOC_BASE, 1> externalAnalogSignal_ADC; // ADC channel 12

typedef Gpio<GPIOA_BASE, 5> pin5_SPI; // SPI pin x
typedef Gpio<GPIOA_BASE, 6> pin6_SPI; // SPI pin x

static Thread *waiting = 0;

char hallEffectSensors_newPosition = 0;
char hallEffectSensors_oldPosition = 0;

float vDutyCycle = 0;
bool vDirection = CCW;
int speedCounter = 0;

float hall_effect_sensors_frequency = 0;
float speed_radiansPerSecond_electric = 0;
float speed_degreesPerSecond_electric = 0;
float speed_RPM_electric = 0;
float speed_radiansPerSecond_mechanic = 0;
float speed_degreesPerSecond_mechanic = 0;
float speed_RPM_mechanic = 0;

bool motorRunning = 0;


int singleADC1value = 0;
int singleADC2value = 0;
bool currentReadingFlag_1 = 0;  // Channel 8
bool currentReadingFlag_2 = 0;  // Channel 9
float ADC_trigger1_point = 0.5;

int currentRead_D = 0;
int currentRead_1_D = 0;
int currentRead_avg = 0;
bool current_D_obtained = 0;
bool current_1_D_obtained = 0;

int currentBuffer_counter = 0;
float currentBuffer_value = 0;
float currentBuffer_average = 0;


uint16_t ADC1_DMA_singleValue = 0;
uint32_t ADC1_DMA_value[1];
uint32_t ADC2_DMA_value[1];
uint32_t ADC3_DMA_value[1];

bool LED_flag = 0;


/**
 * Timer 1 interrupt handler
 */
void TIM1_CC_IRQHandler() {
    TIM1->SR = 0; // Clear interrupt flags
}

/**
 * Timer 2 interrupt handler 
 */
void TIM2_IRQHandler()
{
    TIM2->SR = 0; // Clear interrupt flags
    if (PMACdriver::getMotorStatus()) PMACdriver::trapezoidalDrive();
}

/**
 * Timer 3 interrupt handler
 */
void TIM3_IRQHandler() {
    TIM3->SR = 0; // Clear interrupt flags
}

/**
 * Timer 8 interrupt handler
 */
void TIM8_CC_IRQHandler() {
    TIM8->SR = 0; // Clear interrupt flags
}

/**
 * ADC 1 interrupt handler actual implementation
 */
void __attribute__ ((used)) adcimpl() {
    if (ADC1->SR & ADC_SR_EOC) {
        if (currentReadingFlag_1) {
            currentRead_D = ADC1->DR;
            current_D_obtained = 1;
        } else {
            int dummy = ADC1->DR;
        }
        pwmSignal0::high();
    } else if (ADC1->SR & ADC_SR_JEOC) {
        ADC1->SR &= ~ADC_SR_JEOC;
        if (currentReadingFlag_1) {
            currentRead_1_D = ADC1->DR;
            current_1_D_obtained = 1;
        } else {
            int dummy = ADC1->DR;
        }
        pwmSignal0::low();
    }
    
    if (current_D_obtained && current_1_D_obtained){
        currentRead_avg = (currentRead_D + currentRead_1_D) / 2;
        
        currentBuffer_counter ++;
        currentBuffer_value += currentRead_avg;
        if (currentBuffer_counter >= 1){
            currentBuffer_counter = 0;
            currentBuffer_average = currentBuffer_value / 1;
            currentBuffer_value = 0;
        }
        current_D_obtained = 0;
        current_1_D_obtained = 0;
    }
    if (waiting == 0) return;
    waiting->IRQwakeup();
    if (waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
    waiting = 0;
}

/**
 * ADC interrupt handler
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
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1 Clock
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN; // ADC2 Clock
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN; // ADC3 Clock
            //RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // DMA Clock
            RCC_SYNC();
        }

        // Configure timer 1
        TIM1->CR1 = 0; // Control Register 1 = 0, reset everything related to this timer
        TIM1->CCR1 = 0; // Capture Compare register = 0, 
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 0; // TIM1 CC4 doesn't generate a PWM output signal in a pin, it's used for the ADC

        // Configure interrupt on timer 1 overflow
        TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
        NVIC_SetPriority(TIM1_CC_IRQn, 14); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM1_CC_IRQn);

        // Initialize motor driver specifics
        setupHallSensors();
        enableGate::mode(Mode::OUTPUT);
        faultPin::mode(Mode::INPUT); // Doesn't have a handler yet - TODO: IMPLEMENT HANDLER
        disableDriver(); // Must be enabled later to drive the power MOS gates
        setupControlTimer(CONTROL_TIMER_FREQUENCY); // TIM2
        setupADCTimer(); // TIM8 CC1 and CC2, also TIM1 CC4 must be setup 
        setupADC();
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

            TIM1->CCMR2 |= TIM_CCMR2_OC4M_2
                    | TIM_CCMR2_OC4M_1
                    | TIM_CCMR2_OC4PE;
            TIM1->CCER |= TIM_CCER_CC4E;
            // CC4 doesn't need an output pin

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
        // Erase the value in the capture/compare register
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
            currentReadingFlag_1 = 0;
            currentReadingFlag_2 = 0;
            if (vDirection == CW) {
                if (hallEffectSensors_newPosition == 0b001) {
                    setLowSide(1, 1);
                    currentReadingFlag_1 = 1;
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b101) {
                    setLowSide(1, 1);
                    currentReadingFlag_1 = 1;
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
                    currentReadingFlag_1 = 1;
                    setHighSideWidth(2, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b010) {
                    setLowSide(1, 1);
                    currentReadingFlag_1 = 1;
                    setHighSideWidth(3, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b011) {
                    setLowSide(2, 1);
                    setHighSideWidth(3, vDutyCycle);
                }
            }
            setADCTriggerPosition(vDutyCycle);
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
        // Initialize Timer 2 to apply the trapezoidal drive
        TIM2->CR1 = 0;
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->EGR = TIM_EGR_UG;
        NVIC_SetPriority(TIM2_IRQn, 15); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM2_IRQn);
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
        /*
         * ADC is synchronized with the timers as follows:
         *  TIM8 CC1 falling edge starts the conversion for the regular channels:
         *      - ADC1: phase voltage 3 | current 2 | motor temperature
         *      - ADC2: phase voltage 2 | current 1
         *      - ADC3: phase voltage 1 | board temperature | bus voltage
         *  TIM8 CC2 falling edge starts the conversion for the injected channels:
         *      - ADC2: current 2 and then current 1
         *  TIM1 CC4 falling edge starts the conversion for the injected channels:
         *      - ADC1: current 1 and then current 2
         */

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
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PMAC_PWM_FREQUENCY; // How many steps inside a Period
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
    }

    void PMACdriver::setADCTriggerPosition(float dutyCycle) {
        float firstTrigger = dutyCycle / 2;
        float secondTrigger = dutyCycle + ( ( 1 - dutyCycle ) / 2 );
        TIM8->CCR1 = firstTrigger * PWM_RESOLUTION;
        TIM1->CCR4 = secondTrigger * PWM_RESOLUTION;
        //float secondTrigger = ((3 * dutyCycle) + 1) / 4;
        //float thirdTrigger = (dutyCycle + 1) / 2;
        //
        //TIM8->CCR1 = secondTrigger * PWM_RESOLUTION;
        //TIM8->CCR2 = thirdTrigger * PWM_RESOLUTION;
        //TIM8->CCR1 = ADC_trigger1_point * PWM_RESOLUTION;
    }
    
    void PMACdriver::changeTriggerPoint(float value){
        ADC_trigger1_point = value;
    }
    
    void PMACdriver::setupADC() {
        
        /*Pins setup*/
        currentSense2_ADC::mode(Mode::INPUT_ANALOG); // ADC1,2 channel 8 - Left leg
        //currentSense1_ADC::mode(Mode::INPUT_ANALOG); // ADC1,2 channel 9 - Right leg
        
        /* Clear configuration */
        ADC1->CR1 = 0;
        
        /* Setup timer trigger */
        ADC1->CR2 |= ADC_CR2_EXTEN_1;                                           // External trigger enable for regular channels in falling edge
        ADC1->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3;    // TIM8 CC1 event for regular group
        ADC1->CR2 |= ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in falling edge
        //ADC1->CR2 |= 0; // TIM1 CC4 event for injected group
        
        /*Channel setup*/
        ADC1->SMPR2 |= ADC_SMPR2_SMP8_0;                    // 15 cycles for channel 9 - currentSense1_ADC (ADC1)
        ADC1->SQR1 |= 0;                                    // Regular channel sequence length = 1
        ADC1->SQR3 |= ADC_SQR3_SQ1_3;                       // First conversion is ADC 1 channel 8

        /* Injected channels setup */
        ADC1->JSQR |= 0; // Injected channel sequence length = 0
        ADC1->JSQR |= ADC_JSQR_JSQ1_3; // ADC1 1st injected conversion is channel 8: 01000 - Current sensor 2
        
        
        
        /*Interrupts setup*/
        ADC1->CR1 |= ADC_CR1_DISCEN;                                            // Discontinuous mode regular channels enabled
        ADC1->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC1->CR1 |= ADC_CR1_JEOCIE; // EOC interrupt enabled
        ADC1->CR1 |= ADC_CR1_EOCIE; // EOC interrupt enabled
        //ADC1->CR2 |= ADC_CR2_EOCS;
        NVIC_SetPriority(ADC_IRQn, ADC_IRQN_PRIORITY); //Low priority for timer IRQ
        NVIC_EnableIRQ(ADC_IRQn);
        
        ADC1->CR2 |= ADC_CR2_ADON; // A/D converter ON
    }
    
    int PMACdriver::getADCvalue(){
        //return singleADC1value;
        return currentBuffer_average;
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
        }
        if (((hallEffectSensors_newPosition == 0b001)&&(hallEffectSensors_oldPosition == 0b101)) ||
                ((hallEffectSensors_newPosition == 0b101)&&(hallEffectSensors_oldPosition == 0b001))) {
            
            hall_effect_sensors_frequency = CONTROL_TIMER_FREQUENCY / ( (float)speedCounter );
            
            speed_radiansPerSecond_electric = hall_effect_sensors_frequency * 2 * M_PI;
            speed_degreesPerSecond_electric = hall_effect_sensors_frequency * 360;
            speed_RPM_electric = hall_effect_sensors_frequency * 60;
                    
            speed_radiansPerSecond_mechanic = speed_radiansPerSecond_electric / MOTOR_POLE_PAIRS;
            speed_degreesPerSecond_mechanic = speed_degreesPerSecond_electric / MOTOR_POLE_PAIRS;
            speed_RPM_mechanic = speed_RPM_electric / MOTOR_POLE_PAIRS;
            
            speedCounter = 0;
        }
    }

    float PMACdriver::getSpeed(char type) {
        if (type == 0) return speed_degreesPerSecond_mechanic;
        
        else if (type == 1) return hall_effect_sensors_frequency;
        
        else if (type == 2) return speed_radiansPerSecond_electric;
        else if (type == 3) return speed_degreesPerSecond_electric;
        else if (type == 4) return speed_RPM_electric;
        
        else if (type == 5) return speed_radiansPerSecond_mechanic;
        else if (type == 6) return speed_degreesPerSecond_mechanic;
        else if (type == 7) return speed_RPM_mechanic;
        
        else return 0;
    }
    
    

    char PMACdriver::getMotorStatus() {
        return motorRunning;
    }

    void PMACdriver::setupTimer3(int frequency) {
        // This timer generates a PWM signal and it's setup in the board to drive a servomotor
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
    
    void PMACdriver::setupSPI(){
        /* Parameters */
        // Mode: Receiver Only Master
        // Frame Format: Motorola
        // Data Size: 16 bits
        // First bit: MSB First
        // Pre-scaler for Baud Rate: 256
        // Baud Rate: 328.125 KBits/s
        // Clock Polarity (CPOL): High
        // Clock Phase (CPHA): 1 Edge
        // CRC Calculation: Disabled
        // NSS Signal Type: Software
        
	// vAnglePosition_bits = pSPI;
	// vAnglePosition_bits &= ~0b1000000000000000;
	// vAnglePosition_bits = vAnglePosition_bits >> 1;
        
        /*
         Configuring the SPI in master mode
In the master configuration, the serial clock is generated on the SCK pin.
Procedure
1. Select the BR[2:0] bits to define the serial clock baud rate (see SPI_CR1 register).
2. Select the CPOL and CPHA bits to define one of the four relationships between the
data transfer and the serial clock (see Figure 248). This step is not required when the
TI mode is selected.
3. Set the DFF bit to define 8- or 16-bit data frame format
4. Configure the LSBFIRST bit in the SPI_CR1 register to define the frame format. This
step is not required when the TI mode is selected.
5. If the NSS pin is required in input mode, in hardware mode, connect the NSS pin to a
high-level signal during the complete byte transmit sequence. In NSS software mode,
set the SSM and SSI bits in the SPI_CR1 register. If the NSS pin is required in output
mode, the SSOE bit only should be set. This step is not required when the TI mode is
selected.
6. Set the FRF bit in SPI_CR2 to select the TI protocol for serial communications.
7. The MSTR and SPE bits must be set (they remain set only if the NSS pin is connected
to a high-level signal).
In this configuration the MOSI pin is a data output and the MISO pin is a data input.
 
Transmit sequence
The transmit sequence begins when a byte is written in the Tx Buffer.
The data byte is parallel-loaded into the shift register (from the internal bus) during the first
bit transmission and then shifted out serially to the MOSI pin MSB first or LSB first
depending on the LSBFIRST bit in the SPI_CR1 register. The TXE flag is set on the transfer
of data from the Tx Buffer to the shift register and an interrupt is generated if the TXEIE bit in
the SPI_CR2 register is set.
         * 
         * 
         * 
         * Configuring the SPI for half-duplex communication
         * 
         * 1 clock and 1 data wire (receive-only or transmit-only)
         * 
         * 1 clock and 1 unidirectional data wire (BIDIMODE=0)
         * In receive-only mode, the application can disable the SPI output function by setting the
RXONLY bit in the SPI_CR2 register. In this case, it frees the transmit IO pin (MOSI in
master mode or MISO in slave mode), so it can be used for other purposes.
         * To start the communication in receive-only mode, configure and enable the SPI:
• In master mode, the communication starts immediately and stops when the SPE bit is
cleared and the current reception stops. There is no need to read the BSY flag in this
mode. It is always set when an SPI communication is ongoing.
         * 
         * In unidirectional receive-only mode (BIDIMODE=0 and RXONLY=1)
– The sequence begins as soon as SPE=1
– Only the receiver is activated and the received data on the MISO pin are shifted in
serially to the 8-bit shift register and then parallel loaded into the SPI_DR register
(Rx buffer).
         * 
         * 
         * 
         * 
         * Unidirectional receive-only procedure (BIDIMODE=0 and RXONLY=1)
In this mode, the procedure can be reduced as described below (see Figure 257):
         * 1. Set the RXONLY bit in the SPI_CR2 register.
2. Enable the SPI by setting the SPE bit to 1:
a) In master mode, this immediately activates the generation of the SCK clock, and
data are serially received until the SPI is disabled (SPE=0).
b) In slave mode, data are received when the SPI master device drives NSS low and
generates the SCK clock.
3. Wait until RXNE=1 and read the SPI_DR register to get the received data (this clears
the RXNE bit). Repeat this operation for each data item to be received.
This procedure can also be implemented using dedicated interrupt subroutines launched at
each rising edge of the RXNE flag.
Note: If it is required to disable the SPI after the last transfer, follow the recommendation
described in Section 28.3.8: Disabling the SPI on page 898.
         */
        
        /* pins setup */
        pin5_SPI::alternateFunction(5);
        pin5_SPI::mode(Mode::ALTERNATE);
        pin6_SPI::alternateFunction(5);
        pin6_SPI::mode(Mode::ALTERNATE);
        
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