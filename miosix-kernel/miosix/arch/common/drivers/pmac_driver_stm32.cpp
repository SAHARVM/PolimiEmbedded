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

using namespace std;
using namespace miosix;

typedef Gpio<GPIOB_BASE, 5> pwmSignal0; // TIM3_CH2, this output can drive a servomotor, must now be allocated in another file maybe

typedef Gpio<GPIOA_BASE, 8> pwmSignalH3; // TIM1_CH1
typedef Gpio<GPIOA_BASE, 9> pwmSignalH2; // TIM1_CH2
typedef Gpio<GPIOA_BASE, 10> pwmSignalH1; // TIM1_CH3

#ifdef SINUSOIDAL_DRIVE
typedef Gpio<GPIOB_BASE, 13> pwmSignalL3; // TIM1_CH1N
typedef Gpio<GPIOB_BASE, 14> pwmSignalL2; // TIM1_CH2N
typedef Gpio<GPIOB_BASE, 15> pwmSignalL1; // TIM1_CH3N
#else
typedef Gpio<GPIOB_BASE, 13> outSignalL3;
typedef Gpio<GPIOB_BASE, 14> outSignalL2;
typedef Gpio<GPIOB_BASE, 15> outSignalL1;
#endif // SINUSOIDAL_DRIVE


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
typedef Gpio<GPIOC_BASE, 2> voltageBus_ADC; // ADC channel 12

typedef Gpio<GPIOA_BASE, 5> SPI1_SCK; // SPI SCK
typedef Gpio<GPIOA_BASE, 6> SPI1_MISO; // SPI MISO

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


bool currentReadingFlag_1 = 0; // Channel 8
bool currentReadingFlag_3 = 0; // Channel 9
bool regularChannelReadFlag = 0;
bool injectedChannelReadFlag = 0;

int currentRead_D_branch1 = 0;
int currentRead_1_D_branch1 = 0;
int currentRead_avg_branch1 = 0;
bool current_D_obtained_branch1 = 0;
bool current_1_D_obtained_branch1 = 0;

int injectedADC_CurrentBranch1 = 0;
int injectedADC_CurrentBranch3 = 0;

int currentRead_D_branch3 = 0;
int currentRead_1_D_branch3 = 0;
int currentRead_avg_branch3 = 0;
bool current_D_obtained_branch3 = 0;
bool current_1_D_obtained_branch3 = 0;

int currentBuffer_counter = 0;
float currentBuffer_value = 0;
float currentBuffer_average = 0;

int receivedAngularPosition = 0;
int modifiedAngularPosition_0 = 0;
int modifiedAngularPosition_1 = 0;
float rotorAngularPosition_bits = 0;

float electricalAngle_0 = 0;
float electricalAngle_1 = 0;
float mechanicalAngle_0 = 0;
float mechanicalAngle_1 = 0;
float deltaOrbisSensor_0 = 0;
float deltaOrbisSensor_1 = 0;

char hallEffectSensors_forAngularCalculation_0 = 0;
char hallEffectSensors_forAngularCalculation_1 = 0;

/* To be used for FOC */
float quadratureCurrent_reference = 0;
float directCurrent_reference = 0;
float quadratureCurrent_measured = 0;
float directCurrent_measured = 0;
float quadratureCurrent_error_0 = 0;
float quadratureCurrent_error_1 = 0;
float directCurrent_error_0 = 0;
float directCurrent_error_1 = 0;
float quadratureCurrent_integralError_0 = 0;
float quadratureCurrent_integralError_1 = 0;
float directCurrent_integralError_0 = 0;
float directCurrent_integralError_1 = 0;

float T = 1 / ((float) PMAC_PWM_FREQUENCY); // PWM Period
float theta = 0;
float i_qs_ref = 0;
float i_ds_ref = 0;
float U_alpha = 0;
float U_beta = 0;
float v_alpha = 0;
float v_beta = 0;
float v_qs = 0;
float v_ds = 0;
float v_a = 0;
float v_b = 0;
float v_c = 0;
float i_a = 0;
float i_b = 0;
float i_c = 0;
float i_alpha = 0;
float i_beta = 0;
float i_qs = 0;
float i_ds = 0;
float X = 0;
float Y = 0;
float Z = 0;
float time_phaseA = 0;
float time_phaseB = 0;
float time_phaseC = 0;
float p_term_q = 0;
float i_term_q = 0;
float p_term_d = 0;
float i_term_d = 0;
float dutyCycle_A = 0;
float dutyCycle_B = 0;
float dutyCycle_C = 0;

float testDutyCycle = 0;
float ADC_triggerPoint = 0;
float angularSlip = 0;

float kp_FOC = 0;
float ki_FOC = 0;
float kp_speed = 0;
float ki_speed = 0;

char deltaTheta = 0;
int theta_bits = 0;

float theta_rads = 0;
float theta_rads_1 = 0;

char faultFlag = 0;
bool LED_flag = 0;

float speedCalculationTimeout_Counter = 0;
float speedCalculationTimeout_prevValue = 0;

int busVoltage_bits = 0;
float busVoltage_volts = 24;

shuntCurrent_bits current_measured_bits;
shuntCurrent_amps current_measured_amps;

/**
 * Timer 2 interrupt handler 
 */
void TIM2_IRQHandler() {
    /* This interrupt handles the driving functions and runs at the frequency specified in CONTROL_TIMER_FREQUENCY */
    TIM2->SR = 0; // Clear interrupt flags
#ifdef SINUSOIDAL_DRIVE
    PMACdriver::calculateSpeed();
    PMACdriver::voltageBusCalculation();
    theta_rads = PMACdriver::getTheta();
    if (theta_rads >= 0){
        PMACdriver::fieldOrientedControl(theta_rads);
        PMACdriver::testSignal(1);
    }
    else 
        PMACdriver::testSignal(0);
#else 
    PMACdriver::trapezoidalDrive();
#endif // SINUSOIDAL_DRIVE
}

/**
 * ADC 1 interrupt handler actual implementation
 */
void __attribute__ ((used)) adcimpl() {

#ifdef SINUSOIDAL_DRIVE
    
#if 0
    if (ADC1->SR & ADC_SR_OVR)
        ADC1->SR &= ~ADC_SR_OVR;
    
    else if (ADC1->SR & ADC_SR_JEOC) {
        ADC1->SR &= ~ADC_SR_JEOC;
        injectedADC_CurrentBranch1 = ADC1->JDR1; // store ADC value
        injectedADC_CurrentBranch3 = ADC1->JDR2; // store ADC value
        current_measured_bits.current_branch_A_bits = injectedADC_CurrentBranch1;
        current_measured_bits.current_branch_C_bits = injectedADC_CurrentBranch3;
    }
#else
    
    if (ADC1->SR & ADC_SR_JEOC) {
        ADC1->SR &= ~ADC_SR_JEOC;
        injectedADC_CurrentBranch1 = ADC1->JDR1; // store ADC value
        current_measured_bits.current_branch_A_bits = injectedADC_CurrentBranch1;
    }
    if (ADC2->SR & ADC_SR_JEOC){
        ADC2->SR &= ~ADC_SR_JEOC;
        injectedADC_CurrentBranch3 = ADC2->JDR1; // store ADC value
        current_measured_bits.current_branch_C_bits = injectedADC_CurrentBranch3;
    }
    if (ADC3->SR & ADC_SR_JEOC){
        ADC3->SR &= ~ADC_SR_JEOC;
        busVoltage_bits = ADC3->JDR1; // store ADC value
    }
    
#endif
    
    
    
#else
    int dummy;
    if (ADC1->SR & ADC_SR_OVR) ADC1->SR &= ~ADC_SR_OVR;
    
        
    if (ADC1->SR & ADC_SR_JEOC) {
        ADC1->SR &= ~ADC_SR_JEOC;
        if (currentReadingFlag_1 == 1) { // if current is passing through branch 1
            injectedADC_CurrentBranch1 = ADC1->JDR1; // store ADC value
        } 
//        else {
//            dummy = ADC1->JDR1; // to clear DR
//        }
        if (currentReadingFlag_3 == 1) { // if current is passing through branch 3
            injectedADC_CurrentBranch3 = ADC1->JDR2; // store ADC value
        } 
//        else {
//            dummy = ADC1->JDR2; // to clear DR
//        }

        if (LED_flag) {
            pwmSignal0::high();
            LED_flag = 0;
        } else {
            pwmSignal0::low();
            LED_flag = 1;
        }
    }
    
#if 0
    if (ADC1->SR & ADC_SR_EOC) { // If it's a regular conversion, the current obtained is from 0 to D
        if (regularChannelReadFlag == 0) { // current read was from channel 8, current 1
            if (currentReadingFlag_1 == 1) { // if current is passing through branch 1 (not needed for FOC)
                currentRead_D_branch1 = ADC1->DR; // store ADC value
                current_D_obtained_branch1 = 1; // flag that reading 1/2 of channel 8 is obtained
            } else {
                dummy = ADC1->DR; // to clear DR
            }
        } else if (regularChannelReadFlag == 1) { // current read was from channel 9, current 3
            if (currentReadingFlag_3 == 1) { // if current is passing through branch 1 (not needed for FOC)
                currentRead_D_branch3 = ADC1->DR; // store ADC value
                current_D_obtained_branch3 = 1; // flag that reading 2/2 of channel 8 is obtained
            } else {
                dummy = ADC1->DR; // to clear DR
            }
        }

        if (regularChannelReadFlag == 0)
            regularChannelReadFlag = 1;
        //else 
        //   regularChannelReadFlag = 0;

    } 
    else if (ADC1->SR & ADC_SR_JEOC) { // If it's an injected conversion, the current obtained is from D to 1 
        regularChannelReadFlag = 0;
        ADC1->SR &= ~ADC_SR_JEOC; // clear interrupt flag (needed!!)
        if (currentReadingFlag_1 == 1) { // if current is passing through branch 1 (not needed for FOC)
            currentRead_1_D_branch1 = ADC1->JDR1; // store ADC value
            current_1_D_obtained_branch1 = 1; // flag that reading 1/2 of channel 8 is obtained
        } else {
            dummy = ADC1->JDR1; // to clear DR
        }
        if (currentReadingFlag_3 == 1) { // if current is passing through branch 1 (not needed for FOC)
            currentRead_1_D_branch3 = ADC1->JDR2; // store ADC value
            current_1_D_obtained_branch3 = 1; // flag that reading 2/2 of channel 8 is obtained
        } else {
            dummy = ADC1->JDR2; // to clear DR
        }
    }

    if (current_D_obtained_branch1 && current_1_D_obtained_branch1) {
        currentRead_avg_branch1 = (currentRead_D_branch1 + currentRead_1_D_branch1) / 2;
        current_D_obtained_branch1 = 0;
        current_1_D_obtained_branch1 = 0;
    }

    if (current_D_obtained_branch3 && current_1_D_obtained_branch3) {
        currentRead_avg_branch3 = (currentRead_D_branch3 + currentRead_1_D_branch3) / 2;
        current_D_obtained_branch3 = 0;
        current_1_D_obtained_branch3 = 0;
    }
#endif // if 0
#endif // SINUSOIDAL_DRIVE
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
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // SPI1 Clock
#ifdef DMA
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // DMA Clock
#endif
            RCC_SYNC();
        }
        // Configure timer 1
        TIM1->CR1 = 0;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 0; // TIM1 CC4 doesn't generate a PWM output signal in a pin, it's used for the ADC
#ifdef SINUSOIDAL_DRIVE
        TIM1->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 2 (flags set when counting up)
#endif  // SINUSOIDAL_DRIVE
        //TIM1->CR2 |= TIM_CR2_MMS_0;                     // Master mode: enable
        TIM1->CR2 |= TIM_CR2_MMS_1; // Master mode: update
        //TIM1->CR2 |= TIM_CR2_MMS_2;                     // Master mode: OC1REF used as trigger
        TIM1->SMCR |= TIM_SMCR_MSM; // Master/slave mode

        /* Initialize motor driver specifics */
        setupHallSensors();
        enableGate::mode(Mode::OUTPUT); // TODO: Find a better place for this
        faultPin::mode(Mode::INPUT); // Doesn't have a handler yet - TODO: IMPLEMENT HANDLER
        disableDriver(); // Must be enabled later to drive the power MOS gates
        setupControlTimer(CONTROL_TIMER_FREQUENCY); // TIM2 - Runs the control algorithm
#ifdef SINUSOIDAL_DRIVE
        setupADCTimer(); // TIM8 CC1 and CC2, also TIM1 CC4 must be setup 
        setupADC();
#endif
        setupSPI();
        pwmSignal0::mode(Mode::OUTPUT);
    }

    void PMACdriver::setDrivingFrequency(unsigned int PWM_frequency) {
        uint32_t TIMER_Frequency = SystemCoreClock; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PWM_frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to

        TIM1->PSC = PSC_Value;
        TIM1->ARR = ARR_Value;
    }

    void PMACdriver::enable() {
        {
            FastInterruptDisableLock dLock;

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
            TIM1->CCER |= TIM_CCER_CC1E
                    | TIM_CCER_CC1NE;
            pwmSignalH3::alternateFunction(1);
            pwmSignalH3::mode(Mode::ALTERNATE);

            TIM1->CCMR1 |= TIM_CCMR1_OC2M_2
                    | TIM_CCMR1_OC2M_1
                    | TIM_CCMR1_OC2PE;
            TIM1->CCER |= TIM_CCER_CC2E
                    | TIM_CCER_CC2NE;
            pwmSignalH2::alternateFunction(1);
            pwmSignalH2::mode(Mode::ALTERNATE);

            TIM1->CCMR2 |= TIM_CCMR2_OC3M_2
                    | TIM_CCMR2_OC3M_1
                    | TIM_CCMR2_OC3PE;
            TIM1->CCER |= TIM_CCER_CC3E
                    | TIM_CCER_CC3NE;
            pwmSignalH1::alternateFunction(1);
            pwmSignalH1::mode(Mode::ALTERNATE);

#ifdef SINUSOIDAL_DRIVE
            pwmSignalL1::alternateFunction(1);
            pwmSignalL1::mode(Mode::ALTERNATE);
            pwmSignalL2::alternateFunction(1);
            pwmSignalL2::mode(Mode::ALTERNATE);
            pwmSignalL3::alternateFunction(1);
            pwmSignalL3::mode(Mode::ALTERNATE);
#else
            outSignalL3::mode(Mode::OUTPUT);
            outSignalL3::low();
            outSignalL2::mode(Mode::OUTPUT);
            outSignalL2::low();
            outSignalL1::mode(Mode::OUTPUT);
            outSignalL1::low();
#endif  // SINUSOIDAL_DRIVE

            TIM1->CCMR2 |= TIM_CCMR2_OC4M_2
                    | TIM_CCMR2_OC4M_1
                    | TIM_CCMR2_OC4PE;
            TIM1->CCER |= TIM_CCER_CC4E;
            // CC4 doesn't need an output pin
        }
    }

    void PMACdriver::disable() {
        {
            FastInterruptDisableLock dLock;
            disableDriver();
            pwmSignalH1::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC1E;
            pwmSignalH2::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC2E;
            pwmSignalH3::mode(Mode::INPUT);
            TIM1->CCER &= ~TIM_CCER_CC3E;
#ifndef SINUSOIDAL_DRIVE
            outSignalL3::mode(Mode::INPUT);
            outSignalL2::mode(Mode::INPUT);
            outSignalL1::mode(Mode::INPUT);
#endif  // SINUSOIDAL_DRIVE
        }
    }

    void PMACdriver::start() {
        TIM1->CNT = 0; // Start counter value from 0
        TIM1->EGR |= TIM_EGR_TG | TIM_EGR_UG; // Event Generation Register = Update generation
#ifdef SINUSOIDAL_DRIVE
        TIM1->BDTR |= 0
                //| TIM_BDTR_DTG_0
                //| TIM_BDTR_DTG_1
                //| TIM_BDTR_DTG_2
                //| TIM_BDTR_DTG_3
                //| TIM_BDTR_DTG_4;
                | TIM_BDTR_DTG_5;
                //| TIM_BDTR_DTG_6;
                //| TIM_BDTR_DTG_7; // dead time
#endif
        TIM1->BDTR |= TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM1->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable
    }

    void PMACdriver::stop() {
        disableDriver();
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        {
            FastInterruptDisableLock dLock;
            if (waiting) waiting->IRQwakeup();
            IRQwaitForTimerOverflow(dLock);
        }
        TIM1->CR1 = 0;
    }

    void PMACdriver::setHighSideWidth(char channel, float pulseWidth) {
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
#ifndef SINUSOIDAL_DRIVE
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
#endif

    }

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
        /* TODO: Set up a HMI handler for this flag!! */
        faultFlag = faultPin::value();
    }

    char PMACdriver::getFaultFlag() {
        updateFaultFlag();
        return faultFlag;
    }

    int PMACdriver::trapezoidalDrive() {
#ifndef SINUSOIDAL_DRIVE
        updateHallEffectSensorsValue();
        calculateSpeed();
        if ((hallEffectSensors_oldPosition != hallEffectSensors_newPosition)||(getSpeed(5) < 20)) {
            allGatesLow();
            currentReadingFlag_1 = 0;
            currentReadingFlag_3 = 0;
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
                    currentReadingFlag_3 = 1;
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b011) {
                    setLowSide(3, 1);
                    currentReadingFlag_3 = 1;
                    setHighSideWidth(2, vDutyCycle);
                }
            } else if (vDirection == CCW) {
                if (hallEffectSensors_newPosition == 0b001) {
                    setLowSide(2, 1);
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b101) {
                    setLowSide(3, 1);
                    currentReadingFlag_3 = 1;
                    setHighSideWidth(1, vDutyCycle);
                } else if (hallEffectSensors_newPosition == 0b100) {
                    setLowSide(3, 1);
                    currentReadingFlag_3 = 1;
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
            setADCTriggerPosition(1,vDutyCycle);
            return 1;
        } else {
            return 0;
        }
#else
        return 0;
#endif
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
#ifndef SINUSOIDAL_DRIVE
        setLowSide(1, 0);
        setLowSide(2, 0);
        setLowSide(3, 0);
#endif
    }

    void PMACdriver::setupControlTimer(unsigned int frequency) {
        // Initialize Timer 2 to apply the driving algorithm
        TIM2->CR1 = 0;
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->EGR = TIM_EGR_UG;
        NVIC_SetPriority(TIM2_IRQn, TIM2_IRQn_PRIORITY); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM2_IRQn);
        uint32_t TIMER_Frequency = SystemCoreClock / 2; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * frequency; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to
        TIM2->PSC = PSC_Value;
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
#ifdef SINUSOIDAL_DRIVE
        TIM8->CR1 = 0; // Erase register to set it up

        TIM8->CR1 |= TIM_CR1_CMS_0; // Center-aligned mode 1

        TIM8->CCMR1 |= TIM_CCMR1_OC1M_2
                | TIM_CCMR1_OC1M_1
                | TIM_CCMR1_OC1PE;
        TIM8->CCER |= TIM_CCER_CC1E;

        TIM8->CCMR1 |= TIM_CCMR1_OC2M_2
                | TIM_CCMR1_OC2M_1
                | TIM_CCMR1_OC2PE;
        TIM8->CCER |= TIM_CCER_CC2E;

        uint32_t TIMER_Frequency = SystemCoreClock; // From the data sheet
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PMAC_PWM_FREQUENCY; // How many steps inside a Period
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1; // Pre-scaler
        uint16_t ARR_Value = PWM_RESOLUTION - 1; // Top value to count to
        TIM8->PSC = PSC_Value;
        TIM8->ARR = ARR_Value;

        TIM8->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2; // Trigger mode

        TIM8->CNT = 0; // Start counter value from 0
        TIM8->EGR = TIM_EGR_UG; // Event Generation Register = Update generation
        TIM8->BDTR = TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM8->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable

#else
        TIM8->CR1 = 0; // Erase Control Register 1 to set it up
        TIM8->DIER = TIM_DIER_UIE; // DMA/interrupt enable register: Update interrupt enable
        TIM8->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
        NVIC_SetPriority(TIM8_CC_IRQn, TIM8_CC_IRQn_PRIORITY); //Low priority for timer IRQ
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
        TIM8->SMCR |= TIM_SMCR_SMS_2;

        TIM8->CNT = 0; // Start counter value from 0
        TIM8->EGR = TIM_EGR_UG; // Event Generation Register = Update generation
        TIM8->BDTR = TIM_BDTR_MOE; // Advanced timers need to have the main output enabled after CCxE is set
        TIM8->CR1 = TIM_CR1_CEN; // Counter Register 1 = Counter Enable
#endif
    }

    void PMACdriver::setADCTriggerPosition(char channel, float dutyCycle) {
#ifdef SINUSOIDAL_DRIVE
        TIM1->CCR4 = 0.99 * PWM_RESOLUTION;
        //TIM1->CCR4 = PWM_RESOLUTION;
        //TIM8->CCR2 = dutyCycle * ADC_triggerPoint * PWM_RESOLUTION;

#else
        float firstTrigger = dutyCycle / 2;
        float secondTrigger = dutyCycle + ((1 - dutyCycle) / 2);
        //TIM8->CCR1 = firstTrigger * PWM_RESOLUTION;
        if (dutyCycle > 0.5) TIM1->CCR4 = firstTrigger * PWM_RESOLUTION;
        else if (dutyCycle <= 0.5) TIM1->CCR4 = secondTrigger * PWM_RESOLUTION;
        
#endif
    }

    void PMACdriver::setupADC() {
        /*Pins setup*/
        currentSense2_ADC::mode(Mode::INPUT_ANALOG);    // ADC1,2 channel 8 - Left leg - Working good
        currentSense1_ADC::mode(Mode::INPUT_ANALOG);    // ADC1,2 channel 9 - Right leg - Configuring...
        voltageBus_ADC::mode(Mode::INPUT_ANALOG);       // ADC3 channel 12 - Bus Voltage
        /* Clear configuration */
        ADC1->CR1 = 0;
        ADC2->CR1 = 0;
        ADC3->CR1 = 0;

#ifdef SINUSOIDAL_DRIVE
#if 0
        /* Setup timer trigger */
        //ADC1->CR2 |= ADC_CR2_EXTEN_1; // External trigger enable for regular channels in falling edge
        //ADC1->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3; // TIM8 CC1 event for regular group
        //ADC1->CR2 |= ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in falling edge
        ADC1->CR2 |= ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in rising/falling edge
        ADC1->CR2 |= 0; // TIM1 CC4 event for injected group
        //ADC1->CR2 |= ADC_CR2_JEXTSEL_2 | ADC_CR2_JEXTSEL_3; // TIM8 CC2 event for injected group

        /*Channel setup*/
        ADC1->SMPR2 |= ADC_SMPR2_SMP8_0; // 15 cycles for channel 8 - currentSense2_ADC (ADC1)
        ADC1->SMPR2 |= ADC_SMPR2_SMP9_0; // 15 cycles for channel 9 - currentSense1_ADC (ADC1)
        //ADC1->SQR1 |= ADC_SQR1_L_0; // Regular channel sequence length = 1+1 = 2
        //ADC1->SQR3 |= ADC_SQR3_SQ1_3; // First conversion is ADC 1 channel 8
        //ADC1->SQR3 |= ADC_SQR3_SQ2_0 | ADC_SQR3_SQ2_3; // Second conversion is ADC 1 channel 9

        /* Injected channels setup */
        ADC1->JSQR |= ADC_JSQR_JL_0; // Injected channel sequence length = 1+1
        ADC1->JSQR |= ADC_JSQR_JSQ3_3; // ADC1 1st injected conversion is channel 8: 01000 - Current sensor 2
        ADC1->JSQR |= ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_3; // ADC1 2nd injected conversion is channel 9: 01001 - Current sensor 1

        /*Interrupts setup*/
        //ADC1->CR1 |= ADC_CR1_DISCEN; // Discontinuous mode regular channels enabled
        ADC1->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC1->CR1 |= ADC_CR1_SCAN;
        ADC1->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        //ADC1->CR1 |= ADC_CR1_EOCIE; // EOC interrupt enabled
        ADC1->CR1 |= ADC_CR1_OVRIE; // Overrun interrupt enabled
        ADC1->CR2 |= ADC_CR2_EOCS;
        NVIC_SetPriority(ADC_IRQn, ADC_IRQN_PRIORITY); //Low priority for ADC IRQ
        NVIC_EnableIRQ(ADC_IRQn);
        ADC1->CR2 |= ADC_CR2_ADON; // A/D converter ON
#endif
#if 0
        /* Setup timer trigger */
        ADC1->CR2 |= ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in rising/falling edge
        ADC1->CR2 |= 0; // TIM1 CC4 event for injected group 
        /*Channel setup*/
        ADC1->SMPR2 |= ADC_SMPR2_SMP8_0; // 15 cycles for channel 8 - currentSense2_ADC (ADC1)
        ADC1->SMPR2 |= ADC_SMPR2_SMP9_0; // 15 cycles for channel 9 - currentSense1_ADC (ADC1)
        /* Injected channels setup */
        ADC1->JSQR |= ADC_JSQR_JL_0; // Injected channel sequence length = 1+1
        ADC1->JSQR |= ADC_JSQR_JSQ3_3; // ADC1 1st injected conversion is channel 8: 01000 - Current sensor 2
        ADC1->JSQR |= ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_3; // ADC1 2nd injected conversion is channel 9: 01001 - Current sensor 1
        /*Interrupts setup*/
        ADC1->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC1->CR1 |= ADC_CR1_SCAN;
        ADC1->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        ADC1->CR1 |= ADC_CR1_OVRIE; // Overrun interrupt enabled
        ADC1->CR2 |= ADC_CR2_EOCS;
        NVIC_SetPriority(ADC_IRQn, ADC_IRQN_PRIORITY); //Low priority for ADC IRQ
        NVIC_EnableIRQ(ADC_IRQn);
        ADC1->CR2 |= ADC_CR2_ADON; // A/D converter ON
#else
        /* Triple ADC mode */
        ADC->CCR |= ADC_CCR_MULTI_0 | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_4;
        
        /* Setup timer trigger - All ADCs follow ADC1 */
        ADC1->CR2 |= ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in rising/falling edge
        ADC1->CR2 |= 0; // TIM1 CC4 event for injected group
        
        /*Channel setup*/
        ADC1->SMPR2 |= ADC_SMPR2_SMP8_0; // 15 cycles for channel 8 - currentSense2_ADC (ADC1)
        ADC2->SMPR2 |= ADC_SMPR2_SMP9_0; // 15 cycles for channel 9 - currentSense1_ADC (ADC2)
        ADC3->SMPR1 |= ADC_SMPR1_SMP12_0; // 15 cycles for channel 12 - voltageBus_ADC (ADC3)
        
        /* Injected channels setup */
        ADC1->JSQR |= ADC_JSQR_JSQ4_3;                      // ADC1 1st injected conversion is channel 8: 01000 - Current sensor 2
        ADC2->JSQR |= ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_3;    // ADC2 1st injected conversion is channel 9: 01001 - Current sensor 1
        ADC3->JSQR |= ADC_JSQR_JSQ4_2 | ADC_JSQR_JSQ4_3;    // ADC2 1st injected conversion is channel 12: 01100 - Voltage Bus
        
        /*Interrupts setup*/
        ADC1->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC2->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC3->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC1->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        ADC2->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        ADC3->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        //ADC1->CR1 |= ADC_CR1_OVRIE; // Overrun interrupt enabled
        ADC1->CR2 |= ADC_CR2_EOCS;
        ADC2->CR2 |= ADC_CR2_EOCS;
        ADC3->CR2 |= ADC_CR2_EOCS;
        NVIC_SetPriority(ADC_IRQn, ADC_IRQN_PRIORITY); //Low priority for ADC IRQ
        NVIC_EnableIRQ(ADC_IRQn);
        
        ADC1->CR2 |= ADC_CR2_ADON; // A/D converter ON
        ADC2->CR2 |= ADC_CR2_ADON; // A/D converter ON
        ADC3->CR2 |= ADC_CR2_ADON; // A/D converter ON
#endif
#else   // SINUSOIDAL_DRIVE
        /* Setup timer trigger */
        //ADC1->CR2 |= ADC_CR2_EXTEN_1; // External trigger enable for regular channels in falling edge
        //ADC1->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3; // TIM8 CC1 event for regular group
        ADC1->CR2 |= ADC_CR2_JEXTEN_1; // External trigger enable for injected channels in falling edge
        ADC1->CR2 |= 0; // TIM1 CC4 event for injected group

        /*Channel setup*/
        ADC1->SMPR2 |= ADC_SMPR2_SMP8_0; // 15 cycles for channel 8 - currentSense2_ADC (ADC1)
        ADC1->SMPR2 |= ADC_SMPR2_SMP9_0; // 15 cycles for channel 9 - currentSense1_ADC (ADC1)
        //ADC1->SQR1 |= ADC_SQR1_L_0; // Regular channel sequence length = 1+1 = 2
        //ADC1->SQR3 |= ADC_SQR3_SQ1_3; // First conversion is ADC 1 channel 8
        //ADC1->SQR3 |= ADC_SQR3_SQ2_0 | ADC_SQR3_SQ2_3; // Second conversion is ADC 1 channel 9

        /* Injected channels setup */
        ADC1->JSQR |= ADC_JSQR_JL_0; // Injected channel sequence length = 1+1
        ADC1->JSQR |= ADC_JSQR_JSQ3_3; // ADC1 1st injected conversion is channel 8: 01000 - Current sensor 2
        ADC1->JSQR |= ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_3; // ADC1 2nd injected conversion is channel 9: 01001 - Current sensor 1

        /*Interrupts setup*/
        //ADC1->CR1 |= ADC_CR1_DISCEN; // Discontinuous mode regular channels enabled
        ADC1->CR1 |= ADC_CR1_JDISCEN; // Discontinuous mode injected channels enabled
        ADC1->CR1 |= ADC_CR1_SCAN;
        ADC1->CR1 |= ADC_CR1_JEOCIE; // JEOC interrupt enabled
        //ADC1->CR1 |= ADC_CR1_EOCIE; // EOC interrupt enabled
        ADC1->CR1 |= ADC_CR1_OVRIE; // Overrun interrupt enabled
        ADC1->CR2 |= ADC_CR2_EOCS;
        NVIC_SetPriority(ADC_IRQn, ADC_IRQN_PRIORITY); //Low priority for ADC IRQ
        NVIC_EnableIRQ(ADC_IRQn);
        ADC1->CR2 |= ADC_CR2_ADON; // A/D converter ON
#endif  // SINUSOIDAL_DRIVE
    }

    void PMACdriver::calculateShuntCurrent() {
        if (getMotorStatus()) {
            current_measured_amps.current_branch_A_amps = 
                    ((((float)current_measured_bits.current_branch_A_bits) * 3.3 / 4096 ) - (3.3 / 2))/(10 * 0.001);
            current_measured_amps.current_branch_C_amps = 
                    ((((float)current_measured_bits.current_branch_C_bits) * 3.3 / 4096 ) - (3.3 / 2))/(10 * 0.001);
            current_measured_amps.current_branch_B_amps = 
                    -(current_measured_amps.current_branch_A_amps + current_measured_amps.current_branch_C_amps) + I_OFFSET_B;
        }
    }

    void PMACdriver::changeDutyCycle(float dutyCycle) {
        vDutyCycle = dutyCycle;
    }

    void PMACdriver::changeDirection(float direction) {
        vDutyCycle = direction;
    }
void PMACdriver::calculateSpeed() {
#ifdef SINUSOIDAL_DRIVE
        PMACdriver::updateHallEffectSensorsValue();
#endif // SINUSOIDAL_DRIVE
        if (hallEffectSensors_oldPosition == hallEffectSensors_newPosition) {
            speedCounter++; // this will sum every 1/CONTROL_TIMER_FREQUENCY
            speedCalculationTimeout_Counter++;
            if (speedCalculationTimeout_Counter >= SPEED_CALCULATION_TIMEOUT * CONTROL_TIMER_FREQUENCY) {
                speed_radiansPerSecond_electric = 0;
                speedCalculationTimeout_Counter = 0;
            }
        } 
        else {
            speedCalculationTimeout_Counter = 0;
        }
        if ((hallEffectSensors_newPosition == 0b101)&&(hallEffectSensors_oldPosition == 0b001)) {
            hall_effect_sensors_frequency = -CONTROL_TIMER_FREQUENCY / ((float) speedCounter);
            speed_radiansPerSecond_electric = hall_effect_sensors_frequency * 2 * M_PI;
            speedCounter = 0;
        }
        if ((hallEffectSensors_newPosition == 0b001)&&(hallEffectSensors_oldPosition == 0b101)) {
            hall_effect_sensors_frequency = CONTROL_TIMER_FREQUENCY / ((float) speedCounter);
            speed_radiansPerSecond_electric = hall_effect_sensors_frequency * 2 * M_PI;
            speedCounter = 0;
        }
    }
    
    float PMACdriver::getSpeed(char type) {
        if (type == 2) return speed_radiansPerSecond_electric;
        else if (type == 5) return speed_radiansPerSecond_electric / (float)MOTOR_POLE_PAIRS;  //speed_radiansPerSecond_mechanic;
        else return 0;
    }

    char PMACdriver::getMotorStatus() {
        return motorRunning;
    }

    void PMACdriver::setupTimer3(int frequency) {
        // This timer generates a PWM signal and it's setup in the board to drive a servomotor
        /* Code to call: 
            setupTimer3(1000);
            dutyCycleTimer3(.1);
         */
        TIM3->CR1 = 0;
        TIM3->CCR2 = 0;
        //TIM3->DIER = TIM_DIER_UIE;
        //NVIC_SetPriority(TIM3_IRQn, TIM3_IRQn_PRIORITY); //Low priority for timer IRQ
        //NVIC_EnableIRQ(TIM3_IRQn);

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

    void PMACdriver::setupSPI() {
        SPI1_SCK::alternateFunction(5);
        SPI1_SCK::mode(Mode::ALTERNATE);
        SPI1_MISO::alternateFunction(5);
        SPI1_MISO::mode(Mode::ALTERNATE);
        SPI1->CR1 = 0;
        SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; // 111: fPCLK/256 - The maximum sensor SPI speed is the minimum uC SPI speed...
        SPI1->CR1 |= SPI_CR1_CPOL; // 1: CK to 1 when idle
        SPI1->CR1 |= SPI_CR1_DFF; // 1: 16-bit data frame format is selected for transmission/reception
        SPI1->CR1 |= SPI_CR1_SSM; // 1: Software slave management enabled
        SPI1->CR1 |= SPI_CR1_SSI; // SSI: Internal slave select
        SPI1->CR1 |= SPI_CR1_RXONLY; // 1: Output disabled (Receive-only mode)
        SPI1->CR1 |= SPI_CR1_MSTR; // Master mode
    }

    int PMACdriver::getOrbisSensorValues() {
        SPI1->CR1 |= SPI_CR1_SPE;
        int timeoutCounter = 0;
        while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {
            timeoutCounter++;
            if (timeoutCounter >= 1000) {
                return -1;
                break;
            }
        }
        receivedAngularPosition = SPI1->DR;
        SPI1->CR1 &= ~SPI_CR1_SPE;
        modifiedAngularPosition_1 = modifiedAngularPosition_0;
        modifiedAngularPosition_0 = receivedAngularPosition;
        modifiedAngularPosition_0 &= ~0b1000000000000000;
        modifiedAngularPosition_0 = modifiedAngularPosition_0 >> (1 + 0);
        rotorAngularPosition_bits = modifiedAngularPosition_0;
        return rotorAngularPosition_bits;
    }
    
    float PMACdriver::getTheta() {
        float theta = (float)getOrbisSensorValues();
        if (theta >= 0){
            theta = theta * M_TWOPI / 16384;
            theta += angularSlip * M_TWOPI / 1024;
            if (theta >= M_TWOPI) theta = M_TWOPI - theta;
            if (theta < 0) theta = 0 - theta;
            return theta;
        }
        else
            return -1;
    }

    float PMACdriver::updateElectricalAngularPosition() {

        hallEffectSensors_forAngularCalculation_1 = hallEffectSensors_forAngularCalculation_0;
        hallEffectSensors_forAngularCalculation_0 = getHallEffectSensorsValue();
        if (hallEffectSensors_forAngularCalculation_0 != hallEffectSensors_forAngularCalculation_1) {
            if (hallEffectSensors_forAngularCalculation_0 == 0b001) electricalAngle_0 = (M_TWOPI * 5 / 6);
            else if (hallEffectSensors_forAngularCalculation_0 == 0b101) electricalAngle_0 = (M_TWOPI * 4 / 6);
            else if (hallEffectSensors_forAngularCalculation_0 == 0b100) electricalAngle_0 = (M_TWOPI * 3 / 6);
            else if (hallEffectSensors_forAngularCalculation_0 == 0b110) electricalAngle_0 = (M_TWOPI * 2 / 6);
            else if (hallEffectSensors_forAngularCalculation_0 == 0b010) electricalAngle_0 = (M_TWOPI * 1 / 6);
            else if (hallEffectSensors_forAngularCalculation_0 == 0b011) electricalAngle_0 = 0;
        }
        return M_TWOPI - electricalAngle_0;
    }

    float PMACdriver::getElectricalAngularPosition() {
        return theta;
    }

    float PMACdriver::getMechanicalAngularPosition() {
        mechanicalAngle_0 = 0;
        mechanicalAngle_1 = 0;
        return 0;
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

    int PMACdriver::sinusoidalDrive(float quadratureVoltage, float directVoltage, float theta) {        
        /* Obtain alpha and beta voltages for the Space Vector PWM using Reverse Park and Circle Limitation (the last one not yet) */
        
        v_alpha = (directVoltage * cos(theta)) - (quadratureVoltage * sin(theta));
        v_beta = (directVoltage * sin(theta)) + (quadratureVoltage * cos(theta));
        
        /* Setup the PWM times */
#if 0
        U_alpha = sqrt(3) * T * v_alpha;
        U_beta = -T * v_beta;
        X = U_beta;
        Y = (U_alpha + U_beta) / 2;
        Z = (U_beta - U_alpha) / 2;

        /* Sectors I and IV */
        if (((Y < 0) && (Z >= 0) && (X <= 0)) || ((Y >= 0) && (Z < 0) && (X > 0))) {
            time_phaseA = (T / 4) + (((T / 2) + X - Z) / 2);
            time_phaseB = time_phaseA + Z;
            time_phaseC = time_phaseB - X;
        }/* Sectors II and V */
        else if (((Y < 0) && (Z < 0)) || ((Y >= 0) && (Z >= 0))) {
            time_phaseA = (T / 4) + (((T / 2) + Y - Z) / 2);
            time_phaseB = time_phaseA + Z;
            time_phaseC = time_phaseA - Y;
        }/* Sectors III and VI */
        else if (((Y < 0) && (Z >= 0) && (X > 0)) || ((Y >= 0) && (Z < 0) && (X <= 0))) {
            time_phaseA = (T / 4) + (((T / 2) + Y - X) / 2);
            time_phaseC = time_phaseA - Y;
            time_phaseB = time_phaseC + X;
        }

        dutyCycle_A = (time_phaseA / T);
        dutyCycle_B = (time_phaseB / T);
        dutyCycle_C = (time_phaseC / T);
#else
        v_a = v_alpha;
        v_b = -(v_alpha * 0.5) + (v_beta * .8660254038);
        v_c = -(v_alpha * 0.5) - (v_beta * .8660254038);
        
        dutyCycle_A = ((v_a/busVoltage_volts)+1)/2;
        dutyCycle_B = ((v_b/busVoltage_volts)+1)/2;
        dutyCycle_C = ((v_c/busVoltage_volts)+1)/2;
        
        //if (dutyCycle_A > 0.99) dutyCycle_A = 0.99;
        //if (dutyCycle_B > 0.99) dutyCycle_B = 0.99;
        //if (dutyCycle_C > 0.99) dutyCycle_C = 0.99;
        
#endif
                
        //setADCTriggerPosition(1, 0.1 + (dutyCycle_A / 2));
        setADCTriggerPosition(1, dutyCycle_A);
        setHighSideWidth(1, dutyCycle_A);
        setHighSideWidth(2, dutyCycle_B);
        setHighSideWidth(3, dutyCycle_C);
//        setHighSideWidth(1, 0.5);
//        setHighSideWidth(2, 0.5);
//        setHighSideWidth(3, 0.5);

        return 1;
    }

    void PMACdriver::fieldOrientedControl(float theta) {        
        i_qs_ref = quadratureCurrent_reference;
        i_ds_ref = directCurrent_reference;

        /* Store current in branch 1 and in branch 3 */
        
        calculateShuntCurrent();

        /* Clarke Transform */
        i_alpha = current_measured_amps.current_branch_A_amps;
        i_beta = (0.5773502692) * ((2 * current_measured_amps.current_branch_B_amps) + current_measured_amps.current_branch_A_amps);

        /* Park Transform */
        i_qs = (-i_alpha * sin(theta)) + (i_beta * cos(theta));
        i_ds = (i_alpha * cos(theta)) + (i_beta * sin(theta));

        /* Quadrature Current Control*/
        quadratureCurrent_error_0 = i_qs_ref - i_qs;
        v_qs = ( quadratureCurrent_error_0 * kp_FOC ) + quadratureCurrent_integralError_0;
        if (v_qs > busVoltage_volts) v_qs = busVoltage_volts;
        else if (v_qs < -busVoltage_volts) v_qs = -busVoltage_volts;
        else quadratureCurrent_integralError_0 = quadratureCurrent_integralError_0 + ((ki_FOC * quadratureCurrent_error_0) * (1 / (float)CONTROL_TIMER_FREQUENCY));
        
        /* Direct Current Control */
        directCurrent_error_0 = i_ds_ref - i_ds;
        v_ds = ( directCurrent_error_0 * kp_FOC ) + directCurrent_integralError_0;
        if (v_ds > busVoltage_volts) v_ds = busVoltage_volts;
        else if (v_ds < -busVoltage_volts) v_ds = -busVoltage_volts;
        else directCurrent_integralError_0 = directCurrent_integralError_0 + ((ki_FOC * directCurrent_error_0) * (1 / (float)CONTROL_TIMER_FREQUENCY));

        sinusoidalDrive(v_qs, v_ds, theta);
    }

    float PMACdriver::getFOCvariables(char variable) {
        switch (variable) {
            case HMI_VAR_THETA:
                return theta_rads;
                break;
            case HMI_VAR_I_Q_REF:
                return i_qs_ref;
                break;
            case HMI_VAR_I_D_REF:
                return i_ds_ref;
                break;
            case HMI_VAR_I_A:
                return current_measured_amps.current_branch_A_amps;
                break;
            case HMI_VAR_I_B:
                return current_measured_amps.current_branch_B_amps;
                break;
            case HMI_VAR_I_C:
                return current_measured_amps.current_branch_C_amps;
                break;
            case HMI_VAR_I_ALPHA:
                return i_alpha;
                break;
            case HMI_VAR_I_BETA:
                return i_beta;
                break;
            case HMI_VAR_I_Q:
                return i_qs;
                break;
            case HMI_VAR_I_D:
                return i_ds;
                break;
            case HMI_VAR_I_Q_ERROR:
                return quadratureCurrent_error_0;
                break;
            case HMI_VAR_I_D_ERROR:
                return directCurrent_error_0;
                break;
            case HMI_VAR_I_Q_INTEGRAL_ERROR:
                return quadratureCurrent_integralError_0;
                break;
            case HMI_VAR_I_D_INTEGRAL_ERROR:
                return directCurrent_integralError_0;
                break;
            case HMI_VAR_PROPORTIONAL_Q:
                return p_term_q;
                break;
            case HMI_VAR_PROPORTIONAL_D:
                return p_term_d;
                break;
            case HMI_VAR_INTEGRAL_Q:
                return i_term_q;
                break;
            case HMI_VAR_INTEGRAL_D:
                return i_term_d;
                break;
            case HMI_VAR_V_Q:
                return v_qs;
                break;
            case HMI_VAR_V_D:
                return v_ds;
                break;
            case HMI_VAR_V_ALPHA:
                return v_alpha;
                break;
            case HMI_VAR_V_BETA:
                return v_beta;
                break;
            case HMI_VAR_U_ALPHA:
                return U_alpha;
                break;
            case HMI_VAR_U_BETA:
                return U_beta;
                break;
            case HMI_VAR_X:
                return X;
                break;
            case HMI_VAR_Y:
                return Y;
                break;
            case HMI_VAR_Z:
                return Z;
                break;
            case HMI_VAR_TIME_PHASE_A:
                return time_phaseA;
                break;
            case HMI_VAR_TIME_PHASE_B:
                return time_phaseB;
                break;
            case HMI_VAR_TIME_PHASE_C:
                return time_phaseC;
                break;
            case HMI_VAR_DUTY_CYCLE_A:
                return dutyCycle_A;
                break;
            case HMI_VAR_DUTY_CYCLE_B:
                return dutyCycle_B;
                break;
            case HMI_VAR_DUTY_CYCLE_C:
                return dutyCycle_C;
                break;
            case HMI_VAR_MOTOR_STATUS:
                return getMotorStatus();
                break;
            case HMI_VAR_ANGULAR_SLIP:
                return angularSlip;
                break;
            case HMI_VAR_KP_FOC_CURRENT:
                return kp_FOC;
                break;
            case HMI_VAR_KI_FOC_CURRENT:
                return ki_FOC;
                break;
            case HMI_VAR_KP_FOC_SPEED:
                return kp_speed;
                break;
            case HMI_VAR_KI_FOC_SPEED:
                return ki_speed;
                break;
            case HMI_VAR_DRV8302_FAULT_FLAG:
                return getFaultFlag();
                break;
            case HMI_VAR_SPEED:
                return getSpeed(5);
                break;
            case HMI_VAR_TRIGGER_POSITION:
                return ADC_triggerPoint;
                break;
            case HMI_VAR_VOLTAGE_BUS:
                return busVoltage_volts;
                break;
            default:
                return 0;
                break;
        }
    }

    void PMACdriver::setFOCvariables(char variable, float value) {
        switch (variable) {
            case HMI_SET_VAR_I_Q_REF:
                quadratureCurrent_reference = value / 1000;
                break;
            case HMI_SET_VAR_I_D_REF:
                directCurrent_reference = value / 1000;
                break;
            case HMI_SET_VAR_ANGULAR_SLIP:
                angularSlip = value / 1000;
                break;
            case HMI_SET_VAR_KP_FOC_CURRENT:
                kp_FOC = value / 1000;
                break;
            case HMI_SET_VAR_KI_FOC_CURRENT:
                ki_FOC = value / 1000;
                break;
            case HMI_SET_VAR_KP_FOC_SPEED:
                kp_speed = value / 1000;
                break;
            case HMI_SET_VAR_KI_FOC_SPEED:
                ki_speed = value / 1000;
                break;
            case HMI_SET_VAR_TRIGGER_POSITION:
                ADC_triggerPoint = value / 1000;
                break;
            default:
                break;
        }
    }
    
    void PMACdriver::testSignal (bool flag){
        if (flag == 0) pwmSignal0::low();
        else if (flag == 1) pwmSignal0::high();
    }
    
    void PMACdriver::voltageBusCalculation (){
        busVoltage_volts = (float)busVoltage_bits * 18.727272 * 3.3 / 4096;
    }
}