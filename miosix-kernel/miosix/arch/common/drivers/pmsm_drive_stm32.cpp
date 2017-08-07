/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pmsm_drive_stm32.cpp
 * Author: Arturo Montufar Arreola, based on servo_stm32.cpp
 * 
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#include "pmsm_drive_stm32.h"
#include "kernel/scheduler/scheduler.h"
#include <algorithm>
#include <cstdio>
#include <cmath>

using namespace std;
using namespace miosix;

typedef Gpio<GPIOB_BASE, 5> pwmSignal0; // TIM3_CH2 VESC BOARD

typedef Gpio<GPIOB_BASE, 6> hallSensor1;
typedef Gpio<GPIOB_BASE, 7> hallSensor2;
typedef Gpio<GPIOC_BASE, 11> hallSensor3;



static Thread *waiting = 0;

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
 * Timer 2 interrupt handler actual implementation
 */
void __attribute__ ((used)) tim2impl() {
    TIM2->SR = 0; // Clear interrupt flags
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

namespace miosix {

    //
    // class PMSMdriver
    //

    PMSMdriver& PMSMdriver::instance() {
        static PMSMdriver singleton;
        return singleton;
    }

    PMSMdriver::PMSMdriver() : status(STOPPED) {
        {
            FastInterruptDisableLock dLock;
            // The RCC register should be written with interrupts disabled to
            // prevent race conditions with other threads.
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
            RCC_SYNC();
        }

        // Configure timer
        TIM3->CR1 = 0;
        TIM3->CCR2 = 0;

        // Configure interrupt on timer overflow
        TIM3->DIER = TIM_DIER_UIE;
        NVIC_SetPriority(TIM3_IRQn, 14); //Low priority for timer IRQ
        NVIC_EnableIRQ(TIM3_IRQn);
        setupHallSensors();
    }

    void PMSMdriver::setFrequency(unsigned int PWM_frequency) {
        Lock<FastMutex> l(mutex);
        if (status != STOPPED) return; // If timer enabled ignore the call
        uint32_t TIMER_Frequency = SystemCoreClock / 2;
        uint32_t COUNTER_Frequency = PWM_RESOLUTION * PWM_frequency;
        uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1;
        uint16_t ARR_Value = PWM_RESOLUTION - 1;
        TIM3->PSC = PSC_Value;
        TIM3->ARR = ARR_Value;
    }

    void PMSMdriver::enable() {
        Lock<FastMutex> l(mutex);
        if (status != STOPPED) return; // If timer enabled ignore the call
        {
            FastInterruptDisableLock dLock;
            // Calling the mode() function on a GPIO is subject to race conditions
            // between threads on the STM32, so we disable interrupts

            TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
            TIM3->CCER |= TIM_CCER_CC2E;
#ifndef _ARCH_CORTEXM3_STM32 //Only stm32f2 and stm32f4 have it
            pwmSignal0::alternateFunction(2);
#endif //_ARCH_CORTEXM3_STM32
            pwmSignal0::mode(Mode::ALTERNATE);
        }
    }

    void PMSMdriver::disable() {
        Lock<FastMutex> l(mutex);
        if (status != STOPPED) return; // If timer enabled ignore the call
        {
            FastInterruptDisableLock dLock;
            // Calling the mode() function on a GPIO is subject to race conditions
            // between threads on the STM32, so we disable interrupts

            pwmSignal0::mode(Mode::INPUT);
            TIM3->CCER &= ~TIM_CCER_CC2E;
        }
    }

    void PMSMdriver::setWidth(float pulseWidth) {
        Lock<FastMutex> l(mutex);
        if (status != STARTED) return; // If timer disabled ignore the call
        //switch (channel) {
        //    case 0:
        TIM3->CCR2 = pulseWidth * PWM_RESOLUTION;
        //break;
        //}
    }

    void PMSMdriver::start() {
        Lock<FastMutex> l(mutex);
        if (status != STOPPED) return; // If timer enabled ignore the call

        // While status is starting neither member function callable with timer
        // started nor stopped are allowed
        status = STARTED;
        TIM3->CNT = 0;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->CR1 = TIM_CR1_CEN;
    }

    void PMSMdriver::stop() {
        Lock<FastMutex> l(mutex);
        if (status != STARTED) return; // If timer disabled ignore the call
        status = STOPPED;
        TIM3->CCR2 = 0;
        {
            FastInterruptDisableLock dLock;
            // Wakeup an eventual thread waiting on waitForCycleBegin()
            if (waiting) waiting->IRQwakeup();
            IRQwaitForTimerOverflow(dLock);
        }
        TIM3->CR1 = 0;
    }

    bool PMSMdriver::waitForCycleBegin() {
        // No need to lock the mutex because disabling interrupts is enough to avoid
        // race conditions. Also, locking the mutex here would prevent other threads
        // from calling other member functions of this class
        FastInterruptDisableLock dLock;
        if (status != STARTED) return true;
        IRQwaitForTimerOverflow(dLock);
        return status != STARTED;
    }

    void PMSMdriver::setupHallSensors() {
        hallSensor1::mode(Mode::INPUT);
        hallSensor2::mode(Mode::INPUT);
        hallSensor3::mode(Mode::INPUT);
    }
    
    char PMSMdriver::getHallEffectSensorsValue(){
        hallEffectSensorsPosition = (hallSensor1::value() << 0) | (hallSensor2::value() << 1) | (hallSensor3::value() << 2);
        return hallEffectSensorsPosition;
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

