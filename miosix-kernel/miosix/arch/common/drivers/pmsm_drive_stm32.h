/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pmsm_drive_stm32.h
 * Author: Arturo
 *
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#ifndef PMSM_DRIVE_STM32_H
#define PMSM_DRIVE_STM32_H

#include "miosix.h"

#define PWM_RESOLUTION 1000

namespace miosix {

    /**
     * This class is designed to drive up to 4 servomotors. It generates
     * four square waves that are synchronized with respect to each other,
     * and allows the execution of code that is too synchronized with the
     * waveform generation, to ease the development of closed loop control
     * code using the servos as actuators.
     * This class can be safely accessed by multiple threads, except the
     * waitForCycleBegin() member function.
     */
    class PMSMdriver {
    public:
        /**
         * \return an instance of the SynchronizedServo class (singleton)
         * When first returned, the servo waveform generation is stopped. You must
         * enable at least one channel call start() and setPosition() before the
         * servo driving waveforms will be generated.
         */
        static PMSMdriver& instance();

        /**
         * Enable a channel. Can only be called with the outputs stopped. Even if
         * the channel is enabled, when it is started it will not produce any
         * waveform until the first call to setPosition()
         * \param channel which channel to enable, must be between 0 and 3.
         */
        void enable();

        /**
         * Disable a channel. Can only be called with the outputs stopped.
         * \param channel which channel to disable, must be between 0 and 3.
         */
        void disable();

        /**
         * Start producing the output waveforms.
         */
        void start();

        /**
         * Stop producing the output waveforms. If a thread is waiting in
         * waitForCycleBegin() it will be forecefully woken up.
         * As a side effect, the position of all the channels will be set to NAN.
         * When you restart the timer, you must call setPosition() on each enabled
         * channel before the channel will actually produce a waveform.
         * This function waits until the end of a waveform generation cycle in order
         * to not produce glitches. For this reason, it may take up to
         * 1/getFrequency() to complete, which with the default value of 50Hz is 20ms
         */
        void stop();

        /**
         * Wait until the begin of a waveform generation cycle
         * \return false if a new cycle of waveform generation has begun, or true
         * if another thread called stop(). Only one thread at a time can call this
         * member function. If more than one thread calls this deadlock will occur
         * so don't do it!
         */
        bool waitForCycleBegin();

        /**
         * Set the frequency of the generated waveform. Can only be called
         * with the outputs stopped. The default is 50Hz. Note that due to prescaler
         * resolution, the actual frequency is set to the closest possible value.
         * To know the actual frequency, call getFrequency()
         * \param frequency desired servo update frequency in Hz
         * Must be between 10 and 100Hz
         */
        void setFrequency(unsigned int frequency);
        
        /**
         * 
         */
        void setupHallSensors();
        
        /**
         *  TODO: write something
         * @return Hall effect sensors A, B and C value in the 3 first bits of a byte
         * i.e. 0b 0 0 0 0  0 hC hB hA
         */
        char getHallEffectSensorsValue();

        /*
         * Set the absolute pulse width of the PWM signal
         */
        void setWidth(float pulseWidth);

    private:
        PMSMdriver(const PMSMdriver&);
        PMSMdriver& operator=(const PMSMdriver&);

        /**
         * Constructor
         */
        PMSMdriver();

        /**
         * Wait until the timer overflows from 0xffff to 0. Can only be called with
         * interrupts disabled
         */
        static void IRQwaitForTimerOverflow(FastInterruptDisableLock& dLock);

        //float minWidth, maxWidth; ///< Minimum and maximum pulse widths
        //float a, b, c; ///< Precomputed coefficients
        FastMutex mutex; ///< Mutex to protect from concurrent access

        char hallEffectSensorsPosition = 0;
        
        enum {
            STOPPED, ///< Timer is stopped
            STARTED ///< Timer is started
        } status;
    };

} //namespace miosix


#endif /* PMSM_DRIVE_STM32_H */