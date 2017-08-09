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
     * This class is designed to drive a Permanent Magnet Synchronous Machine (PMSM)
     * by generating 3 PWM signals that are managed by a specific control method
     * This class also handles:
     *  -Signals to work with Texas Instruments PMSM driver DRV8302
     *  -ADC signals
     *  -Hall effect sensors signals
     *  -Absolute position encoder (later)
     *  -CAN
     *  -etc...
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
         * Initialize the ports for the hall effect sensors
         */
        void setupHallSensors();

        /**
         * Obtains the value of the hall effect sensors connected to PB6, 7 and PC11
         * @return Hall effect sensors A, B and C value in the 3 LSB of a byte
         * i.e. 0b 0 0 0 0  0 hC hB hA
         */
        char getHallEffectSensorsValue();

        /**
         * 
         */
        void updateHallEffectSensorsValue();

        /**
         * Set the absolute pulse width of the PWM signal
         * @param pulseWidth float from 0 to 1
         */
        void setWidth(char channel, float pulseWidth);

        /**
         * Sets up the main driving timer, which reads the hall effect sensors,
         * If there is any change, change the driving gates
         * @param frequency
         */
        void setupDriverTimer(unsigned int frequency);

        /**
         * 
         * @param referenceSpeed
         */
        void speedControl(float referenceSpeed);

        /**
         * 
         */
        void enableDriver();

        /**
         * 
         */
        void disableDriver();

        /**
         * 
         * @param channel
         * @param value
         * @return 
         */
        void setLowSide(char channel, bool value);

        /**
         * 
         */
        void updateFaultFlag();

        /**
        * 
        * @return 
        */
        bool getFaultFlag();
        
        int trapezoidalDrive(float dutyCycle);

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

        char hallEffectSensors_newPosition = 0;
        char hallEffectSensors_oldPosition = 0;
        bool motorRunning = 0;
        bool faultFlag = 0;

        enum {
            STOPPED, ///< Timer is stopped
            STARTED ///< Timer is started
        } status;
    };

} //namespace miosix


#endif /* PMSM_DRIVE_STM32_H */