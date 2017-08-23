/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pmac_driver_stm32.h
 * Author: Arturo
 *
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#ifndef PMAC_DRIVER_STM32_H
#define PMAC_DRIVER_STM32_H

#include "miosix.h"

#define PWM_RESOLUTION 100
#define CONTROL_TIMER_FREQUENCY 50000
#define MOTOR_POLE_PAIRS 8
#define CW 0
#define CCW 1

namespace miosix {

    /**
     * This class is designed to drive a Permanent Magnet Synchronous Machine (PMAC)
     * by generating 3 PWM signals that are managed by a specific control method
     * This class also handles:
     *  -Signals to work with Texas Instruments PMAC driver DRV8302
     *  -ADC signals
     *  -Hall effect sensors signals
     *  -Absolute position encoder (later)
     *  -CAN
     *  -etc...
     */
    class PMACdriver {
    public:
        /**
         * \return an instance of the SynchronizedServo class (singleton)
         * When first returned, the servo waveform generation is stopped. You must
         * enable at least one channel call start() and setPosition() before the
         * servo driving waveforms will be generated.
         */
        static PMACdriver& instance();

        /**
         * Enable a channel. Can only be called with the outputs stopped. Even if
         * the channel is enabled, when it is started it will not produce any
         * waveform until the first call to setPosition()
         * \param channel which channel to enable, must be between 0 and 3.
         */
        static void enable();

        /**
         * Disable a channel. Can only be called with the outputs stopped.
         * \param channel which channel to disable, must be between 0 and 3.
         */
        static void disable();

        /**
         * Start producing the output waveforms.
         */
        static void start();

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
        static void stop();

        /**
         * Wait until the begin of a waveform generation cycle
         * \return false if a new cycle of waveform generation has begun, or true
         * if another thread called stop(). Only one thread at a time can call this
         * member function. If more than one thread calls this deadlock will occur
         * so don't do it!
         */
        bool waitForCycleBegin();
        
        /**
         * Set the output frequency. Only to be called with outputs stopped.
         * @param frequency in Hz
         */
        static void setFrequency(unsigned int frequency);

        /**
         * Initialize the ports for the hall effect sensors
         */
        static void setupHallSensors();

        /**
         * Obtains the value of the hall effect sensors connected to PB6, 7 and PC11
         * @return Hall effect sensors A, B and C value in the 3 LSB of a byte
         * i.e. 0b 0 0 0 0  0 hC hB hA
         */
        static char getHallEffectSensorsValue();

        /**
         * 
         */
        static void updateHallEffectSensorsValue();

        /**
         * Set the absolute pulse width of the PWM signal
         * @param pulseWidth float from 0 to 1
         */
        static void setHighSideWidth(char channel, float pulseWidth);

        /**
         * Sets up the main driving timer, which reads the hall effect sensors,
         * If there is any change, change the driving gates
         * @param frequency
         */
        static void setupDriverTimer(unsigned int frequency);

        /**
         * 
         * @param referenceSpeed
         */
        static void speedControl(float referenceSpeed);

        /**
         * 
         */
        static void enableDriver();

        /**
         * 
         */
        static void disableDriver();
       
        /**
         * 
         * @param channel
         * @param value
         * @return 
         */
        static void setLowSide(char channel, bool value);

        /**
         * 
         */
        static void updateFaultFlag();

        /**
        * 
        * @return 
        */
        static bool getFaultFlag();
        
        /**
         * 
         * @param dutyCycle
         * @return 
         */
        static int trapezoidalDrive();
        
        static void allGatesLow();
        
        static void highSideGatesLow();
        
        static void lowSideGatesLow();
        
        static void changeDutyCycle (float dutyCycle);
        
        static void changeDirection (float direction);
        
        static float getSpeed (char type);
        
        static void calculateSpeed ();
        
        static char getMotorStatus();
        
        static bool faultFlag;// = 0;  

    private:
        PMACdriver(const PMACdriver&);
        PMACdriver& operator=(const PMACdriver&);

        /**
         * Constructor
         */
        PMACdriver();

        /**
         * Wait until the timer overflows from 0xffff to 0. Can only be called with
         * interrupts disabled
         */
        static void IRQwaitForTimerOverflow(FastInterruptDisableLock& dLock);
        
        
        /**
         * 
         * @param frequency
         */
        static void setupControlTimer(unsigned int frequency);
        
        /**
         * 
         */
        static void setupADC();
        
        FastMutex mutex; ///< Mutex to protect from concurrent access


        /*enum {
            STOPPED, ///< Timer is stopped
            STARTED ///< Timer is started
        } status;*/
    };

} //namespace miosix


#endif /* PMAC_DRIVER_STM32_H */