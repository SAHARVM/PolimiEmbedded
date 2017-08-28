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
#define PMAC_PWM_FREQUENCY 25000
#define CONTROL_TIMER_FREQUENCY 50000
#define MOTOR_POLE_PAIRS 8
#define CW 0
#define CCW 1


#ifndef ANTILIB_GPIO_H
#define ANTILIB_GPIO_H

#define GPIO_CNF_INPUT_ANALOG  0
#define GPIO_CNF_INPUT_FLOATING  1
#define GPIO_CNF_INPUT_PULLUPDOWN 2

#define GPIO_CNF_OUTPUT_PUSHPULL 0
#define GPIO_CNF_OUTPUT_OPENDRAIN 1
#define GPIO_CNF_AFIO_PUSHPULL  2
#define GPIO_CNF_AFIO_OPENDRAIN  3

#define GPIO_MODE_INPUT    0
#define GPIO_MODE_OUTPUT10MHz  1
#define GPIO_MODE_OUTPUT2MHz  2
#define GPIO_MODE_OUTPUT50MHz  3

#define GPIOCONF(mode, cnf) ((cnf << 2) | (mode))
#define GPIOPINCONFL(pin, conf) (conf << (pin * 4))
#define GPIOPINCONFH(pin, conf) (conf << ((pin - 8) * 4))

#define CONFMASKL(pin) ((u32)~(15 << (pin * 4)))
#define CONFMASKH(pin) ((u32)~(15 << ((pin - 8) * 4)))

#endif

#ifndef ANTILIB_ADC_H
#define ANTILIB_ADC_H

#define SAMPLE_TIME_1_5  0
#define SAMPLE_TIME_7_5  1
#define SAMPLE_TIME_13_5 2
#define SAMPLE_TIME_28_5 3
#define SAMPLE_TIME_41_5 4
#define SAMPLE_TIME_55_5 5
#define SAMPLE_TIME_71_5 6
#define SAMPLE_TIME_239_5 7


#define ADC_SAMPLE_TIME0(x)   (x << 0)
#define ADC_SAMPLE_TIME1(x)   (x << 3)
#define ADC_SAMPLE_TIME2(x)   (x << 6)
#define ADC_SAMPLE_TIME3(x)   (x << 9)
#define ADC_SAMPLE_TIME4(x)   (x << 12)
#define ADC_SAMPLE_TIME5(x)   (x << 15)
#define ADC_SAMPLE_TIME6(x)   (x << 18)
#define ADC_SAMPLE_TIME7(x)   (x << 21)
#define ADC_SAMPLE_TIME8(x)   (x << 24)
#define ADC_SAMPLE_TIME9(x)   (x << 27)

#define ADC_SAMPLE_TIME10(x)  (x << 0)
#define ADC_SAMPLE_TIME11(x)  (x << 3)
#define ADC_SAMPLE_TIME12(x)  (x << 6)
#define ADC_SAMPLE_TIME13(x)  (x << 9)
#define ADC_SAMPLE_TIME14(x)  (x << 12)
#define ADC_SAMPLE_TIME15(x)  (x << 15)
#define ADC_SAMPLE_TIME16(x)  (x << 18)
#define ADC_SAMPLE_TIME17(x)  (x << 21)


#define ADC_SEQUENCE_LENGTH(x) (x << 20)

// SQR3
#define ADC_SEQ1(x)  (x << 0)
#define ADC_SEQ2(x)  (x << 5)
#define ADC_SEQ3(x)  (x << 10)
#define ADC_SEQ4(x)  (x << 15)
#define ADC_SEQ5(x)  (x << 20)
#define ADC_SEQ6(x)  (x << 25)
// SQR2
#define ADC_SEQ7(x)  (x << 0)
#define ADC_SEQ8(x)  (x << 5)
#define ADC_SEQ9(x)  (x << 10)
#define ADC_SEQ10(x)  (x << 15)
#define ADC_SEQ11(x) (x << 20)
#define ADC_SEQ12(x)  (x << 25)
// SQR1
#define ADC_SEQ13(x)  (x << 0)
#define ADC_SEQ14(x)  (x << 5)
#define ADC_SEQ15(x)  (x << 10)
#define ADC_SEQ16(x) (x << 15)

#endif

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
        static void setDrivingFrequency(unsigned int frequency);

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

        static void changeDutyCycle(float dutyCycle);

        static void changeDirection(float direction);

        static float getSpeed(char type);

        static void calculateSpeed();

        static char getMotorStatus();
        
        static int getBatteryVoltage();
        
        static void setupTimer3(int frequency);
        
        static void dutyCycleTimer3(float dutyCycle);

        static bool faultFlag; // = 0;  
        

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

        /**
         * 
         * @param frequency
         */
        static void setupADCTimer();

        FastMutex mutex; ///< Mutex to protect from concurrent access


        /*enum {
            STOPPED, ///< Timer is stopped
            STARTED ///< Timer is started
        } status;*/
    };

} //namespace miosix


#endif /* PMAC_DRIVER_STM32_H */