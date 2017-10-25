/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   pmac_driver_stm32.h
 * Author: Arturo Montufar Arreola
 *
 * Created on 5 de agosto de 2017, 09:11 PM
 */

#ifndef PMAC_DRIVER_STM32_H
#define PMAC_DRIVER_STM32_H

#include "miosix.h"

//#define SINUSOIDAL_DRIVE
#define MOTOR_HUBMOTOR
#define ROTOR_DIRECT_VECTOR_ANGULAR_SLIP (0.0)

#ifdef SINUSOIDAL_DRIVE
#define CONTROL_TIMER_FREQUENCY 10000
#define PWM_RESOLUTION 100
#define PMAC_PWM_FREQUENCY 25000
#else
#define CONTROL_TIMER_FREQUENCY 50000
#define PWM_RESOLUTION 100
#define PMAC_PWM_FREQUENCY 15000
#endif // SINUSOIDAL_DRIVE

#ifdef MOTOR_HUBMOTOR
#define MOTOR_POLE_PAIRS 10
#define MOTOR_PHASE_RESISTANCE 0.160
#define MOTOR_PHASE_INDUCTANCE .076
#define MOTOR_ELECTRICAL_CONSTANT 0.03775
#define DC_BUS 24   // Must be measured later
#else
#define MOTOR_POLE_PAIRS 8
#define MOTOR_PHASE_RESISTANCE 0.1
#define MOTOR_PHASE_INDUCTANCE .05
#define MOTOR_ELECTRICAL_CONSTANT 0.05
#endif

#define CW 0
#define CCW 1

#define THERMISTOR_B_VALUE 3380

// ADC start address = 0x4001 2000
// ADC1 offset address is 0x000
// ADC2 offset address is 0x100
// ADC3 offset address is 0x200
// ADC_DR offset address is 0x4C
// ADC1_DR_Address should be 0x4001 2000 + 0x000 + 0x4C = 0x4001 204C
//#define ADC1_DR_Address    ((uint32_t)0x4001204C)

/* Interrupts priority table */
//#define TIM1_CC_IRQN_PRIORITY   14
#define TIM8_CC_IRQn_PRIORITY   21
#define TIM2_IRQn_PRIORITY      16
#define TIM3_IRQn_PRIORITY      17
#define ADC_IRQN_PRIORITY       18
#define DMA_IRQN_PRIORITY       19
#define SPI1_IRQN_PRIORITY      20

#define HMI_VAR_THETA                   0
#define HMI_VAR_I_Q_REF                 1
#define HMI_VAR_I_D_REF                 2
#define HMI_VAR_I_A                     3
#define HMI_VAR_I_B                     4
#define HMI_VAR_I_C                     5
#define HMI_VAR_I_ALPHA                 6
#define HMI_VAR_I_BETA                  7
#define HMI_VAR_I_Q                     8
#define HMI_VAR_I_D                     9
#define HMI_VAR_I_Q_ERROR               10
#define HMI_VAR_I_D_ERROR               11
#define HMI_VAR_I_Q_INTEGRAL_ERROR      12
#define HMI_VAR_I_D_INTEGRAL_ERROR      13
#define HMI_VAR_PROPORTIONAL_Q          14
#define HMI_VAR_PROPORTIONAL_D          15
#define HMI_VAR_INTEGRAL_Q              16
#define HMI_VAR_INTEGRAL_D              17
#define HMI_VAR_V_Q                     18
#define HMI_VAR_V_D                     19
#define HMI_VAR_V_ALPHA                 20
#define HMI_VAR_V_BETA                  21
#define HMI_VAR_U_ALPHA                 22
#define HMI_VAR_U_BETA                  23
#define HMI_VAR_X                       24
#define HMI_VAR_Y                       25
#define HMI_VAR_Z                       26
#define HMI_VAR_TIME_PHASE_A            27
#define HMI_VAR_TIME_PHASE_B            28
#define HMI_VAR_TIME_PHASE_C            29
#define HMI_VAR_DUTY_CYCLE_A            30
#define HMI_VAR_DUTY_CYCLE_B            31
#define HMI_VAR_DUTY_CYCLE_C            32
#define HMI_VAR_MOTOR_STATUS            33
#define HMI_VAR_ANGULAR_SLIP            34
#define HMI_VAR_KP_FOC_CURRENT          35
#define HMI_VAR_KI_FOC_CURRENT          36
#define HMI_VAR_KP_FOC_SPEED            37
#define HMI_VAR_KI_FOC_SPEED            38
#define HMI_VAR_DRV8302_FAULT_FLAG      39
#define HMI_VAR_SPEED                   40
#define HMI_VAR_TRIGGER_POSITION        41

#define HMI_SET_VAR_I_Q_REF             1
#define HMI_SET_VAR_I_D_REF             2
#define HMI_SET_VAR_ANGULAR_SLIP        3
#define HMI_SET_VAR_KP_FOC_CURRENT      4
#define HMI_SET_VAR_KI_FOC_CURRENT      5
#define HMI_SET_VAR_KP_FOC_SPEED        6
#define HMI_SET_VAR_KI_FOC_SPEED        7
#define HMI_SET_VAR_TRIGGER_POSITION    8


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
        static char getFaultFlag();

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

        static void setupTimer3(int frequency);

        static void dutyCycleTimer3(float dutyCycle);

        static void setADCTriggerPosition(char channel, float dutyCycle);

        static int getADCvalue();

        static void changeTriggerPoint(float value);

        static void setupSPI();

        static int getOrbisSensorValues();

        static float updateElectricalAngularPosition();

        static float getElectricalAngularPosition();

        static float getMechanicalAngularPosition();

        static int sinusoidalDrive(float quadratureVoltage, float directVoltage, float theta);
        
        static void fieldOrientedControl(float theta);

        static float getShuntCurrent(char branch);

        static float getFOCvariables(char variable);

        static void setFOCvariables(char variable, float value);

        static void spaceVectorModulation_setTimes();
        
        static void setTheta(int orbisSensorAngle);
        
        static void testSignal (bool flag);
        
        static float getTheta ();


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

        static void setupSimpleADC();


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