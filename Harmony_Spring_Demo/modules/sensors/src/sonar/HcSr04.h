/*
 * HcSr04.h
 *
 *  Created on: Mar 28, 2016
 *      Author: ssdharan
 */

#ifndef HCSR04_H_
#define HCSR04_H_



#pragma once

#include <string>
#include <mraa/aio.h>
#include <mraa/gpio.h>
#include <mraa/pwm.h>
#include <sys/time.h>

#define CM 1
#define INC 0

/**
 * @brief HC-SR04 Ultrasonic Sensor library
 * @defgroup hcsr04 libupm-hcsr04
 * @ingroup generic gpio sound
 */

/**
 * @library hcsr04
 * @sensor hcsr04
 * @comname HC-SR04 Ultrasonic Sensor
 * @type sound
 * @man generic
 * @con gpio
 *
 * @brief API for the HC-SR04 Ultrasonic Sensor
 *
 * This module defines the HC-SR04 interface for libhcsr04
 *
 * @snippet hcsr04.cxx Interesting
 */
class HCSR04 {
    public:
        /**
         * Instantiates an HCSR04 object
         *
         * @param triggerPin Pin to trigger the sensor for distance
         * @param echoPin Pulse response to triggering
         * @param fptr Function pointer to handle rising-edge and
         * falling-edge interrupts
         */

        HCSR04 (uint8_t triggerPin, uint8_t echoPin, void (*fptr)(void *));

        /**
         * HCSR04 object destructor
         */
        ~HCSR04 ();

        /**
         * Gets the distance from the sensor
         */
        double getDistance (int sys);

        /**
         * On each interrupt, this function detects if the interrupt
         * was falling-edge or rising-edge.
         * Should be called from the interrupt handler.
         */
        void ackEdgeDetected ();

        uint8_t m_doWork; /**< Flag to control blocking function while waiting for a falling-edge interrupt */

        /**
         * Returns the name of the sensor
         */
        std::string name()
        {
            return m_name;
        }

    private:

        double timing();
        mraa_gpio_context   m_triggerPinCtx;
        mraa_gpio_context   m_echoPinCtx;

        long    m_RisingTimeStamp;
        long    m_FallingTimeStamp;
        uint8_t m_InterruptCounter;

        std::string         m_name;
};




#endif /* HCSR04_H_ */
