#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <stdlib.h>
#include <functional>
#include "HcSr04.h"


HCSR04::HCSR04 (uint8_t triggerPin, uint8_t echoPin, void (*fptr)(void *)) {
    //mraa_result_t error  = MRAA_SUCCESS;
    m_name              = "HCSR04";
    m_doWork = 0;
    m_InterruptCounter = 0;
    m_FallingTimeStamp = 0;
    m_RisingTimeStamp = 0;

    m_triggerPinCtx     = mraa_gpio_init (triggerPin);
    if (m_triggerPinCtx == NULL) {
        throw std::invalid_argument(std::string(__FUNCTION__) +
                                    ": mraa_pwm_init() failed, invalid pin?");
        return;
    }

    mraa_gpio_dir(m_triggerPinCtx, MRAA_GPIO_OUT);
    mraa_gpio_write (m_triggerPinCtx, 0);

    m_echoPinCtx = mraa_gpio_init(echoPin);
    if (m_echoPinCtx == NULL) {
        throw std::invalid_argument(std::string(__FUNCTION__) +
                                    ": mraa_gpio_init() failed, invalid pin?");
        return;
    }

    mraa_gpio_dir(m_echoPinCtx, MRAA_GPIO_IN);
    mraa_gpio_isr(m_echoPinCtx, MRAA_GPIO_EDGE_BOTH, fptr, NULL);
}

HCSR04::~HCSR04 () {
    mraa_result_t error = MRAA_SUCCESS;

    error = mraa_gpio_close (m_triggerPinCtx);
    if (error != MRAA_SUCCESS) {
        mraa_result_print (error);
    }

    error = mraa_gpio_close (m_echoPinCtx);
    if (error != MRAA_SUCCESS) {
        mraa_result_print (error);
    }
}

double
HCSR04::timing() {
    mraa_gpio_write (m_triggerPinCtx, 1);
    usleep(10);
    mraa_gpio_write (m_triggerPinCtx, 0);

    m_doWork = 0;
    m_InterruptCounter = 0;
    while (!m_doWork) {
        usleep (5);
    }

    return m_FallingTimeStamp - m_RisingTimeStamp;
}

void
HCSR04::ackEdgeDetected () {
    struct timeval timer;
    gettimeofday(&timer, NULL);

    ++m_InterruptCounter;
    if (!(m_InterruptCounter % 2)) {
        m_FallingTimeStamp  = 1000000 * timer.tv_sec + timer.tv_usec;
        m_doWork = 1;
    } else {
        m_RisingTimeStamp = 1000000 * timer.tv_sec + timer.tv_usec;
    }
}

double
HCSR04::getDistance(int sys)
{
    double _timing = timing();
    if (sys)
    {
        return (_timing/2) / 29.1;
    } else {
        return (_timing/2) / 74.1;
    }
}
