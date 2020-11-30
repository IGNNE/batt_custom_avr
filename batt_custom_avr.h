/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file batt_custom_avr.h
 *
 * Custom battery monitor with an AVR "co-processor" over I2C
 */

#pragma once

#include <lib/drivers/smbus/SMBus.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

using namespace time_literals;


class BatteryCustomAvr : public device::I2C, public I2CSPIDriver<BatteryCustomAvr>
{
public:
    BatteryCustomAvr(I2CSPIBusOption bus_option, const int bus, int bus_frequency);

    ~BatteryCustomAvr();

    int init();

    static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                         int runtime_instance);
    static void print_usage();

    void RunImpl();

    /**
     * default address for the avr mcu, no deeper reason, set your own if you want
     * this needs to be in the public part so the main function can access it
     */
    const static uint8_t I2C_DEFAULT_ADDRESS = 0x4;

private:
    /**
     * the reported battery value is the average of the last values,
     * to remove outliers
     */
    constexpr static uint8_t AVERAGE_VALUES = 3;
    float last_values_V[AVERAGE_VALUES];
    uint8_t last_values_counter = 0;

    /**
     * how often to poll the device: the adc runs with more than 10 kHz,
     * but there is no need to waste I2C time and AVR cycles
     */
    constexpr static auto POLLING_INVERVALL = 500_ms;

    orb_advert_t battery_topic;

    /** register address for battery value */
    const uint8_t BATTERY_REGISTER = 0x1;

    /** voltage range for the adc values */
    constexpr static float VOLTAGE_RANGE = 5.0;

    // just to be safe, here are some default values for 1S lipos
    constexpr static float LOW_V = 3.6;
    constexpr static float CRITICAL_V = 3.5;
    constexpr static float EMERGENCY_V = 3.3;

};
