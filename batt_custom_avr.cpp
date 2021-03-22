/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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


#include "batt_custom_avr.h"

extern "C" __EXPORT int batt_custom_avr_main(int argc, char *argv[]);

BatteryCustomAvr::BatteryCustomAvr(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
    I2C(DRV_BAT_CUSTOM_AVR, MODULE_NAME, bus, I2C_DEFAULT_ADDRESS, bus_frequency),
    I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, I2C_DEFAULT_ADDRESS)
{
    battery_status_s new_report = {};
    battery_topic = orb_advertise(ORB_ID(battery_status), &new_report);
}

BatteryCustomAvr::~BatteryCustomAvr()
{
    orb_unadvertise(battery_topic);
}

int BatteryCustomAvr::init()
{
    int ret = I2C::init();

    if (ret != PX4_OK) {
        PX4_WARN("I2C::init failed: (%i)", ret);
    }

    uint8_t id_address = 0;
	uint8_t id_value = 0;
        // for some reason, my avr needs two separate tranfers
    // first write the "register"/key, then read the date
    ret = transfer(&id_address, 1, nullptr, 0);
    ret += transfer(nullptr, 0, &id_value, 1);
    if (ret != PX4_OK || id_value != I2C_DEFAULT_ADDRESS) {
        PX4_WARN("Controller does not respond, address returned %i, should be %i", id_value, I2C_DEFAULT_ADDRESS);
    }

    return ret;
}

void BatteryCustomAvr::RunImpl()
{
    // TODO: this function feels not very efficient

    int ret = PX4_OK;

    uint8_t tempval = 0;
    // for some reason, my avr needs two separate tranfers
    // first write the "register"/key, then read the date
    ret = transfer(&BATTERY_REGISTER, 1, nullptr, 0);
    ret += transfer(nullptr, 0, &tempval, 1);

    // did everything work, and did we get a meaningful value back?
    if(ret == PX4_OK && tempval != 0 && tempval != 255) {
        // 0-255 is mapped to the voltage range
        last_values_V[last_values_counter++] = tempval * (VOLTAGE_RANGE / 255.0f);
        if(last_values_counter >= AVERAGE_VALUES) {
            last_values_counter = 0;
        }

        // create average and report
        float sum_values_V = 0;
        for (auto value_V : last_values_V)
        {
            sum_values_V += value_V;
        }
        battery_status_s new_report = {};
        new_report.timestamp = hrt_absolute_time();
        new_report.connected = true;
        new_report.cell_count = CELL_COUNT;

        float avg_value_V = sum_values_V/3;
        // HACK:
        new_report.voltage_v  = new_report.voltage_cell_v[0] = avg_value_V;
        if(avg_value_V > LOW_V) {
            new_report.warning = battery_status_s::BATTERY_WARNING_NONE;
        } else if(avg_value_V > CRITICAL_V) {
            new_report.warning = battery_status_s::BATTERY_WARNING_LOW;
        } else if(avg_value_V > EMERGENCY_V) {
            new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;
        } else {
            new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
        }

        orb_publish(ORB_ID(battery_status), battery_topic, &new_report);
    } else {
        // something broke
        PX4_WARN("Can't access battery sensor: transfer result %x, register content %x", ret, tempval);
    }
}

void BatteryCustomAvr::print_usage()
{
        PRINT_MODULE_DESCRIPTION("Simple custom battery driver via I2C \n"\
"This driver polls an external AVR MCU that reads the battery voltage via ADC.\n"\
"There is not much to configure, you probably want to edit the source code directly\n"\
"(most people probably use a real FC ADC channel or an actual fuel gauge IC). The\n"\
"default values assume an 1S lipo battery directly attached to a 5V AVR ADC.");

        PRINT_MODULE_USAGE_NAME("batt_custom_avr", "driver");

        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BatteryCustomAvr::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                      int runtime_instance)
{
    BatteryCustomAvr * instance = new BatteryCustomAvr(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);
    if (!instance) {
        PX4_ERR("alloc failed");
        return nullptr;
    }

    // start running, there is no real init code anyway
    if(instance->init()) {
        PX4_ERR("Can't open I2C device");
    }
    instance->ScheduleOnInterval(POLLING_INVERVALL);

    return instance;
}

extern "C" __EXPORT int batt_custom_avr_main(int argc, char *argv[])
{
        using ThisDriver = BatteryCustomAvr;
        BusCLIArguments cli{true, false};
        cli.default_i2c_frequency = 400000;
        cli.i2c_address = BatteryCustomAvr::I2C_DEFAULT_ADDRESS;

        const char *verb = cli.parseDefaultArguments(argc, argv);
        if (!verb) {
                ThisDriver::print_usage();
                return -1;
        }

        BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_CUSTOM_AVR);

        if (!strcmp(verb, "start")) {
                return ThisDriver::module_start(cli, iterator);
        }

        if (!strcmp(verb, "stop")) {
                return ThisDriver::module_stop(iterator);
        }

        ThisDriver::print_usage();
        return -1;
}
