#!/usr/bin/env python
#
# Weather Board Test File
# Version 1.8 August 22, 2016
#
# SwitchDoc Labs
# www.switchdoc.com
#
#

# =========================================================================
#  System Imports....
# =========================================================================

import sys
import time
from datetime import datetime
import random
import binascii
import struct
import subprocess

# =========================================================================
#  Application imports
# =========================================================================

import config
import logging

# =========================================================================
#  Raspberry PI imports
# =========================================================================

enable_pi_emulator = False

try:
    import RPi.GPIO as GPIO
    import smbus
except ImportError:
    enable_pi_emulator = True

sys.path.append('./SDL_Pi_SSD1306')
sys.path.append('./SDL_Pi_INA3221')
sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./Adafruit_Python_SSD1306')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./SDL_Pi_FRAM')
sys.path.append('./SDL_Pi_TCA9545')
sys.path.append('./RaspberryPi-AS3935/RPi_AS3935')

import SDL_DS3231
import Adafruit_BMP.BMP280 as BMP280
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack
import SDL_Pi_FRAM
from RPi_AS3935 import RPi_AS3935
import SDL_Pi_INA3221
import SDL_Pi_TCA9545

# /*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
TCA9545_ADDRESS = 0x73  # 1110011 (A0+A1=VDD)
# /*=========================================================================*/

# /*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
TCA9545_REG_CONFIG = 0x00
#    /*---------------------------------------------------------------------*/

TCA9545_CONFIG_BUS0 = 0x01  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS1 = 0x02  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS2 = 0x04  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS3 = 0x08  # 1 = enable, 0 = disable

# /*=========================================================================*/


################
# Device Present State Variables
###############

# indicate interrupt has happened from as3936

as3935_Interrupt_Happened = False

# set to true if you are building the solar powered version
config.SolarPower_Mode = False

config.SunAirPlus_Present = False
config.AS3935_Present = False
config.DS3231_Present = False
config.BMP280_Present = False
config.FRAM_Present = False
config.HTU21DF_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False
config.WXLink_Present = False


###############
# setup lightning i2c mux
##############

def returnStatusLine(device, state):
    return_string = device
    if state:
        return_string += ":   \t\tPresent"
    else:
        assert isinstance(return_string, object)
        return_string += ":   \t\tNot Present"
    return return_string


###############

# WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#

anemometerPin = 21
rainPin = 20

# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1  # internally, the library checks for ADS1115 or ADS1015 if found

# sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
# Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(anemometerPin, rainPin, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
# weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)

################

# WXLink Test Setup

WXLink = smbus.SMBus(1)
try:
    data = WXLink.read_i2c_block_data(0x08, 0)
    config.WXLink_Present = True
except:
    config.WXLink_Present = False

################

# DS3231/AT24C32 Setup
filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
start_time = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)

try:

    # comment out the next line after the clock has been initialized
    ds3231.write_now()
    print "DS3231=\t\t%s" % ds3231.read_datetime()
    config.DS3231_Present = True
    print "----------------- "
    print "----------------- "
    print " AT24C32 EEPROM"
    print "----------------- "
    print "writing first 4 addresses with random data"
    for x in range(0, 4):
        value = random.randint(0, 255)
        print "address = %i writing value=%i" % (x, value)
        ds3231.write_AT24C32_byte(x, value)
    print "----------------- "

    print "reading first 4 addresses"
    for x in range(0, 4):
        print "address = %i value = %i" % (x, ds3231.read_AT24C32_byte(x))
    print "----------------- "

except IOError as e:
    #    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.DS3231_Present = False
# do the AT24C32 eeprom

################

# BMP280 Setup

try:
    bmp280 = BMP280.BMP280()
    config.BMP280_Present = True

except IOError as e:

    #    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.BMP280_Present = False

################

# HTU21DF Detection
try:
    HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib", "-l"])
    config.HTU21DF_Present = True
except:
    config.HTU21DF_Present = False


################

def respond_to_as3935_interrupt():
    # switch to BUS1 - lightning detector is on Bus1
    print "in respond to as3935 interrupt"
    tca9545.write_control_register(TCA9545_CONFIG_BUS1)
    time.sleep(0.003)
    global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
    reason = as3935.get_interrupt()
    as3935LastInterrupt = reason
    if reason == 0x01:
        as3935LastStatus = "Noise Floor too low. Adjusting"
        as3935.raise_noise_floor()
    elif reason == 0x04:
        as3935LastStatus = "Disturber detected - masking"
        as3935.set_mask_disturber(True)
    elif reason == 0x08:
        now = datetime.now().strftime('%H:%M:%S - %Y/%m/%d')
        distance = as3935.get_distance()
        as3935LastDistance = distance
        as3935LastStatus = "Lightning Detected " + str(distance) + "km away. (%s)" % now
    # switch back to BUS0
    tca9545.write_control_register(TCA9545_CONFIG_BUS0)


def get_weather_data():
    # GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)

    ###############

    # Set up FRAM

    fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr=0x50)
    # FRAM Detection
    try:
        fram.read8(0)
        config.FRAM_Present = True
    except:
        config.FRAM_Present = False

    ###############

    if config.SolarPower_Mode:

        try:
            # switch to BUS2 -  SunAirPlus is on Bus2
            tca9545.write_control_register(TCA9545_CONFIG_BUS2)
            sunAirPlus = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)
            # the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
            LIPO_BATTERY_CHANNEL = 1
            SOLAR_CELL_CHANNEL = 2
            OUTPUT_CHANNEL = 3

            bus_voltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
            config.SunAirPlus_Present = True
        except:
            config.SunAirPlus_Present = False

        tca9545.write_control_register(TCA9545_CONFIG_BUS0)

    ###############

    # Detect AM2315
    try:
        from tentacle_pi.AM2315 import AM2315

        try:
            am2315 = AM2315(0x5c, "/dev/i2c-1")
            temperature, humidity, crc_check = am2315.sense()
            print "AM2315 =", temperature
            config.AM2315_Present = True
            if (crc_check == -1):
                config.AM2315_Present = False
        except:
            config.AM2315_Present = False
    except:
        config.AM2315_Present = False
        print "------> See Readme to install tentacle_pi"

    ###########
    # WXLink functions

    def hex2float(s):
        return struct.unpack('<f', binascii.unhexlify(s))[0]

    def hex2int(s):
        return struct.unpack('<L', binascii.unhexlify(s))[0]

    # Main Loop - sleeps 10 seconds
    # Tests all I2C and WeatherRack devices on Weather Board
    # Main Program

    print ""
    print "Weather Board Demo / Test Version 1.7 - SwitchDoc Labs"
    print ""
    print ""
    print "Program Started at:" + time.strftime("%Y-%m-%d %H:%M:%S")
    print ""

    totalRain = 0

    print "----------------------"
    print returnStatusLine("DS3231", config.DS3231_Present)
    print returnStatusLine("BMP280", config.BMP280_Present)
    print returnStatusLine("FRAM", config.FRAM_Present)
    print returnStatusLine("HTU21DF", config.HTU21DF_Present)
    print returnStatusLine("AM2315", config.AM2315_Present)
    print returnStatusLine("ADS1015", config.ADS1015_Present)
    print returnStatusLine("ADS1115", config.ADS1115_Present)
    print returnStatusLine("AS3935", config.AS3935_Present)
    # noinspection PyUnresolvedReferences
    print returnStatusLine("SunAirPlus", config.SunAirPlus_Present)
    print returnStatusLine("WXLink", config.WXLink_Present)
    print "----------------------"

    block1 = ""
    block2 = ""

    response = {}

    print "---------------------------------------- "
    print "----------------- "
    if config.DS3231_Present:
        print " DS3231 Real Time Clock"
    else:
        print " DS3231 Real Time Clock Not Present"

    print "----------------- "
    #

    if config.DS3231_Present:
        current_time = datetime.utcnow()

        delta_time = current_time - start_time

        print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")
        print "DS3231=\t\t%s" % ds3231.read_datetime()

        print "DS3231 Temperature= \t%0.2f C" % ds3231.getTemp()
        print "----------------- "

        response['DS3231'] = {'raspberry_pi': time.strftime("%Y-%m-%d %H:%M:%S"), 'time': "%s" % ds3231.read_datetime(),
                              'temperature': ds3231.getTemp()}

    print "----------------- "
    print " WeatherRack Weather Sensors"
    if config.WXLink_Present:
        print " WXLink Remote WeatherRack"
    else:
        print " WeatherRack Local"
    print "----------------- "
    #
    print "----------------- "
    if config.AM2315_Present:
        print " AM2315 Temperature/Humidity Sensor"
    else:
        print " AM2315 Temperature/Humidity  Sensor Not Present"
    print "----------------- "

    if config.AM2315_Present:
        # noinspection PyUnboundLocalVariable
        temperature, humidity, crc_check = am2315.sense()
        print "AM2315 temperature: %0.1f" % temperature
        print "AM2315 humidity: %0.1f" % humidity
        print "AM2315 crc: %s" % crc_check
        response['AM2315'] = {'temperature': "%0.1f" % temperature,
                              'humidity': "%0.1f" % humidity,
                              'crc': "%s" % crc_check}
    print "----------------- "
    print "----------------- "

    if not config.WXLink_Present:

        currentWindSpeed = weatherStation.current_wind_speed() / 1.6
        currentWindGust = weatherStation.get_wind_gust() / 1.6
        totalRain += weatherStation.get_current_rain_total() / 25.4
        print "Rain Total=\t%0.2f in" % totalRain
        print 'Wind Speed=\t%0.2f MPH' % currentWindSpeed
        # noinspection PyUnresolvedReferences

        print "MPH wind_gust=\t%0.2f MPH" % currentWindGust

        if config.ADS1015_Present or config.ADS1115_Present:
            print "Wind Direction=\t\t\t %0.2f Degrees" % weatherStation.current_wind_direction()
            print "Wind Direction Voltage=\t\t %0.3f V" % weatherStation.current_wind_direction_voltage()

        response['weather_rack'] = {'rain_total': "%0.2f" % totalRain, 'wind_speed': "%0.2f" % currentWindSpeed,
                                    'wind_direction': "%0.2f" % weatherStation.current_wind_direction(),
                                    'wind_voltage': "%0.3f" % weatherStation.current_wind_direction_voltage()}

    if config.WXLink_Present:
        old_block1 = block1
        old_block2 = block2
        try:
            print "-----------"
            print "block 1"
            block1 = WXLink.read_i2c_block_data(0x08, 0)
            print ''.join('{:02x}'.format(x) for x in block1)
            block1 = bytearray(block1)
            print "block 2"
            block2 = WXLink.read_i2c_block_data(0x08, 1)
            block2 = bytearray(block2)
            print ''.join('{:02x}'.format(x) for x in block2)
            print "-----------"
        except:
            print "WXLink Read failed - Old Data Kept"
            block1 = old_block1
            block2 = old_block2

        currentWindSpeed = struct.unpack('f', str(block1[9:13]))[0] / 1.6

        currentWindGust = 0.0  # not implemented in Solar WXLink version

        totalRain = struct.unpack('l', str(block1[17:21]))[0] / 25.4

        print "Rain Total=\t%0.2f in" % totalRain
        print "Wind Speed=\t%0.2f MPH" % currentWindSpeed
        # noinspection PyUnresolvedReferences

        currentWindDirection = struct.unpack('H', str(block1[7:9]))[0]
        print "Wind Direction=\t\t\t %i Degrees" % currentWindDirection

        # now do the AM2315 Temperature
        temperature = struct.unpack('f', str(block1[25:29]))[0]
        elements = [block1[29], block1[30], block1[31], block2[0]]
        outHByte = bytearray(elements)
        humidity = struct.unpack('f', str(outHByte))[0]
        print "AM2315 from WXLink temperature: %0.1f" % temperature
        print "AM2315 from WXLink humidity: %0.1f" % humidity

        # now read the SunAirPlus Data from WXLink

        batteryVoltage = struct.unpack('f', str(block2[1:5]))[0]
        batteryCurrent = struct.unpack('f', str(block2[5:9]))[0]
        loadCurrent = struct.unpack('f', str(block2[9:13]))[0]
        solarPanelVoltage = struct.unpack('f', str(block2[13:17]))[0]
        solarPanelCurrent = struct.unpack('f', str(block2[17:21]))[0]

        auxA = struct.unpack('f', str(block2[21:25]))[0]

        print "WXLink batteryVoltage = %6.2f" % batteryVoltage
        print "WXLink batteryCurrent = %6.2f" % batteryCurrent
        print "WXLink loadCurrent = %6.2f" % loadCurrent
        print "WXLink solarPanelVoltage = %6.2f" % solarPanelVoltage
        print "WXLink solarPanelCurrent = %6.2f" % solarPanelCurrent
        print "WXLink auxA = %6.2f" % auxA

        # message ID
        MessageID = struct.unpack('l', str(block2[25:29]))[0]
        print "WXLink Message ID %i" % MessageID

    print "----------------- "
    print "----------------- "
    if config.BMP280_Present:
        print " BMP280 Barometer"
    else:
        print " BMP280 Barometer Not Present"
    print "----------------- "

    if config.BMP280_Present:
        print 'Temperature = \t{0:0.2f} C'.format(bmp280.read_temperature())
        print 'Pressure = \t{0:0.2f} KPa'.format(bmp280.read_pressure() / 1000)
        print 'Altitude = \t{0:0.2f} m'.format(bmp280.read_altitude())
        print 'Sealevel Pressure = \t{0:0.2f} KPa'.format(bmp280.read_sealevel_pressure() / 1000)
        # noinspection PyUnresolvedReferences
    print "----------------- "

    print "----------------- "
    response['BMP280'] = {'Temperature': '{0:0.2f}'.format(bmp280.read_temperature()),
                          'Pressure': '{0:0.2f}'.format(bmp280.read_pressure() / 1000),
                          'Altitude': '{0:0.2f}'.format(bmp280.read_altitude()),
                          'SeaLevelPressure': '{0:0.2f}'.format(bmp280.read_sealevel_pressure() / 1000)}

    if config.HTU21DF_Present:
        print " HTU21DF Temp/Hum"
    else:
        print " HTU21DF Temp/Hum Not Present"
    print "----------------- "

    # We use a C library for this device as it just doesn't play well with Python and smbus/I2C libraries
    if config.HTU21DF_Present:
        HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib", "-l"])
        split_string = HTU21DFOut.split()

        htu_temperature = float(split_string[0])
        htu_humidity = float(split_string[1])
        print "Temperature XXXX = \t%0.2f C" % htu_temperature
        print "Humidity = \t%0.2f %%" % htu_humidity
        # noinspection PyUnresolvedReferences
    print "----------------- "

    return response


while True:
    print get_weather_data()
    print "Sleeping 10 seconds..."
    time.sleep(10.0)
