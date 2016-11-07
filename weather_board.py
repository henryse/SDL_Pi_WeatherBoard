# =========================================================================
#  System Imports....
# =========================================================================

import sys
import time
from datetime import datetime
import random
import traceback
import config

# =========================================================================
#  Raspberry PI imports
# =========================================================================

enable_pi_emulator = False

try:
    # noinspection PyUnresolvedReferences
    import RPi.GPIO as GPIO
    # noinspection PyUnresolvedReferences
    import smbus
except ImportError:
    enable_pi_emulator = True

sys.path.append('./SDL_Pi_INA3221')
sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./SDL_Pi_TCA9545')

# noinspection PyUnresolvedReferences
import SDL_DS3231
# noinspection PyUnresolvedReferences
import Adafruit_BMP.BMP280 as BMP280
# noinspection PyUnresolvedReferences
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack

# =========================================================================
#    I2C ADDRESS/BITS
# =========================================================================
TCA9545_ADDRESS = 0x73  # 1110011 (A0+A1=VDD)
# =========================================================================

# =========================================================================
#    CONFIG REGISTER (R/W)
# =========================================================================
TCA9545_REG_CONFIG = 0x00
TCA9545_CONFIG_BUS0 = 0x01  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS1 = 0x02  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS2 = 0x04  # 1 = enable, 0 = disable
TCA9545_CONFIG_BUS3 = 0x08  # 1 = enable, 0 = disable

# =========================================================================


# =========================================================================
# Device Present State Variables
# indicate interrupt has happened from as3936
# =========================================================================

as3935_Interrupt_Happened = False

# =========================================================================
# set to true if you are building the solar powered version
# =========================================================================

config.SolarPower_Mode = False

config.DS3231_Present = False
config.BMP280_Present = False
config.AM2315_Present = False
config.ADS1015_Present = False
config.ADS1115_Present = False


# =========================================================================
# setup lightning i2c mux
# =========================================================================

def returnStatusLine(device, state):
    return_string = device
    if state:
        return_string += ":   \t\tPresent"
    else:
        assert isinstance(return_string, object)
        return_string += ":   \t\tNot Present"
    return return_string


# =========================================================================
# WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
# =========================================================================

anemometerPin = 21
rainPin = 20

# =========================================================================
# internally, the library checks for ADS1115 or ADS1015 if found
# =========================================================================

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1

# =========================================================================
# sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
# =========================================================================
SDL_MODE_SAMPLE = 0

# =========================================================================
# Delay mode means to wait for sampleTime and the average after that time.
# =========================================================================
SDL_MODE_DELAY = 1

weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(anemometerPin, rainPin, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
# weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)

# =========================================================================
# DS3231/AT24C32 Setup
# =========================================================================
filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
start_time = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)

try:
    # =========================================================================
    # comment out the next line after the clock has been initialized
    # =========================================================================
    ds3231.write_now()
    print "DS3231=\t\t%s" % ds3231.read_datetime()
    config.DS3231_Present = True
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
    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.DS3231_Present = False

# =========================================================================
# BMP280 Setup
# =========================================================================

try:
    bmp280 = BMP280.BMP280()
    config.BMP280_Present = True

except IOError as e:
    print "I/O error({0}): {1}".format(e.errno, e.strerror)
    config.BMP280_Present = False


def output_config():
    print "----------------------"
    print returnStatusLine("DS3231", config.DS3231_Present)
    print returnStatusLine("BMP280", config.BMP280_Present)
    print returnStatusLine("AM2315", config.AM2315_Present)
    print returnStatusLine("ADS1015", config.ADS1015_Present)
    print returnStatusLine("ADS1115", config.ADS1115_Present)
    print returnStatusLine("DS3231", config.DS3231_Present)
    print "----------------------"


def check_weather_health():
    output_config()
    return config.DS3231_Present and config.BMP280_Present and config.ADS1115_Present


def get_weather_data():
    response = {}

    try:
        # =========================================================================
        # Detect AM2315
        # =========================================================================
        try:
            # noinspection PyUnresolvedReferences,PyPackageRequirements
            from tentacle_pi.AM2315 import AM2315
            try:
                am2315 = AM2315(0x5c, "/dev/i2c-1")
                temperature, humidity, crc_check = am2315.sense()
                print "AM2315 =", temperature
                config.AM2315_Present = True
                if crc_check == -1:
                    config.AM2315_Present = False
            except:
                config.AM2315_Present = False
        except:
            config.AM2315_Present = False
            print "------> See Readme to install tentacle_pi"

        if config.DS3231_Present:
            response['DS3231'] = {'raspberry_pi': time.strftime("%Y-%m-%d %H:%M:%S"),
                                  'time': "%s" % ds3231.read_datetime(),
                                  'temperature': ds3231.getTemp()}

        if config.AM2315_Present:
            # noinspection PyUnboundLocalVariable
            temperature, humidity, crc_check = am2315.sense()
            response['AM2315'] = {'temperature': "%0.1f" % temperature,
                                  'humidity': "%0.1f" % humidity,
                                  'crc': "%s" % crc_check}

        currentWindSpeed = weatherStation.current_wind_speed()
        currentWindGust = weatherStation.get_wind_gust()
        totalRain = weatherStation.get_current_rain_total()

        if config.ADS1015_Present or config.ADS1115_Present:
            response['ADS1015'] = {'wind_direction': "%0.2f" % weatherStation.current_wind_direction(),
                                   'wind_voltage': "%0.3f" % weatherStation.current_wind_direction_voltage()}

        response['weather_rack'] = {'rain_total': "%0.2f" % totalRain,
                                    'wind_speed': "%0.2f" % currentWindSpeed,
                                    'wind_gust': "%0.2f" % currentWindGust}

        if config.BMP280_Present:
            response['BMP280'] = {'temperature': '{0:0.2f}'.format(bmp280.read_temperature()),
                                  'pressure': '{0:0.2f}'.format(bmp280.read_pressure() / 1000),
                                  'altitude': '{0:0.2f}'.format(bmp280.read_altitude()),
                                  'sea_level_pressure': '{0:0.2f}'.format(bmp280.read_sealevel_pressure() / 1000)}
    except Exception as error:
        traceback.print_exc()
        print error.message

    return response


if __name__ == '__main__':
    while True:
        print get_weather_data()
        print "Sleeping 10 seconds..."
        time.sleep(10.0)
