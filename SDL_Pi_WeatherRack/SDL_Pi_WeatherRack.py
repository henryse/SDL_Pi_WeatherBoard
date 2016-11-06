#
#
#
#  SDL_Pi_WeatherRack.py - Raspberry Pi Python Library for SwitchDoc Labs WeatherRack.
#
#  SparkFun Weather Station Meters
#  Argent Data Systems
#  Created by SwitchDoc Labs February 13, 2015 
#  Released into the public domain.
#    Version 1.3 - remove 300ms Bounce
#    Version 2.0 - Update for WeatherPiArduino V2
#    

# imports

try:
    import config
except:
    import NoWPAConfig.py as config

import sys
import time as time_
import traceback

sys.path.append('./Adafruit_ADS1x15')

enable_pi_emulator = False
try:
    from Adafruit_ADS1x15 import ADS1x15
    import RPi.GPIO as GPIO

    GPIO.setwarnings(False)
except ImportError:
    enable_pi_emulator = True

# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1

# sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
# Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

WIND_FACTOR = 2.400


# Helper Functions

def fuzzyCompare(compareValue, value):
    VARYVALUE = 0.05

    if (value > (compareValue * (1.0 - VARYVALUE))) and (value < (compareValue * (1.0 + VARYVALUE))):
        return True

    return False


def voltageToDegrees(value, defaultWindDirection):
    # Note: The original documentation for the wind vane says 16 positions.
    # Typically only recieve 8 positions.  And 315 degrees was wrong.

    # For 5V, use 1.0.  For 3.3V use 0.66
    ADJUST3OR5 = 0.66

    if fuzzyCompare(3.84 * ADJUST3OR5, value):
        return 0.0

    if fuzzyCompare(1.98 * ADJUST3OR5, value):
        return 22.5

    if fuzzyCompare(2.25 * ADJUST3OR5, value):
        return 45

    if fuzzyCompare(0.41 * ADJUST3OR5, value):
        return 67.5

    if fuzzyCompare(0.45 * ADJUST3OR5, value):
        return 90.0

    if fuzzyCompare(0.32 * ADJUST3OR5, value):
        return 112.5

    if fuzzyCompare(0.90 * ADJUST3OR5, value):
        return 135.0

    if fuzzyCompare(0.62 * ADJUST3OR5, value):
        return 157.5

    if fuzzyCompare(1.40 * ADJUST3OR5, value):
        return 180

    if fuzzyCompare(1.19 * ADJUST3OR5, value):
        return 202.5

    if fuzzyCompare(3.08 * ADJUST3OR5, value):
        return 225

    if fuzzyCompare(2.93 * ADJUST3OR5, value):
        return 247.5

    if fuzzyCompare(4.62 * ADJUST3OR5, value):
        return 270.0

    if fuzzyCompare(4.04 * ADJUST3OR5, value):
        return 292.5

    if fuzzyCompare(4.34 * ADJUST3OR5, value):  # chart in manufacturers documentation wrong
        return 315.0

    if fuzzyCompare(3.43 * ADJUST3OR5, value):
        return 337.5

    return defaultWindDirection  # return previous value if not found


# return current microseconds
def micros():
    microseconds = int(round(time_.time() * 1000000))
    return microseconds


class SDL_Pi_WeatherRack:
    if not enable_pi_emulator:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    # instance variables
    _currentWindCount = 0
    _currentRainCount = 0
    _shortestWindTime = 0

    _pinAnem = 0
    _pinRain = 0
    _ADChannel = 0
    _ADMode = 0

    _currentWindSpeed = 0.0
    _currentWindDirection = 0.0
    _lastWindTime = 0

    _sampleTime = 5.0
    _selectedMode = SDL_MODE_SAMPLE
    _startSampleTime = 0

    _currentRainMin = 0
    _lastRainTime = 0

    _ads1015 = 0

    def __init__(self, pinAnem, pinRain, ADMode):

        if not enable_pi_emulator:
            GPIO.setup(pinAnem, GPIO.IN)
            GPIO.setup(pinRain, GPIO.IN)

        # when a falling edge is detected on port pinAnem, regardless of whatever
        # else is happening in the program, the function callback will be run
        if not enable_pi_emulator:
            GPIO.add_event_detect(pinAnem, GPIO.RISING, callback=self.serviceInterruptAnem)
            GPIO.add_event_detect(pinRain, GPIO.RISING, callback=self.serviceInterruptRain)

        ADS1015 = 0x00  # 12-bit ADC
        ADS1115 = 0x01  # 16-bit ADC
        # Select the gain
        self.gain = 6144  # +/- 6.144V
        # self.gain = 4096  # +/- 4.096V

        # Select the sample rate
        self.sps = 250  # 250 samples per second

        # Initialise the ADC using the default mode (use default I2C address)
        # Set this to ADS1015 or ADS1115 depending on the ADC you are using!
        if not enable_pi_emulator:
            self.ads1015 = ADS1x15(ic=ADS1015, address=0x48)

        # determine if device present
        try:
            self.ads1015.readRaw(1, self.gain, self.sps)  # AIN1 wired to wind vane on WeatherPiArduino
            time_.sleep(1.0)
            value = self.ads1015.readRaw(1, self.gain, self.sps)  # AIN1 wired to wind vane on WeatherPiArduino

            # now figure out if it is an ADS1015 or ADS1115
            if (0x0F & value) == 0:
                config.ADS1015_Present = True
                config.ADS1115_Present = False
                # check again (1 out 16 chance of zero)
                value = self.ads1015.readRaw(0, self.gain, self.sps)  # AIN1 wired to wind vane on WeatherPiArduino
                if ((0x0F & value) == 0):
                    config.ADS1015_Present = True
                    config.ADS1115_Present = False

                else:
                    config.ADS1015_Present = False
                    config.ADS1115_Present = True
                    if not enable_pi_emulator:
                        self.ads1015 = ADS1x15(ic=ADS1115, address=0x48)
            else:
                config.ADS1015_Present = False
                config.ADS1115_Present = True
                if not enable_pi_emulator:
                    self.ads1015 = ADS1x15(ic=ADS1115, address=0x48)

        except TypeError as e:
            traceback.print_exc()
            print e.message
            config.ADS1015_Present = False
            config.ADS1115_Present = False

        self._ADMode = ADMode

    # Wind Direction Routines

    def current_wind_direction(self):

        if self._ADMode == SDL_MODE_I2C_ADS1015:
            value = self.ads1015.readADCSingleEnded(1, self.gain,
                                                    self.sps)  # AIN1 wired to wind vane on WeatherPiArduino

            voltageValue = value / 1000

        else:
            # user internal A/D converter
            voltageValue = 0.0

        direction = voltageToDegrees(voltageValue, self._currentWindDirection)
        return direction

    def current_wind_direction_voltage(self):

        if self._ADMode == SDL_MODE_I2C_ADS1015:
            value = self.ads1015.readADCSingleEnded(1, self.gain,
                                                    self.sps)  # AIN1 wired to wind vane on WeatherPiArduino

            voltageValue = value / 1000

        else:
            # user internal A/D converter
            voltageValue = 0.0

        return voltageValue

    # Utility methods

    def reset_rain_total(self):
        self._currentRainCount = 0

    def accessInternalCurrentWindDirection(self):
        return self._currentWindDirection

    def reset_wind_gust(self):
        self._shortestWindTime = 0xffffffff

    def startWindSample(self, sampleTime):

        self._startSampleTime = micros()

        self._sampleTime = sampleTime

    # get current wind
    def get_current_wind_speed_when_sampling(self):

        compareValue = self._sampleTime * 1000000

        if micros() - self._startSampleTime >= compareValue:
            # sample time exceeded, calculate currentWindSpeed
            timeSpan = (micros() - self._startSampleTime)

            self._currentWindSpeed = (float(self._currentWindCount) / float(
                timeSpan)) * WIND_FACTOR * 1000000.0

            # print "SDL_CWS = %f, self._shortestWindTime = %i, CWCount=%i TPS=%f" %
            # (self._currentWindSpeed,self._shortestWindTime,
            # self._currentWindCount, float(self._
            # currentWindCount)/float(self._sampleTime))

            self._currentWindCount = 0

            self._startSampleTime = micros()

            # print "self._currentWindSpeed=", self._currentWindSpeed
        return self._currentWindSpeed

    def setWindMode(self, selectedMode, sampleTime):  # time in seconds

        self._sampleTime = sampleTime
        self._selectedMode = selectedMode

        if self._selectedMode == SDL_MODE_SAMPLE:
            self.startWindSample(self._sampleTime)

            # def get current values

    def get_current_rain_total(self):
        rain_amount = 0.2794 * float(self._currentRainCount)
        self._currentRainCount = 0
        return rain_amount

    def current_wind_speed(self):  # in milliseconds

        if self._selectedMode == SDL_MODE_SAMPLE:
            self._currentWindSpeed = self.get_current_wind_speed_when_sampling()
        else:
            # km/h * 1000 msec

            self._currentWindCount = 0
            delay(self._sampleTime * 1000)
            self._currentWindSpeed = (float(self._currentWindCount) / float(
                self._sampleTime)) * WIND_FACTOR

        return self._currentWindSpeed

    def get_wind_gust(self):

        latestTime = self._shortestWindTime
        self._shortestWindTime = 0xffffffff
        time = latestTime / 1000000.0  # in microseconds
        if time == 0:
            return 0
        else:
            return (1.0 / float(time)) * WIND_FACTOR

    # Interrupt Routines

    def serviceInterruptAnem(self, channel):

        # print "Anem Interrupt Service Routine"

        currentTime = (micros() - self._lastWindTime)

        self._lastWindTime = micros()

        if currentTime > 1000:  # debounce
            self._currentWindCount += 1

            if currentTime < self._shortestWindTime:
                self._shortestWindTime = currentTime

    def serviceInterruptRain(self, channel):

        # print "Rain Interrupt Service Routine"

        currentTime = (micros() - self._lastRainTime)

        self._lastRainTime = micros()
        if currentTime > 500:  # debounce
            self._currentRainCount += 1
            if currentTime < self._currentRainMin:
                self._currentRainMin = currentTime