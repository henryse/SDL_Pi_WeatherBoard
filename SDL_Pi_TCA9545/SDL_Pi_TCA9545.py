#!/usr/bin/env python

# SDL_Pi_TCA9545.py Python Driver Code
# SwitchDoc Labs April 1, 2015	 
# V 1.2


# encoding: utf-8

enable_pi_emulator = False
try:
    # noinspection PyUnresolvedReferences
    import smbus
except ImportError:
    enable_pi_emulator = True

# constants

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


class SDL_Pi_TCA9545:
    # =========================================================================
    # TCA9545 Code
    # =========================================================================
    def __init__(self, twi=1, addr=TCA9545_ADDRESS, bus_enable=TCA9545_CONFIG_BUS0):
        if not enable_pi_emulator:
            self._bus = smbus.SMBus(twi)
        self._addr = addr
        config = bus_enable
        self._write(TCA9545_REG_CONFIG, config)

    def _write(self, register, data):
        # print "addr =0x%x register = 0x%x data = 0x%x " % (self._addr, register, data)
        if not enable_pi_emulator:
            self._bus.write_byte_data(self._addr, register, data)

    def _read(self):
        return_data = 0
        if not enable_pi_emulator:
            return_data = self._bus.read_byte(self._addr)
        # print "addr = 0x%x return_data = 0x%x " % (self._addr, return_data)
        return return_data

    # public functions

    def read_control_register(self):
        # Reads Control Register

        value = self._read()
        return value

    def write_control_register(self, config):
        # Writes Control Register

        self._write(TCA9545_REG_CONFIG, config)
