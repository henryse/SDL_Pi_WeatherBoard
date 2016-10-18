#!/usr/bin/env python

# SDL_Pi_FRAM.py Python Driver Code
# SwitchDoc Labs 02/16/2014
# V 1.2

enable_pi_emulator = False
try:
    # noinspection PyUnresolvedReferences
    import smbus
except ImportError:
    enable_pi_emulator = True


class SDL_Pi_FRAM:
    ###########################
    # SDL_Pi_FRAM Code
    ###########################
    def __init__(self, twi=1, addr=0x50):
        if not enable_pi_emulator:
            self._bus = smbus.SMBus(twi)
        self._addr = addr

    def write8(self, address, data):
        # print "addr =0x%x address = 0x%x data = 0x%x  " % (self._addr, address, data)
        if not enable_pi_emulator:
            self._bus.write_i2c_block_data(self._addr, address >> 8, [address % 256, data])

    def read8(self, address):
        if not enable_pi_emulator:
            self._bus.write_i2c_block_data(self._addr, address >> 8, [address % 256])

        return_data = [0]
        if not enable_pi_emulator:
            return_data = self._bus.read_byte(self._addr)
        # this will read at the current address pointer, which we on the previous line
        # print "addr = 0x%x address = 0x%x %i return_data = 0x%x " % (self._addr, address, address, return_data)
        return return_data
