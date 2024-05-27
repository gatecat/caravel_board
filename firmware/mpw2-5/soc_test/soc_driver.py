#!/usr/bin/env python3

# from pyftdi.ftdi import Ftdi
import time
import sys, os
# from pyftdi.spi import SpiController
from array import array as Array
import binascii
from io import StringIO

from nucleo_api import Test, ProgSupply

from machine import Pin, SPI, SoftSPI, sleep, I2C, SoftI2C

CARAVEL_PASSTHRU = 0xC4
CARAVEL_STREAM_READ = 0x40
CARAVEL_STREAM_WRITE = 0x80
CARAVEL_REG_READ = 0x48
CARAVEL_REG_WRITE = 0x88
CARAVEL_REG_READ2 = 0x50
CARAVEL_REG_WRITE2 = 0x90


class SPI:
    def __init__(self, enabled=True):

        if enabled:
            self.cs = Pin('SPI5_CS', mode=Pin.OUT, value=1)
            self.sck = Pin('SPI5_SCK', mode=Pin.OUT, value=0)
            self.mosi = Pin('SPI5_MISO', mode=Pin.OUT)  # PF9 = IO[2] = caravel input
            self.miso = Pin('SPI5_MOSI', mode=Pin.IN)  # PF8 = IO[1] = caravel output
            self.spi = SoftSPI(baudrate=00000, polarity=0, phase=0, sck=self.sck, mosi=self.mosi, miso=self.miso)
        else:
            self.cs = Pin('SPI5_CS', mode=Pin.IN, pull=None)
            self.sck = Pin('SPI5_SCK', mode=Pin.IN, pull=None)
            self.mosi = Pin('SPI5_MISO', mode=Pin.IN, pull=None)  # PF9 = IO[2] = caravel input
            self.miso = Pin('SPI5_MOSI', mode=Pin.IN, pull=None)  # PF8 = IO[1] = caravel output
            self.spi = None

    def write(self, buf):
        txdata = bytearray(buf)
        self.cs.value(0)
        self.spi.write(txdata)
        self.cs.value(1)

    def exchange(self, buf, n):
        
        txdata = bytearray(buf)
        txdata += '\0'*(n)
        m = len(txdata)
        rxdata = bytearray(m)

        self.cs.value(0)
        self.spi.write_readinto(txdata, rxdata)
        self.cs.value(1)
        return rxdata[m-n:m]

    def set_gpio(self, value):
        for i in range(4):
            self.write([CARAVEL_STREAM_WRITE, 0x6d-i, (value >> (8*i)) & 0xFF])

    def set_mprj_mode(self, io, value):
        self.write([CARAVEL_STREAM_WRITE, 0x1d+2*io+0, (value >> 8) & 0x1F])
        self.write([CARAVEL_STREAM_WRITE, 0x1d+2*io+1, value & 0xFF])

    def yeet_mprj(self):
        self.write([CARAVEL_STREAM_WRITE, 0x13, 1])
        time.sleep(0.1)

def run():
    print("Powering up...")
    slave = SPI()
    # in some cases, you may need to comment or uncomment this line
    #slave.write([CARAVEL_REG_WRITE, 0x0b, 0x01])
    # ------------

    print(" ")
    print("Caravel data:")
    mfg = slave.exchange([CARAVEL_STREAM_READ, 0x01], 2)
    print("   mfg        = {:04x}".format(int.from_bytes(mfg, 'big')))

    product = slave.exchange([CARAVEL_REG_READ, 0x03], 1)
    print("   product    = {:02x}".format(int.from_bytes(product, 'big')))

    data = slave.exchange([CARAVEL_STREAM_READ, 0x04], 4)
    print("   project ID = {:08x}".format(int.from_bytes(data, 'big')))

    # disable HKSPI
    slave.write([CARAVEL_REG_WRITE, 0x6f, 0xff])

    data = slave.exchange([CARAVEL_STREAM_READ, 0x03], 4)
    print("   disabled ID = {:08x} (should be 00)".format(int.from_bytes(data, 'big')))

    slave.__init__(enabled=False)

    supply = ProgSupply()
    # set vcc to 1.8V for better fabric perf
    voltage = 1.8
    R2 = 360 / ((voltage / 1.25) - 1)
    Rpot = (1 / (1 / R2 - 1 / 5000)) - 500
    P = Rpot / 38.911
    Pval = int(P)

    print('Writing ' + str(Pval) + ' to potentiometer.')
    supply.write_1v8(Pval)

    soc_clk  = Pin('IO_0', mode=Pin.OUT, value=0)
    soc_rstn = Pin('IO_1', mode=Pin.OUT, value=1)
    soc_flash_clk = Pin('IO_2', mode=Pin.IN, pull=0)
    soc_flash_csn = Pin('IO_3', mode=Pin.IN, pull=0)
    soc_flash_d0 = Pin('IO_4', mode=Pin.IN, pull=0)
    soc_flash_d1 = Pin('IO_5', mode=Pin.IN, pull=0)
    soc_flash_d2 = Pin('IO_6', mode=Pin.IN, pull=0)
    soc_flash_d3 = Pin('IO_7', mode=Pin.IN, pull=0)

    heartbeat_0  = Pin('IO_29', mode=Pin.IN, pull=0)
    heartbeat_1  = Pin('IO_30', mode=Pin.IN, pull=0)
    rst_inv_out  = Pin('IO_31', mode=Pin.IN, pull=0)
    rst_sync_out = Pin('IO_32', mode=Pin.IN, pull=0)

    def clk():
        soc_clk.value(0)
        soc_clk.value(1)

    soc_rstn.value(0)
    print(f"rst_inv_out={rst_inv_out.value()} rst_sync_out={rst_sync_out.value()}")
    soc_rstn.value(1)
    clk()
    clk()
    print(f"rst_inv_out={rst_inv_out.value()} rst_sync_out={rst_sync_out.value()}")
    clk()
    print(f"heartbeat_0={heartbeat_0.value()}")
    clk()
    print(f"heartbeat_0={heartbeat_0.value()}")
    clk()
    print(f"heartbeat_0={heartbeat_0.value()}")
