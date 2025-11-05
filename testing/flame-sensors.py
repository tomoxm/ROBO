#!/usr/bin/env python3
# Jednoduchý test 5CH detektora plameňa cez I2C multiplexer (0x70, kanál 5)
# a digitálny expander (predpoklad PCF8574 na adrese 0x20).
# Logika: 0 = plameň, 1 = nič.

import time
from smbus2 import SMBus

I2C_BUS  = 1
MUX_ADDR = 0x70
MUX_CH   = 5
PCF_ADDR = 0x20           # zmeň podľa potreby
PINS     = [0, 1, 2, 3, 4]
POLL_MS  = 100

def mux_select(i2c, ch):
    i2c.write_byte(MUX_ADDR, 1 << ch)

def main():
    with SMBus(I2C_BUS) as i2c:
        mux_select(i2c, MUX_CH)
        i2c.write_byte(PCF_ADDR, 0xFF)  # nastaviť všetky piny ako vstupy

        last = None
        while True:
            raw = i2c.read_byte(PCF_ADDR)  # bity P7..P0
            states = {p: 0 if ((raw >> p) & 1) == 0 else 1 for p in PINS}  # 0=plameň
            if states != last:
                active = [f"D{p}" for p in PINS if states[p] == 0]
                print(f"{'🔥 ' + ', '.join(active) if active else '– Žiadny plameň'}  (raw=0b{raw:08b})")
                last = states
            time.sleep(POLL_MS / 1000)

if __name__ == "__main__":
    main()
