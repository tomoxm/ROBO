#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 3× VL53L1X cez TCA9548A (0x70) — 1 riadok v konzole (prepísanie)
import time, sys
import board, busio
import adafruit_vl53l1x

MUX_ADDR = 0x70
CHANNELS = [1, 2, 3]
REFRESH  = 0.1  # s

CSI = "\033["  # ANSI escape

def mux_select(i2c, ch):
    i2c.writeto(MUX_ADDR, bytes([1 << ch]))
    time.sleep(0.002)

def clear_line():
    # vymaž aktuálny riadok a vráť kurzor na začiatok
    sys.stdout.write(CSI + "2K" + "\r")

def main():
    i2c = busio.I2C(board.SCL, board.SDA)

    # init senzorov
    sensors = {}
    for ch in CHANNELS:
        try:
            mux_select(i2c, ch)
            vl = adafruit_vl53l1x.VL53L1X(i2c)
            vl.start_ranging()
            sensors[ch] = vl
        except Exception as e:
            sensors[ch] = None
            # nechceme nové riadky, tak len raz vypíšeme a potom ticho
            sys.stderr.write(f"\n[WARN] ch{ch}: {e}\n")

    # info header (1x)
    sys.stdout.write("VL53L1X @ TCA9548A 0x70 — Ctrl+C na ukončenie\n")

    try:
        while True:
            parts = []
            for ch in CHANNELS:
                vl = sensors.get(ch)
                if vl is None:
                    parts.append(f"ch{ch}: n/a")
                    continue
                try:
                    mux_select(i2c, ch)
                    if vl.data_ready:
                        d = vl.distance  # cm
                        vl.clear_interrupt()
                        parts.append(f"ch{ch}: {d:>4} cm" if d is not None else f"ch{ch}: ----")
                    else:
                        parts.append(f"ch{ch}:  ...")
                except Exception as e:
                    parts.append(f"ch{ch}: err")
            line = " | ".join(parts)

            clear_line()
            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(REFRESH)
    except KeyboardInterrupt:
        pass
    finally:
        for vl in sensors.values():
            if vl:
                try: vl.stop_ranging()
                except: pass
        try: i2c.writeto(MUX_ADDR, b"\x00")
        except: pass
        clear_line()
        sys.stdout.write("Koniec.\n")
        sys.stdout.flush()

if __name__ == "__main__":
    main()
