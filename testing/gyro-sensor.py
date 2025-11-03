#!/usr/bin/env python3
import time, sys, signal, termios, tty
from smbus2 import SMBus, i2c_msg

MUX_ADDR = 0x70
MUX_CH   = 4
MPU_ADDR = None  # autodetect 0x68/0x69

PWR_MGMT_1  = 0x6B
SMPLRT_DIV  = 0x19
CONFIG      = 0x1A
GYRO_CONFIG = 0x1B
WHO_AM_I    = 0x75
GYRO_ZOUT_H = 0x47

GYR_SENS = 131.0  # ±250 dps

def mux_select(bus, ch): bus.write_byte(MUX_ADDR, 1 << ch)

def mpu_w(bus, reg, val): bus.write_byte_data(MPU_ADDR, reg, val)

def rd16s(bus, reg_h):
    h, l = bus.read_i2c_block_data(MPU_ADDR, reg_h, 2)
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v

def init_mpu(bus):
    mpu_w(bus, PWR_MGMT_1, 0x00)  # wake
    time.sleep(0.05)
    mpu_w(bus, PWR_MGMT_1, 0x01)  # clock = PLL X axis
    mpu_w(bus, CONFIG, 0x03)      # DLPF ~44Hz
    mpu_w(bus, SMPLRT_DIV, 4)     # ~200/(1+4) ≈ 40 Hz
    mpu_w(bus, GYRO_CONFIG, 0x00) # ±250 dps
    time.sleep(0.05)

def kbhit():
    import select
    return select.select([sys.stdin], [], [], 0)[0]

def main():
    global MPU_ADDR
    with SMBus(1) as bus:
        mux_select(bus, MUX_CH)

        # autodetect 0x68/0x69
        for addr in (0x68, 0x69):
            try:
                bus.read_byte_data(addr, WHO_AM_I)
                MPU_ADDR = addr
                break
            except: pass
        if MPU_ADDR is None:
            print("MPU-6500 nenájdený na CH0 (skús 0x68/0x69).")
            return

        who = bus.read_byte_data(MPU_ADDR, WHO_AM_I)
        if who != 0x70:
            print(f"Pozor: WHO_AM_I=0x{who:02X} (očak. 0x70 pre MPU-6500). Pokračujem…")

        init_mpu(bus)

        # kalibrácia biasu
        N, s = 200, 0.0
        for _ in range(N):
            mux_select(bus, MUX_CH)
            s += rd16s(bus, GYRO_ZOUT_H) / GYR_SENS
            time.sleep(0.003)
        gz_bias = s / N

        yaw, t_last = 0.0, time.perf_counter()

        # terminál: skry kurzor + raw mód pre čítanie kláves
        sys.stdout.write("\033[?25l")
        sys.stdout.flush()
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        def tidy_exit(*_):
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
            sys.exit(0)
        signal.signal(signal.SIGINT, tidy_exit)
        signal.signal(signal.SIGTERM, tidy_exit)

        print(f"Gz[°/s]   |  YAW[°]   (MPU @ 0x{MPU_ADDR:02X}, MUX CH{MUX_CH})   [r = zero yaw]")
        while True:
            mux_select(bus, MUX_CH)
            try:
                gz = rd16s(bus, GYRO_ZOUT_H) / GYR_SENS - gz_bias
            except Exception:
                gz = float('nan')  # dočasná I2C chyba

            t = time.perf_counter()
            dt, t_last = t - t_last, t
            if not (gz != gz):  # not NaN
                yaw += gz * dt
                if yaw > 180 or yaw < -180:
                    yaw = (yaw + 180) % 360 - 180

            sys.stdout.write("\r%+8.2f | %+8.2f" % (gz, yaw))
            sys.stdout.flush()

            if kbhit():
                ch = sys.stdin.read(1)
                if ch.lower() == 'r':
                    yaw = 0.0
                elif ch == '\x03' or ch == 'q':
                    tidy_exit()

            time.sleep(0.02)

if __name__ == "__main__":
    try:
        main()
    finally:
        sys.stdout.write("\033[?25h")
        sys.stdout.flush()
