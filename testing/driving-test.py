#!/usr/bin/env python3
# four_wheels_gyro_hold_pro_yawdiff_guarded_autostop.py
#
# 'pro + yaw-diff' with robust I2C guard + FRONT LIDAR AUTO-STOP:
# - retries, backoff, re-init on mux/gyro I2C failures
# - control-loop watchdog -> brake + recover
# - VL53L1X on mux ch3, auto-stops if < FRONT_STOP_CM ahead while moving forward
#
# Extra keys:
#   R : force I2C/gyro re-init
#   h : toggle yaw-differential assist
#   f : toggle front auto-stop

import sys, time, signal, termios, tty, math
from math import copysign

# -------- Robot HAT / GPIO / I2C --------
try:
    from robot_hat import Pin as RH_Pin, PWM as RH_PWM
    from gpiozero import OutputDevice
    from smbus2 import SMBus
except Exception as e:
    print(f"[FATAL] dependency import failed: {e}")
    sys.exit(1)

# Adafruit VL53L1X (front distance)
try:
    import board, busio
    import adafruit_vl53l1x
    HAS_VL53 = True
except Exception as e:
    print(f"[WARN] VL53L1X libs not available: {e}")
    HAS_VL53 = False

# ================== CONFIG (pins & parms) ==================
# TB6612 #1 (FR/FL) DIR on BCM:
GPIO_FR_AIN1 = 8    # CS
GPIO_FR_AIN2 = 11   # SCK
GPIO_FL_BIN1 = 10   # MOSI
GPIO_FL_BIN2 = 9    # MISO

PWM_FR = 7   # PWMA -> Front Right
PWM_FL = 6   # PWMB -> Front Left

# TB6612 #2 (RR/RL) DIR on Robot HAT D*:
D_RR_AIN1 = "D0"
D_RR_AIN2 = "D1"
D_RL_BIN1 = "D2"
D_RL_BIN2 = "D3"

PWM_RR = 4   # PWMA -> Rear Right
PWM_RL = 5   # PWMB -> Rear Left

# Wheel inversion / trims / side gains
INVERT_FL = False
INVERT_FR = True
INVERT_RL = False
INVERT_RR = False

LEFT_GAIN  = 1.00
RIGHT_GAIN = 1.00
TRIM_FL = 1.00
TRIM_FR = 1.00
TRIM_RL = 1.00
TRIM_RR = 1.00

# PWM
FRONT_PWM_FREQ = 1000
REAR_PWM_FREQ  = 1000

# UI / speed
BASE_SPEED   = 50
BASE_STEP    = 5
PRINT_HZ     = 25.0
CONTROL_HZ   = 200.0

# =============== Gyro (MPU-6500) over TCA9548A ===============
MUX_ADDR = 0x70
MUX_CH   = 4   # gyro on ch4
WHO_AM_I    = 0x75
PWR_MGMT_1  = 0x6B
SMPLRT_DIV  = 0x19
CONFIG      = 0x1A
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47
GYR_SENS = 131.0  # LSB/(°/s) @ ±250 dps

# ======= Heading-hold controller =======
PID_KP = 0.020
PID_KI = 0.0006
PID_KD = 0.0030
I_CLAMP       = 25.0
DEADBAND_DEG  = 0.5
SCHMITT_ENTER = 1.2
SCHMITT_EXIT  = 0.6
E_SOFT = 10.0
KD_RATE = 0.0025
D_LP_ALPHA = 0.20
I_LEAK_PER_S = 0.10

OMEGA_MAX = 1.0
OMEGA_SLEW_MAX = 3.0

# ---- Polarity knobs ----
GZ_SIGN = +1
CORR_SIGN = -1

# ======= Yaw-differential assist =======
YAW_DIFF_GAIN_PER_DEG = 0.008
YAW_DIFF_MAX_GAIN     = 0.18
YAW_DIFF_ON = True

# ======= I2C Guard / Watchdog =======
I2C_RETRIES       = 3
I2C_BACKOFF_S     = 0.002
I2C_MAX_FAILS     = 25
I2C_REINIT_SLEEP  = 0.06
LOOP_WATCHDOG_S   = 0.5

# ======= Front distance auto-stop (VL53L1X) =======
FRONT_CH          = 3        # your front sensor
FRONT_STOP_CM     = 25.0     # stop if closer than this
FRONT_HYST_CM     = 2.0      # release block when >= STOP+HYST
FRONT_REFRESH_S   = 0.05     # 20 Hz sampling
FRONT_AUTOSTOP_ON = True

# ================== Helpers ==================
def clamp(v, lo, hi): return max(lo, min(hi, v))
def angle_wrap_deg(a): return (a + 180.0) % 360.0 - 180.0
def angle_diff_deg(a, b): return angle_wrap_deg(a - b)

def kbhit():
    import select
    return select.select([sys.stdin], [], [], 0)[0]

def _set_pwm_freq(p, f):
    for attr in ("freq", "frequency", "setfreq", "set_frequency"):
        m = getattr(p, attr, None)
        if callable(m):
            try: m(f); return
            except: pass
    try: p.freq = f
    except: pass

def _set_pwm_duty(p, percent):
    percent = max(0, min(100, int(percent)))
    for attr in ("duty", "set_duty", "pulse_width_percent", "write"):
        m = getattr(p, attr, None)
        if callable(m):
            try: m(percent); return
            except: pass
    dc = getattr(p, "duty_cycle", None)
    if callable(dc):
        try: dc(percent/100.0); return
        except: pass
    if hasattr(p, "value"):
        try: p.value = int(4095*(percent/100.0)); return
        except: pass

# ================== Motor drivers ==================
class TB6612GPIOZeroDir:
    def __init__(self, in1_bcm, in2_bcm, pwm_idx, invert=False, freq=1000, trim=1.0):
        self.in1 = OutputDevice(in1_bcm, active_high=True, initial_value=False)
        self.in2 = OutputDevice(in2_bcm, active_high=True, initial_value=False)
        self.pwm = RH_PWM(pwm_idx)
        self.invert = invert; self.trim = trim
        _set_pwm_freq(self.pwm, freq); self.brake()
    def _set_dir(self, fwd):
        fwd ^= self.invert
        if fwd: self.in1.on();  self.in2.off()
        else:   self.in1.off(); self.in2.on()
    def run(self, signed_pct):
        v = int(clamp(signed_pct, -100, 100) * self.trim)
        self._set_dir(v >= 0); _set_pwm_duty(self.pwm, abs(v))
    def coast(self): self.in1.off(); self.in2.off(); _set_pwm_duty(self.pwm, 0)
    def brake(self): self.in1.on();  self.in2.on();  _set_pwm_duty(self.pwm, 0)
    def close(self):
        try: self.in1.close(); self.in2.close()
        except: pass

class TB6612HatPinsDir:
    def __init__(self, in1_name, in2_name, pwm_idx, invert=False, freq=1000, trim=1.0):
        self.in1 = RH_Pin(in1_name, RH_Pin.OUT)
        self.in2 = RH_Pin(in2_name, RH_Pin.OUT)
        self.pwm = RH_PWM(pwm_idx)
        self.invert = invert; self.trim = trim
        _set_pwm_freq(self.pwm, freq); self.brake()
    def _set_dir(self, fwd):
        fwd ^= self.invert
        if fwd: self.in1.high(); self.in2.low()
        else:   self.in1.low();  self.in2.high()
    def run(self, signed_pct):
        v = int(clamp(signed_pct, -100, 100) * self.trim)
        self._set_dir(v >= 0); _set_pwm_duty(self.pwm, abs(v))
    def coast(self): self.in1.low(); self.in2.low(); _set_pwm_duty(self.pwm, 0)
    def brake(self): self.in1.high(); self.in2.high(); _set_pwm_duty(self.pwm, 0)

# Wheels
FR = TB6612GPIOZeroDir(GPIO_FR_AIN1, GPIO_FR_AIN2, PWM_FR, invert=INVERT_FR, freq=FRONT_PWM_FREQ, trim=TRIM_FR)
FL = TB6612GPIOZeroDir(GPIO_FL_BIN1, GPIO_FL_BIN2, PWM_FL, invert=INVERT_FL, freq=FRONT_PWM_FREQ, trim=TRIM_FL)
RR = TB6612HatPinsDir  (D_RR_AIN1, D_RR_AIN2, PWM_RR, invert=INVERT_RR, freq=REAR_PWM_FREQ,  trim=TRIM_RR)
RL = TB6612HatPinsDir  (D_RL_BIN1, D_RL_BIN2, PWM_RL, invert=INVERT_RL, freq=REAR_PWM_FREQ,  trim=TRIM_RL)

def set_wheels(fl, fr, rl, rr):
    FL.run(int(clamp(fl * LEFT_GAIN,  -100, 100)))
    FR.run(int(clamp(fr * RIGHT_GAIN, -100, 100)))
    RL.run(int(clamp(rl * LEFT_GAIN,  -100, 100)))
    RR.run(int(clamp(rr * RIGHT_GAIN, -100, 100)))

def all_brake():
    FL.brake(); FR.brake(); RL.brake(); RR.brake()

# =============== Yaw-diff bias helper ===============
def apply_yaw_diff_bias(fl, fr, rl, rr, yaw_err_deg, vx, vy):
    if not YAW_DIFF_ON:
        return fl, fr, rl, rr
    forward_dom = abs(vy) / max(1e-6, (abs(vx) + abs(vy)))
    if forward_dom < 0.15:
        return fl, fr, rl, rr
    g = clamp(YAW_DIFF_GAIN_PER_DEG * yaw_err_deg, -YAW_DIFF_MAX_GAIN, YAW_DIFF_MAX_GAIN)
    g *= forward_dom
    fr2 = fr * (1.0 + g); rr2 = rr * (1.0 + g)
    fl2 = fl * (1.0 - g); rl2 = rl * (1.0 - g)
    m = max(1.0, abs(fl2), abs(fr2), abs(rl2), abs(rr2))
    return fl2/m, fr2/m, rl2/m, rr2/m

# ================== Mecanum mixing ==================
def mecanum_drive(vx, vy, omega, base_percent, yaw_err_deg=0.0):
    fl = vy + vx + omega
    fr = vy - vx - omega
    rl = vy - vx + omega
    rr = vy + vx - omega
    maxmag = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))
    fl /= maxmag; fr /= maxmag; rl /= maxmag; rr /= maxmag
    fl, fr, rl, rr = apply_yaw_diff_bias(fl, fr, rl, rr, yaw_err_deg, vx, vy)
    scale = clamp(base_percent, 0, 100)
    set_wheels(fl*scale, fr*scale, rl*scale, rr*scale)

# ================== I2C Guard ==================
class I2CGuard:
    def __init__(self, mux_addr, mux_ch):
        self.mux_addr = mux_addr
        self.mux_ch   = mux_ch
        self.bus = None
        self.mpu_addr = None
        self.gz_bias = 0.0
        self.fail_count = 0

    def open(self):
        if self.bus:
            try: self.bus.close()
            except: pass
        time.sleep(0.01)
        self.bus = SMBus(1)
        time.sleep(0.01)

    def mux_write(self, val):
        for _ in range(I2C_RETRIES):
            try:
                self.bus.write_byte(self.mux_addr, val)
                return True
            except Exception:
                time.sleep(I2C_BACKOFF_S)
        return False

    def mux_select(self):
        # select default (gyro) channel
        return self.mux_select_ch(self.mux_ch)

    def mux_select_ch(self, ch):
        # deselect then select channel 'ch'
        if not self.mux_write(0x00): return False
        time.sleep(0.001)
        return self.mux_write(1 << ch)

    def read_word(self, addr, reg):
        for _ in range(I2C_RETRIES):
            try:
                h, l = self.bus.read_i2c_block_data(addr, reg, 2)
                v = (h << 8) | l
                return v - 65536 if v & 0x8000 else v
            except Exception:
                time.sleep(I2C_BACKOFF_S)
        return None

    def write_reg(self, addr, reg, val):
        for _ in range(I2C_RETRIES):
            try:
                self.bus.write_byte_data(addr, reg, val)
                return True
            except Exception:
                time.sleep(I2C_BACKOFF_S)
        return False

    def init_gyro(self):
        # autodetect 0x68/0x69
        self.mpu_addr = None
        for a in (0x68, 0x69):
            try:
                _ = self.bus.read_byte_data(a, WHO_AM_I)
                self.mpu_addr = a
                break
            except Exception:
                pass
        if self.mpu_addr is None:
            return False
        # configure
        ok =  self.write_reg(self.mpu_addr, PWR_MGMT_1, 0x00)
        ok &= self.write_reg(self.mpu_addr, PWR_MGMT_1, 0x01)
        ok &= self.write_reg(self.mpu_addr, CONFIG,      0x03)
        ok &= self.write_reg(self.mpu_addr, SMPLRT_DIV,  4)
        ok &= self.write_reg(self.mpu_addr, GYRO_CONFIG, 0x00)
        if not ok:
            return False
        time.sleep(0.02)
        # bias
        N = 200; s = 0.0; n_ok = 0
        for _ in range(N):
            if not self.mux_select():
                continue
            w = self.read_word(self.mpu_addr, GYRO_ZOUT_H)
            if w is None:
                continue
            s += (w / GYR_SENS); n_ok += 1
            time.sleep(0.003)
        self.gz_bias = (s / n_ok) if n_ok else 0.0
        return n_ok > 5

    def reinit_all(self):
        try:
            self.open()
            if not self.mux_select():
                return False
            if not self.init_gyro():
                return False
            self.fail_count = 0
            return True
        except Exception:
            return False

# ================== UI text ==================
HELP = '''
--------------- 4-Wheel Mecanum + Gyro Hold (pro + yaw-diff + I2C-guard + FRONT AUTOSTOP) ---------------

NUMPAD:
  7 8 9    8=fwd, 2=back, 4/6=strafe L/R, diagonals 7/9/1/3
  4 5 6    5=stop
  1 2 3    0=rotate left (CCW), .=rotate right (CW)
  +/-      increase/decrease BASE speed

Gyro:
  g        toggle heading lock ON/OFF
  r        set target heading = current yaw
  y        zero yaw estimator to 0°
  o        flip GZ_SIGN (gyro Z sign)
  p        flip CORR_SIGN (correction sign)
  h        toggle yaw-differential assist
  f        toggle FRONT auto-stop
  R        force I2C/gyro re-init
  [SPACE]  stop            [q]/Ctrl+C: quit
'''

def print_status(base, vx, vy, omg_cmd, yaw, tgt, lock_on, gz, omega_corr,
                 yawdiff_on, gz_sign, corr_sign, i2c_fails,
                 front_cm, f_on, f_blocked, stop_cm):
    sys.stdout.write("\033[H\033[J")
    print(HELP)
    print(f"BASE={base:>3}% | vec (vx,vy)=({vx:+.2f},{vy:+.2f}) | ω_user={omg_cmd:+.2f} | ω_corr={omega_corr:+.2f} | yaw-diff={'ON ' if yawdiff_on else 'OFF'}")
    print(f"Yaw={yaw:+7.2f}° | Target={tgt:+7.2f}° | Lock={'ON ' if lock_on else 'OFF'} | Gz={gz:+7.2f} °/s")
    fc = ("----" if (front_cm is None) else f"{front_cm:4.0f} cm")
    print(f"Front={fc} | AutoStop={'ON ' if f_on else 'OFF'} (th={stop_cm:.0f} cm) | BLOCKED={'YES' if f_blocked else 'no '} | I2C fails={i2c_fails}")
    print(f"Signs: GZ_SIGN={gz_sign:+d}  CORR_SIGN={corr_sign:+d}")

# ================== Main ==================
def main():
    global BASE_SPEED, GZ_SIGN, CORR_SIGN, YAW_DIFF_ON, FRONT_AUTOSTOP_ON

    vx = 0.0; vy = 0.0; omg_user = 0.0
    yaw = 0.0; yaw_target = 0.0; lock_on = True

    e_prev = 0.0
    i_term = 0.0
    gz_dlp = 0.0
    corr_active = False
    omega_prev = 0.0

    # Front sensor state
    front_cm = None
    front_blocked = False
    t_front_next = 0.0
    vl = None
    i2c_busio = None

    all_brake()
    sys.stdout.write("\033[?25l"); sys.stdout.flush()
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setcbreak(fd)

    def tidy_exit(*_):
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write("\033[?25h\n"); sys.stdout.flush()
        all_brake()
        # stop ranging + deselect mux
        if vl:
            try: vl.stop_ranging()
            except: pass
        try:
            # deselect all mux channels via SMBus (safe even if guard closed)
            if ig and ig.bus:
                ig.mux_write(0x00)
        except: pass
        sys.exit(0)

    signal.signal(signal.SIGINT, tidy_exit)
    signal.signal(signal.SIGTERM, tidy_exit)

    # --- I2C guard init (gyro on mux ch4) ---
    ig = I2CGuard(MUX_ADDR, MUX_CH)
    if not ig.reinit_all():
        print("[FATAL] I2C/gyro init failed. Check wiring/mux channel.")
        tidy_exit()

    # --- Front VL53L1X init on mux ch3 ---
    if HAS_VL53:
        try:
            i2c_busio = busio.I2C(board.SCL, board.SDA)
            # select ch3 for sensor, then create sensor on that path
            if ig.mux_select_ch(FRONT_CH):
                vl = adafruit_vl53l1x.VL53L1X(i2c_busio)
                # optional: faster refresh for close range responsiveness
                try:
                    vl.distance_mode = 1  # 1=short, 2=long (library dependent)
                except Exception:
                    pass
                try:
                    vl.timing_budget = 33  # ~30-40ms
                except Exception:
                    pass
                vl.start_ranging()
            # switch back to gyro channel for normal loop
            ig.mux_select()
        except Exception as e:
            print(f"[WARN] Front VL53L1X init failed: {e}")
            vl = None

    t_prev = time.perf_counter()
    t_print_next = 0.0
    dt_goal = 1.0 / CONTROL_HZ

    print_status(BASE_SPEED, vx, vy, omg_user, yaw, yaw_target, lock_on, 0.0, 0.0,
                 YAW_DIFF_ON, GZ_SIGN, CORR_SIGN, ig.fail_count,
                 front_cm, FRONT_AUTOSTOP_ON, front_blocked, FRONT_STOP_CM)

    try:
        while True:
            t_now = time.perf_counter()
            dt = t_now - t_prev

            # Loop watchdog
            if dt > LOOP_WATCHDOG_S:
                all_brake()
            # Maintain target rate
            if dt < dt_goal:
                time.sleep(dt_goal - dt)
                t_now = time.perf_counter()
                dt = t_now - t_prev
            t_prev = t_now

            # --- Safe gyro read ---
            gz = 0.0
            got = False
            if ig.mux_select():
                w = ig.read_word(ig.mpu_addr, GYRO_ZOUT_H)
                if w is not None:
                    gz_raw = w / GYR_SENS
                    gz = (gz_raw - ig.gz_bias) * GZ_SIGN
                    got = True

            if not got:
                ig.fail_count += 1
                if ig.fail_count >= I2C_MAX_FAILS:
                    all_brake()
                    time.sleep(I2C_REINIT_SLEEP)
                    if not ig.reinit_all():
                        time.sleep(0.2)
                        if not ig.reinit_all():
                            print("[FATAL] I2C could not recover. Exiting safely.")
                            tidy_exit()
                    continue
            else:
                ig.fail_count = 0

            # integrate yaw + filter
            if got:
                yaw += gz * dt
                yaw = angle_wrap_deg(yaw)
                gz_dlp = D_LP_ALPHA * gz + (1.0 - D_LP_ALPHA) * gz_dlp

            # --- Front sensor sample (rate-limited) ---
            if vl and t_now >= t_front_next:
                # hop to front channel, read if ready, then hop back to gyro ch
                if ig.mux_select_ch(FRONT_CH):
                    try:
                        if vl.data_ready:
                            d = vl.distance  # cm (int/float or None)
                            vl.clear_interrupt()
                            front_cm = float(d) if d is not None else None
                    except Exception:
                        # keep last value; don't kill the loop
                        pass
                ig.mux_select()  # return to gyro path for next cycle
                t_front_next = t_now + FRONT_REFRESH_S

            # --- Auto-stop logic (forward only) with hysteresis ---
            if FRONT_AUTOSTOP_ON and front_cm is not None:
                if front_blocked:
                    if front_cm >= (FRONT_STOP_CM + FRONT_HYST_CM):
                        front_blocked = False
                else:
                    if vy > 0.05 and front_cm < FRONT_STOP_CM:
                        # immediate stop & brake
                        vx = 0.0; vy = 0.0; omg_user = 0.0
                        all_brake()
                        front_blocked = True

            # --- Heading-hold control ---
            omega_corr_raw = 0.0
            moving = (abs(vx) + abs(vy)) > 1e-3
            if lock_on and moving:
                e = angle_diff_deg(yaw_target, yaw)
                if abs(e) < DEADBAND_DEG: e = 0.0

                if corr_active:
                    if abs(e) < SCHMITT_EXIT: corr_active = False
                else:
                    if abs(e) > SCHMITT_ENTER: corr_active = True

                e_shaped = e / (1.0 + abs(e)/E_SOFT)

                if corr_active:
                    i_term = clamp(i_term + (PID_KI * e_shaped), -I_CLAMP, I_CLAMP)
                else:
                    i_term -= i_term * I_LEAK_PER_S * dt

                d_err = (e - e_prev) / dt if dt > 0 else 0.0
                e_prev = e

                pid = PID_KP * e_shaped + i_term + PID_KD * d_err
                omega_corr_raw = pid - KD_RATE * gz_dlp
            else:
                e = 0.0
                e_prev = 0.0
                i_term -= i_term * I_LEAK_PER_S * dt
                corr_active = False

            omega_corr = clamp(CORR_SIGN * omega_corr_raw, -OMEGA_MAX, OMEGA_MAX)

            # Slew-limit total omega
            omega_des = clamp(omg_user + omega_corr, -1.0, +1.0)
            max_step = OMEGA_SLEW_MAX * dt
            omega = clamp(omega_des, omega_prev - max_step, omega_prev + max_step)
            omega_prev = omega

            # If blocked, keep motors braked; otherwise drive
            if not front_blocked:
                mecanum_drive(vx, vy, omega, BASE_SPEED, yaw_err_deg=e)

            # --- Keyboard (non-blocking) ---
            if kbhit():
                ch = sys.stdin.read(1)
                if   ch == '8': vy, vx, omg_user = +1.0, 0.0,  0.0
                elif ch == '2': vy, vx, omg_user = -1.0, 0.0,  0.0
                elif ch == '4': vx, vy, omg_user = -1.0, 0.0,  0.0
                elif ch == '6': vx, vy, omg_user = +1.0, 0.0,  0.0
                elif ch == '7': vx, vy, omg_user = -1.0, +1.0, 0.0
                elif ch == '9': vx, vy, omg_user = +1.0, +1.0, 0.0
                elif ch == '1': vx, vy, omg_user = -1.0, -1.0, 0.0
                elif ch == '3': vx, vy, omg_user = +1.0, -1.0, 0.0
                elif ch == '5': vx, vy, omg_user = 0.0, 0.0,  0.0
                elif ch == '0': omg_user = +1.0
                elif ch == '.': omg_user = -1.0
                elif ch in ('+', '='): BASE_SPEED = clamp(BASE_SPEED + BASE_STEP, 0, 100)
                elif ch in ('-', '_'): BASE_SPEED = clamp(BASE_SPEED - BASE_STEP, 0, 100)

                # Gyro / guard controls
                elif ch.lower() == 'g':
                    lock_on = not lock_on
                    if lock_on:
                        yaw_target = yaw
                        i_term = 0.0; e_prev = 0.0; corr_active = False
                elif ch.lower() == 'r':
                    yaw_target = yaw
                    i_term = 0.0; e_prev = 0.0; corr_active = False
                elif ch.lower() == 'y':
                    yaw = 0.0; yaw_target = 0.0
                    i_term = 0.0; e_prev = 0.0; corr_active = False
                elif ch.lower() == 'o':
                    GZ_SIGN *= -1
                elif ch.lower() == 'p':
                    CORR_SIGN *= -1
                elif ch.lower() == 'h':
                    YAW_DIFF_ON = not YAW_DIFF_ON
                elif ch.lower() == 'f':
                    FRONT_AUTOSTOP_ON = not FRONT_AUTOSTOP_ON
                    if not FRONT_AUTOSTOP_ON:
                        front_blocked = False
                elif ch.upper() == 'R':
                    # Force gyro re-init; also ensure mux returns to gyro channel
                    all_brake()
                    time.sleep(I2C_REINIT_SLEEP)
                    ig.reinit_all()
                    yaw = 0.0; yaw_target = 0.0
                    i_term = 0.0; e_prev = 0.0; corr_active = False
                    front_blocked = False
                elif ch == ' ':
                    vx = vy = omg_user = 0.0
                    all_brake()
                elif ch.lower() == 'q' or ch == '\x03':
                    tidy_exit()

                if lock_on and abs(omg_user) < 1e-6 and (abs(vx)+abs(vy)) > 1e-3:
                    yaw_target = yaw

            # --- Status print ---
            if t_now >= t_print_next:
                print_status(BASE_SPEED, vx, vy, omg_user, yaw, yaw_target, lock_on, gz, omega_corr,
                             YAW_DIFF_ON, GZ_SIGN, CORR_SIGN, ig.fail_count,
                             front_cm, FRONT_AUTOSTOP_ON, front_blocked, FRONT_STOP_CM)
                t_print_next = t_now + 1.0/PRINT_HZ

    finally:
        tidy_exit()

if __name__ == "__main__":
    main()
