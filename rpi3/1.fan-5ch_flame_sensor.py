### code for rpi3 test ROBOT

#!/usr/bin/env python3
import time
import signal
import sys
import RPi.GPIO as GPIO

# === Piny ===
SENSOR_PINS = [26, 19, 13, 6, 5]   # 5CH flame sensor (digitálne výstupy)
FAN_PIN = 17                       # TRIG/IN na MOSFETe

# === Nastavenia ===
HOLD_SECONDS = 3.0                 # dobeh ventilátora po poslednej detekcii
DEBOUNCE_SEC = 0.02                # loop interval ~50 Hz
CALIBRATION_TIME = 1.0             # koľko s snímať "neutrálny" stav na začiatku
FAN_ACTIVE_LOW = False             # ak tvoj MOSFET zapína pri LOW -> nechaj True
VERBOSE = True

GPIO.setmode(GPIO.BCM)

# --- Ventilátor: nastavíme menovité úrovne podľa typu modulu ---
fan_off_level = GPIO.HIGH if FAN_ACTIVE_LOW else GPIO.LOW
fan_on_level  = GPIO.LOW  if FAN_ACTIVE_LOW else GPIO.HIGH
GPIO.setup(FAN_PIN, GPIO.OUT, initial=fan_off_level)

# --- Senzory: necháme PUD_OFF, aby sme sa nebili s modulom (má vlastné komparátory) ---
for p in SENSOR_PINS:
    GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def fan_on():
    GPIO.output(FAN_PIN, fan_on_level)

def fan_off():
    GPIO.output(FAN_PIN, fan_off_level)

def read_pins():
    return {p: GPIO.input(p) for p in SENSOR_PINS}

def cleanup_and_exit(code=0):
    try:
        fan_off()
    finally:
        GPIO.cleanup()
    sys.exit(code)

def sigint_handler(sig, frame):
    print("\nStopping…")
    cleanup_and_exit(0)

signal.signal(signal.SIGINT, sigint_handler)

# --- 1) KALIBRÁCIA: zistíme neutrál (bez plameňa) ---
print("Calibrating neutral states (no flame)…")
t0 = time.time()
samples = {p: {0: 0, 1: 0} for p in SENSOR_PINS}
while time.time() - t0 < CALIBRATION_TIME:
    for p in SENSOR_PINS:
        v = GPIO.input(p)
        samples[p][v] += 1
    time.sleep(0.005)  # rýchle vzorkovanie

neutral = {p: (0 if samples[p][0] >= samples[p][1] else 1) for p in SENSOR_PINS}
if VERBOSE:
    print("Neutral states:", {p: neutral[p] for p in SENSOR_PINS})

# Aktívny stav definujeme ako "odlišný od neutrálneho"
def flame_on_pin(pin, val=None):
    if val is None:
        val = GPIO.input(pin)
    return val != neutral[pin]

# Stavové premenné
last_vals = read_pins()
last_any_flame = any(flame_on_pin(p, last_vals[p]) for p in SENSOR_PINS)
fan_deadline = 0.0

if VERBOSE:
    print("Initial pin states:", last_vals)
    print("Initial any_flame:", last_any_flame)
print("Running. Show flame and watch logs…  (Ctrl+C to exit)")

try:
    while True:
        now = time.time()
        vals = read_pins()

        # Loguj zmeny pinov (užitočné na ladenie zapojenia/logiky)
        for p in SENSOR_PINS:
            if vals[p] != last_vals[p]:
                print(f"[{time.strftime('%H:%M:%S')}] GPIO{p} changed {last_vals[p]} -> {vals[p]} | "
                      f"{'FLAME' if flame_on_pin(p, vals[p]) else 'no flame'}")
        last_vals = vals

        # Zisti „plameň“ podľa odchýlky od neutrálneho stavu
        any_flame_now = any(flame_on_pin(p, vals[p]) for p in SENSOR_PINS)

        # Ak plameň, zapni ventilátor a posuň deadline
        if any_flame_now:
            if VERBOSE and not last_any_flame:
                print(f"[{time.strftime('%H:%M:%S')}] Any flame = TRUE -> FAN ON (hold {HOLD_SECONDS}s)")
            fan_on()
            fan_deadline = now + HOLD_SECONDS

        # Po dobehu vypni
        if fan_deadline and now >= fan_deadline and GPIO.input(FAN_PIN) == fan_on_level:
            if VERBOSE:
                print(f"[{time.strftime('%H:%M:%S')}] Hold elapsed -> FAN OFF")
            fan_off()
            fan_deadline = 0.0

        last_any_flame = any_flame_now
        time.sleep(DEBOUNCE_SEC)

except Exception as e:
    print(f"Error: {e}")
    cleanup_and_exit(1)
