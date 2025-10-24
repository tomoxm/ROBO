# ROBO

Raspberry Pi robotics project (SunFounder Robot HAT, mecanum wheels, sensors, fan control).  
Goals:
- 4x motor control (mecanum)
- Sensor suite: VL53L1X laser, flame, grayscale, gyroscope and line sensor
- Fan (MOSFET) for candle/fire 
- Structured code with tests and CI

## Quick start

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python -m software.robot.main
