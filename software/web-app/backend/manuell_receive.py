import requests
import time

API_URL = "http://localhost:8888/api/analytics"

while True:
    try:
        r = requests.get(API_URL, timeout=2)
        d = r.json()
        print(f"[seq {d['sequence']:>6}]  bat={d['battery_voltage']:.2f}V  {d['battery_percentage']}%  "
              f"amps={d['esc_amps']:.2f}A  pitch={d['pitch_angle']:.2f}  "
              f"roll={d['roll_angle']:.2f}  yaw={d['yaw_rate']:.2f}")
    except Exception as e:
        print(f"Error: {e}")

    time.sleep(2)
