import requests

response = requests.post(
    "http://localhost:8888/api/control",
    json={
        "target_pitch": 0,
        "target_roll": 0,
        "target_yaw": 0,
        "base_throttle": 200,
        "armed": True,
    }
)

print(response.status_code)
print(response.json())