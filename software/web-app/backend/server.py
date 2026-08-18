import asyncio
import socket
import struct
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

ESP_IP        = "10.181.187.231"
ESP_PORT      = 5555
SEND_RATE_HZ  = 50
MAGIC         = 0x5E1FC9A3
PACKET_FORMAT = "<IIfffhBB"
PACKET_SIZE   = struct.calcsize(PACKET_FORMAT)

ANALYTICS_MAGIC  = 0x48EAD63
ANALYTICS_FORMAT = "<IIfBffff3x"
ANALYTICS_SIZE   = struct.calcsize(ANALYTICS_FORMAT)

class AnalyticsState:
    def __init__(self):
        self.battery_voltage = 0.0
        self.battery_percentage = 0
        self.esc_amps = 0.0
        self.pitch_angle = 0.0
        self.roll_angle = 0.0
        self.yaw_rate = 0.0
        self.sequence = 0
        self.last_update = 0.0
        self.lock = asyncio.Lock()

    async def update(self, seq, voltage, percentage, amps, pitch, roll, yaw):
        async with self.lock:
            self.sequence = seq
            self.battery_voltage = voltage
            self.battery_percentage = percentage
            self.esc_amps = amps
            self.pitch_angle = pitch
            self.roll_angle = roll
            self.yaw_rate = yaw
            self.last_update = time.time()

    async def snapshot(self):
        async with self.lock:
            return {
                "battery_voltage": round(self.battery_voltage, 2),
                "battery_percentage": self.battery_percentage,
                "esc_amps": round(self.esc_amps, 2),
                "pitch_angle": round(self.pitch_angle, 2),
                "roll_angle": round(self.roll_angle, 2),
                "yaw_rate": round(self.yaw_rate, 2),
                "sequence": self.sequence,
                "last_update": self.last_update,
            }

analytics = AnalyticsState()

# Current state of all the values
class ControlState:
    def __init__(self):
        self.target_pitch    = 0.0     # deg
        self.target_roll     = 0.0     # deg 
        self.target_yaw      = 0.0     # deg/s
        self.base_throttle   = 0       # 0 to 2047
        self.armed           = False   
        self.lock = asyncio.Lock()
    
    async def update(self, pitch, roll, yaw, throttle, armed):
        async with self.lock:
            self.target_pitch    = pitch
            self.target_roll     = roll
            self.target_yaw      = yaw
            self.base_throttle   = throttle
            self.armed           = armed
    
    async def snapshot(self):
        async with self.lock:
            return (self.target_pitch, self.target_roll, self.target_yaw, self.base_throttle, self.armed)
        
state = ControlState()

async def udp_sender_loop(sock):
    sequence = 0
    interval = 1.0 / SEND_RATE_HZ

    print(f"UDP sender started: {ESP_IP}:{ESP_PORT} @ {SEND_RATE_HZ} Hz")

    try:
        while True:
            pitch, roll, yaw, throttle, armed = await state.snapshot()

            flags = 0x01 if armed else 0x00

            packet = struct.pack(
                PACKET_FORMAT,
                MAGIC,
                sequence,
                pitch,
                roll,
                yaw,
                throttle,
                flags,
                0,
            )

            try:
                sock.sendto(packet, (ESP_IP, ESP_PORT))
            except OSError as e:
                print(f"UDP send error: {e}")

            sequence = (sequence + 1) & 0xFFFFFFFF
            await asyncio.sleep(interval)

    except asyncio.CancelledError:
        print("UDP sender stopping")


async def udp_receiver_loop(sock):
    print("UDP receiver started")

    try:
        while True:
            try:
                while True:
                    data, _ = sock.recvfrom(256)
                    if len(data) != ANALYTICS_SIZE:
                        continue
                    fields = struct.unpack(ANALYTICS_FORMAT, data)
                    magic, seq, voltage, percentage, amps, pitch, roll, yaw = fields
                    if magic != ANALYTICS_MAGIC:
                        continue
                    await analytics.update(seq, voltage, percentage, amps, pitch, roll, yaw)
            except BlockingIOError:
                pass
            await asyncio.sleep(0.01)

    except asyncio.CancelledError:
        print("UDP receiver stopping")

@asynccontextmanager
async def lifespan(app: FastAPI):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    sender_task = asyncio.create_task(udp_sender_loop(sock))
    receiver_task = asyncio.create_task(udp_receiver_loop(sock))
    print("Server ready")
    yield

    sender_task.cancel()
    receiver_task.cancel()
    await asyncio.gather(sender_task, receiver_task, return_exceptions=True)
    sock.close()

app = FastAPI(lifespan=lifespan)


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],          
    allow_methods=["*"],
    allow_headers=["*"],
)

# Input structure for web app  
class ControlInput(BaseModel):
    target_pitch:    float = Field(ge=-30.0, le=30.0)
    target_roll:     float = Field(ge=-30.0, le=30.0)
    target_yaw: float = Field(ge=-180.0, le=180.0)
    base_throttle:   int   = Field(ge=0, le=2047)
    armed:           bool  = False

# routes
@app.post("/api/control")
async def update_control(input: ControlInput):
    await state.update(
        input.target_pitch,
        input.target_roll,
        input.target_yaw,
        input.base_throttle,
        input.armed,
    )
    return {"ok": True}

@app.get("/api/state")
async def get_state():
    pitch, roll, yaw, throttle, armed = await state.snapshot()
    return {
        "target_pitch": pitch,
        "target_roll": roll,
        "target_yaw": yaw,
        "base_throttle": throttle,
        "armed": armed,
    }

@app.get("/api/analytics")
async def get_analytics():
    return await analytics.snapshot()

@app.post("/api/disarm")
async def emergency_disarm():
    await state.update(0.0, 0.0, 0.0, 0, False)
    return {"ok": True, "message": "Disarmed"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8888)
