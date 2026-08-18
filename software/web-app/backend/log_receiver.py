"""
ESP32 network log receiver.

Subscribes to the ESP's log server (UDP port 5556) and prints
all ESP_LOG output with color-coded log levels.

Usage:
    python log_receiver.py [ESP_IP]
"""

import socket
import sys
import re
ESP_IP   = "192.168.2.250"
LOG_PORT = 5556

MAX_RETRIES    = 10
RETRY_TIMEOUT  = 1.0

COLORS = {
    "E": "\033[91m",  # red
    "W": "\033[93m",  # yellow
    "I": "\033[92m",  # green
    "D": "\033[96m",  # cyan
    "V": "\033[37m",  # white
}
RESET = "\033[0m"

LEVEL_RE = re.compile(r"^([EWIDV]) \(")


def colorize(line: str) -> str:
    m = LEVEL_RE.match(line)
    if m:
        color = COLORS.get(m.group(1), "")
        return f"{color}{line}{RESET}"
    return line


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else ESP_IP

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.settimeout(RETRY_TIMEOUT)

    print(f"Subscribing to log stream at {target}:{LOG_PORT} ...")

    for attempt in range(1, MAX_RETRIES + 1):
        sock.sendto(b"subscribe", (target, LOG_PORT))
        try:
            data, _ = sock.recvfrom(256)
            print(f"Connected: {data.decode(errors='replace').strip()}")
            break
        except (socket.timeout, OSError):
            print(f"  attempt {attempt}/{MAX_RETRIES} — no response, retrying...")
    else:
        print("Failed to connect. Is the ESP running and on WiFi?")
        sock.close()
        return

    sock.settimeout(None)
    print("-" * 60)

    try:
        while True:
            data, _ = sock.recvfrom(4096)
            for line in data.decode(errors="replace").splitlines():
                print(colorize(line))
    except KeyboardInterrupt:
        print(f"\n{RESET}Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
