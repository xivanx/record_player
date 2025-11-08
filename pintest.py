"""
pintest.py
Poll GPIO 21/22/23 wired to a rotary switch with a shared ground.
Pressing a switch drives the line low (0); idle reads high (1) via the internal pull-up.
Run with: mpremote connect [port] run pintest.py
"""

from machine import Pin
import time

# GPIOs tied to the rotary switch; configure as pull-ups because the common lead goes to GND.
TEST_PINS = (
    ("GPIO21", Pin(21, Pin.IN, Pin.PULL_UP)),
    ("GPIO22", Pin(22, Pin.IN, Pin.PULL_UP)),
    ("GPIO23", Pin(23, Pin.IN, Pin.PULL_UP)),
)

POLL_INTERVAL_MS = 50


def snapshot():
    """Return current logic level for each monitored pin."""
    return tuple((name, pin.value()) for name, pin in TEST_PINS)


def main():
    print("Monitoring GPIO21/GPIO22/GPIO23 (0 = pressed to GND, 1 = released)")
    last = snapshot()
    print(last)

    while True:
        current = snapshot()
        if current != last:
            print(current)
            last = current
        time.sleep_ms(POLL_INTERVAL_MS)


if __name__ == "__main__":
    main()
