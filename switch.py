"""
switch.py
Map a three-position rotary switch on GPIO21/22/23 to JogController actions from classtest.py.

Combinations (active-low):
- GPIO21 = 0, GPIO22 = GPIO23 = 1 ⟹ stop motor immediately.
- GPIO23 = 0, others = 1 ⟹ ramp to 3.49 rad/s.
- GPIO22 = 0, others = 1 ⟹ ramp to 4.7123889 rad/s.

Run with: mpremote connect [port] run switch.py
"""

from machine import Pin
import time

import classtest

# Configure the controller used for all switch actions. Fallback to direct JogController
# construction if the helper is unavailable on the device.
if hasattr(classtest, "create_controller"):
    controller = classtest.create_controller(accel_step=0.1, accel_interval=0.05)
else:
    controller = classtest.JogController(
        classtest.motor, accel_step=0.1, accel_interval=0.05
    )
motor = classtest.motor

# Map switch pins (wired common to GND, so use internal pull-ups).
PIN_MAP = {
    "GPIO21": Pin(21, Pin.IN, Pin.PULL_UP),
    "GPIO22": Pin(22, Pin.IN, Pin.PULL_UP),
    "GPIO23": Pin(23, Pin.IN, Pin.PULL_UP),
}

TARGET_LOW_23 = 3.49          # rad/s when GPIO23 is pulled low
TARGET_LOW_22 = 4.7123889     # rad/s when GPIO22 is pulled low
POLL_INTERVAL_MS = 50
def read_switch():
    """Return the active command based on the current pin levels."""
    levels = {name: pin.value() for name, pin in PIN_MAP.items()}
    if levels["GPIO21"] == 0 and levels["GPIO22"] == 1 and levels["GPIO23"] == 1:
        return "STOP"
    if levels["GPIO23"] == 0 and levels["GPIO21"] == 1 and levels["GPIO22"] == 1:
        return "RUN_3_49"
    if levels["GPIO22"] == 0 and levels["GPIO21"] == 1 and levels["GPIO23"] == 1:
        return "RUN_4_71"
    return "IDLE"


def send_velocity(target):
    """Send the requested velocity immediately."""
    controller._set_limit_current(45)
    controller.set_target(target)
    controller.current_vel = controller.target_vel
    controller._send_velocity(controller.current_vel)


def apply_command(command):
    """Dispatch the requested command to the jog controller."""
    if command == "STOP":
        print("Switch -> STOP: set velocity 0 rad/s")
        send_velocity(0.0)
    elif command == "RUN_3_49":
        print("Switch -> SPEED 3.49 rad/s (instant)")
        send_velocity(TARGET_LOW_23)
    elif command == "RUN_4_71":
        print("Switch -> SPEED 4.7123889 rad/s (instant)")
        send_velocity(TARGET_LOW_22)
    else:
        # Nothing to do for IDLE; leave controller at its last state.
        pass


def main():
    last_command = None
    print("Monitoring GPIO21/22/23 rotary switch for motor commands...")
    print("Only one pin should be LOW at a time. Current levels:")
    print({name: pin.value() for name, pin in PIN_MAP.items()})

    try:
        while True:
            command = read_switch()
            if command != last_command:
                if command != "IDLE":
                    apply_command(command)
                last_command = command
            time.sleep_ms(POLL_INTERVAL_MS)
    except KeyboardInterrupt:
        print("Switch loop interrupted, soft-stopping motor...")
        controller.soft_stop()
    finally:
        motor.disable()
        print("Motor disabled.")


if __name__ == "__main__":
    main()
