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
CURRENT_LIMIT_PERCENT = 0.010  # limit to 10% of the default limit
DEFAULT_LIMIT_CURRENT = 45
LIMIT_CURRENT_AMPS = DEFAULT_LIMIT_CURRENT * CURRENT_LIMIT_PERCENT

DEBOUNCE_COUNT = 3
_last_raw_command = None
_raw_repeat = 0
_stable_command = "IDLE"

def _read_raw_command():
    """Interpret the current pin levels without debounce."""
    levels = {name: pin.value() for name, pin in PIN_MAP.items()}
    if levels["GPIO21"] == 0 and levels["GPIO22"] == 1 and levels["GPIO23"] == 1:
        return "STOP"
    if levels["GPIO23"] == 0 and levels["GPIO21"] == 1 and levels["GPIO22"] == 1:
        return "RUN_3_49"
    if levels["GPIO22"] == 0 and levels["GPIO21"] == 1 and levels["GPIO23"] == 1:
        return "RUN_4_71"
    return "IDLE"


def read_switch():
    """Return the debounced command based on the current pin levels."""
    global _last_raw_command, _raw_repeat, _stable_command
    raw_command = _read_raw_command()
    if raw_command == _last_raw_command:
        _raw_repeat += 1
    else:
        _raw_repeat = 1
        _last_raw_command = raw_command

    if raw_command != _stable_command and _raw_repeat >= DEBOUNCE_COUNT:
        _stable_command = raw_command

    return _stable_command


def send_velocity(target):
    """Send the requested velocity immediately."""
    controller._set_limit_current(LIMIT_CURRENT_AMPS)
    controller.set_target(target)
    controller.current_vel = controller.target_vel
    controller._send_velocity(controller.current_vel)


def apply_command(command):
    """Dispatch the requested command to the jog controller."""
    if command == "STOP":
        print("Switch -> STOP: set velocity 0 rad/s")
        send_velocity(0.0)
        motor.disable()
        print("motor disabled")
    elif command == "RUN_3_49":
        print("Switch -> SPEED 3.49 rad/s (instant)")
        motor.enable()
        send_velocity(TARGET_LOW_23)
    elif command == "RUN_4_71":
        print("Switch -> SPEED 4.7123889 rad/s (instant)")
        motor.enable()
        send_velocity(TARGET_LOW_22)
    else:
        # Nothing to do for IDLE; leave controller at its last state.
        pass
