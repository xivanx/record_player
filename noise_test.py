"""
noise_test.py
Use the three-position rotary switch (GPIO21/22/23) to evaluate multiple ramp
algorithms defined in classtest.JogController:

- GPIO21 = LOW  : stop the motor (soft stop).
- GPIO23 = LOW  : trigger the next ramp algorithm in the list.
- GPIO22 = LOW  : optional, run the test at the higher target velocity (4.7123889).

Each time GPIO23 transitions from HIGH → LOW, the script cycles to the next
algorithm and ramps the motor from the current speed to the requested target.
"""

from machine import Pin
import time

import classtest

# Reuse the shared controller from classtest.
if hasattr(classtest, "create_controller"):
    controller = classtest.create_controller(accel_step=0.1, accel_interval=0.05)
else:
    controller = classtest.JogController(
        classtest.motor, accel_step=0.1, accel_interval=0.05
    )
motor = classtest.motor

# Switch pins (common to GND, so use pull-ups).
PIN_MAP = {
    "GPIO21": Pin(21, Pin.IN, Pin.PULL_UP),  # stop
    "GPIO22": Pin(22, Pin.IN, Pin.PULL_UP),  # optional high-speed selector
    "GPIO23": Pin(23, Pin.IN, Pin.PULL_UP),  # cycle algorithms
}

TARGET_LOW = 3.49
TARGET_HIGH = 4.7123889
POLL_INTERVAL_MS = 40

# Define the algorithms to evaluate (name, JogController method, kwargs).
ALGORITHMS = (
    ("jerk-limited S-curve", "ramp_to_target_jerk", {"duration": 2.2, "sample_interval": 0.02}),
    ("exp-tail trapezoid", "ramp_to_target_exp", {"duration": 2.0, "accel_portion": 0.3, "sample_interval": 0.03}),
    ("PID velocity loop", "ramp_to_target_pid", {"run_time": 2.5, "sample_interval": 0.02}),
    ("filtered linear ramp", "ramp_to_target_filtered", {"duration": 1.8, "sample_interval": 0.02, "window": 6}),
)
ALGO_COUNT = 0
for _entry in ALGORITHMS:
    ALGO_COUNT += 1


def read_levels():
    return {name: pin.value() for name, pin in PIN_MAP.items()}


def stop_motor():
    controller.set_target(0.0)
    if hasattr(controller, "ramp_to_target_filtered"):
        controller.ramp_to_target_filtered(duration=1.2, sample_interval=0.02, window=5)
    elif hasattr(controller, "ramp_to_target_soft"):
        controller.ramp_to_target_soft(duration=1.5, sample_interval=0.04)
    else:
        controller.ramp_to_target()


def run_algorithm(index, target_speed):
    name, method_name, kwargs = ALGORITHMS[index]
    method = getattr(controller, method_name, None)
    if method is None:
        method = controller.ramp_to_target
        kwargs = {}
    controller.set_target(target_speed)
    method(**kwargs)
    print("[{}] target={:.3f} rad/s, final={:.3f}".format(name, target_speed, controller.current_vel))


def main():
    algo_index = 0
    prev_levels = read_levels()

    print("noise_test ready. GPIO23 cycles algorithms (GPIO22 high=3.49, low=4.7123889).")

    try:
        while True:
            levels = read_levels()

            if levels["GPIO21"] == 0:
                stop_motor()

            if levels["GPIO23"] == 0 and prev_levels["GPIO23"] == 1:
                # Determine which speed to use depending on GPIO22.
                target = TARGET_HIGH if levels["GPIO22"] == 0 else TARGET_LOW
                run_algorithm(algo_index, target)
                algo_index = (algo_index + 1) % ALGO_COUNT

            prev_levels = levels
            time.sleep_ms(POLL_INTERVAL_MS)

    except KeyboardInterrupt:
        print("Interrupted. Soft stopping...")
        controller.soft_stop()
    finally:
        motor.disable()
        print("Motor disabled.")


if __name__ == "__main__":
    main()
