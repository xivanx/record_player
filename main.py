"""Primary runtime entry point integrating the rotary switch controller."""

import time

import switch

POLL_INTERVAL_MS = 50


def main():
    """Monitor GPIO21/22/23 and dispatch jog controller actions."""
    last_command = None
    print("Monitoring GPIO21/22/23 rotary switch for motor commands...")
    print("Only one pin should be LOW at a time. Current levels:")
    print({name: pin.value() for name, pin in switch.PIN_MAP.items()})

    while True:
        command = switch.read_switch()
        if command != last_command:
            if command != "IDLE":
                switch.apply_command(command)
            last_command = command
        time.sleep_ms(POLL_INTERVAL_MS)


if __name__ == "__main__":
    main()
