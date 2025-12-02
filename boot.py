# This file is executed on every boot (including wake-boot from deepsleep)
# import esp
# esp.osdebug(None)
# import webrepl
# webrepl.start()

try:
    import main

    if hasattr(main, "main"):
        main.main()
    else:
        print("Warning: main module has no main() function.")
except Exception as exc:
    # Print the exception to the REPL so failures during boot aren't silent.
    import sys

    sys.print_exception(exc)
