from machine import Pin
import esp32

# 列出所有可用 GPIO
for i in range(0, 40):
    try:
        p = Pin(i)
        print(i, p)
    except:
        pass