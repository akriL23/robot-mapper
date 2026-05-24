#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Пины [TRIG, ECHO]
SENSORS = {
    "Front Left":  {"trig": 4,  "echo": 7},
    "Front Right": {"trig": 16, "echo": 26}, # <-- Изменено!
    "Back Left":   {"trig": 27, "echo": 14},
    "Back Right":  {"trig": 15, "echo": 10}
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def get_distance(trig, echo):
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    # 1. Ждём перехода 0 -> 1
    start = time.time()
    timeout = start + 0.04
    while GPIO.input(echo) == 0:
        if time.time() > timeout:
            return -1.0  # Нет старта импульса

    # 2. Ждём перехода 1 -> 0
    start = time.time()
    timeout = start + 0.04
    while GPIO.input(echo) == 1:
        if time.time() > timeout:
            break

    duration = time.time() - start
    return round(duration * 171.5, 3)

try:
    print("📡 Диагностика УЗ-датчиков... (Ctrl+C для выхода)")
    for name, pins in SENSORS.items():
        GPIO.setup(pins["trig"], GPIO.OUT)
        GPIO.setup(pins["echo"], GPIO.IN)
        GPIO.output(pins["trig"], GPIO.LOW)
        time.sleep(0.1)

    while True:
        print("-" * 45)
        for name, pins in SENSORS.items():
            state = GPIO.input(pins["echo"])
            dist = get_distance(pins["trig"], pins["echo"])
            status = f"{dist} м" if dist != -1.0 else "❌ Нет старта"
            stuck = " [STUCK HIGH]" if state == 1 and dist == -1.0 else ""
            print(f"{name:<12}: {status}{stuck}")
        print("-" * 45)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\n👋 Готово")
    GPIO.cleanup()
