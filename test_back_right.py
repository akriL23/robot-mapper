#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

# Пины для BACK_RIGHT (из твоего кода)
ENB = 13
IN3 = 20
IN4 = 21

print("⚙️  Настройка GPIO...")
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# PWM для управления скоростью
pwm = GPIO.PWM(ENB, 500)
pwm.start(0)

try:
    print("▶️  Включаю BACK_RIGHT на 50% скорости (3 сек)...")
    # Направление: IN3=HIGH, IN4=LOW
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm.ChangeDutyCycle(50)
    
    time.sleep(3)
    
    print("🔄  Меняю направление (3 сек)...")
    # Обратное направление
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    time.sleep(3)
    
    print("🛑  Остановка.")
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

except KeyboardInterrupt:
    print("\n️  Прервано пользователем.")
finally:
    pwm.stop()
    GPIO.cleanup()
    print("✅ Тест завершен.")
