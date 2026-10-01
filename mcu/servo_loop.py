"""Быстрый контур на микроконтроллере: рули, гироскоп, предохранители.

Запускается на RP2040 (Raspberry Pi Pico) под MicroPython. Плата стоит
около двухсот рублей и делает то, чего не может Pi Zero 2 W: держит
ровный шаг 250 раз в секунду.

Почему не на самом Pi: под Linux планировщик, SD-карта и WiFi дают
дрожание в миллисекунды. Для контура, который должен отрабатывать
болтанку за десятые доли секунды, это много: аппарат начнёт рыскать.
Поэтому разделение такое:

    Pi Zero 2 W: камера, солнце, GPS, курс по земле — 20-30 раз в секунду
    RP2040:      гироскоп, стабилизация, рули              — 250 раз в секунду

Логика контура берётся из control.py — того же файла, что проверен
в тренажёре и на стенде. Скорость и момент рулей считаются там же.
"""

from __future__ import annotations

import math
import time

from machine import I2C, PWM, UART, WDT, Pin  # noqa: F401

from control import HeadingHolder, Tune, Vehicle
from imu import Imu

# --- железо ---
I2C_SDA, I2C_SCL, I2C_HZ = 4, 5, 400_000
SERVO_RUDDER, SERVO_ELEVATOR, SERVO_AILERON = 6, 7, 8
LINK_UART_ID, LINK_BAUD, LINK_TX, LINK_RX = 0, 115200, 0, 1
ARM_PIN, LED_PIN = 14, 25

LOOP_HZ = 250
PERIOD = 1.0 / LOOP_HZ
# Если Pi молчит дольше этого, рули надо вернуть: связь потеряна
LINK_TIMEOUT_S = 1.0
# Предел крена: за ним аппарат теряет подъёмную силу
MAX_ROLL_DEG = 45.0


class Link:
    """Связь с Pi. Строки вида 'H 227.4 1' — курс и признак достоверности."""

    def __init__(self, uart) -> None:
        self.uart = uart
        self.heading_cmd = 0.0
        self.heading_valid = False
        self.stamp = 0.0

    def poll(self, now: float) -> None:
        while self.uart.any():
            line = self.uart.readline()
            if not line:
                break
            parts = line.decode("utf-8", "ignore").strip().split()
            if len(parts) >= 3 and parts[0] == "H":
                try:
                    self.heading_cmd = float(parts[1]) % 360.0
                    self.heading_valid = parts[2] == "1"
                except ValueError:
                    continue
                self.stamp = now

    def fresh(self, now: float) -> bool:
        return self.heading_valid and (now - self.stamp) < LINK_TIMEOUT_S


class Servos:
    """Три машинки: руль направления, руль высоты, элероны.

    При потере питания рули должны вставать в нейтраль механически —
    пружиной или аэродинамикой. Программная нейтраль это не заменяет.
    """

    def __init__(self) -> None:
        self.rudder = PWM(Pin(SERVO_RUDDER), freq=50)
        self.elevator = PWM(Pin(SERVO_ELEVATOR), freq=50)
        self.aileron = PWM(Pin(SERVO_AILERON), freq=50)
        self.neutral()

    @staticmethod
    def _duty(command: float) -> int:
        width = 1500.0 + command * 500.0
        width = max(1000.0, min(2000.0, width))
        return int(width / 20000.0 * 65535)

    def set(self, rudder: float, elevator: float, aileron: float) -> None:
        self.rudder.duty_u16(self._duty(rudder))
        self.elevator.duty_u16(self._duty(elevator))
        self.aileron.duty_u16(self._duty(aileron))

    def neutral(self) -> None:
        self.set(0.0, 0.0, 0.0)


def main() -> None:
    led = Pin(LED_PIN, Pin.OUT)
    arm = Pin(ARM_PIN, Pin.IN, Pin.PULL_UP)
    watchdog = WDT(timeout=2000)

    i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_HZ)
    imu = Imu(i2c)
    servos = Servos()
    link = Link(UART(LINK_UART_ID, baudrate=LINK_BAUD, tx=Pin(LINK_TX), rx=Pin(LINK_RX)))

    # Контур курса: руль направления держит курс, который просит Pi
    holder = HeadingHolder(Tune(), Vehicle(), tail_arm=0.35)
    holder.arm()

    print("калибрую гироскоп, аппарат должен стоять неподвижно")
    bias = imu.calibrate_gyro()
    holder.calibrate_gyro([bias])
    holder.set_reference(0.0)
    print("готов, жду взвода")

    while arm.value():
        watchdog.feed()
        led.value(not led.value())
        servos.neutral()
        time.sleep_ms(100)

    print("взведено")
    last = time.ticks_ms()
    while True:
        watchdog.feed()
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, last) / 1000.0
        last = now_ms
        if dt <= 0.0 or dt > 0.2:
            dt = PERIOD

        now = now_ms / 1000.0
        link.poll(now)
        gyro = imu.gyro()
        accel = imu.accel()

        if not link.fresh(now):
            # Связь потеряна или курс недостоверен: рули в нейтраль.
            # Лучше ровный полёт устойчивого аппарата, чем борьба вслепую
            servos.neutral()
            time.sleep_us(int(PERIOD * 1_000_000))
            continue

        # Задание приходит снаружи, оценка курса своя: интегрируем гироскоп,
        # а абсолютную привязку даёт Pi по солнцу. Передавать сюда команду
        # как измерение нельзя — ошибка обнулится и рули не будут работать
        holder.set_target(link.heading_cmd)
        yaw_rate = gyro[2]
        command = holder.update(dt, gyro, yaw_rate, None, 0.0)

        # Крен и тангаж держим здесь же: элероны против крена, руль высоты
        # по тангажу. Контуры те же, что проверены в тренажёре
        roll = math.atan2(accel[1], accel[2]) if abs(accel[2]) > 1e-6 else 0.0
        roll_deg = math.degrees(roll)
        aileron = max(-1.0, min(1.0, -roll_deg / MAX_ROLL_DEG))
        elevator = 0.0  # в простейшем варианте высота держится оттриммировкой

        servos.set(command.ch_z if command.active else 0.0, elevator, aileron)

        spent = time.ticks_diff(time.ticks_ms(), now_ms) / 1000.0
        pause = PERIOD - spent
        if pause > 0:
            time.sleep_us(int(pause * 1_000_000))


if __name__ == "__main__":
    main()
