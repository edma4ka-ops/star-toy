"""Прошивка аппарата: держит курс, зафиксированный по звезде.

Запускается на ESP32 или RP2040 под MicroPython. Логика управления берётся
из control.py — того же файла, который проверен в тренажёре и на стенде.
Здесь только железо: датчик, рулевые машинки, приём курса от звёздного
датчика и предохранители.

Порядок работы:

    1. Включение. Ждём прогрева датчика, калибруем уход гироскопа.
    2. Ждём курса от звёздного датчика (он решает задачу по звёздам,
       пока аппарат стоит неподвижно). Без курса пуск не разрешается.
    3. Взводим и ждём старта: старт определяется по перегрузке
       от двигателя. До старта рули в нейтрали.
    4. Разгон: контур держит курс, пока работает двигатель.
    5. Выгорание: рули в нейтраль, аппарат устойчив сам.
    6. Посадка: ждём покоя, выключаемся.

Чего в этой прошивке нет и что нужно доделать перед железом:
  * нет проверки на реальном датчике — числа регистров взяты из описания;
  * нет защиты от повторного пуска без перезагрузки (сейчас одна попытка);
  * нет записи журнала на флеш — после посадки данные теряются;
  * звёздный датчик описан протоколом, но не реализован.
"""

from __future__ import annotations

import time

from machine import I2C, PWM, UART, WDT, Pin  # noqa: F401

from control import CourseHolder, Tune, Vehicle, q_from_axes
from imu import Imu

# --- Настройки железа ------------------------------------------------------

I2C_SDA = 21
I2C_SCL = 22
I2C_HZ = 400_000

SERVO_Z_PIN = 25  # пара рулей на оси +-z
SERVO_Y_PIN = 26  # пара рулей на оси +-y

UART_ID = 1
UART_BAUD = 115200  # линия от звёздного датчика
UART_TX = 17
UART_RX = 16

ARM_PIN = 32  # тумблер взвода: пуск только при замкнутом контакте
LED_PIN = 2

LOOP_HZ = 200
CONTROL_PERIOD = 1.0 / LOOP_HZ

# Перегрузка, по которой видим старт: двигатель даёт больше 3 g
LAUNCH_ACCEL = 3.0 * 9.80665
# Ниже этой перегрузки считаем, что двигатель отработал
BURNOUT_ACCEL = 1.6 * 9.80665
# Дольше этого времени после старта управление не нужно ни при каких условиях
MAX_FLIGHT_S = 30.0


class Servos:
    """Две рулевые машинки. Нейтраль — середина хода.

    Машинки ставятся так, чтобы при потере питания рули вставали в нейтраль
    (пружина или аэродинамика), а не заклинивались в крайнем положении.
    """

    def __init__(self, pin_z: int, pin_y: int) -> None:
        self.pwm_z = PWM(Pin(pin_z), freq=50)
        self.pwm_y = PWM(Pin(pin_y), freq=50)
        self.neutral()

    @staticmethod
    def _duty(command: float) -> int:
        """Команда -1..1 в скважность для 50 Гц.

        1,0 мс — один край, 2,0 мс — другой, середина 1,5 мс.
        """
        width_us = 1500.0 + command * 500.0
        width_us = max(1000.0, min(2000.0, width_us))
        return int(width_us / 20000.0 * 65535)

    def set(self, ch_z: float, ch_y: float) -> None:
        self.pwm_z.duty_u16(self._duty(ch_z))
        self.pwm_y.duty_u16(self._duty(ch_y))

    def neutral(self) -> None:
        self.set(0.0, 0.0)


class StarLink:
    """Приём курса от звёздного датчика.

    Протокол строкой, потому что отлаживать проще всего:
        LOCK <x> <y> <z> <y> <y_y> <y_z> <z_x> <z_y> <z_z>
    где первые три числа — ось аппарата (куда смотрит нос), остальные шесть —
    оси «верх» и «бок» в инерциальной системе. Датчик отдаёт их, решив задачу
    по звёздам.
    """

    def __init__(self, uart) -> None:
        self.uart = uart
        self.attitude = None
        self.stamp = 0.0

    def poll(self, now: float) -> bool:
        if not self.uart.any():
            return False
        line = self.uart.readline()
        if not line:
            return False
        parts = line.decode("utf-8", "ignore").strip().split()
        if len(parts) != 10 or parts[0] != "LOCK":
            return False
        try:
            values = [float(v) for v in parts[1:]]
        except ValueError:
            return False
        self.attitude = q_from_axes(
            (values[0], values[1], values[2]),
            (values[3], values[4], values[5]),
            (values[6], values[7], values[8]),
        )
        self.stamp = now
        return True

    def fresh(self, now: float, limit_s: float = 120.0) -> bool:
        return self.attitude is not None and (now - self.stamp) < limit_s


def main() -> None:
    led = Pin(LED_PIN, Pin.OUT)
    arm_switch = Pin(ARM_PIN, Pin.IN, Pin.PULL_UP)
    watchdog = WDT(timeout=4000)

    i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=I2C_HZ)
    imu = Imu(i2c)
    servos = Servos(SERVO_Z_PIN, SERVO_Y_PIN)
    stars = StarLink(UART(UART_ID, baudrate=UART_BAUD, tx=Pin(UART_TX), rx=Pin(UART_RX)))

    holder = CourseHolder(Tune(), Vehicle())

    print("калибрую гироскоп: аппарат должен стоять неподвижно")
    bias = imu.calibrate_gyro()
    holder.calibrate_gyro([bias])
    print("уход гироскопа, °/с:", [round(v * 57.2958, 3) for v in bias])

    # --- ожидание курса от звёздного датчика ---
    print("жду курс от звёздного датчика")
    start = time.ticks_ms()
    while True:
        watchdog.feed()
        now = time.ticks_diff(time.ticks_ms(), start) / 1000.0
        if stars.poll(now) and stars.fresh(now):
            break
        if now > 300.0:
            print("звёздный датчик не дал курс за 5 минут — выключаюсь")
            return
        time.sleep_ms(20)

    if not holder.set_attitude(stars.attitude, stars_ok=True):
        print("курс не принят")
        return
    holder.lock_course()
    print("курс зафиксирован по звёздам")

    # --- ожидание взвода ---
    print("переведите тумблер взвода, чтобы разрешить пуск")
    while arm_switch.value():
        watchdog.feed()
        led.value(not led.value())
        time.sleep_ms(200)
    holder.arm()
    led.value(1)
    print("взведено, жду старт")

    # --- полёт ---
    flying = False
    burnout = False
    last = time.ticks_ms()
    flight_start = 0
    while True:
        watchdog.feed()
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, last) / 1000.0
        last = now_ms
        if dt <= 0.0 or dt > 0.5:
            dt = CONTROL_PERIOD

        gyro = imu.gyro()
        accel = imu.accel()
        magnitude = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5

        if not flying:
            # До старта рули в нейтрали, датчик не трогаем: старт виден
            # по перегрузке от двигателя
            if magnitude > LAUNCH_ACCEL:
                flying = True
                flight_start = now_ms
                print("старт")
            else:
                servos.neutral()
                time.sleep_ms(int(CONTROL_PERIOD * 1000))
                continue

        flight_time = time.ticks_diff(now_ms, flight_start) / 1000.0
        if not burnout and magnitude < BURNOUT_ACCEL and flight_time > 0.3:
            burnout = True
            print("двигатель отработал, рули в нейтраль")

        # Скоростной напор точно не измеряется: берём оценку по времени
        # разгона — этого достаточно, чтобы регулятор не перепутал масштаб
        q_dyn = 0.5 * 1.2 * (120.0 * min(flight_time, 1.6)) ** 2

        command = holder.update(dt, gyro, q_dyn, burning=not burnout)
        servos.set(command.ch_z, command.ch_y)

        if flight_time > MAX_FLIGHT_S:
            servos.neutral()
            print("полёт длится слишком долго — рули в нейтраль, выключаюсь")
            return

        # Задержка до следующего шага с учётом затраченного времени
        spent = time.ticks_diff(time.ticks_ms(), now_ms) / 1000.0
        pause = CONTROL_PERIOD - spent
        if pause > 0:
            time.sleep_us(int(pause * 1_000_000))


if __name__ == "__main__":
    main()
