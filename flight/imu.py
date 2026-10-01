"""Драйвер гироскопа и акселерометра по I2C.

По умолчанию MPU6050 — самый распространённый и дешёвый. Для аппарата лучше
ICM-42688-P: у него меньше шум и уход, и он держит курс заметно точнее.
Разница для нашей задачи такая: MPU6050 после калибровки уезжает примерно
на 0,1 °/с, за 2 секунды разгона это 0,2° — терпимо; ICM-42688 уезжает
в разы меньше, и это запас на более долгий полёт.

Чип задаётся константой CHIP, остальной код от этого не зависит.
"""

from __future__ import annotations

import struct
import time

CHIP = "mpu6050"  # "mpu6050" или "icm42688"

# MPU6050
MPU_ADDR = 0x68
MPU_WHO_AM_I = 0x75
MPU_PWR_MGMT_1 = 0x6B
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_GYRO_OUT = 0x43
MPU_ACCEL_OUT = 0x3B

# Масштабы: +-250 °/с и +-2 g — самые чувствительные диапазоны.
# На аппарате перегрузки бывают до 10 g, поэтому акселерометр лучше
# поставить на +-16 g, а гироскоп оставить на +-250 °/с
GYRO_SCALE = 1.0 / 131.0  # °/с на единицу
ACCEL_SCALE = 1.0 / 16384.0  # g на единицу
G = 9.80665


class Imu:
    """Угловые скорости в радианах в секунду, ускорение в м/с²."""

    def __init__(self, i2c, address: int = MPU_ADDR) -> None:
        self.i2c = i2c
        self.address = address
        self.gyro_bias = (0.0, 0.0, 0.0)
        self._init_chip()

    def _init_chip(self) -> None:
        who = self.i2c.readfrom_mem(self.address, MPU_WHO_AM_I, 1)[0]
        if who not in (0x68, 0x70, 0x71, 0x73, 0x74, 0x98):
            raise OSError(f"неизвестный датчик, WHO_AM_I = 0x{who:02X}")
        # Вывести из спящего режима и тактировать от внутреннего генератора
        self.i2c.writeto_mem(self.address, MPU_PWR_MGMT_1, b"\x00")
        time.sleep_ms(50)
        # Фильтр нижних частот: на аппарате вибрация от двигателя, без фильтра
        # гироскоп показывает шум двигателя, а не поворот
        self.i2c.writeto_mem(self.address, 0x1A, b"\x03")  # 44 Гц
        self.i2c.writeto_mem(self.address, MPU_GYRO_CONFIG, b"\x00")
        self.i2c.writeto_mem(self.address, MPU_ACCEL_CONFIG, b"\x00")

    def _read_vector(self, register: int, scale: float) -> tuple:
        raw = self.i2c.readfrom_mem(self.address, register, 6)
        x, y, z = struct.unpack(">hhh", raw)
        return (x * scale, y * scale, z * scale)

    def gyro(self) -> tuple:
        """Угловые скорости в радианах в секунду, с учётом ухода."""
        import math

        gx, gy, gz = self._read_vector(MPU_GYRO_OUT, GYRO_SCALE)
        return (
            math.radians(gx) - self.gyro_bias[0],
            math.radians(gy) - self.gyro_bias[1],
            math.radians(gz) - self.gyro_bias[2],
        )

    def accel(self) -> tuple:
        """Ускорение в м/с². На столе показывает направление «вверх»."""
        ax, ay, az = self._read_vector(MPU_ACCEL_OUT, ACCEL_SCALE)
        return (ax * G, ay * G, az * G)

    def calibrate_gyro(self, samples: int = 400) -> tuple:
        """Средний уход по выборке неподвижных измерений.

        Аппарат должен стоять неподвижно. На прогрев датчика уходит время:
        если калибровать сразу после включения, уход уедет при прогреве,
        и курс уведёт за собой.
        """
        import math

        time.sleep_ms(300)
        sums = [0.0, 0.0, 0.0]
        for _ in range(samples):
            gx, gy, gz = self._read_vector(MPU_GYRO_OUT, GYRO_SCALE)
            sums[0] += gx
            sums[1] += gy
            sums[2] += gz
            time.sleep_ms(2)
        self.gyro_bias = (
            math.radians(sums[0] / samples),
            math.radians(sums[1] / samples),
            math.radians(sums[2] / samples),
        )
        return self.gyro_bias
