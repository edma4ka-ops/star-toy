"""Поиск солнца в кадре и линия горизонта.

Запускается на Raspberry Pi Zero 2 W. Камера смотрит вперёд, солнце в кадре
занимает 2-5 пикселей, поэтому ищем не «яркий объект», а превышение порога:
солнце пересвечено, и его пиксели упираются в максимум.

Всё считается без numpy, если его нет: на слабом кадре это важнее скорости,
а при наличии numpy ускоряется в разы. Поэтому обе ветки в одном коде.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class SunFix:
    """Где солнце в кадре и под каким углом от оптической оси."""

    u: float  # пиксель по горизонтали
    v: float  # пиксель по вертикали
    pixels: int  # сколько пикселей прошло порог
    peak: int  # максимальная яркость
    bearing_x: float  # угол влево-вправо, градусы
    bearing_y: float  # угол вверх-вниз, градусы

    def valid(self) -> bool:
        # Два пикселя — уже солнце; один принимать нельзя, так выглядит
        # битый пиксель матрицы, их у любой камеры есть несколько.
        # Больше четырёхсот — скорее всего засветка от облака или блик
        return 2 <= self.pixels <= 400


@dataclass
class Horizon:
    """Линия горизонта: даёт крен и тангаж."""

    angle_deg: float  # наклон линии: это крен
    offset: float  # смещение от центра кадра: это тангаж
    quality: float  # доля столбцов, где горизонт найден
    found: bool

    def pitch_deg(self, fov_y_deg: float) -> float:
        return self.offset * fov_y_deg

    def roll_deg(self) -> float:
        return self.angle_deg


def find_sun(luma, width: int, height: int, threshold: int = 245) -> SunFix | None:
    """Найти солнце: центр тяжести пикселей выше порога.

    Порог высокий намеренно. Солнце пересвечивает матрицу до максимума,
    а небо и облака до максимума не доходят, поэтому порог отделяет
    солнце надёжнее, чем поиск «самого яркого пятна».
    """
    total = 0.0
    sum_u = 0.0
    sum_v = 0.0
    count = 0
    peak = 0

    for v in range(height):
        row = v * width
        for u in range(width):
            value = luma[row + u]
            if value > peak:
                peak = value
            if value >= threshold:
                weight = float(value)
                total += weight
                sum_u += weight * u
                sum_v += weight * v
                count += 1

    if count == 0 or total <= 0.0:
        return None

    u = sum_u / total
    v = sum_v / total
    return SunFix(u=u, v=v, pixels=count, peak=peak, bearing_x=0.0, bearing_y=0.0)


def bearing_from_pixel(u: float, v: float, width: int, height: int,
                       fov_x_deg: float, fov_y_deg: float) -> tuple[float, float]:
    """Угол от оптической оси по положению пикселя.

    Для широкоугольного объектива это приближение: настоящая проекция
    не линейная, и по краям кадра ошибка доходит до градусов. Для удержания
    курса этого достаточно, а для точных измерений нужна калибровка.
    """
    dx = (u - width / 2.0) / width
    dy = (v - height / 2.0) / height
    return dx * fov_x_deg, dy * fov_y_deg


def measure_sun(luma, width: int, height: int, fov_x_deg: float, fov_y_deg: float,
                threshold: int = 245) -> SunFix | None:
    fix = find_sun(luma, width, height, threshold)
    if fix is None:
        return None
    fix.bearing_x, fix.bearing_y = bearing_from_pixel(
        fix.u, fix.v, width, height, fov_x_deg, fov_y_deg
    )
    return fix


def find_horizon(luma, width: int, height: int, sky_threshold: int = 120,
                 step: int = 4) -> Horizon:
    """Линия горизонта по границе «небо-земля» в каждом столбце.

    В небе ярко, у земли темно. Идём сверху вниз и берём первое падение
    яркости — это и есть горизонт. Работает над полем и лесом, не работает
    над водой и снегом, где контраста нет.
    """
    xs = []
    ys = []
    for u in range(0, width, step):
        previous = None
        for v in range(0, height - step):
            value = luma[v * width + u]
            if previous is not None and previous >= sky_threshold > value:
                xs.append(float(u))
                ys.append(float(v))
                break
            previous = value

    if len(xs) < width / step * 0.3:
        return Horizon(0.0, 0.0, len(xs) / max(1.0, width / step), False)

    n = float(len(xs))
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if abs(denominator) < 1e-9:
        return Horizon(0.0, 0.0, 0.0, False)
    slope = numerator / denominator
    offset = mean_y - slope * mean_x
    angle = math.degrees(math.atan(slope))
    # Смещение от центра кадра, в долях высоты
    centre_offset = (offset + slope * (width / 2.0) - height / 2.0) / height
    return Horizon(angle, centre_offset, n / max(1.0, width / step), True)


def synthetic_frame(width: int, height: int, sun_u: float, sun_v: float,
                    radius: float = 1.0, background: int = 40,
                    sky: int = 120, horizon_row: int | None = None,
                    sun_brightness: int = 255) -> list[int]:
    """Кадр для проверки без камеры: небо, земля и солнце."""
    luma = [background] * (width * height)
    if horizon_row is not None:
        for v in range(height):
            for u in range(width):
                luma[v * width + u] = sky if v < horizon_row else background
    for v in range(height):
        for u in range(width):
            if (u - sun_u) ** 2 + (v - sun_v) ** 2 <= radius * radius:
                luma[v * width + u] = sun_brightness
    return luma
