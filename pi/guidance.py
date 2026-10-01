"""Курс по солнцу: астрономия, ветер, снос.

Запускается на Raspberry Pi Zero 2 W. Здесь считается, куда лететь и на
сколько развернуть нос против ветра, чтобы трек по земле был прямым.

Главное, что надо понимать про ветер: держать нос на солнце и лететь прямо —
разные вещи. Боковой ветер 5 м/с при скорости 25 м/с уносит аппарат
на 300 м за минуту. Прямую даёт разворот носа на ветер, и угол этого
разворота надо откуда-то взять. Взять его можно только из измерения
движения по земле — из GPS. Нет GPS — нет и поправки, и аппарат честно
уходит с курса, о чём программа сообщает.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

DEG = math.pi / 180.0


def wrap180(angle: float) -> float:
    """Привести угол к диапазону -180..180."""
    return (angle + 180.0) % 360.0 - 180.0


def julian_day(year: int, month: int, day: int, hour_ut: float) -> float:
    if month <= 2:
        year -= 1
        month += 12
    a = year // 100
    b = 2 - a + a // 4
    return (
        int(365.25 * (year + 4716))
        + int(30.6001 * (month + 1))
        + day
        + b
        - 1524.5
        + hour_ut / 24.0
    )


def sun_azimuth(year: int, month: int, day: int, hour_ut: float,
                latitude: float, longitude: float) -> tuple[float, float]:
    """Азимут и высота солнца в градусах. Азимут от севера по часовой стрелке.

    Нужно, потому что «лететь на солнце» — это курс в земной системе,
    а не направление в кадре. Азимут солнца известен заранее: он зависит
    только от времени и места, и GPS их даёт.
    """
    jd = julian_day(year, month, day, hour_ut)
    t = (jd - 2451545.0) / 36525.0

    l0 = 280.46646 + t * (36000.76983 + t * 0.0003032)
    m = 357.52911 + t * (35999.05029 - 0.0001537 * t)
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)
    c = (
        math.sin(m * DEG) * (1.914602 - t * (0.004817 + 0.000014 * t))
        + math.sin(2 * m * DEG) * (0.019993 - 0.000101 * t)
        + math.sin(3 * m * DEG) * 0.000289
    )
    true_long = l0 + c
    omega = 125.04 - 1934.136 * t
    apparent_long = true_long - 0.00569 - 0.00478 * math.sin(omega * DEG)

    mean_obliq = 23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0
    obliq = mean_obliq + 0.00256 * math.cos(omega * DEG)
    declination = math.asin(math.sin(obliq * DEG) * math.sin(apparent_long * DEG))

    y = math.tan(obliq * DEG / 2.0) ** 2
    eq_time = 4.0 * math.degrees(
        y * math.sin(2 * l0 * DEG)
        - 2 * e * math.sin(m * DEG)
        + 4 * e * y * math.sin(m * DEG) * math.cos(2 * l0 * DEG)
        - 0.5 * y * y * math.sin(4 * l0 * DEG)
        - 1.25 * e * e * math.sin(2 * m * DEG)
    )
    true_solar = (hour_ut * 60.0 + eq_time + 4.0 * longitude) % 1440.0
    hour_angle = true_solar / 4.0 - 180.0

    lat = latitude * DEG
    zen = math.acos(
        math.sin(lat) * math.sin(declination)
        + math.cos(lat) * math.cos(declination) * math.cos(hour_angle * DEG)
    )
    elevation = 90.0 - math.degrees(zen)
    sin_zen = math.sin(zen)
    if abs(sin_zen) < 1e-9:
        return 0.0, elevation
    cos_az = (math.sin(lat) * math.cos(zen) - math.sin(declination)) / (math.cos(lat) * sin_zen)
    cos_az = max(-1.0, min(1.0, cos_az))
    azimuth = math.degrees(math.acos(cos_az))
    if hour_angle > 0:
        azimuth = 360.0 - azimuth
    return azimuth, elevation


@dataclass
class Tune:
    """Настройки контура курса по земле."""

    kp: float = 0.9  # градус разворота на градус ошибки трека
    ki: float = 0.08  # 1/с: накапливает угол сноса
    crab_limit_deg: float = 45.0  # предел накопленного сноса
    deadband_deg: float = 0.5  # мёртвая зона по треку
    max_heading_rate_deg: float = 30.0  # ограничение скорости разворота


class CourseGuidance:
    """Считает, куда должен смотреть нос, чтобы трек по земле шёл на солнце.

    Схема простая и надёжная. Целевой трек — азимут солнца. Ошибка трека —
    разница между целевым и текущим треком по земле. Разворот носа на ветер
    (угол сноса) накапливается интегратором: программа сама находит, насколько
    надо задрать нос, чтобы ветер перестал сносить. Пропорциональная часть
    разворачивает быстро, интегральная убирает остаточный снос.

    Почему именно так: угол сноса заранее неизвестен — он зависит от ветра,
    скорости и загрузки. Зато его можно накопить по факту ошибки трека,
    и это надёжнее, чем считать ветер по формуле с неточными данными.
    """

    def __init__(self, tune: Tune | None = None) -> None:
        self.tune = tune or Tune()
        self.crab = 0.0
        self.heading_cmd = 0.0
        self.track_error = 0.0
        self.have_track = False

    def reset(self) -> None:
        self.crab = 0.0
        self.track_error = 0.0

    def update(self, dt: float, target_track_deg: float, track_deg: float | None,
               heading_deg: float | None = None) -> float:
        """Вернуть команду по курсу носа в градусах.

        track_deg — трек по земле из GPS. Если его нет, снос неизвестен,
        и программа честно держит нос на солнце, сообщая об этом флагом.
        """
        if track_deg is None:
            # Без движения по земле поправку на ветер взять неоткуда
            self.have_track = False
            self.heading_cmd = target_track_deg
            return self.heading_cmd

        self.have_track = True
        error = wrap180(target_track_deg - track_deg)
        self.track_error = error

        if abs(error) > self.tune.deadband_deg:
            self.crab += self.tune.ki * error * dt
            self.crab = max(-self.tune.crab_limit_deg,
                            min(self.tune.crab_limit_deg, self.crab))

        wanted = target_track_deg + self.tune.kp * error + self.crab

        # Ограничение скорости разворота: резкая команда раскачивает аппарат
        if heading_deg is not None:
            step = wrap180(wanted - heading_deg)
            limit = self.tune.max_heading_rate_deg * dt
            if abs(step) > limit:
                wanted = heading_deg + math.copysign(limit, step)

        self.heading_cmd = wanted % 360.0
        return self.heading_cmd

    def crab_angle(self) -> float:
        """Накопленный угол сноса: сколько градусов нос смотрит на ветер."""
        return self.crab


def crab_estimate(wind_cross: float, airspeed: float) -> float:
    """Угол сноса по известному боковому ветру. Для справки и проверки."""
    if airspeed <= 1e-6:
        return 0.0
    return math.degrees(math.asin(max(-1.0, min(1.0, wind_cross / airspeed))))


def nose_azimuth_from_sun(sun_azimuth_deg: float, sun_bearing_x_deg: float) -> float:
    """Азимут носа по положению солнца в кадре.

    Это и есть главный смысл камеры: она даёт абсолютный курс без компаса.
    Если солнце в кадре левее центра на угол b, значит нос смотрит правее
    солнца на этот же угол.
    """
    return (sun_azimuth_deg - sun_bearing_x_deg) % 360.0
