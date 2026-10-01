"""Полёт по направлению к солнцу: азимут, ветер, засветка камеры.

    python sim/sun.py

Три расчёта, и все три нужны до первого полёта.

1. Астрономия: где солнце и как быстро меняется его азимут. Если азимут
   за время полёта уходит на градусы, «лететь на солнце» перестаёт быть
   постоянным курсом.

2. Ветер: держать нос на солнце и лететь прямо — не одно и то же.
   Боковой ветер сносит аппарат вбок, и чтобы трек был прямым, нос надо
   заранее развернуть на ветер. Насколько — считается здесь.

3. Засветка: если камера смотрит туда же, куда летит аппарат, то при
   полёте на солнце она смотрит прямо в солнце. Что при этом будет
   с матрицей — считается по облучённости.
"""

from __future__ import annotations

import math
import sys

DEG = math.pi / 180.0


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


def solar_position(year: int, month: int, day: int, hour_ut: float,
                   latitude: float, longitude: float) -> tuple[float, float]:
    """Азимут и высота солнца в градусах. Азимут от севера по часовой."""
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

    # Истинное солнечное время в минутах
    true_solar = (hour_ut * 60.0 + eq_time + 4.0 * longitude) % 1440.0
    hour_angle = true_solar / 4.0 - 180.0

    lat = latitude * DEG
    zen = math.acos(
        math.sin(lat) * math.sin(declination)
        + math.cos(lat) * math.cos(declination) * math.cos(hour_angle * DEG)
    )
    elevation = 90.0 - math.degrees(zen)

    # Азимут
    sin_zen = math.sin(zen)
    if abs(sin_zen) < 1e-9:
        return 0.0, elevation
    cos_az = (math.sin(lat) * math.cos(zen) - math.sin(declination)) / (
        math.cos(lat) * sin_zen
    )
    cos_az = max(-1.0, min(1.0, cos_az))
    azimuth = math.degrees(math.acos(cos_az))
    if hour_angle > 0:
        azimuth = 360.0 - azimuth
    return azimuth, elevation


def air_mass(elevation_deg: float) -> float:
    """Масса атмосферы на пути луча: у горизонта свет проходит в десятки раз больше."""
    if elevation_deg <= 0.0:
        return 38.0
    z = 90.0 - elevation_deg
    return 1.0 / (math.cos(z * DEG) + 0.50572 * (96.07995 - z) ** -1.6364)


def direct_irradiance(elevation_deg: float) -> float:
    """Прямая облучённость от солнца, Вт/м². В зените около 1000."""
    if elevation_deg <= 0.0:
        return 0.0
    return 1353.0 * 0.7 ** (air_mass(elevation_deg) ** 0.678)


def main() -> int:
    # Москва, вечер 21 июня
    latitude, longitude, tz = 55.75, 37.62, 3.0
    year, month, day = 2026, 6, 21

    print("Солнце: азимут и высота вечером 21 июня, Москва")
    print(f"  {'время':>8}{'азимут':>10}{'высота':>10}{'облучённость':>15}")
    settings = []
    for hour_local in (19.0, 20.0, 21.0, 21.5, 22.0):
        azimuth, elevation = solar_position(
            year, month, day, hour_local - tz, latitude, longitude
        )
        irradiance = direct_irradiance(elevation)
        mark = ""
        if 0.0 < elevation < 30.0:
            mark = "  <- закатное"
        print(f"  {hour_local:>6.1f} ч{azimuth:>9.1f}°{elevation:>9.1f}°"
              f"{irradiance:>13.0f} Вт/м²{mark}")
        if elevation > 0.0:
            settings.append((hour_local, azimuth, elevation))
    print()

    print("Как быстро уходит азимут (для полёта это важно)")
    print(f"  {'время':>8}{'высота':>10}{'азимут через 1 мин':>22}{'уход':>10}")
    for hour_local in (20.0, 21.0, 21.4):
        az1, el1 = solar_position(year, month, day, hour_local - tz, latitude, longitude)
        az2, _el2 = solar_position(
            year, month, day, hour_local - tz + 1.0 / 60.0, latitude, longitude
        )
        print(f"  {hour_local:>6.1f} ч{el1:>9.1f}°{az2:>20.2f}°{az2 - az1:>9.2f}°")
    print("  За минуту азимут уходит на десятые доли градуса, за полёт в пару")
    print("  минут — меньше градуса. Значит солнце как задатчик курса годится:")
    print("  курс не успевает уехать вместе с ним.")
    print()

    # --- ветер и прямолинейность ---
    print("Ветер: держать нос на солнце и лететь прямо — не одно и то же")
    speed = 25.0
    print(f"  воздушная скорость аппарата: {speed:.0f} м/с")
    print(f"  {'боковой ветер':>15}{'угол на ветер':>15}"
          f"{'снос за 60 с':>15}{'снос за 120 с':>16}")
    for wind in (2.0, 5.0, 8.0):
        crab = math.degrees(math.asin(min(1.0, wind / speed)))
        print(f"  {wind:>13.1f} м/с{crab:>13.1f}°"
              f"{wind * 60:>13.0f} м{wind * 120:>14.0f} м")
    print("  Если просто держать нос на солнце, боковой ветер 5 м/с уносит")
    print("  аппарат на 300 м за минуту. Прямолинейность даёт не курс носа,")
    print("  а разворот носа на ветер — на угол из второго столбца.")
    print()

    # --- засветка камеры ---
    print("Засветка: что будет с камерой, если смотреть прямо на солнце")
    focal = 3.6e-3  # м, типовой объектив FPV
    aperture = 2.0e-3  # м, диаметр входного зрачка
    sun_angular = 0.532 * DEG
    image_diameter = focal * sun_angular
    image_area = math.pi * (image_diameter / 2.0) ** 2
    collected_factor = math.pi * (aperture / 2.0) ** 2 / image_area
    print(f"  объектив: фокус {focal * 1000:.1f} мм, зрачок {aperture * 1000:.1f} мм")
    print(f"  изображение солнца: {image_diameter * 1e6:.0f} мкм "
          f"(пиксель обычно 3-6 мкм, то есть солнце жжёт одну точку)")
    print(f"  {'высота солнца':>15}{'облучённость':>15}{'на матрице':>15}{'вывод':>24}")
    for elevation in (60.0, 30.0, 10.0, 3.0, 0.5):
        irradiance = direct_irradiance(elevation)
        sensor = irradiance * collected_factor / 1e4  # Вт/см²
        if sensor > 100.0:
            verdict = "сгорит мгновенно"
        elif sensor > 10.0:
            verdict = "сгорит за секунды"
        elif sensor > 1.0:
            verdict = "сгорит за минуты"
        else:
            verdict = "вытерпит"
        print(f"  {elevation:>13.1f}°{irradiance:>13.0f} Вт/м²"
              f"{sensor:>13.1f} Вт/см²{verdict:>24}")
    print("  Расчёт по геометрическому изображению солнца. Реальный объектив")
    print("  размазывает пятно по нескольким пикселям, поэтому в жизни выходит")
    print("  в разы меньше — но всё равно выше порога повреждения. Вывод тот же:")
    print("  смотреть камерой на солнце нельзя ни на закате, ни днём.")
    print()

    print("Что из этого следует для конструкции")
    print("  1. Солнце как задатчик курса годится: азимут за полёт не уходит.")
    print("  2. Прямолинейность в ветер даёт разворот носа против ветра.")
    print("     Измерять снос надо по земле: по картинке с камеры вперёд он")
    print("     виден плохо, потому что смотрят вперёд, а сносит вбок.")
    print("     Хорошо измеряют снос GPS или датчик потока, смотрящий вниз.")
    print("  3. Смотреть камерой прямо на солнце нельзя ни при какой высоте:")
    print("     днём матрица сгорает мгновенно, на закате за секунды.")
    print("     Решение простое и ничего не стоит: задавать курс ПО солнцу,")
    print("     а лететь не НА него, а под углом 30-90 градусов. Тогда солнце")
    print("     работает ориентиром и не мешает камере.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
