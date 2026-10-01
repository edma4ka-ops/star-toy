"""Проверка программы для Pi без Raspberry: на синтетических кадрах.

    python pi/check.py

Проверяется всё, что можно проверить на компьютере: поиск солнца в кадре,
линия горизонта, астрономия и — самое важное — удержание трека в боковом
ветре. Полёт в ветре проверяется отдельной моделью: аппарат летит по
команде, ветер сносит, а программа должна сама найти угол сноса.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from guidance import (  # noqa: E402
    CourseGuidance,
    Tune,
    crab_estimate,
    nose_azimuth_from_sun,
    sun_azimuth,
    wrap180,
)
from sun_vision import find_horizon, measure_sun, synthetic_frame  # noqa: E402

RESULTS: list[tuple[bool, str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((ok, name, detail))
    print(f"  {'OK  ' if ok else 'ПЛОХО'} {name}" + (f" — {detail}" if detail and not ok else ""))


def test_sun() -> None:
    print("--- поиск солнца ---")
    width, height = 640, 480
    fov_x, fov_y = 120.0, 90.0
    for u, v in ((320.0, 240.0), (100.0, 60.0), (600.0, 200.0), (20.0, 460.0)):
        frame = synthetic_frame(width, height, u, v, radius=1.0, horizon_row=260)
        fix = measure_sun(frame, width, height, fov_x, fov_y)
        if fix is None:
            check(f"солнце найдено в ({u:.0f}, {v:.0f})", False, "не найдено")
            continue
        error = math.hypot(fix.u - u, fix.v - v)
        check(
            f"солнце найдено в ({u:.0f}, {v:.0f})",
            fix.valid() and error < 0.6,
            f"центр дал ({fix.u:.1f}, {fix.v:.1f}), промах {error:.2f} пикселя",
        )

    # Солнце размером в пять пикселей — это штатный случай
    frame = synthetic_frame(width, height, 300.0, 200.0, radius=1.0)
    fix = measure_sun(frame, width, height, fov_x, fov_y)
    check("солнце из пяти пикселей найдено",
          fix is not None and fix.valid() and fix.pixels == 5,
          "не найдено" if fix is None else f"пикселей {fix.pixels}")

    # Один яркий пиксель — это битый пиксель матрицы, а не солнце
    frame = synthetic_frame(width, height, 300.0, 200.0, radius=0.0)
    fix = measure_sun(frame, width, height, fov_x, fov_y)
    check("один пиксель отбракован как битый",
          fix is not None and not fix.valid(),
          "принят за солнце" if fix is not None else "не найден")

    # Большая яркая область — это не солнце, а засветка от облака
    frame = synthetic_frame(width, height, 300.0, 200.0, radius=30.0)
    fix = measure_sun(frame, width, height, fov_x, fov_y)
    check("засветка от облака отбракована", fix is not None and not fix.valid(),
          "принята за солнце" if fix is not None else "не найдена")

    # Угол от оптической оси
    frame = synthetic_frame(width, height, 320.0, 240.0, radius=1.0)
    fix = measure_sun(frame, width, height, fov_x, fov_y)
    check("солнце по центру даёт нулевой угол",
          fix is not None and abs(fix.bearing_x) < 0.5 and abs(fix.bearing_y) < 0.5,
          "" if fix is None else f"углы {fix.bearing_x:.2f}, {fix.bearing_y:.2f}")
    frame = synthetic_frame(width, height, 480.0, 240.0, radius=1.0)
    fix = measure_sun(frame, width, height, fov_x, fov_y)
    expected = (480.0 - 320.0) / 640.0 * fov_x
    check("смещение вправо даёт правильный угол",
          fix is not None and abs(fix.bearing_x - expected) < 1.0,
          "" if fix is None else f"получено {fix.bearing_x:.1f}°, ждали {expected:.1f}°")


def test_horizon() -> None:
    print("--- линия горизонта ---")
    width, height = 640, 480
    frame = synthetic_frame(width, height, 320.0, 100.0, radius=1.0, horizon_row=200)
    horizon = find_horizon(frame, width, height)
    check("горизонт найден", horizon.found, f"качество {horizon.quality:.2f}")
    check("горизонт без наклона", abs(horizon.angle_deg) < 2.0,
          f"наклон {horizon.angle_deg:.1f}°")


def test_astro() -> None:
    print("--- астрономия ---")
    azimuth, elevation = sun_azimuth(2026, 6, 21, 18.0, 55.75, 37.62)
    check("азимут солнца над Москвой вечером 21 июня",
          abs(azimuth - 227.0) < 1.5, f"получено {azimuth:.1f}°, ждали 227°")
    check("высота солнца на закате мала", -1.0 < elevation < 3.0,
          f"получено {elevation:.1f}°")

    # Нос по солнцу в кадре: солнце левее центра, значит нос правее солнца
    nose = nose_azimuth_from_sun(227.0, -10.0)
    check("азимут носа по солнцу в кадре", abs(wrap180(nose - 237.0)) < 0.01,
          f"получено {nose:.1f}°, ждали 237°")


def test_course_in_wind() -> None:
    """Главная проверка: удержит ли программа трек в боковом ветре."""
    print("--- удержание трека в боковом ветре ---")
    for wind in (0.0, 5.0, 8.0, 12.0):
        airspeed = 25.0
        target_track = 0.0
        # Ветер дует на восток, значит трек уводит вправо
        heading = target_track
        track = heading
        guidance = CourseGuidance(Tune())
        dt = 0.02
        for _ in range(int(120.0 / dt)):
            # Аппарат идёт по команде с запаздыванием первого порядка
            command = guidance.update(dt, target_track, track, heading)
            heading += (wrap180(command - heading)) * dt / 1.0
            # Трек по земле: воздушная скорость по носу плюс ветер
            vx = airspeed * math.sin(math.radians(heading)) + wind
            vy = airspeed * math.cos(math.radians(heading))
            track = math.degrees(math.atan2(vx, vy))
        final_error = abs(wrap180(target_track - track))
        expected_crab = -crab_estimate(wind, airspeed)
        crab_error = abs(wrap180(guidance.crab_angle() - expected_crab))
        check(
            f"трек удержан при ветре {wind:.0f} м/с",
            final_error < 1.0,
            f"остаточная ошибка трека {final_error:.2f}°",
        )
        check(
            f"угол сноса найден при ветре {wind:.0f} м/с",
            crab_error < 2.5,
            f"накоплено {guidance.crab_angle():.1f}°, "
            f"а надо {expected_crab:.1f}°",
        )

    # Без GPS снос неизвестен: программа держит нос на солнце и говорит об этом
    guidance = CourseGuidance(Tune())
    command = guidance.update(0.02, 227.0, None, 227.0)
    check("без GPS команда равна курсу на солнце",
          abs(wrap180(command - 227.0)) < 0.01 and not guidance.have_track,
          f"команда {command:.1f}°")


def main() -> int:
    print("Проверка программы курса по солнцу")
    test_sun()
    test_horizon()
    test_astro()
    test_course_in_wind()
    failed = [r for r in RESULTS if not r[0]]
    print()
    print(f"проверок: {len(RESULTS)}, успешно: {len(RESULTS) - len(failed)}, "
          f"провалено: {len(failed)}")
    for _ok, name, detail in failed:
        print(f"  провал: {name} — {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
