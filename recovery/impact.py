"""Расчёт спасения ракеты парашютом по крупному объекту в кадре.

    python recovery/impact.py

Условия задачи: парашют считать неубиваемым, система включается через
3 секунды после старта, признак срабатывания — крупный объект в кадре.

Считаем три вещи, и каждая может закрыть идею:

1. Дальность обнаружения. Камера видит объект, только когда он занимает
   достаточно пикселей. Значит есть дистанция, ближе которой замечаем.

2. Время реакции. Кадр, подтверждение по нескольким кадрам, задержка
   запала, раскрытие купола. Всё это аппарат проходит, не тормозя.

3. Рывок при раскрытии. Парашют не останавливает мягко: на скорости
   он даёт перегрузку, и вопрос в том, переживут ли её не купол,
   а аппаратура внутри.
"""

from __future__ import annotations

import math
import sys

G = 9.81
RHO = 1.2

MASS = 2.4  # кг, полная масса аппарата
CANOPY = 1.4  # м, диаметр купола
CANOPY_CD = 0.75
SPEED = 50.0  # м/с, скорость на разгоне

FOV_X_DEG = 120.0
WIDTH, HEIGHT = 640, 480
FPS = 30.0
CONFIRM_FRAMES = 3
PYRO_DELAY = 0.08  # с, задержка вышибного заряда
INFLATION = 1.2  # с, раскрытие купола

# Сколько пикселей должно быть у объекта, чтобы назвать его крупным
BIG_PIXELS = 8

# Что аппаратура переживает без разрушения, перегрузка в единицах g
EQUIPMENT_LIMIT_G = 15.0


def detection_range(size_m: float, pixels: int = BIG_PIXELS) -> float:
    """Дальность, с которой объект займёт нужное число пикселей."""
    angle_per_pixel = math.radians(FOV_X_DEG) / WIDTH
    return size_m / (pixels * angle_per_pixel)


def reaction_distance(speed: float) -> tuple[float, float]:
    """Сколько метров аппарат пройдёт, пока система решит и раскроет купол."""
    time_s = 1.0 / FPS + CONFIRM_FRAMES / FPS + PYRO_DELAY + INFLATION
    return time_s, time_s * speed


def dual_deployment(drogue: float = 0.6, main_canopy: float = CANOPY,
                    speed: float = SPEED) -> dict:
    """Двойной выпуск: сначала маленький купол, потом основной.

    Смысл двойного выпуска в том, что большой купол раскрывается уже
    на малой скорости. Тогда рывок получается мягким, и его переживает
    не только корпус, но и аппаратура внутри.

    Считаем по шагам: рывок на вытяжном, скорость спуска на вытяжном,
    рывок на основном при этой скорости, конечная скорость спуска.
    """
    def chute(diameter: float, at_speed: float) -> tuple[float, float]:
        area = math.pi * diameter ** 2 / 4.0
        force = 0.5 * RHO * at_speed ** 2 * area * CANOPY_CD
        return area, force

    def terminal(area: float) -> float:
        return math.sqrt(2.0 * MASS * G / (RHO * area * CANOPY_CD))

    drogue_area, drogue_force = chute(drogue, speed)
    drogue_g = drogue_force / MASS / G
    drogue_terminal = terminal(drogue_area)

    main_area, main_force = chute(main_canopy, drogue_terminal)
    main_g = main_force / MASS / G
    main_terminal = terminal(main_area)

    return {
        "drogue_area": drogue_area,
        "drogue_force": drogue_force,
        "drogue_g": drogue_g,
        "drogue_terminal": drogue_terminal,
        "main_area": main_area,
        "main_force": main_force,
        "main_g": main_g,
        "main_terminal": main_terminal,
    }


def canopy_load(speed: float, mass: float = MASS, canopy: float = CANOPY) -> tuple[float, float, float]:
    """Рывок при раскрытии: сила, перегрузка, тормозной путь."""
    area = math.pi * canopy ** 2 / 4.0
    force = 0.5 * RHO * speed ** 2 * area * CANOPY_CD
    accel = force / mass
    return force, accel / G, speed ** 2 / (2.0 * accel)


def main() -> int:
    print("Спасение ракеты парашютом по крупному объекту")
    print(f"  аппарат {MASS} кг, купол {CANOPY} м, скорость {SPEED:.0f} м/с")
    print(f"  камера {WIDTH}x{HEIGHT}, поле зрения {FOV_X_DEG:.0f}°, {FPS:.0f} кадров/с")
    print()

    print("1. С какой дистанции камера вообще замечает объект")
    print(f"  {'объект':<28}{'размер':>9}{'дистанция':>13}")
    objects = (
        ("птица", 0.3),
        ("дрон", 0.5),
        ("дерево", 10.0),
        ("дом", 20.0),
        ("земля при пикировании", 500.0),
    )
    for name, size in objects:
        print(f"  {name:<28}{size:>7.1f} м{detection_range(size):>11.0f} м")
    print("  Птица становится «крупным объектом» с 11 м. На такой дистанции")
    print("  при скорости 50 м/с до неё остаётся 0,22 секунды.")
    print()

    print("2. Время реакции и путь за это время")
    time_s, distance = reaction_distance(SPEED)
    print(f"  кадр + подтверждение:        {1.0 / FPS + CONFIRM_FRAMES / FPS:.2f} с")
    print(f"  задержка запала:             {PYRO_DELAY:.2f} с")
    print(f"  раскрытие купола:            {INFLATION:.2f} с")
    print(f"  итого:                       {time_s:.2f} с")
    print(f"  за это время аппарат пройдёт:{distance:>6.0f} м")
    print()

    print("3. Что делает купол, когда раскрылся")
    force, g_load, stop_distance = canopy_load(SPEED)
    print(f"  сила на куполе:              {force:.0f} Н")
    print(f"  перегрузка:                  {g_load:.0f} g")
    print(f"  тормозной путь:              {stop_distance:.1f} м")
    print()
    print(f"  Купол останавливает аппарат почти мгновенно: за {stop_distance:.1f} м.")
    print(f"  Но цена — перегрузка {g_load:.0f} g. Аппаратура переживает примерно")
    print(f"  {EQUIPMENT_LIMIT_G:.0f} g. Значит купол цел, а камера, платы и батарея — нет.")
    print()

    print("3б. Двойной выпуск: вытяжной, затем основной")
    d = dual_deployment()
    print(f"  вытяжной купол 0,6 м:")
    print(f"    рывок при {SPEED:.0f} м/с:        {d['drogue_force']:>6.0f} Н "
          f"({d['drogue_g']:.0f} g) {'переживёт' if d['drogue_g'] < EQUIPMENT_LIMIT_G else 'НЕ переживёт'}")
    print(f"    скорость спуска:           {d['drogue_terminal']:>6.1f} м/с")
    print(f"  основной купол {CANOPY:.1f} м раскрывается на {d['drogue_terminal']:.0f} м/с:")
    print(f"    рывок:                     {d['main_force']:>6.0f} Н "
          f"({d['main_g']:.1f} g) {'переживёт' if d['main_g'] < EQUIPMENT_LIMIT_G else 'НЕ переживёт'}")
    print(f"    скорость спуска:           {d['main_terminal']:>6.1f} м/с")
    print()
    print(f"  Вы правы: двойной выпуск снимает перегрузку. Вместо {g_load:.0f} g")
    print(f"  получается {d['drogue_g']:.0f} g, а потом {d['main_g']:.1f} g — аппаратура цела.")
    print()


    print(f"  {'объект':<24}{'замечаем':>10}{'нужно':>9}{'успеет?':>12}{'перегрузка':>13}")
    needed = distance + stop_distance
    for name, size in objects:
        found = detection_range(size)
        ok = "да" if found >= needed else "нет"
        print(f"  {name:<24}{found:>8.0f} м{needed:>7.0f} м{ok:>12}{g_load:>11.0f} g")
    print()
    print(f"  Чтобы успеть, объект надо заметить с {needed:.0f} м. Столько дают")
    print("  только дерево, дом и земля. Птицу и дрон — нет, они слишком мелкие.")
    print()

    print("5. Ложные срабатывания: что камера считает крупным объектом")
    print("  Угол траектории и доля кадра, занятая землёй:")
    for angle in (90.0, 60.0, 45.0, 30.0, 15.0, 5.0):
        # Чем ниже нос, тем больше земли в кадре; при 45° примерно половина
        share = min(1.0, max(0.0, (90.0 - angle) / 90.0 * 1.6))
        verdict = "сработает" if share * WIDTH * HEIGHT > BIG_PIXELS * 100 else "молчит"
        print(f"    {angle:>4.0f}°   {share * 100:>5.0f}% кадра   {verdict}")
    print("  Земля — самый крупный объект в кадре и появляется она всегда,")
    print("  как только нос опускается. То есть признак «крупный объект»")
    print("  срабатывает не на препятствие, а на снижение.")
    print()

    print("6. Что будет, если включить систему через 3 секунды после старта")
    altitude = 30.0 * 3.0 - 0.5 * G * 1.0 ** 2
    print(f"  через 3 с аппарат на высоте около {altitude:.0f} м и идёт на {SPEED:.0f} м/с")
    print(f"  раскрытие купола там даёт те же {g_load:.0f} g и гасит скорость почти сразу,")
    print(f"  поэтому аппарат опускается под точкой раскрытия — рядом с пусковой")
    print(f"  площадкой, где стоят люди. При спуске {6:.0f} м/с с {altitude:.0f} м это")
    print(f"  около {altitude / 6:.0f} секунд на то, чтобы отойти.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
