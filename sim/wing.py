"""Подбор крыла: площадь, размах, скорость срыва, скороподъёмность.

    python sim/wing.py

Крыло меняет всё. Без него аппарат держался только тягой, и потому был
обязан лететь почти вертикально. С крылом подъёмную силу даёт крыло,
тяга нужна только против сопротивления — и появляется тот самый полёт
по горизонту с набором высоты.

Считается по обычным формулам самолёта:

    подъёмная сила   Y = 0,5 * rho * V^2 * S * CL
    сопротивление    X = 0,5 * rho * V^2 * S * (CD0 + CL^2 / (pi * e * AR))
    скорость срыва   Vs = sqrt(2 * вес / (rho * S * CLmax))

Масса берётся в двух вариантах: 2,0 кг (как говорили раньше) и 2,8 кг
(если 800 г оборудования добавляются к изделию, а не входят в него).
"""

from __future__ import annotations

import math
import sys

RHO = 1.2
G = 9.81


def wing_area(weight: float, speed: float, cl: float) -> float:
    """Площадь крыла, нужная чтобы нести вес на данной скорости."""
    return 2.0 * weight / (RHO * speed * speed * cl)


def stall_speed(weight: float, area: float, cl_max: float) -> float:
    return math.sqrt(2.0 * weight / (RHO * area * cl_max))


def drag(weight: float, area: float, speed: float, cl: float, cd0: float, ar: float) -> float:
    induced = cl * cl / (math.pi * 0.8 * ar)
    return 0.5 * RHO * speed * speed * area * (cd0 + induced)


def main() -> int:
    equipment = 0.8  # кг оборудования, которое должно лететь
    cl_max = 1.1
    cd0 = 0.035
    ar = 6.0

    print("Оборудование, которое должно лететь: 800 г")
    print("Считаю два варианта массы, потому что это меняет размер крыла")
    print()

    for mass in (2.8, 2.0):
        weight = mass * G
        print(f"=== Масса {mass:.1f} кг, вес {weight:.1f} Н ===")
        if mass > 2.0:
            print("    (изделие 2,0 кг плюс 800 г оборудования)")
        else:
            print("    (800 г оборудования уже внутри 2,0 кг)")

        area_target = wing_area(weight, 22.0, 0.5)
        span = math.sqrt(area_target * ar)
        chord = area_target / span
        vs = stall_speed(weight, area_target, cl_max)

        print(f"  площадь крыла:            {area_target * 1e4:.0f} см²")
        print(f"  размах при удлинении {ar:.0f}:     {span * 100:.0f} см")
        print(f"  хорда:                    {chord * 100:.0f} см")
        print(f"  скорость срыва:           {vs:.1f} м/с")
        print(f"  сход с катапульты:        19 м/с — "
              f"{'запас есть' if 19.0 > vs * 1.25 else 'ЗАПАС МАЛ, нужен размах больше'}")
        print()

        print(f"  {'скорость':>10}{'CL':>7}{'сопротивление':>16}{'нужна тяга':>13}")
        for speed in (18.0, 22.0, 26.0, 30.0, 35.0, 40.0):
            cl = 2.0 * weight / (RHO * speed * speed * area_target)
            if cl > cl_max:
                print(f"  {speed:>8.0f} м/с{cl:>7.2f}{'срыв':>16}{'—':>13}")
                continue
            x = drag(weight, area_target, speed, cl, cd0, ar)
            print(f"  {speed:>8.0f} м/с{cl:>7.2f}{x:>14.1f} Н{x:>11.1f} Н")
        print()

        print("  Что даёт двигатель")
        for name, thrust in (("форсаж 5 кг", 5.0 * G), ("крейсер 2 кг", 2.0 * G)):
            print(f"    {name}: {thrust:.1f} Н")
            for speed in (22.0, 26.0, 30.0):
                cl = 2.0 * weight / (RHO * speed * speed * area_target)
                if cl > cl_max:
                    continue
                x = drag(weight, area_target, speed, cl, cd0, ar)
                excess = thrust - x
                if excess <= 0.0:
                    print(f"      на {speed:.0f} м/с: тяги не хватает даже "
                          f"на горизонталь (нужно {x:.1f} Н)")
                    continue
                climb_angle = math.degrees(math.asin(min(1.0, excess / weight)))
                print(f"      на {speed:.0f} м/с: запас {excess:.1f} Н -> "
                      f"подъём под {climb_angle:.0f}°, "
                      f"{speed * math.sin(math.radians(climb_angle)):.0f} м/с по высоте")
        print()

    print("Рекомендуемый размер с учётом массы самого крыла")
    print("  Крыло невесомым не бывает: пенопластовое с лонжероном весит")
    print("  около 1,2 кг на квадратный метр площади. Считаем с этим.")
    launch_speed = 19.0
    for base_mass, label in ((2.8, "изделие 2,0 кг + оборудование 800 г"),
                             (2.0, "оборудование внутри 2,0 кг")):
        area = 0.15
        for _ in range(6):  # пара пересчётов, сходится быстро
            wing_mass = area * 1.2
            total = base_mass + wing_mass
            area = 2.0 * total * G / (RHO * 22.0 ** 2 * 0.5)
        wing_mass = area * 1.2
        total = base_mass + wing_mass
        span = math.sqrt(area * ar)
        # Площадь, при которой срыв отстоит от схода в 1,3 раза
        safe_area = 2.0 * total * G / (RHO * (launch_speed / 1.3) ** 2 * cl_max)
        safe_span = math.sqrt(safe_area * ar)
        print(f"  {label}:")
        print(f"    масса крыла:            {wing_mass * 1000:.0f} г")
        print(f"    полная масса:           {total:.2f} кг")
        print(f"    площадь крыла:          {area * 1e4:.0f} см², "
              f"размах {span * 100:.0f} см, хорда {area / span * 100:.0f} см")
        print(f"    скорость срыва:         "
              f"{stall_speed(total * G, area, cl_max):.1f} м/с "
              f"(сход {launch_speed:.0f} м/с, запас "
              f"{launch_speed / stall_speed(total * G, area, cl_max):.2f}x)")
        print(f"    для запаса 1,3 нужно:   {safe_area * 1e4:.0f} см², "
              f"размах {safe_span * 100:.0f} см")
        print()

    print("Что из этого следует")
    print("  1. Крыло делает полёт по горизонту возможным при тяге меньше веса.")
    print("     Крейсерских 2 кг хватает с большим избытком: на 26-30 м/с запас")
    print("     уходит в подъём под 34-38° со скороподъёмностью 13-17 м/с.")
    print("  2. Скорость срыва надо знать обязательно. Ниже неё аппарат падает")
    print("     даже с работающим двигателем: крыло перестаёт нести.")
    print("  3. Сход с катапульты должен быть быстрее срыва минимум в 1,3 раза.")
    print("     Резинки дают 19 м/с, значит срыв должен быть не выше 14,6 м/с.")
    print("  4. Форсаж 5 кг для крылатого аппарата избыточен: запаса 46 Н хватает")
    print("     на вертикальный подъём. Крыло становится не нужно, и аппарат")
    print("     ведёт себя как ракета. Форсаж стоит ослабить до 1,5-2 весов.")
    print()
    print("Чего крыло не решает")
    print("  Крыло даёт подъёмную силу только на угле атаки, значит появляется")
    print("  тангаж. Одним рулём направления его не удержать: аппарат либо")
    print("  задерёт нос и свалится, либо опустит и разгонится в пикирование.")
    print("  Нужно одно из двух:")
    print("    * руль высоты и оттриммированный аппарат — правильный путь;")
    print("    * или жёсткая оттриммировка без управления по тангажу, но тогда")
    print("      угол подъёма задаётся тягой, а не программой.")
    print()
    print("  И про высоту: подъём 13-17 м/с за 18 секунд работы двигателя —")
    print("  это 250-300 м. Нужен ограничитель: либо по времени, либо по высоте.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
