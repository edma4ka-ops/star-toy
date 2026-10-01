"""Массовая сводка и продольная центровка.

    python sim/mass.py

Две задачи, и вторая важнее первой.

1. Сложить массы и получить полную: от неё зависит размер крыла, а размер
   крыла добавляет массу — поэтому считаем парой пересчётов, пока не сойдётся.

2. Найти, где окажется центр масс. У крылатого аппарата это не мелочь:
   крыло даёт подъёмную силу в своей аэродинамической фокусе, и если центр
   масс окажется позади него, аппарат станет неустойчив по тангажу —
   задерёт нос, потеряет скорость и свалится. Тяжёлый двигатель на хвосте
   тянет центр масс назад, поэтому крыло придётся ставить дальше назад
   или грузить нос оборудованием.
"""

from __future__ import annotations

import math
import sys

RHO = 1.2
G = 9.81

# Компоненты: название, масса (кг), положение центра (м от носа).
# Положения — предположение; их надо будет уточнить по чертежу.
COMPONENTS = (
    ("оборудование 800 г", 0.80, 0.25),
    ("двигатель с топливом", 0.80, 0.95),
    ("корпус, хвост, оперение", 0.35, 0.55),
    ("машинки и тяги", 0.09, 0.50),
    ("питание и электроника", 0.10, 0.30),
)

FUSELAGE_LENGTH = 1.10
CL_MAX = 1.1
AR = 6.0
WING_MASS_PER_M2 = 1.2


def wing_area(weight: float, speed: float, cl: float) -> float:
    return 2.0 * weight / (RHO * speed * speed * cl)


def main() -> int:
    fixed_mass = sum(m for _n, m, _x in COMPONENTS)
    moment = sum(m * x for _n, m, x in COMPONENTS)

    print("Массовая сводка")
    print(f"  {'компонент':<26}{'масса':>9}{'положение':>12}{'момент':>10}")
    for name, mass, position in COMPONENTS:
        print(f"  {name:<26}{mass * 1000:>7.0f} г{position * 100:>10.0f} см"
              f"{mass * position:>9.2f}")
    print(f"  {'итого без крыла':<26}{fixed_mass * 1000:>7.0f} г"
          f"{moment / fixed_mass * 100:>10.0f} см{moment:>9.2f}")
    print()

    # Крыло подбираем вместе с его массой: площадь зависит от массы,
    # масса зависит от площади — пара пересчётов сходится
    area = 0.20
    for _ in range(8):
        wing_mass = area * WING_MASS_PER_M2
        total = fixed_mass + wing_mass
        area = wing_area(total * G, 22.0, 0.5)
    wing_mass = area * WING_MASS_PER_M2
    total = fixed_mass + wing_mass
    span = math.sqrt(area * AR)
    chord = area / span
    stall = math.sqrt(2.0 * total * G / (RHO * area * CL_MAX))

    print("Полная масса и крыло")
    print(f"  крыло:                      {wing_mass * 1000:.0f} г")
    print(f"  полная масса:               {total:.2f} кг")
    print(f"  площадь крыла:              {area * 1e4:.0f} см²")
    print(f"  размах:                     {span * 100:.0f} см")
    print(f"  хорда:                      {chord * 100:.0f} см")
    print(f"  скорость срыва:             {stall:.1f} м/с")
    print(f"  запас на сходе (19 м/с):    {19.0 / stall:.2f}x "
          f"({'достаточно' if 19.0 / stall >= 1.3 else 'МАЛО, нужен размах больше'})")
    safe_area = 2.0 * total * G / (RHO * (19.0 / 1.3) ** 2 * CL_MAX)
    print(f"  для запаса 1,3 нужно:       {safe_area * 1e4:.0f} см², "
          f"размах {math.sqrt(safe_area * AR) * 100:.0f} см")
    print()

    # --- продольная центровка ---
    print("Продольная центровка")
    # Крыло ставим так, чтобы центр масс оказался на четверти хорды:
    # это привычное и безопасное положение для простого профиля
    cg_without_wing = moment / fixed_mass
    # Центр масс с крылом: крыло ставим передней кромкой в x, его центр
    # тяжести примерно на половине хорды
    def cg_for(leading_edge: float) -> float:
        wing_position = leading_edge + chord * 0.5
        return (moment + wing_mass * wing_position) / total

    leading_edge = cg_without_wing - chord * 0.25
    for _ in range(10):
        cg = cg_for(leading_edge)
        leading_edge = cg - chord * 0.25

    cg = cg_for(leading_edge)
    wing_position = leading_edge + chord * 0.5
    print(f"  центр масс без крыла:       {cg_without_wing * 100:.0f} см от носа")
    print(f"  центр масс с крылом:        {cg * 100:.0f} см от носа")
    print(f"  передняя кромка крыла:      {leading_edge * 100:.0f} см от носа")
    print(f"  четверть хорды (фокус):     {(leading_edge + chord * 0.25) * 100:.0f} см от носа")
    print(f"  хвост аппарата:             {FUSELAGE_LENGTH * 100:.0f} см от носа")
    if leading_edge + chord > FUSELAGE_LENGTH:
        print("  ВНИМАНИЕ: крыло не помещается на корпус — он слишком короткий")
    if leading_edge < 0.0:
        print("  ВНИМАНИЕ: центр масс ушёл вперёд настолько, что крыло некуда ставить")
    print()

    print("Что это значит")
    print(f"  Двигатель 800 г сидит на {COMPONENTS[1][2] * 100:.0f} см и тянет центр масс")
    print(f"  назад, к {cg_without_wing * 100:.0f} см от носа. Крыло встаёт на "
          f"{leading_edge * 100:.0f} см — это задняя половина корпуса, как у")
    print("  аппарата с толкающим двигателем. За крылом остаётся "
          f"{(FUSELAGE_LENGTH - leading_edge - chord) * 100:.0f} см на хвост,")
    print("  и этого хватает для руля и оперения.")
    print()
    print("  Рычаги, если центровка не устроит:")
    print("    1. перенести оборудование вперёд — самый сильный рычаг:")
    equipment_mass = COMPONENTS[0][1]
    shift = equipment_mass * 0.10 / total * 100
    print(f"       сдвиг 800 г на 10 см вперёд сдвигает центр масс на {shift:.1f} см;")
    print("    2. укоротить корпус за двигателем;")
    print("    3. взять более лёгкий двигатель или меньше топлива.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
