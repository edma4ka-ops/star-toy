"""Подбор площади рулей: сколько нужно, чтобы аппарат держал курс.

    python sim/size_canards.py

Главное правило проекта. Оперерение разворачивает аппарат по потоку — это
и есть устойчивость. Рули должны этот разворот пересилить, иначе они
бесполезны: никакое быстродействие контура не поможет, если рули физически
слабее оперения.

Сравнивать надо моменты на радиан угла:

    оперение:  M_оп = q * S_корп * CN_альфа * d_фокус
    рули:      M_рул = q * S_рул * CN_дельта * L_рул

Скоростной напор q в обеих формулах один и тот же, поэтому отношение
не зависит от скорости: либо рули сильнее на любой скорости, либо слабее.
Отсюда требование:

    S_рул * CN_дельта * L_рул >= k * S_корп * CN_альфа * d_фокус,  k ~ 1,5

Запас k = 1,5 нужен потому, что рули должны не только удержать равновесие,
но и перебороть его с запасом на порыв ветра.
"""

from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from control import CourseHolder, Tune, Vehicle  # noqa: E402
from rocket import Conditions, Rocket, prepare_holder, simulate  # noqa: E402

MARGIN = 1.5


def airframe_spring(rocket: Rocket) -> float:
    """Момент оперения на радиан угла атаки, Н·м/рад."""
    return rocket.area * rocket.cn_alpha * abs(rocket.aero_arm)


def canard_moment(vehicle: Vehicle) -> float:
    """Момент рулей на радиан отклонения, Н·м/рад."""
    return vehicle.canard_area * vehicle.cn_delta * vehicle.canard_arm


def required_area(rocket: Rocket, vehicle: Vehicle, margin: float = MARGIN) -> float:
    """Какая площадь пары рулей нужна, чтобы пересилить оперение с запасом."""
    return margin * airframe_spring(rocket) / (vehicle.cn_delta * vehicle.canard_arm)


def main() -> int:
    rocket = Rocket()
    vehicle = rocket.vehicle
    spring = airframe_spring(rocket)
    moment = canard_moment(vehicle)
    need = required_area(rocket, vehicle)

    print("Расчёт рулей")
    print(f"  калибр аппарата:            {rocket.diameter * 1000:.0f} мм")
    print(f"  площадь корпуса:            {rocket.area * 1e4:.1f} см²")
    print(f"  фокус за центром масс:      {abs(rocket.aero_arm) * 100:.0f} см")
    print(f"  момент оперения:            {spring:.5f} Н·м/рад")
    print(f"  рули сейчас:                {vehicle.canard_area * 1e4:.0f} см² "
          f"({vehicle.canard_area * 1e4 / 4:.1f} см² на руль)")
    print(f"  момент рулей:               {moment:.5f} Н·м/рад")
    print(f"  запас:                      {moment / spring:.2f}x при нужных {MARGIN:.2f}x")
    print(f"  нужная площадь пары рулей:  {need * 1e4:.0f} см² "
          f"({need * 1e4 / 4:.1f} см² на руль)")
    if vehicle.canard_area < need:
        print("  ВЫВОД: рули слабее оперения — курс в ветре держаться не будет")
    else:
        print("  ВЫВОД: рули сильнее оперения — курс держать можно")

    print()
    print("Проверка прогоном: снос при ветре 3 м/с и порыве 3 м/с")
    conditions = Conditions(wind=3.0, gust=3.0)
    passive = simulate("без управления", Rocket(vehicle=Vehicle()), conditions, False)
    print(f"  без управления:             {passive.max_deviation:6.1f} м")
    print(f"  {'площадь':>10}{'запас':>8}{'снос':>9}{'ошибка':>9}{'рули':>8}{'лучше':>8}")
    for area in (0.0024, 0.004, 0.005, 0.008, 0.012):
        vehicle = Vehicle(canard_area=area)
        rocket = Rocket(vehicle=vehicle)
        holder = CourseHolder(Tune(), vehicle)
        prepare_holder(rocket, conditions, holder)
        result = simulate("прогон", rocket, conditions, True, holder)
        reserve = canard_moment(vehicle) / spring
        print(
            f"  {area * 1e4:>8.0f} см²{reserve:>7.2f}x{result.max_deviation:>8.1f} м"
            f"{result.max_err_deg:>8.1f}°{result.max_delta_deg:>7.1f}°"
            f"{passive.max_deviation / max(result.max_deviation, 1e-9):>7.1f}x"
        )
    print()
    print("Как видно из таблицы, до запаса 1,5 улучшение почти не растёт:")
    print("аппарат борется с ветром и всё равно уходит с курса.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
