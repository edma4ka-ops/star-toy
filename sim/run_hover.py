"""Прогон аппарата с винтом: держит ли он курс вокруг вертикали.

    python sim/run_hover.py

Печатает три вещи, которые решают судьбу конструкции:
  * насколько сильно аппарат закручивает реактивный момент винта;
  * может ли пара рулей этот момент перебороть;
  * что происходит с курсом за 40 секунд полёта.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from control import HeadingHolder, HeadingTune, HoverCraft  # noqa: E402
from hover import Conditions, Hover, simulate  # noqa: E402


def scenario(name: str, craft: Hover, control_on: bool, light: bool = True, note: str = ""):
    cond = Conditions(light_available=light, gyro_noise=math.radians(0.1))
    holder = None
    if control_on:
        holder = HeadingHolder(HeadingTune(), craft.craft)
        holder.arm()
    result = simulate(name, craft, cond, control_on, holder)
    result.note = note
    return result


def balance(craft: Hover) -> None:
    """Главный расчёт: кто кого — винт или рули."""
    torque = craft.motor_torque()
    moment_max = craft.rudder_moment(craft.craft.rudder_max)
    print("Баланс момента винта и рулей")
    print(f"  масса и вес:                {craft.mass * 1000:.0f} г, {craft.weight:.1f} Н")
    print(f"  тяга винта:                 {craft.thrust():.1f} Н "
          f"({craft.thrust_ratio:.2f} от веса)")
    print(f"  диаметр винта:              {craft.prop_diameter * 1000:.0f} мм")
    print(f"  напор в струе за винтом:    {craft.slipstream_q():.0f} Па")
    print(f"  реактивный момент винта:    {torque:.3f} Н·м")
    print(f"  рули:                       {craft.fin_area * 1e4:.0f} см² каждый, "
          f"смещение {craft.fin_radius * 100:.0f} см, ход ±{craft.rudder_max_deg:.0f}°")
    print(f"  момент рулей на полном ходу:{moment_max:.3f} Н·м")
    ratio = moment_max / max(torque, 1e-9)
    print(f"  рули против винта:          {ratio:.2f}x")
    if ratio < 1.0:
        print("  ВЫВОД: рули слабее винта — аппарат будет крутиться даже при")
        print(f"         полностью отклонённых рулях. Нужно в {1.0 / max(ratio, 1e-9):.1f} раза больше момента.")
    else:
        print("  ВЫВОД: рули сильнее винта, курс удержать можно")

    need = 1.5 * torque / (2.0 * craft.fin_radius * craft.slipstream_q()
                           * craft.fin_cn_delta * craft.craft.rudder_max)
    print(f"  для запаса 1,5 нужно по:    {need * 1e4:.0f} см² на руль")
    if need <= craft.fin_area:
        print("         (текущей площади хватает)")
    else:
        print(f"         это в {need / craft.fin_area:.1f} раза больше текущей")
    side = craft.rudder_side_force(craft.craft.rudder_max)
    print(f"  цена удержания курса:       боковая сила {side:.2f} Н, "
          f"это снос около {side / craft.mass:.1f} м/с²")
    print()


def main() -> int:
    craft = Hover()
    balance(craft)

    runs = [
        scenario("без управления", craft, control_on=False,
                 note="реактивный момент винта никто не перебарывает"),
        scenario("курс держится, свет виден", craft, control_on=True, light=True,
                 note="датчик света даёт абсолютный курс"),
        scenario("курс держится, света нет", craft, control_on=True, light=False,
                 note="курс держится только гироскопом и медленно уплывает"),
    ]

    print(f"{'прогон':<30}{'высота':>9}{'вращение':>11}{'уход курса':>12}"
          f"{'рули':>8}{'снос вбок':>11}")
    print("-" * 84)
    for r in runs:
        print(
            f"{r.name:<30}{r.altitude:>8.0f} м{r.spin_end:>10.0f}°/с"
            f"{r.heading_drift:>11.1f}°{r.max_rudder_deg:>7.1f}°{r.side_drift:>10.1f} м"
        )
    print()
    for r in runs:
        if r.note:
            print(f"  {r.name}: {r.note}")
    print()
    print("Курс считается удержанным, если уход не больше 10° за 40 с полёта.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
