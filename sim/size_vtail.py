"""Подбор рулей для аппарата, висящего на тяге винта.

    python sim/size_vtail.py

Главное правило здесь то же, что у ракеты, но враг другой. У ракеты рули
боролись с оперением; здесь они борются с реактивным моментом винта.
Пока момент рулей меньше момента винта, аппарат крутится — и никакая
программа этого не изменит.

Момент рулей:  M_рул = 2 * R * q * S * CN * d
где R — смещение руля от оси, q — напор в струе за винтом (T / A_винта),
S — площадь одного руля, d — угол отклонения.

Момент винта:  M_винт = P / w, где P — мощность в струе, w — угловая
скорость вала. Мощность считается по импульсной теории: P = T * v / КПД,
где v = sqrt(T / (2 * rho * A_винта)).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from hover import G, RHO, Hover  # noqa: E402

MARGIN = 1.5


def fin_geometry(craft: Hover, span: float, chord: float) -> tuple[float, float]:
    """Площадь руля и смещение от оси по его размаху и хорде.

    Смещение R — это расстояние от оси аппарата до центра давления руля:
    радиус корпуса плюс половина размаха. Отсюда важное свойство: длинный
    узкий руль даёт момент двойной выгодой — и площадь растёт, и плечо.
    """
    area = span * chord
    radius = craft.diameter / 2.0 + span / 2.0
    return area, radius


def moment_of(craft: Hover, area: float, radius: float) -> float:
    return 2.0 * radius * craft.slipstream_q() * area * craft.fin_cn_delta * craft.craft.rudder_max


def required_area(craft: Hover, radius: float, margin: float = MARGIN) -> float:
    return margin * craft.motor_torque() / (
        2.0 * radius * craft.slipstream_q() * craft.fin_cn_delta * craft.craft.rudder_max
    )


def main() -> int:
    craft = Hover()
    torque = craft.motor_torque()
    print("Что нужно, чтобы удержать курс")
    print(f"  аппарат:                    {craft.mass * 1000:.0f} г, "
          f"корпус {craft.diameter * 1000:.0f} мм")
    print(f"  винт:                       {craft.prop_diameter * 1000:.0f} мм, "
          f"{craft.prop_rpm:.0f} об/мин")
    print(f"  тяга:                       {craft.thrust():.1f} Н")
    print(f"  напор в струе:              {craft.slipstream_q():.0f} Па")
    print(f"  реактивный момент винта:    {torque:.3f} Н·м")
    print(f"  нужен момент рулей:         {MARGIN * torque:.3f} Н·м (запас {MARGIN})")
    print()

    print("Вариант 1: рули V-образно. Размах решает больше, чем площадь")
    print(f"  {'размах':>8}{'хорда':>8}{'площадь':>10}{'плечо R':>10}{'момент':>10}{'запас':>8}")
    for span, chord in ((0.03, 0.07), (0.06, 0.07), (0.12, 0.06), (0.18, 0.05), (0.25, 0.04)):
        area, radius = fin_geometry(craft, span, chord)
        moment = moment_of(craft, area, radius)
        print(
            f"  {span * 100:>6.0f} см{chord * 100:>6.0f} см{area * 1e4:>8.0f} см²"
            f"{radius * 100:>8.1f} см{moment:>8.3f} Н·м{moment / torque:>7.2f}x"
        )
    print("  Читается так: короткий широкий руль почти бесполезен, потому что")
    print("  плечо R мало. Длинный узкий руль даёт момент двойной выгодой.")
    print()

    print("Вариант 2: хвостовой винт на штанге — рычаг в разы длиннее")
    boom = 0.30
    print(f"  штанга {boom * 100:.0f} см; нужная тяга винта:")
    need_force = MARGIN * torque / boom
    print(f"    {need_force:.2f} Н — это винт {50} мм на малых оборотах или")
    print(f"    моторчик размером с тот, что стоит в квадрокоптере 100 мм")
    for force in (0.2, 0.5, 1.0):
        print(f"    тяга {force:.1f} Н -> момент {force * boom:.3f} Н·м "
              f"({force * boom / torque:.2f}x от винта)")
    print()

    print("Вариант 3: два винта встречного вращения — момент гасится сам")
    print("  Реактивные моменты двух винтов направлены в разные стороны")
    print("  и вычитаются. Бороться становится не с чем: рули нужны только")
    print("  против ветра. Это самый чистый вариант, но нужен второй мотор")
    print("  и реверс одного винта (или редуктор с двумя валами).")
    print()

    print("Вариант 4: два винта рядом с разной тягой — курс делается разностью")
    print("  Момент = разность тяг * плечо между винтами. Плечо здесь")
    print("  полразмаха, то есть десятки сантиметров, поэтому момент огромный,")
    print("  а заодно появляется управление наклоном — то, чего у аппарата")
    print("  с одним винтом нет вообще.")
    for lever in (0.15, 0.25):
        for delta in (0.2, 0.5):
            print(f"    плечо {lever * 100:.0f} см, разность тяг {delta:.1f} Н -> "
                  f"{lever * delta:.3f} Н·м ({lever * delta / torque:.2f}x)")
    print()

    print("Как меняется задача при другом винте")
    print(f"  {'винт':>8}{'напор':>9}{'момент винта':>15}{'нужно рулей':>14}")
    for diameter in (0.08, 0.10, 0.15, 0.20, 0.25):
        probe = Hover(prop_diameter=diameter)
        area, radius = fin_geometry(probe, 0.12, 0.06)
        print(
            f"  {diameter * 1000:>6.0f} мм{probe.slipstream_q():>8.0f} Па"
            f"{probe.motor_torque():>13.3f} Н·м{required_area(probe, radius) * 1e4:>11.0f} см²"
        )
    print("  Правило простое: маленький винт даёт плотную струю (рулям легче)")
    print("  и одновременно большой момент (рулям тяжелее). Смотрите на")
    print("  последний столбец: это площадь руля при размахе 12 см.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
