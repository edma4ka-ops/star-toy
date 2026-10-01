"""Расчёт профиля: старт 5 кг две секунды, катапульта 2,5 м, затем 2 кг.

    python sim/size_boost.py

Здесь считаются четыре вещи, которые решают судьбу аппарата:

  1. С какой скоростью он сходит с катапульты и что происходит за 2 секунды
     форсированной тяги.
  2. Каким будет установившийся полёт при тяге, равной весу: с каким углом
     к горизонту и с какой скоростью.
  3. Каков реактивный момент винта на старте и в крейсере.
  4. Хватает ли рулей перебороть этот момент в каждой фазе.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

G = 9.81
RHO = 1.2


def induced_velocity(thrust: float, prop_area: float) -> float:
    """Скорость струи за винтом по импульсной теории."""
    return math.sqrt(max(thrust, 0.0) / (2.0 * RHO * prop_area))


def motor_torque(thrust: float, prop_area: float, rpm: float, efficiency: float) -> float:
    """Реактивный момент винта на корпус, Н·м."""
    power = thrust * induced_velocity(thrust, prop_area) / efficiency
    omega = rpm * 2.0 * math.pi / 60.0
    return power / max(omega, 1e-6)


def slipstream_q(thrust: float, prop_area: float) -> float:
    """Динамический напор в струе за винтом, Па."""
    return thrust / prop_area


def rudder_moment(q_total: float, area: float, radius: float, cn: float, deg: float) -> float:
    force = q_total * area * cn * math.radians(deg)
    return 2.0 * radius * force


def main() -> int:
    # --- исходные данные ---
    mass = 2.0
    weight = mass * G
    body_diameter = 0.12
    body_area = math.pi * body_diameter ** 2 / 4.0
    cd = 0.45
    drag_area = cd * body_area
    prop_diameter = 0.25
    prop_area = math.pi * prop_diameter ** 2 / 4.0
    prop_rpm = 5500.0
    efficiency = 0.6

    boost_force = 5.0 * G  # 5 кг
    boost_time = 2.0
    cruise_force = 2.0 * G  # 2 кг
    catapult_length = 2.5
    catapult_force = 90.0  # Н, типичная резинка на такую длину
    launch_elevation = math.radians(60.0)

    # Рули: размах и хорда
    fin_span = 0.14
    fin_chord = 0.05
    fin_area = fin_span * fin_chord
    fin_radius = body_diameter / 2.0 + fin_span / 2.0
    fin_cn = 2.0
    rudder_deg = 20.0

    print("Исходные данные")
    print(f"  масса {mass:.1f} кг, вес {weight:.1f} Н")
    print(f"  старт: {boost_force:.0f} Н ({boost_force / weight:.1f} веса) в течение {boost_time:.0f} с")
    print(f"  крейсер: {cruise_force:.0f} Н ({cruise_force / weight:.2f} веса)")
    print(f"  катапульта {catapult_length:.1f} м, усилие около {catapult_force:.0f} Н")
    print(f"  винт {prop_diameter * 1000:.0f} мм, {prop_rpm:.0f} об/мин")
    print(f"  рули: размах {fin_span * 100:.0f} см, хорда {fin_chord * 100:.0f} см, "
          f"площадь {fin_area * 1e4:.0f} см², плечо {fin_radius * 100:.1f} см")
    print()

    # --- 1. разгон на катапульте ---
    print("Фаза 1: катапульта и форсаж")
    a_rail = (boost_force + catapult_force) / mass - G * math.sin(launch_elevation)
    v_exit = math.sqrt(max(0.0, 2.0 * a_rail * catapult_length))
    t_rail = v_exit / max(a_rail, 1e-6)
    print(f"  ускорение на направляющей:  {a_rail:.1f} м/с²")
    print(f"  сход с катапульты:          {v_exit:.1f} м/с за {t_rail:.2f} с")

    boost_left = max(0.0, boost_time - t_rail)
    # За оставшийся форсаж скорость растёт, но её съедают сопротивление
    # и составляющая веса; считаем шагами по 0,01 с
    speed = v_exit
    height = catapult_length * math.sin(launch_elevation)
    step = 0.01
    t = t_rail
    while t < boost_time:
        drag = 0.5 * RHO * speed * speed * drag_area
        accel = (boost_force - drag) / mass - G * math.sin(launch_elevation)
        speed += accel * step
        height += speed * math.sin(launch_elevation) * step
        t += step
    print(f"  к концу форсажа ({boost_time:.0f} с): {speed:.1f} м/с, высота {height:.0f} м")
    print()

    # --- 2. установившийся полёт при тяге, равной весу ---
    print("Фаза 2: тяга равна весу")
    print("  Угол к горизонту, при котором thrust = вес * sin(y) + сопротивление:")
    print(f"  {'скорость':>10}{'сопротивление':>15}{'угол к горизонту':>18}{'верт. скорость':>17}")
    for v in (60.0, 50.0, 40.0, 30.0, 20.0):
        drag = 0.5 * RHO * v * v * drag_area
        excess = cruise_force - drag
        if excess > weight:
            print(f"  {v:>8.0f} м/с{drag:>13.1f} Н{'нет решения':>18}{'':>17}")
            continue
        if excess <= 0:
            print(f"  {v:>8.0f} м/с{drag:>13.1f} Н{'падает':>18}{'—':>17}")
            continue
        angle = math.degrees(math.asin(excess / weight))
        climb = v * math.sin(math.radians(angle))
        print(f"  {v:>8.0f} м/с{drag:>13.1f} Н{angle:>15.0f}°{climb:>14.0f} м/с")
    print("  Читается так: с падением скорости сопротивление уменьшается,")
    print("  угол растёт, и полёт сам собой переходит в вертикальный подъём.")
    print("  Полёта строго по горизонту при тяге, равной весу, не бывает.")
    print()

    # --- 3 и 4. момент винта и рули ---
    print("Фаза 3: хватает ли рулей")
    print(f"  {'режим':<22}{'тяга':>7}{'напор струи':>13}{'момент винта':>14}"
          f"{'напор потока':>14}{'момент рулей':>14}{'запас':>8}")
    for name, thrust, speed in (
        ("старт у катапульты", boost_force, v_exit),
        ("конец форсажа", boost_force, speed),
        ("крейсер 50 м/с", cruise_force, 50.0),
        ("крейсер 30 м/с", cruise_force, 30.0),
        ("зависание", cruise_force, 0.0),
    ):
        torque = motor_torque(thrust, prop_area, prop_rpm, efficiency)
        q_slip = slipstream_q(thrust, prop_area)
        q_free = 0.5 * RHO * speed * speed
        q_total = q_slip + q_free
        moment = rudder_moment(q_total, fin_area, fin_radius, fin_cn, rudder_deg)
        print(
            f"  {name:<22}{thrust:>5.0f} Н{q_slip:>11.0f} Па{torque:>12.2f} Н·м"
            f"{q_free:>12.0f} Па{moment:>12.2f} Н·м{moment / torque:>7.2f}x"
        )
    print()
    print("  Напор потока и напор струи складываются: на скорости рули работают")
    print("  в основном от набегающего потока, а в зависании — только от струи.")
    print("  Отсюда вывод: чем быстрее летит аппарат, тем легче рулям.")

    need_low = 1.5 * motor_torque(boost_force, prop_area, prop_rpm, efficiency) / (
        2.0 * fin_radius * (slipstream_q(boost_force, prop_area) + 0.5 * RHO * v_exit ** 2)
        * fin_cn * math.radians(rudder_deg)
    )
    need_high = 1.5 * motor_torque(cruise_force, prop_area, prop_rpm, efficiency) / (
        2.0 * fin_radius * (slipstream_q(cruise_force, prop_area) + 0.5 * RHO * 50.0 ** 2)
        * fin_cn * math.radians(rudder_deg)
    )
    print()
    print(f"  Для запаса 1,5 нужно площади руля: {need_low * 1e4:.0f} см² на сходе "
          f"с катапульты и {need_high * 1e4:.0f} см² в крейсере")
    print(f"  Сейчас стоит {fin_area * 1e4:.0f} см².")
    return 0


if __name__ == "__main__":
    sys.exit(main())
