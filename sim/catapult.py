# -*- coding: utf-8 -*-
"""Расчёт резиновой катапульты: сколько даст разгона и какая перегрузка.

Модель: несколько прядей латексной трубки тянут каретку с ракетой по
направляющей длиной L. Сила пряди растёт с растяжением:

    F(x) = F0 + k·x        (F0 — предварительное натяжение, Н)

Работа резины = площадь под кривой силы, минус потери на трение каретки.

    python sim/catapult.py
"""
import math

G = 9.81


def strand_force(x, f0, k):
    return max(0.0, f0 + k * x)


def launch(rocket_mass=3.0, sled_mass=0.6, strands=4, stroke=3.0,
           f0_per_strand=40.0, k_per_strand=20.0, friction=0.08):
    """Возвращает словарь с параметрами разгона."""
    m = rocket_mass + sled_mass
    forces = [strands * strand_force(x / 100 * stroke, f0_per_strand, k_per_strand)
              for x in range(0, 101)]
    avg = sum(forces) / len(forces)
    peak = max(forces)

    work = avg * stroke
    energy = work * (1.0 - friction)
    v = math.sqrt(2 * energy / m)
    a_max = peak / m
    t = 2 * stroke / max(v, 1e-6)          # при равноускоренном движении

    return {
        "m": m,
        "work_j": work,
        "energy_j": energy,
        "v_exit": v,
        "a_max_g": a_max / G,
        "peak_n": peak,
        "avg_n": avg,
        "t_rail": t,
        "speed_after_0_5s_motor": v,       # двигатель считаем отдельно
    }


def table():
    print("Резиновая катапульта: варианты прядей (ракета 3 кг, каретка 0.6 кг, ход 3 м)")
    print(f"{'прядей':>7} {'пик, Н':>8} {'средн., Н':>10} {'энергия, Дж':>12} "
          f"{'v выхода, м/с':>14} {'перегрузка':>11}")
    for n in (2, 3, 4, 6, 8):
        r = launch(strands=n)
        print(f"{n:>7} {r['peak_n']:>8.0f} {r['avg_n']:>10.0f} {r['energy_j']:>12.0f} "
              f"{r['v_exit']:>14.1f} {r['a_max_g']:>10.1f} g")

    print("\nВлияние длины хода (4 пряди):")
    print(f"{'ход, м':>8} {'v выхода, м/с':>14} {'перегрузка':>11} {'время на направляющей, с':>26}")
    for L in (1.5, 2.0, 2.5, 3.0, 3.5):
        r = launch(stroke=L)
        print(f"{L:>8.1f} {r['v_exit']:>14.1f} {r['a_max_g']:>10.1f} g {r['t_rail']:>26.2f}")

    print("\nВлияние массы ракеты (4 пряди, ход 3 м):")
    print(f"{'масса, кг':>10} {'v выхода, м/с':>14} {'перегрузка':>11}")
    for mass in (1.0, 2.0, 3.0, 5.0):
        r = launch(rocket_mass=mass)
        print(f"{mass:>10.1f} {r['v_exit']:>14.1f} {r['a_max_g']:>10.1f} g")


if __name__ == "__main__":
    table()
    r = launch()
    print("\nВыбранный вариант (4 пряди, ход 3 м):")
    print(f"  пик силы:      {r['peak_n']:.0f} Н")
    print(f"  работа:        {r['work_j']:.0f} Дж, до каретки доходит {r['energy_j']:.0f} Дж")
    print(f"  скорость:      {r['v_exit']:.1f} м/с")
    print(f"  перегрузка:    {r['a_max_g']:.1f} g")
    print(f"  время разгона: {r['t_rail']*1000:.0f} мс")
