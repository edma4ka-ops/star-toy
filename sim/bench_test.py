"""Стендовая проверка контура: то, что не видно в числах полёта.

    python sim/bench_test.py

Проверяем не «летит ли вообще», а вещи, которые ломаются тихо:

  1. Знак коррекции. Если знак перепутан, аппарат не просто плохо держит
     курс — он уходит с него быстро и в обратную сторону. В полёте это
     выглядит как «управление не помогает», и причину так не найти.
  2. Отсечка по большой ошибке: рули уходят в нейтраль.
  3. Поведение без звёздной привязки: курса нет, управление не включается.
  4. Калибровка гироскопа: с уходом датчика оценка ориентации не уезжает.
  5. Мёртвая зона: мелкая ошибка не дёргает рули.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from control import (  # noqa: E402
    CourseHolder,
    Tune,
    Vehicle,
    q_axes,
    q_from_axes,
    q_integrate,
    v_cross,
    v_norm,
)
from rocket import Rocket, canard_force_body, canard_moment_body  # noqa: E402

RESULTS: list[tuple[bool, str, str]] = []
ROCKET = Rocket(vehicle=Vehicle())  # один аппарат на весь стенд


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((ok, name, detail))
    mark = "OK  " if ok else "ПЛОХО"
    print(f"  {mark} {name}" + (f" — {detail}" if detail and not ok else ""))


def launch_attitude(elevation_deg: float = 75.0):
    elevation = math.radians(elevation_deg)
    x_axis = (math.cos(elevation), 0.0, math.sin(elevation))
    z_axis = (-math.sin(elevation), 0.0, math.cos(elevation))
    return q_from_axes(x_axis, (0.0, 1.0, 0.0), z_axis)


def error_deg(holder: CourseHolder) -> float:
    x_axis = q_axes(holder.attitude)[0]
    return math.degrees(v_norm(v_cross(holder.course_dir, x_axis)))


def step_physics(holder: CourseHolder, cmd, q_dyn: float, dt: float = 0.005) -> tuple:
    """Один шаг стенда: рули дают момент, момент доворачивает оценку ориентации.

    Это не полёт, а проверка знака: берём только рули и инерцию по тангажу
    и рысканию, без аэродинамики и тяги.
    """
    vehicle = ROCKET.vehicle
    limit = vehicle.delta_max
    force = canard_force_body(ROCKET, q_dyn, cmd.ch_z * limit, cmd.ch_y * limit)
    torque = canard_moment_body(ROCKET, force)
    omega = (
        torque[0] / vehicle.inertia_x * dt,
        torque[1] / vehicle.inertia_y * dt,
        torque[2] / vehicle.inertia_z * dt,
    )
    holder.attitude = q_integrate(holder.attitude, omega, 1.0)
    # Гироскоп стенда показывает тот же поворот, что получил аппарат
    return omega


def bench_sign(axis_name: str) -> tuple[bool, float, float]:
    """Отклонить аппарат вокруг одной оси и проверить, что ошибка уменьшается."""
    holder = CourseHolder(vehicle=ROCKET.vehicle)
    holder.calibrate_gyro([(0.0, 0.0, 0.0)] * 5)
    holder.set_attitude(launch_attitude())
    holder.lock_course()
    holder.arm()

    # Вносим ошибку курса: аппарат «уехал» на 2° вокруг оси z или y
    skew = math.radians(2.0)
    axis = (0.0, 0.0, skew) if axis_name == "z" else (0.0, skew, 0.0)
    holder.attitude = q_integrate(holder.attitude, axis, 1.0)
    start = error_deg(holder)

    q_dyn = 3000.0
    rate = (0.0, 0.0, 0.0)
    for _ in range(400):
        cmd = holder.update(0.005, rate, q_dyn)
        if not cmd.active:
            break
        rate = step_physics(holder, cmd, q_dyn)
    finish = error_deg(holder)
    return finish < start * 0.5, start, finish


def main() -> int:
    print("Стендовая проверка контура управления")
    print("--- знак коррекции ---")
    for axis_name, title in (("z", "рыскание"), ("y", "тангаж")):
        ok, start, finish = bench_sign(axis_name)
        check(
            f"ошибка по {title} уменьшается",
            ok,
            f"было {start:.2f}°, стало {finish:.2f}° — знак коррекции перепутан",
        )

    print("--- отсечки и запреты ---")
    holder = CourseHolder(vehicle=ROCKET.vehicle)
    holder.calibrate_gyro([(0.0, 0.0, 0.0)] * 5)
    holder.set_attitude(launch_attitude())
    holder.lock_course()
    cmd = holder.update(0.005, (0, 0, 0), 3000.0)
    check("без взвода рули в нейтрали", not cmd.active and cmd.ch_z == 0.0, cmd.reason)

    holder.arm()
    holder.attitude = q_integrate(holder.attitude, (0.0, math.radians(70.0), 0.0), 1.0)
    cmd = holder.update(0.005, (0, 0, 0), 3000.0)
    check(
        "большая ошибка отключает управление",
        not cmd.active and "рули в нейтраль" in cmd.reason,
        f"ошибка {cmd.err_deg:.1f}°: {cmd.reason}",
    )

    holder2 = CourseHolder(vehicle=ROCKET.vehicle)
    holder2.calibrate_gyro([(0.0, 0.0, 0.0)] * 5)
    ok = holder2.set_attitude(launch_attitude(), stars_ok=False)
    holder2.lock_course()
    holder2.arm()
    cmd = holder2.update(0.005, (0, 0, 0), 3000.0)
    check("без звёзд управление не включается", (not ok) and not cmd.active, cmd.reason)

    holder3 = CourseHolder(vehicle=ROCKET.vehicle)
    holder3.arm()
    cmd = holder3.update(0.005, (0, 0, 0), 3000.0)
    check("без калибровки гироскопа нет взвода", not cmd.active, cmd.reason)

    print("--- калибровка гироскопа ---")
    bias = (0.0, 0.0, math.radians(0.5))
    holder4 = CourseHolder(vehicle=ROCKET.vehicle)
    holder4.calibrate_gyro([bias] * 5)
    holder4.set_attitude(launch_attitude())
    holder4.lock_course()
    holder4.arm()
    # Аппарат стоит неподвижно, датчик врёт на 0,5 °/с
    for _ in range(200):
        holder4.update(0.005, bias, 3000.0)
    drift = error_deg(holder4)
    check(
        "с калибровкой курс не уезжает",
        drift < 0.05,
        f"уехал на {drift:.3f}°",
    )

    holder5 = CourseHolder(vehicle=ROCKET.vehicle)
    # Калибровка сделана по нулям, а датчик врёт: так бывает, когда
    # калибруют на холодном гироскопе, а потом он прогревается
    holder5.calibrate_gyro([(0.0, 0.0, 0.0)] * 5)
    holder5.set_attitude(launch_attitude())
    holder5.lock_course()
    holder5.arm()
    for _ in range(200):
        holder5.update(0.005, bias, 3000.0)
    drift5 = error_deg(holder5)
    check(
        "без калибровки курс уезжает (так и должно быть)",
        drift5 > 0.3,
        f"уехал всего на {drift5:.3f}° — похоже, уход не учитывается",
    )

    print("--- мёртвая зона ---")
    holder6 = CourseHolder(Tune(deadband_deg=1.0))
    holder6.calibrate_gyro([(0.0, 0.0, 0.0)] * 5)
    holder6.set_attitude(launch_attitude())
    holder6.lock_course()
    holder6.arm()
    holder6.attitude = q_integrate(holder6.attitude, (0.0, 0.0, math.radians(0.4)), 1.0)
    cmd = holder6.update(0.005, (0, 0, 0), 3000.0)
    check("мелкая ошибка не дёргает рули", cmd.active and abs(cmd.ch_z) < 1e-9, f"ch_z={cmd.ch_z}")

    failed = [r for r in RESULTS if not r[0]]
    print()
    print(f"проверок: {len(RESULTS)}, успешно: {len(RESULTS) - len(failed)}, провалено: {len(failed)}")
    for _, name, detail in failed:
        print(f"  провал: {name} — {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
