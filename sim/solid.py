"""Модель аппарата с двигателем, который работает 20 секунд.

    python sim/solid.py

Профиль двигателя, который заказан: 5 кг первые две секунды, потом 2 кг
ещё восемнадцать. Изделие 2 кг, запуск катапультой 2,5 м.

Почему эта модель отдельная от sim/hover.py
-------------------------------------------

1. **Нет реактивного момента.** Твердотопливный двигатель не крутит корпус.
   Поэтому главная проблема прошлой модели — момент винта 2,9 Н·м на старте,
   который рули не могли перебороть — здесь просто отсутствует.

2. **Другая геометрия рулей.** Дротик летит вдоль оси, руль стоит на хвосте,
   и его плечо — расстояние от центра масс до хвоста (десятки сантиметров),
   а не смещение от оси. Момент получается в разы больше при той же площади.

3. **Нет струи винта.** Рули работают только набегающим потоком, поэтому
   на малой скорости они бессильны, а на большой сильны. Направляющая
   и катапульта здесь важны именно поэтому.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from control import (  # noqa: E402
    HeadingHolder,
    HeadingTune,
    HoverCraft,
    q_axes,
    q_from_axes,
    q_integrate,
    q_rotate,
    q_rotate_inv,
    v_add,
    v_cross,
    v_dot,
    v_norm,
    v_scale,
    v_sub,
    v_unit,
)

G = 9.81
RHO = 1.2


class Motor:
    """Профиль тяги: форсаж, затем длинный крейсерский участок, затем ничего."""

    def __init__(
        self,
        boost_force: float = 5.0 * G,
        boost_time: float = 2.0,
        sustain_force: float = 2.0 * G,
        sustain_time: float = 18.0,
        propellant: float = 0.25,
    ) -> None:
        self.boost_force = boost_force
        self.boost_time = boost_time
        self.sustain_force = sustain_force
        self.sustain_time = sustain_time
        self.propellant = propellant  # масса топлива, кг

    @property
    def burn_time(self) -> float:
        return self.boost_time + self.sustain_time

    def thrust(self, t: float) -> float:
        if t < self.boost_time:
            return self.boost_force
        if t < self.burn_time:
            return self.sustain_force
        return 0.0

    def impulse(self) -> float:
        return self.boost_force * self.boost_time + self.sustain_force * self.sustain_time

    def mass_left(self, t: float) -> float:
        """Сколько топлива осталось: оно выгорает за время работы."""
        if t >= self.burn_time:
            return 0.0
        return self.propellant * (1.0 - t / self.burn_time)


class Dart:
    """Изделие: корпус с оперением и рулём на хвосте."""

    def __init__(self) -> None:
        self.mass_dry = 1.75
        self.motor = Motor()
        self.diameter = 0.12
        self.length = 1.00
        self.cd = 0.45
        # Оперение: чем больше запас устойчивости, тем труднее рулям
        self.fin_cn_alpha = 6.0  # производная силы оперения по углу атаки
        self.aero_arm = -0.12  # аэродинамический фокус позади центра масс
        self.cm_q = 6.0  # демпфирование вращения
        self.inertia_y = 0.060
        self.inertia_z = 0.060
        # Руль на хвосте
        self.rudder_area = 0.0070  # 70 см²
        self.rudder_cn = 2.0
        self.rudder_arm = 0.35  # от центра масс до руля
        self.rudder_max_deg = 20.0
        self.craft = HoverCraft(
            fin_area=self.rudder_area,
            fin_cn_delta=self.rudder_cn,
            rudder_max_deg=self.rudder_max_deg,
            inertia_x=0.0050,
        )
        # Запуск: резинки 2,5 м. Сила резинки падает по мере разгона,
        # поэтому считаем её профилем, а не одной цифрой
        self.bungee_length = 2.5
        self.bungee_peak = 160.0  # Н в начале разгона
        self.bungee_end = 40.0  # Н в конце
        self.launch_elevation_deg = 35.0
        self.launch_rate_deg = 5.0
        self.rail_length = 2.5

    def bungee_force(self, along: float) -> float:
        """Сила резинок в точке разгона. Линейно падает от начала к концу."""
        if along >= self.bungee_length:
            return 0.0
        share = max(0.0, along) / self.bungee_length
        return self.bungee_peak + (self.bungee_end - self.bungee_peak) * share

    @property
    def area(self) -> float:
        return math.pi * self.diameter ** 2 / 4.0

    @property
    def mass_launch(self) -> float:
        return self.mass_dry + self.motor.propellant

    def mass(self, t: float) -> float:
        return self.mass_dry + self.motor.mass_left(t)

    def aero_spring(self, q: float) -> float:
        """Момент оперения на радиан угла атаки: с ним борются рули."""
        return q * self.area * self.fin_cn_alpha * abs(self.aero_arm)

    def rudder_moment_per_rad(self, q: float) -> float:
        """Момент руля на хвосте на радиан отклонения."""
        return q * self.rudder_area * self.rudder_cn * self.rudder_arm


def launch_attitude(elevation_deg: float) -> tuple:
    elevation = math.radians(elevation_deg)
    x_axis = (math.cos(elevation), 0.0, math.sin(elevation))
    z_axis = (-math.sin(elevation), 0.0, math.cos(elevation))
    return q_from_axes(x_axis, (0.0, 1.0, 0.0), z_axis)


def heading_of(q) -> float:
    _, y_axis, _ = q_axes(q)
    return math.atan2(y_axis[1], y_axis[0])


def fly(dart: Dart, control_on: bool, wind: float = 3.0, gust: float = 0.0,
        flight_time: float = 60.0, heading_light: bool = True,
        elevation_deg: float | None = None):
    """Прогон полёта. Возвращает журнал и итоги."""
    dt = 0.001
    control_period = 1.0 / 200.0
    since_control = 0.0
    if elevation_deg is not None:
        dart.launch_elevation_deg = elevation_deg

    attitude = launch_attitude(dart.launch_elevation_deg)
    rail_dir = q_axes(attitude)[0]
    position = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    omega = (0.0, 0.0, 0.0)
    on_rail = True
    mass = dart.mass_launch
    exit_speed = 0.0

    holder = None
    if control_on:
        holder = HeadingHolder(HeadingTune(omega_n=6.0), dart.craft, tail_arm=dart.rudder_arm)
        holder.set_reference(heading_of(attitude))
        holder.arm()

    heading0 = heading_of(attitude)
    journal = []
    max_err = 0.0
    max_rudder = 0.0
    apogee = 0.0
    burnout_speed = 0.0
    active_steps = total_steps = 0
    max_aero = 0.0
    max_cmd_moment = 0.0

    t = 0.0
    while t < flight_time and position[2] >= -0.01:
        x_b, y_b, z_b = q_axes(attitude)
        speed_rel = v_norm(v_sub(velocity, (0.0, wind, 0.0)))
        v_rel = v_sub(velocity, (0.0, wind, 0.0))
        q_dyn = 0.5 * RHO * speed_rel ** 2
        thrust = dart.motor.thrust(t)
        mass = dart.mass(t)

        force = v_add((0.0, 0.0, -G * mass), v_scale(x_b, thrust))
        if speed_rel > 1e-6:
            force = v_add(
                force,
                v_scale(v_unit(v_rel), -0.5 * RHO * speed_rel ** 2 * dart.cd * dart.area),
            )

        v_perp = v_sub(v_rel, v_scale(x_b, v_dot(v_rel, x_b)))
        normal_body = (0.0, 0.0, 0.0)
        if speed_rel > 1e-6:
            cn = dart.fin_cn_alpha * (v_norm(v_perp) / speed_rel)
            normal_inertial = v_scale(v_unit(v_perp), -q_dyn * dart.area * cn)
            force = v_add(force, normal_inertial)
            normal_body = q_rotate_inv(attitude, normal_inertial)

        # --- управление ---
        rudder = 0.0
        cmd_err = 0.0
        if control_on and holder is not None:
            since_control += dt
            if since_control >= control_period:
                since_control = 0.0
                gyro = omega
                command = holder.update(
                    control_period,
                    gyro,
                    v_dot(omega, x_b),
                    heading_of(attitude) if heading_light else None,
                    q_dyn,
                    now=t,
                )
                total_steps += 1
                cmd_err = command.err_deg
                max_cmd_moment = max(max_cmd_moment, command.authority)
                if command.active:
                    active_steps += 1
                    rudder = command.ch_z * math.radians(dart.rudder_max_deg)
                    max_err = max(max_err, command.err_deg)
                    max_rudder = max(max_rudder, math.degrees(abs(rudder)))

        # --- моменты ---
        rudder_force = q_dyn * dart.rudder_area * dart.rudder_cn * rudder
        torque = (0.0, 0.0, 0.0)
        torque = v_add(torque, v_cross((-dart.rudder_arm, 0.0, 0.0), (0.0, rudder_force, 0.0)))
        torque = v_add(torque, v_cross((dart.aero_arm, 0.0, 0.0), normal_body))
        damp = 0.5 * RHO * speed_rel * dart.area * dart.length ** 2 * dart.cm_q
        torque = v_add(torque, (0.0, -damp * omega[1], -damp * omega[2]))
        max_aero = max(max_aero, dart.aero_spring(q_dyn))

        velocity = v_add(velocity, v_scale(v_scale(force, 1.0 / mass), dt))
        position = v_add(position, v_scale(velocity, dt))
        omega = v_add(
            omega,
            (
                0.0,
                torque[1] / dart.inertia_y * dt,
                torque[2] / dart.inertia_z * dt,
            ),
        )
        attitude = q_integrate(attitude, omega, dt)

        if on_rail:
            along = max(0.0, v_dot(position, rail_dir))
            position = v_scale(rail_dir, along)
            velocity = v_scale(rail_dir, max(0.0, v_dot(velocity, rail_dir)))
            attitude = launch_attitude(dart.launch_elevation_deg)
            omega = (0.0, 0.0, 0.0)
            # Пока аппарат на направляющей, его толкают резинки
            bungee = dart.bungee_force(along)
            if bungee > 0.0:
                velocity = v_add(
                    velocity,
                    v_scale(rail_dir, bungee / mass * dt),
                )
            if along >= dart.rail_length:
                on_rail = False
                exit_speed = v_norm(velocity)
                omega = (0.0, math.radians(dart.launch_rate_deg), 0.0)

        apogee = max(apogee, position[2])
        if t >= dart.motor.burn_time and burnout_speed == 0.0:
            burnout_speed = v_norm(velocity)
        if abs(t * 10 - round(t * 10)) < 1e-9:
            horizontal = math.hypot(velocity[0], velocity[1])
            path_angle = math.degrees(math.atan2(velocity[2], max(horizontal, 1e-9)))
            journal.append(
                (t, position[2], v_norm(velocity), path_angle, rudder, cmd_err)
            )
        t += dt

    return {
        "journal": journal,
        "apogee": apogee,
        "impact": math.hypot(position[0], position[1]),
        "flight_time": t,
        "heading_drift": math.degrees(
            (heading_of(attitude) - heading0 + math.pi) % (2.0 * math.pi) - math.pi
        ),
        "max_err": max_err,
        "max_rudder": max_rudder,
        "burnout_speed": burnout_speed,
        "exit_speed": exit_speed,
        "active_share": 100.0 * active_steps / max(total_steps, 1),
        "max_aero": max_aero,
        "control_on": control_on,
    }


def main() -> int:
    dart = Dart()
    motor = dart.motor

    print("Двигатель, который заказан: 5 кг две секунды, затем 2 кг восемнадцать")
    print(f"  суммарный импульс:          {motor.impulse():.0f} Н·с")
    print(f"  полное время работы:        {motor.burn_time:.0f} с")
    print(f"  топлива в нём:              {motor.propellant * 1000:.0f} г")
    print(f"  взлётная масса:             {dart.mass_launch:.2f} кг, "
          f"без топлива {dart.mass_dry:.2f} кг")
    print(f"  начальная тяговооружённость:{motor.boost_force / (dart.mass_launch * G):.2f}")
    print(f"  крейсерская:                {motor.sustain_force / (dart.mass_launch * G):.2f} веса")
    print()

    print("Сколько скорости даёт такой двигатель")
    print("  Формула Циолковского без гравитации и сопротивления:")
    dv_ideal = motor.impulse() / dart.mass_launch
    print(f"    предел по импульсу:       {dv_ideal:.0f} м/с")
    print("  В реальности меньше: часть импульса уходит на борьбу с весом")
    print("  и сопротивлением воздуха. Считаем моделью ниже.")
    print()

    print("Пуск резинками 2,5 м")
    print(f"  сила резинок:               от {dart.bungee_peak:.0f} Н до "
          f"{dart.bungee_end:.0f} Н по мере разгона")
    print(f"  энергия резинок:            "
          f"{(dart.bungee_peak + dart.bungee_end) / 2.0 * dart.bungee_length:.0f} Дж")
    print()

    print("Как угол пуска влияет на полёт")
    print(f"  {'угол':>7}{'сход':>9}{'апогей':>10}{'падение':>11}{'время':>9}"
          f"{'угол держится':>16}")
    for elevation in (30.0, 35.0, 40.0, 45.0):
        probe = Dart()
        run = fly(probe, control_on=True, wind=3.0, elevation_deg=elevation)
        # Сколько времени траектория остаётся выше 20 градусов
        above = 0.0
        for t, _height, _speed, angle, _rudder, _err in run["journal"]:
            if angle >= 20.0:
                above = t
        print(
            f"  {elevation:>6.0f}°{run['exit_speed']:>7.0f} м/с{run['apogee']:>8.0f} м"
            f"{run['impact']:>9.0f} м{run['flight_time']:>7.1f} с{above:>14.1f} с"
        )
    print("  Последний столбец — сколько секунд траектория остаётся круче 20°.")
    print()

    passive = fly(Dart(), control_on=False, wind=3.0)
    active = fly(Dart(), control_on=True, wind=3.0)

    print("Профиль полёта с управлением")
    print(f"  {'t, с':>6}{'высота, м':>11}{'скорость, м/с':>15}{'угол траектории':>17}"
          f"{'руль':>9}{'ошибка курса':>14}")
    for t, height, speed, angle, rudder, err in active["journal"]:
        if t > 24.0:
            break
        print(f"  {t:>6.1f}{height:>11.0f}{speed:>15.1f}{angle:>16.0f}°"
              f"{math.degrees(rudder):>8.1f}°{err:>13.1f}°")
    print()

    print("Итоги")
    print(f"  {'':<26}{'без управления':>16}{'с управлением':>16}")
    rows = (
        ("высота в верхней точке", "apogee", "м", 0),
        ("скорость после выгорания", "burnout_speed", "м/с", 0),
        ("уход курса за полёт", "heading_drift", "°", 1),
        ("ошибка курса в пике", "max_err", "°", 1),
        ("руль в пике", "max_rudder", "°", 1),
        ("дальность падения", "impact", "м", 0),
    )
    for title, key, unit, digits in rows:
        a = passive[key]
        b = active[key]
        print(f"  {title:<26}{a:>14.{digits}f} {unit}{b:>14.{digits}f} {unit}")
    print(f"  {'управление работало':<26}{'—':>16}{active['active_share']:>14.0f} %")
    print()

    print("Баланс: может ли руль перебороть оперение")
    # Берём не конец полёта, а середину разгона: там напор уже рабочий
    q = 0.5 * RHO * 35.0 ** 2
    spring = dart.aero_spring(q)
    rudder = dart.rudder_moment_per_rad(q)
    print(f"  скоростной напор при 35 м/с: {q:.0f} Па")
    print(f"  оперение на радиан:          {spring:.3f} Н·м/рад")
    print(f"  руль на радиан:              {rudder:.3f} Н·м/рад")
    print(f"  руль против оперения:        {rudder / max(spring, 1e-9):.2f}x")
    if rudder < 1.5 * spring:
        need = 1.5 * spring / (q * dart.rudder_cn * dart.rudder_arm)
        margin = abs(dart.aero_arm) / dart.diameter
        print(f"  ВЫВОД: рулю не хватает. Нужно {need * 1e4:.0f} см² вместо "
              f"{dart.rudder_area * 1e4:.0f} см²,")
        print(f"         либо уменьшить запас устойчивости: сейчас он "
              f"{margin:.1f} калибра, а для управляемого аппарата")
        print("         достаточно 0,5-1 калибра. Это дешевле, чем растить руль:")
        print("         сдвинуть центр масс назад или подрезать оперение.")
    else:
        print("  ВЫВОД: руль сильнее оперения — курс держать можно")
    print()

    print("Почему траектория сама загибается вниз")
    print("  При тяге, равной весу, скорость поворачивает гравитация:")
    print("    d(угол)/dt = -вес * cos(угол) / (масса * скорость)")
    print(f"  {'скорость':>10}{'угол к горизонту':>18}{'поворот вниз':>15}")
    for v in (70.0, 50.0, 35.0, 25.0):
        for angle in (60.0, 30.0, 10.0):
            turn = math.degrees(
                -dart.mass_launch * G * math.cos(math.radians(angle))
                / (dart.mass_launch * v)
            )
            print(f"  {v:>8.0f} м/с{angle:>16.0f}°{turn:>13.1f}°/с")
        print()
    print("  Тяга, направленная вдоль скорости, этот поворот не останавливает:")
    print("  она разгоняет, но не поднимает. Остановить его может только сила,")
    print("  направленная поперёк скорости — то есть подъёмная сила крыла")
    print("  или наклон вектора тяги. Ни того, ни другого у дротика нет.")
    print()
    print("Про высоту: чем выше забирается аппарат, тем важнее ограничитель.")
    print("В России беспилотники выше 150 м требуют разрешения.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
