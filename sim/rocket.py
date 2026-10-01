"""Модель полёта: проверка контура управления без железа.

Модель считается независимо от программы управления: здесь физика,
а не то, что программа про себя думает. Если знаки рулей или коэффициенты
подобраны неверно, аппарат в модели уйдёт с курса — и это будет видно.

Что учтено:
  * тяга двигателя с перекосом (двигатель посажен неровно — реальная причина
    ухода ракеты с курса);
  * сила лобового сопротивления;
  * восстанавливающий момент оперения (аппарат устойчив сам по себе);
  * силы двух пар рулей и моменты от них;
  * демпфирование по тангажу, рысканию и крену;
  * ветер с порывом (боковой);
  * уход гироскопа и шум датчика.

Система координат инерциальная: x — направление пуска, y — вправо, z — вверх.
Начало — точка старта. Связанные оси: x — вдоль аппарата к носу, y и z —
поперечные.

Геометрия рулей и моменты инерции берутся из control.Vehicle — из того же
места, что и программа управления. Двух источников правды здесь быть не должно:
разойдутся — и тренажёр начнёт подтверждать неверный контур.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from control import (
    Vehicle,
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


@dataclass
class Rocket:
    """Параметры аппарата. Числа — правдоподобная модель класса «детская ракета»."""

    vehicle: Vehicle = field(default_factory=Vehicle)

    mass_dry: float = 0.75
    mass_prop: float = 0.35
    burn_time: float = 1.6
    thrust: float = 45.0
    diameter: float = 0.057
    length: float = 0.70
    aero_arm: float = -0.10  # аэродинамический фокус позади центра масс
    cn_alpha: float = 10.0  # производная коэффициента силы корпуса с оперением
    cd: float = 0.45
    cm_q: float = 8.0  # демпфирование по тангажу и рысканию
    croll_p: float = 0.05  # демпфирование по крену
    # Перекос тяги и начальное возмущение схода с направляющей
    thrust_misalign_deg: float = 0.8
    rail_rate_deg: float = 12.0
    rail_length: float = 4.0  # длина направляющей, м

    @property
    def area(self) -> float:
        return math.pi * self.diameter ** 2 / 4.0

    def mass(self, t: float) -> float:
        if t >= self.burn_time:
            return self.mass_dry
        return self.mass_dry + self.mass_prop * (1.0 - t / self.burn_time)

    def thrust_now(self, t: float) -> float:
        return self.thrust if t < self.burn_time else 0.0


@dataclass
class Conditions:
    """Внешние условия и неисправности, которые мы хотим пережить."""

    wind: float = 4.0  # боковой ветер, м/с
    gust: float = 3.0  # порыв, м/с
    gust_start: float = 0.6
    gust_end: float = 1.8
    gyro_bias: tuple = (0.0, 0.0, 0.0)  # уход гироскопа, рад/с
    gyro_noise: float = 0.0  # шум гироскопа, рад/с
    elevation_deg: float = 75.0  # угол пуска к горизонту
    dt: float = 0.001
    control_hz: float = 200.0
    seed: int = 7


@dataclass
class Result:
    name: str
    samples: list = field(default_factory=list)
    apogee: float = 0.0
    impact_y: float = 0.0
    impact_x: float = 0.0
    max_deviation: float = 0.0
    max_err_deg: float = 0.0
    max_delta_deg: float = 0.0
    active_steps: int = 0
    total_steps: int = 0
    control_on: bool = False
    note: str = ""


def wind_at(t: float, cond: Conditions) -> tuple:
    speed = cond.wind
    if cond.gust_start <= t <= cond.gust_end:
        speed += cond.gust
    return (0.0, speed, 0.0)


def canard_force_body(rocket: Rocket, q_dyn: float, delta_z: float, delta_y: float) -> tuple:
    """Сила двух пар рулей в связанных осях — это геометрия аппарата.

    Пара рулей на оси +-z даёт силу по y, пара на оси +-y даёт силу по z.
    Знак отклонения задан механикой качалки и проверяется на стенде
    (sim/bench_test.py): если перепутан, аппарат будет уходить с курса
    вдвое быстрее, чем без управления.
    """
    vehicle = rocket.vehicle
    scale = q_dyn * vehicle.canard_area * vehicle.cn_delta
    return (0.0, scale * delta_z, scale * delta_y)


def canard_moment_body(rocket: Rocket, force_body: tuple) -> tuple:
    """Момент рулей относительно центра масс: они стоят впереди него."""
    return v_cross((rocket.vehicle.canard_arm, 0.0, 0.0), force_body)


def launch_attitude(elevation_deg: float) -> tuple:
    """Кватернион стартового положения: нос под углом к горизонту, верх в зенит."""
    elevation = math.radians(elevation_deg)
    x_axis = (math.cos(elevation), 0.0, math.sin(elevation))
    z_axis = (-math.sin(elevation), 0.0, math.cos(elevation))
    return q_from_axes(x_axis, (0.0, 1.0, 0.0), z_axis)


def simulate(
    name: str,
    rocket: Rocket,
    cond: Conditions,
    control_on: bool,
    holder=None,
    stars_ok: bool = True,
) -> Result:
    """Один прогон полёта. holder — CourseHolder или None."""
    dt = cond.dt
    control_period = 1.0 / cond.control_hz
    since_control = 0.0
    rng = random.Random(cond.seed)

    attitude = launch_attitude(cond.elevation_deg)
    rail_dir = q_axes(attitude)[0]
    position = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    omega = (0.0, 0.0, 0.0)
    on_rail = True
    tilt = math.radians(rocket.thrust_misalign_deg)
    launch_attitude_fixed = attitude

    result = Result(name=name, control_on=control_on)
    t = 0.0
    while True:
        x_b, y_b, z_b = q_axes(attitude)

        # --- ветер и скорость относительно воздуха ---
        wind = wind_at(t, cond)
        v_rel = v_sub(velocity, wind)
        speed = v_norm(v_rel)
        v_dir = v_unit(v_rel)
        q_dyn = 0.5 * RHO * speed * speed

        # --- силы ---
        mass = rocket.mass(t)
        force = (0.0, 0.0, -G * mass)

        # Тяга: направление перекошено относительно оси аппарата
        thrust = rocket.thrust_now(t)
        if thrust > 0.0:
            thrust_dir = v_unit(v_add(x_b, v_scale(y_b, math.tan(tilt))))
            force = v_add(force, v_scale(thrust_dir, thrust))

        # Лобовое сопротивление
        if speed > 1e-6:
            drag = -0.5 * RHO * speed * speed * rocket.cd * rocket.area
            force = v_add(force, v_scale(v_dir, drag))

        # Корпус с оперением: сила перпендикулярна оси и направлена против
        # поперечной составляющей скорости — это и есть устойчивость
        v_perp = v_sub(v_rel, v_scale(x_b, v_dot(v_rel, x_b)))
        normal_body = (0.0, 0.0, 0.0)
        if speed > 1e-6:
            cn = rocket.cn_alpha * (v_norm(v_perp) / speed)
            normal_dir = v_unit(v_perp)
            normal_inertial = v_scale(normal_dir, -q_dyn * rocket.area * cn)
            force = v_add(force, normal_inertial)
            normal_body = q_rotate_inv(attitude, normal_inertial)

        # --- управление ---
        delta_z = delta_y = 0.0
        cmd_err = 0.0
        cmd_active = False
        ch_z = ch_y = 0.0
        cmd_reason = ""
        if control_on and holder is not None:
            since_control += dt
            if since_control >= control_period:
                since_control = 0.0
                gyro_meas = (
                    omega[0] + cond.gyro_bias[0] + rng.gauss(0.0, cond.gyro_noise),
                    omega[1] + cond.gyro_bias[1] + rng.gauss(0.0, cond.gyro_noise),
                    omega[2] + cond.gyro_bias[2] + rng.gauss(0.0, cond.gyro_noise),
                )
                burning = t < rocket.burn_time
                command = holder.update(control_period, gyro_meas, q_dyn, burning)
                result.total_steps += 1
                cmd_err = command.err_deg
                cmd_active = command.active
                cmd_reason = command.reason
                ch_z, ch_y = command.ch_z, command.ch_y
                if command.active:
                    limit = holder.vehicle.delta_max
                    delta_z = command.ch_z * limit
                    delta_y = command.ch_y * limit
                    result.max_err_deg = max(result.max_err_deg, command.err_deg)
                    result.max_delta_deg = max(
                        result.max_delta_deg,
                        math.degrees(max(abs(delta_z), abs(delta_y))),
                    )
                    result.active_steps += 1

        canard_body = canard_force_body(rocket, q_dyn, delta_z, delta_y)
        force = v_add(force, q_rotate(attitude, canard_body))

        # --- моменты в связанных осях ---
        torque = canard_moment_body(rocket, canard_body)
        # от корпуса с оперением
        torque = v_add(torque, v_cross((rocket.aero_arm, 0.0, 0.0), normal_body))
        # демпфирование
        vehicle = rocket.vehicle
        damp = 0.5 * RHO * speed * rocket.area * rocket.length ** 2
        torque = v_add(
            torque,
            (
                -0.5 * RHO * speed * rocket.area * rocket.diameter ** 2 * rocket.croll_p * omega[0],
                -damp * rocket.cm_q * omega[1],
                -damp * rocket.cm_q * omega[2],
            ),
        )

        # --- интегрирование ---
        velocity = v_add(velocity, v_scale(v_scale(force, 1.0 / mass), dt))
        position = v_add(position, v_scale(velocity, dt))

        omega = v_add(
            omega,
            (
                torque[0] / vehicle.inertia_x * dt,
                torque[1] / vehicle.inertia_y * dt,
                torque[2] / vehicle.inertia_z * dt,
            ),
        )
        attitude = q_integrate(attitude, omega, dt)

        if on_rail:
            # Пока аппарат на направляющей, он не может ни повернуться,
            # ни уйти вбок: направляющая держит его. Это важно для модели:
            # на малой скорости рули ещё ничего не могут, и без направляющей
            # аппарат успевает уйти с курса ветром задолго до того,
            # как управление станет работоспособным
            along = max(0.0, v_dot(position, rail_dir))
            position = v_scale(rail_dir, along)
            velocity = v_scale(rail_dir, max(0.0, v_dot(velocity, rail_dir)))
            attitude = launch_attitude_fixed
            omega = (0.0, 0.0, 0.0)
            if along >= rocket.rail_length:
                on_rail = False
                # Сход с направляющей с небольшим возмущением
                omega = (0.0, math.radians(rocket.rail_rate_deg), 0.0)

        result.samples.append(
            (
                t,
                position,
                velocity,
                q_axes(attitude)[0],
                q_dyn,
                cmd_err,
                ch_z,
                ch_y,
                cmd_active,
                cmd_reason,
            )
        )
        result.apogee = max(result.apogee, position[2])
        result.max_deviation = max(result.max_deviation, abs(position[1]))

        t += dt
        if position[2] < 0.0 and t > 0.05:
            result.impact_x = position[0]
            result.impact_y = position[1]
            return result
        if t > 60.0:
            result.note = "полёт не закончился за 60 с"
            return result


def prepare_holder(rocket: Rocket, cond: Conditions, holder, stars_ok: bool = True):
    """То, что делает борт до пуска: калибровка гироскопа, привязка по звёздам."""
    holder.calibrate_gyro([cond.gyro_bias] * 5)
    holder.set_attitude(launch_attitude(cond.elevation_deg), stars_ok=stars_ok)
    holder.lock_course()
    holder.arm()
    return holder
