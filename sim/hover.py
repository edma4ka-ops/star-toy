"""Модель аппарата с винтом: висит на тяге, держит курс вокруг вертикали.

Считается независимо от программы управления. Задача модели — ответить
на вопрос, который нельзя решить на словах: хватает ли двух рулей, чтобы
удержать аппарат от закручивания винтом.

Чем эта модель отличается от ракетной (sim/rocket.py)
-----------------------------------------------------

1. Тяга не короткая, а непрерывная и примерно равна весу. Аппарат не летит
   по параболе, он висит или медленно поднимается, и ось его смотрит вверх.
2. Главное возмущение — реактивный момент винта. Он постоянный, и именно
   его рули должны перебороть.
3. Рули работают не в набегающем потоке, а в струе от винта. Это важно:
   струя есть и когда аппарат висит на месте, поэтому рули не теряют силу
   на нулевой скорости. Динамический напор в струе примерно равен тяге,
   делённой на площадь винта.
4. Момент рулей по рысканию возникает потому, что рули смещены от оси
   на расстояние R. Плечо мало (это радиус оперения, 5-7 см), поэтому
   момент невелик: 2 * R * F.

Система координат инерциальная: x и y — горизонталь, z — вверх.
Связанные оси: x — вдоль аппарата к носу (при висении вверх), y и z —
поперечные.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from control import (
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


@dataclass
class Hover:
    """Параметры аппарата. Числа — правдоподобная модель игрушки на 0,9 кг."""

    # Геометрия рулей и инерция вокруг оси берутся из того же места,
    # что и программой управления: двух источников правды здесь быть не должно
    craft: HoverCraft = field(default_factory=HoverCraft)

    mass: float = 2.0
    diameter: float = 0.12
    length: float = 0.70
    prop_diameter: float = 0.25
    # Тяга как доля веса. На старте форсаж, потом крейсер
    thrust_ratio: float = 1.0  # крейсер: тяга равна весу
    boost_ratio: float = 2.5  # старт: 5 кг при весе 2 кг
    boost_time: float = 2.0
    # Винт: обороты и КПД нужны, чтобы посчитать реактивный момент
    prop_rpm: float = 5500.0
    prop_efficiency: float = 0.6
    # Оперение как стабилизатор: возвращает аппарат по потоку
    fin_cn_alpha: float = 4.0
    aero_arm: float = -0.15  # аэродинамический фокус позади центра масс
    cd: float = 0.45
    # Инерция по поперечным осям (по оси — в HoverCraft)
    inertia_y: float = 0.060
    inertia_z: float = 0.060
    # Запуск катапультой: 2,5 м разгона дают около 17 м/с
    launch_speed: float = 17.5
    launch_elevation_deg: float = 60.0
    launch_rate_deg: float = 5.0  # возмущение при сходе

    @property
    def fin_area(self) -> float:
        return self.craft.fin_area

    @property
    def fin_radius(self) -> float:
        return self.craft.fin_radius

    @property
    def fin_cn_delta(self) -> float:
        return self.craft.fin_cn_delta

    @property
    def rudder_max_deg(self) -> float:
        return self.craft.rudder_max_deg

    @property
    def inertia_x(self) -> float:
        return self.craft.inertia_x

    @property
    def area(self) -> float:
        return math.pi * self.diameter ** 2 / 4.0

    @property
    def prop_area(self) -> float:
        return math.pi * self.prop_diameter ** 2 / 4.0

    @property
    def weight(self) -> float:
        return self.mass * G

    def thrust(self) -> float:
        """Тяга на крейсерском режиме, Н."""
        return self.thrust_ratio * self.weight

    def thrust_now(self, t: float) -> float:
        """Тяга в текущий момент: сначала форсаж, потом крейсер."""
        ratio = self.boost_ratio if t < self.boost_time else self.thrust_ratio
        return ratio * self.weight

    def motor_torque_now(self, t: float) -> float:
        """Реактивный момент винта в текущий момент, Н·м.

        Момент считается по импульсной теории при текущей тяге: он растёт
        быстрее тяги, поэтому форсаж опасен именно моментом, а не тягой.
        """
        thrust = self.thrust_now(t)
        induced = math.sqrt(max(thrust, 0.0) / (2.0 * RHO * self.prop_area))
        power = thrust * induced / max(self.prop_efficiency, 1e-6)
        omega = self.prop_rpm * 2.0 * math.pi / 60.0
        return power / max(omega, 1e-6)

    def slipstream_q(self, t: float = 0.0) -> float:
        """Динамический напор в струе за винтом, Па.

        По импульсной теории тяга примерно равна напору в струе, умноженному
        на площадь винта: T = q * A. Отсюда q = T / A. Это значит, что
        маленький винт даёт плотную струю и сильные рули — но требует
        большой мощности, а значит и большого реактивного момента.
        """
        return self.thrust_now(t) / self.prop_area

    def rudder_q(self, t: float, speed: float) -> float:
        """Напор, в котором работают рули: струя винта плюс набегающий поток.

        На старте поток слабый и рули живут только струёй; в крейсере
        наоборот — потока в разы больше. Поэтому на скорости рулям легко,
        а на малой тяжело.
        """
        return self.slipstream_q(t) + 0.5 * RHO * speed * speed

    def motor_torque(self) -> float:
        """Реактивный момент винта на корпус, Н·м.

        Мощность в струе: T * v_струи, где v_струи = sqrt(T / (2 * rho * A)).
        Момент — это мощность, делённая на угловую скорость вала.
        """
        induced = math.sqrt(self.thrust() / (2.0 * RHO * self.prop_area))
        power = self.thrust() * induced / max(self.prop_efficiency, 1e-6)
        omega = self.prop_rpm * 2.0 * math.pi / 60.0
        return power / max(omega, 1e-6)

    def rudder_moment(self, deflection_rad: float) -> float:
        """Момент рулей по рысканию, Н·м.

        Два руля смещены от оси на R, поэтому их силы дают момент 2 * R * F.
        """
        return self.craft.moment_per_rad(self.slipstream_q()) * deflection_rad

    def rudder_side_force(self, deflection_rad: float) -> float:
        """Побочная боковая сила рулей, Н.

        Рули V-образно стоят под 45°, их силы складываются не только
        в момент, но и в боковую силу. Чтобы удержать курс, аппарат
        приходится толкать вбок — это цена такой схемы.
        """
        q = self.slipstream_q()
        force = q * self.fin_area * self.fin_cn_delta * deflection_rad
        return 2.0 * force * math.cos(math.radians(45.0))


@dataclass
class Conditions:
    wind: float = 2.0
    gust: float = 2.0
    gust_start: float = 5.0
    gust_end: float = 9.0
    gyro_bias: float = 0.0  # уход гироскопа по рысканию, рад/с
    gyro_noise: float = 0.0
    dt: float = 0.001
    control_hz: float = 200.0
    flight_time: float = 40.0
    # Виден ли источник света (солнце или маяк). Если нет — курс держится
    # только гироскопом, и это видно по уходу
    light_available: bool = True
    seed: int = 11


@dataclass
class Result:
    name: str
    samples: list = field(default_factory=list)
    spin_end: float = 0.0  # установившееся вращение, °/с
    spin_max: float = 0.0
    heading_drift: float = 0.0  # уход курса от начального, градусы
    max_heading_error: float = 0.0
    max_rudder_deg: float = 0.0
    altitude: float = 0.0
    side_drift: float = 0.0
    control_on: bool = False
    note: str = ""


def launch_attitude(elevation_deg: float) -> tuple:
    elevation = math.radians(elevation_deg)
    x_axis = (math.cos(elevation), 0.0, math.sin(elevation))
    z_axis = (-math.sin(elevation), 0.0, math.cos(elevation))
    return q_from_axes(x_axis, (0.0, 1.0, 0.0), z_axis)


def heading_of(quaternion) -> float:
    """Курс: куда смотрит «бок» аппарата в горизонтальной плоскости, радианы."""
    _, y_axis, _ = q_axes(quaternion)
    return math.atan2(y_axis[1], y_axis[0])


def simulate(
    name: str,
    craft: Hover,
    cond: Conditions,
    control_on: bool,
    holder=None,
) -> Result:
    dt = cond.dt
    control_period = 1.0 / cond.control_hz
    since_control = 0.0
    rng = random.Random(cond.seed)

    attitude = launch_attitude(craft.launch_elevation_deg)
    x_axis = q_axes(attitude)[0]
    position = (0.0, 0.0, 0.0)
    velocity = v_scale(x_axis, craft.launch_speed)
    omega = (0.0, 0.0, math.radians(craft.launch_rate_deg))

    result = Result(name=name, control_on=control_on)
    heading0 = heading_of(attitude)

    if holder is not None:
        holder.set_reference(heading0, q_dyn=craft.rudder_q(0.0, craft.launch_speed))

    t = 0.0
    while t < cond.flight_time:
        x_b, y_b, z_b = q_axes(attitude)

        wind = (0.0, cond.wind + (cond.gust if cond.gust_start <= t <= cond.gust_end else 0.0), 0.0)
        v_rel = v_sub(velocity, wind)
        speed = v_norm(v_rel)
        v_dir = v_unit(v_rel)
        q_dyn = 0.5 * RHO * speed * speed
        thrust = craft.thrust_now(t)
        motor_torque = craft.motor_torque_now(t)
        rudder_q = craft.rudder_q(t, speed)

        # --- силы ---
        force = v_add((0.0, 0.0, -craft.weight), v_scale(x_b, thrust))
        if speed > 1e-6:
            drag = -0.5 * RHO * speed * speed * craft.cd * craft.area
            force = v_add(force, v_scale(v_dir, drag))

        # Оперение возвращает аппарат по потоку (устойчивость дротика)
        v_perp = v_sub(v_rel, v_scale(x_b, v_dot(v_rel, x_b)))
        normal_body = (0.0, 0.0, 0.0)
        if speed > 1e-6:
            cn = craft.fin_cn_alpha * (v_norm(v_perp) / speed)
            normal_dir = v_unit(v_perp)
            normal_inertial = v_scale(normal_dir, -q_dyn * craft.area * cn)
            force = v_add(force, normal_inertial)
            normal_body = q_rotate_inv(attitude, normal_inertial)

        # --- управление ---
        rudder = 0.0
        cmd_err = 0.0
        if control_on and holder is not None:
            since_control += dt
            if since_control >= control_period:
                since_control = 0.0
                heading = heading_of(attitude)
                gyro_yaw = v_dot(omega, x_b)  # вращение вокруг оси аппарата
                gyro = (
                    omega[0] + rng.gauss(0.0, cond.gyro_noise),
                    omega[1] + rng.gauss(0.0, cond.gyro_noise),
                    omega[2] + cond.gyro_bias + rng.gauss(0.0, cond.gyro_noise),
                )
                command = holder.update(
                    control_period,
                    gyro,
                    gyro_yaw + cond.gyro_bias,
                    heading if cond.light_available else None,
                    rudder_q,
                    now=t,
                )
                cmd_err = command.err_deg
                if command.active:
                    rudder = command.ch_z * math.radians(craft.rudder_max_deg)
                    result.max_rudder_deg = max(
                        result.max_rudder_deg, math.degrees(abs(rudder))
                    )
                    result.max_heading_error = max(result.max_heading_error, command.err_deg)
                # Программа знает тягу, значит знает и момент винта: сообщаем
                # его контуру, чтобы рули отклонялись заранее
                holder.set_motor_torque(motor_torque)

        # --- момент рулей и реактивный момент винта ---
        # Рули работают в суммарном напоре: струя винта плюс набегающий поток
        rudder_authority = craft.craft.moment_per_rad(rudder_q)
        rudder_moment = rudder_authority * rudder
        rudder_force = (
            rudder_q * craft.fin_area * craft.fin_cn_delta * rudder
        )
        rudder_force_body = (0.0, rudder_force * math.cos(math.radians(45.0)) * 2.0, 0.0)
        torque = (
            # Реактивный момент винта крутит корпус вокруг своей оси
            -motor_torque,
            # Момент рулей по рысканию действует вокруг оси аппарата
            rudder_moment,
            0.0,
        )
        # Побочная боковая сила рулей приложена сзади и наклоняет аппарат
        torque = v_add(
            torque,
            v_cross((-craft.length * 0.4, 0.0, craft.fin_radius), rudder_force_body),
        )
        # Оперение как демпфер поворота вокруг оси: на скорости оно тормозит
        # вращение, потому что рули движутся поперёк потока
        rotational_damp = (
            2.0
            * 0.5
            * RHO
            * math.sqrt(max(2.0 * rudder_q / RHO, 1.0))
            * craft.fin_radius ** 2
            * craft.fin_area
            * craft.fin_cn_alpha
        )
        torque = v_add(
            torque,
            (
                -rotational_damp * omega[0],
                -0.5 * RHO * speed * craft.area * craft.length ** 2 * 3.0 * omega[1],
                -0.5 * RHO * speed * craft.area * craft.length ** 2 * 3.0 * omega[2],
            ),
        )
        torque = v_add(torque, v_cross((craft.aero_arm, 0.0, 0.0), normal_body))

        # --- интегрирование ---
        velocity = v_add(velocity, v_scale(v_scale(force, 1.0 / craft.mass), dt))
        position = v_add(position, v_scale(velocity, dt))
        omega = v_add(
            omega,
            (
                torque[0] / craft.inertia_x * dt,
                torque[1] / craft.inertia_y * dt,
                torque[2] / craft.inertia_z * dt,
            ),
        )
        attitude = q_integrate(attitude, omega, dt)

        spin = math.degrees(v_dot(omega, x_b))
        result.spin_max = max(result.spin_max, abs(spin))
        result.samples.append((t, position, velocity, heading_of(attitude), spin, cmd_err, rudder))
        result.altitude = position[2]
        result.side_drift = math.hypot(position[0], position[1])
        result.spin_end = spin
        result.heading_drift = math.degrees(
            (heading_of(attitude) - heading0 + math.pi) % (2.0 * math.pi) - math.pi
        )
        t += dt

    return result
