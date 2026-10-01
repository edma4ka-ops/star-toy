"""Управление рулями: аппарат держит курс, зафиксированный по звезде.

Как это работает по сути
------------------------

1. До пуска аппарат стоит неподвижно. Камера-звёздный датчик решает задачу
   по звёздам и отдаёт ориентацию: куда смотрит ось аппарата и где у него
   «верх». Этот момент фиксируется как КУРС.
2. Дальше курс держит гироскоп. Он измеряет повороты, программа сравнивает
   текущее направление оси с зафиксированным и отклоняет рули так, чтобы
   вернуть аппарат на курс.
3. По звёздам в полёте не наводимся: на скорости звезда смазывается. Датчик
   нужен только до старта — и этого достаточно, потому что аппарат держит
   уже заданный курс, а не ищет звезду заново.

Почему рулей два, а осей три
----------------------------

Две рулевые машинки отклоняют две пары рулей: одна пара сидит на оси ±z
и даёт силу по y (рыскание), вторая на оси ±y и даёт силу по z (тангаж).
Крен при этом свободный: аппарат может вращаться вокруг своей оси, и это
не мешает — программа знает угол крена от гироскопа и разворачивает команду
в связанные оси. Именно поэтому без гироскопа такая схема не работает.

Почему коэффициенты считаются, а не подбираются на глаз
-------------------------------------------------------

Сила рулей растёт как скоростной напор: на малой скорости рули почти ничего
не могут, на большой — переворачивают аппарат за десятые доли секунды.
Поэтому регулятор не «крутит ручку на минус-плюс», а каждый раз считает,
какое отклонение рулей даст нужное угловое ускорение:

    момент на радиан отклонения  M = q * S * CN * L
    угловое ускорение на радиан  a = M / I
    нужно ускорение              a_нужн = w0^2 * ошибка - 2*z*w0 * скорость
    отклонение                   d = a_нужн / a

Здесь w0 — желаемая частота контура, z — демпфирование. Такой регулятор
одинаково работает и на старте, и на разгоне, и на плановом участке.

Файл не зависит от железа: на вход идут угловые скорости и скоростной напор,
на выходе — два управляющих сигнала. Поэтому один и тот же код работает
и в тренажёре на компьютере, и на борту.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Векторы и кватернионы. Без внешних библиотек, чтобы код шёл и на борту.
# Кватернион записан как (w, x, y, z) и переводит вектор из связанных осей
# в инерциальные: v_ин = q * v_св * q*.
# ---------------------------------------------------------------------------


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a):
    n = v_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_unit(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_rotate_by(q, v):
    """Повернуть вектор из связанных осей в инерциальные."""
    # v' = v + 2 * w * (q_v x v) + 2 * q_v x (q_v x v)
    w, x, y, z = q
    qv = (x, y, z)
    t = v_scale(v_cross(qv, v), 2.0)
    return v_add(v, v_add(v_scale(t, w), v_cross(qv, t)))


def q_rotate(q, v):
    return q_rotate_by(q, v)


def q_rotate_inv(q, v):
    """Повернуть вектор из инерциальных осей в связанные."""
    return q_rotate_by(q_conj(q), v)


def q_from_omega(omega, dt):
    """Приращение кватерниона от угловой скорости за шаг dt."""
    magnitude = v_norm(omega)
    angle = magnitude * dt
    if angle < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    axis = v_scale(omega, 1.0 / magnitude)
    half = angle * 0.5
    s = math.sin(half)
    return (math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s)


def q_integrate(q, omega, dt):
    """Довернуть кватернион на угловую скорость в связанных осях."""
    return q_unit(q_mul(q, q_from_omega(omega, dt)))


def q_axes(q):
    """Оси тела, выраженные в инерциальной системе: (x_ось, y_ось, z_ось)."""
    return (
        q_rotate_by(q, (1.0, 0.0, 0.0)),
        q_rotate_by(q, (0.0, 1.0, 0.0)),
        q_rotate_by(q, (0.0, 0.0, 1.0)),
    )


def q_from_axes(x_axis, y_axis, z_axis):
    """Кватернион из трёх ортонормированных осей.

    Оси тела в инерциальных координатах — это СТОЛБЦЫ матрицы поворота.
    Перепутать их со строками легко: получается транспонированная матрица,
    то есть обратный поворот, и аппарат стартует носом вниз.
    """
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return q_unit(((s * 0.25), (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s))
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return q_unit(((m21 - m12) / s, s * 0.25, (m01 + m10) / s, (m02 + m20) / s))
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return q_unit(((m02 - m20) / s, (m01 + m10) / s, s * 0.25, (m12 + m21) / s))
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return q_unit(((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s * 0.25))


def deadband(value: float, band: float) -> float:
    if abs(value) <= band:
        return 0.0
    return value - math.copysign(band, value)


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


# ---------------------------------------------------------------------------
# Аппарат и настройки контура
# ---------------------------------------------------------------------------


@dataclass
class Vehicle:
    """Геометрия и инерция — то, без чего нельзя посчитать отклонение рулей.

    Эти же числа использует модель полёта: один источник правды на программу
    и на тренажёр, иначе они разойдутся и тренажёр начнёт врать.
    """

    canard_area: float = 0.005  # суммарная площадь пары рулей, м² (50 см²)
    cn_delta: float = 2.0  # производная коэффициента силы рулей, на радиан
    canard_arm: float = 0.40  # расстояние от центра масс до рулей, м
    inertia_x: float = 0.0004  # момент инерции относительно оси, кг·м²
    inertia_y: float = 0.022  # момент инерции, кг·м²
    inertia_z: float = 0.022
    delta_max_deg: float = 12.0  # предельное отклонение рулей

    @property
    def delta_max(self) -> float:
        return math.radians(self.delta_max_deg)

    def authority(self, q_dyn: float, q_min: float = 0.0) -> float:
        """Момент на радиан отклонения рулей при данном скоростном напоре."""
        return max(q_dyn, q_min) * self.canard_area * self.cn_delta * self.canard_arm


@dataclass
class Tune:
    """Настройки контура. Частота и демпфирование — в физических единицах."""

    omega_n: float = 10.0  # желаемая частота контура, рад/с (около 1,6 Гц)
    zeta: float = 0.7  # демпфирование: 0,7 — без перерегулирования
    deadband_deg: float = 0.15  # мёртвая зона по ошибке курса
    # Предел ошибки, после которого управление отключается. Это защита от
    # отказа датчика (оценка ориентации разошлась), а не рабочий режим:
    # в ветре аппарат законно уходит на десятки градусов, и бросать курс
    # в этот момент нельзя — именно тогда рули и нужны
    err_off_deg: float = 60.0
    q_min: float = 50.0  # напор, ниже которого рули бесполезны, Па
    accel_trust: float = 0.05  # доверие к акселерометру на столе
    # Интегратор. Оперение тянет аппарат по потоку, и пропорционального
    # регулятора не хватает: он уравновешивает это тянущее усилие с остаточной
    # ошибкой — тем большей, чем сильнее ветер. Замер на модели: без интегратора
    # аппарат летит с задранным на 9,5° носом и сносит на 45 м, с интегратором
    # ошибка снимается до нуля
    k_integral: float = 2.0  # 1/с
    integral_max_deg: float = 10.0  # предел накопленного угла, градусы
    # Держать ли курс и после выгорания топлива. По умолчанию нет: на плановом
    # участке аппарат обязан задирать нос вниз вслед за траекторией, и держать
    # ориентацию нельзя — это не ошибка курса, а нормальный полёт
    hold_in_coast: bool = False


@dataclass
class Command:
    """Что подать на рулевые машинки и что записать в журнал."""

    ch_z: float = 0.0  # пара рулей на оси +-z, -1..1
    ch_y: float = 0.0  # пара рулей на оси +-y, -1..1
    active: bool = False
    reason: str = "не взведено"
    err_deg: float = 0.0  # ошибка курса
    roll_deg: float = 0.0  # крен вокруг оси аппарата
    q_dyn: float = 0.0  # скоростной напор, Па
    authority: float = 0.0  # момент на радиан отклонения, Н·м
    saturated: bool = False  # рули упёрлись в предел

    def neutral(self, reason: str) -> "Command":
        self.ch_z = 0.0
        self.ch_y = 0.0
        self.active = False
        self.saturated = False
        self.reason = reason
        return self


@dataclass
class HoverCraft:
    """Геометрия рулей аппарата, который висит на тяге винта.

    Здесь ловушка, которую видно только на геометрии. У аппарата, висящего
    вертикально, «курс» — это поворот вокруг своей же оси. Два руля, двигаясь
    вместе, дают такой поворот только потому, что они СМЕЩЕНЫ от оси на
    расстояние R: сила руля F даёт момент R * F каждый. Если рули поставить
    на саму ось, они не смогут повернуть аппарат вообще.

    Отсюда: момент рулей = 2 * R * q * S * CN * d, и плечо R — рычаг,
    которым можно набирать момент, не увеличивая рули.
    """

    fin_area: float = 0.0070  # площадь одного руля, м² (70 см²)
    fin_radius: float = 0.13  # смещение руля от оси, м
    fin_cn_delta: float = 2.0  # производная силы руля по углу
    rudder_max_deg: float = 20.0
    inertia_x: float = 0.0050  # момент инерции вокруг оси, кг·м²

    @property
    def rudder_max(self) -> float:
        return math.radians(self.rudder_max_deg)

    def moment_per_rad(self, q: float, tail_arm: float = 0.0) -> float:
        """Момент рулей на радиан отклонения при данном напоре.

        Две разные геометрии, и путать их нельзя:

        * `tail_arm > 0` — руль на хвосте дротика, летящего вдоль оси.
          Плечо — расстояние от центра масс до руля, то есть десятки
          сантиметров. Момент = q * S * CN * L.
        * `tail_arm = 0` — рули V-образно на аппарате, висящем на тяге.
          Плечо — смещение руля от оси, то есть сантиметры.
          Момент = 2 * R * q * S * CN. Здесь работают только на смещении.
        """
        if tail_arm > 0.0:
            return q * self.fin_area * self.fin_cn_delta * tail_arm
        return 2.0 * self.fin_radius * q * self.fin_area * self.fin_cn_delta


@dataclass
class HeadingTune:
    """Настройки контура курса для висящего аппарата."""

    omega_n: float = 4.0  # частота контура, рад/с: аппарат тяжёлый и медленный
    zeta: float = 0.8
    k_integral: float = 1.5  # 1/с: снимает постоянный момент винта
    integral_max_deg: float = 25.0
    err_off_deg: float = 90.0  # защита от расхождения оценки ориентации
    # Сколько секунд можно держать курс одним гироскопом, пока нет света.
    # Дальше честно сообщаем, что курс держится по памяти и уплывает
    dead_reckoning_s: float = 10.0


class HeadingHolder:
    """Держит курс вокруг вертикали одной парой рулей.

    Порядок работы:

        holder = HeadingHolder()
        holder.set_reference(heading_from_light_sensor, q_slip=...)
        while flight:
            cmd = holder.update(dt, gyro, yaw_rate, measured_heading, q_slip)

    Курс измеряет датчик света (солнце или маяк). Между его показаниями
    курс держит гироскоп — и это ограничение по времени, а не удобство:
    чем дольше нет света, тем больше уплывает курс.
    """

    def __init__(
        self,
        tune: HeadingTune | None = None,
        craft: HoverCraft | None = None,
        tail_arm: float = 0.0,
    ) -> None:
        self.tune = tune or HeadingTune()
        self.craft = craft or HoverCraft()
        # 0 — рули V-образно по бокам висящего аппарата; больше нуля —
        # руль на хвосте дротика, и тогда плечо это расстояние до хвоста
        self.tail_arm = tail_arm
        self.reference = 0.0
        self.estimate = 0.0
        self.integral = 0.0
        self.locked = False
        self.armed = False
        self.last_fix = -1e9
        self.motor_torque = 0.0  # ожидаемый реактивный момент винта, Н·м
        self._saturated = False
        self._cmd = Command()

    # -- подготовка ---------------------------------------------------------

    def set_reference(self, heading_rad: float, q_dyn: float = 0.0, now: float = 0.0) -> None:
        """Задать курс по датчику света: солнце или маяк."""
        self.reference = heading_rad
        self.estimate = heading_rad
        self.integral = 0.0
        self.last_fix = now
        self.locked = True

    def set_target(self, heading_rad: float) -> None:
        """Обновить заданный курс, не трогая оценку.

        Отличие от set_reference важно. set_reference ставит и задание,
        и оценку — это разовая привязка при включении. А когда задание
        приходит снаружи двадцать раз в секунду, оценку трогать нельзя:
        иначе ошибка каждый раз обнуляется и рули ничего не делают.
        """
        self.reference = heading_rad

    def arm(self) -> None:
        self.armed = True

    def set_motor_torque(self, torque_nm: float) -> None:
        """Сообщить ожидаемый реактивный момент винта.

        Программа сама управляет тягой, значит знает и момент: он растёт
        вместе с тягой. Зная его заранее, рули можно отклонить на нужный
        угол сразу, не дожидаясь, пока аппарат начнёт закручиваться. Это
        особенно важно на старте, когда форсаж даёт момент в разы больше,
        чем рули могут создать на малой скорости.
        """
        self.motor_torque = torque_nm

    # -- основной шаг -------------------------------------------------------

    def update(
        self,
        dt: float,
        gyro,
        yaw_rate: float,
        measured_heading: float | None = None,
        q_slip: float = 0.0,
        now: float = 0.0,
    ) -> Command:
        cmd = self._cmd

        if not self.armed:
            return cmd.neutral("не взведено")
        if not self.locked:
            return cmd.neutral("нет курса: датчик света не дал направление")

        # Оценка курса: гироскоп ведёт, датчик света поправляет.
        # Поправку вносим не рывком, а долей за шаг — иначе рывок
        # от неточного измерения дёрнет рули
        self.estimate = (self.estimate + yaw_rate * dt) % (2.0 * math.pi)
        if measured_heading is not None:
            fix = (measured_heading - self.estimate + math.pi) % (2.0 * math.pi) - math.pi
            self.estimate = (self.estimate + 0.3 * fix) % (2.0 * math.pi)
            self.last_fix = now

        error = (self.reference - self.estimate + math.pi) % (2.0 * math.pi) - math.pi
        cmd.err_deg = math.degrees(error)
        cmd.roll_deg = 0.0
        cmd.q_dyn = q_slip

        if abs(cmd.err_deg) > self.tune.err_off_deg:
            return cmd.neutral(f"ошибка курса {cmd.err_deg:.0f}° — рули в нейтраль")

        # Постоянный реактивный момент винта пропорциональный регулятор
        # не снимет: он оставит ошибку. Поэтому интеграл здесь не улучшение,
        # а необходимость
        if not self._saturated:
            self.integral += error * dt
            limit = math.radians(self.tune.integral_max_deg)
            self.integral = clamp(self.integral, -limit, limit)

        wanted = (
            self.tune.omega_n ** 2 * error
            - 2.0 * self.tune.zeta * self.tune.omega_n * yaw_rate
            + self.tune.k_integral * self.integral
        ) * self.craft.inertia_x

        authority = self.craft.moment_per_rad(q_slip, self.tail_arm)
        cmd.authority = authority
        if authority <= 1e-9:
            return cmd.neutral("потока нет — рули бесполезны")

        # Прямая компенсация момента винта: рули отклоняются на нужный угол
        # сразу, а не после того, как аппарат раскрутится
        wanted_moment = wanted + self.motor_torque
        deflection = wanted_moment / authority
        limit_rad = self.craft.rudder_max
        cmd.saturated = abs(deflection) > limit_rad
        self._saturated = cmd.saturated
        cmd.ch_z = clamp(deflection, -limit_rad, limit_rad) / limit_rad
        cmd.ch_y = 0.0
        cmd.active = True
        # Честно про качество удержания: со светом курс абсолютный,
        # без света он держится по памяти и медленно уплывает
        if now - self.last_fix > self.tune.dead_reckoning_s:
            cmd.reason = "света нет: курс держится по памяти"
        else:
            cmd.reason = "курс держится"
        return cmd


class CourseHolder:
    """Держит курс, зафиксированный по звезде, отклоняя две пары рулей.

    Порядок работы:

        holder = CourseHolder()
        holder.calibrate_gyro(samples)          # на столе, аппарат неподвижен
        holder.set_attitude(q, stars_ok=True)   # ориентация от звёздного датчика
        holder.lock_course()                    # этот момент = курс
        holder.arm()
        while flight:
            cmd = holder.update(dt, gyro, q_dyn)
    """

    def __init__(self, tune: Tune | None = None, vehicle: Vehicle | None = None) -> None:
        self.tune = tune or Tune()
        self.vehicle = vehicle or Vehicle()
        self.attitude = (1.0, 0.0, 0.0, 0.0)
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.armed = False
        self.locked = False
        self.star_fix_valid = False
        self.course_dir = (1.0, 0.0, 0.0)
        self.course_y = (0.0, 1.0, 0.0)
        self.course_z = (0.0, 0.0, 1.0)
        self.calibrated = False
        self.integral = (0.0, 0.0, 0.0)
        self._saturated = False
        self._cmd = Command()

    # -- подготовка ---------------------------------------------------------

    def calibrate_gyro(self, samples) -> None:
        """Средний уход гироскопа по выборке неподвижных измерений.

        Без этого шага уход датчика за несколько секунд полёта превращается
        в ошибку курса: 0,5 °/с за 3 с — это 1,5°, а на 300 м это 8 м.
        """
        samples = [tuple(s) for s in samples]
        if not samples:
            return
        n = float(len(samples))
        self.gyro_bias = (
            sum(s[0] for s in samples) / n,
            sum(s[1] for s in samples) / n,
            sum(s[2] for s in samples) / n,
        )
        self.calibrated = True

    def set_attitude(self, q, stars_ok: bool = True) -> bool:
        """Принять ориентацию от звёздного датчика.

        Курс задаёт только звёздный датчик: акселерометр даёт наклон
        к вертикали, но не даёт азимут, а нужен именно он. Без датчика
        курса нет — и управление не включится.
        """
        if not stars_ok:
            self.star_fix_valid = False
            return False
        self.attitude = q_unit(q)
        self.star_fix_valid = True
        x_axis, y_axis, z_axis = q_axes(self.attitude)
        self.course_dir, self.course_y, self.course_z = x_axis, y_axis, z_axis
        self.locked = True
        return True

    def lock_course(self) -> bool:
        """Зафиксировать текущую ориентацию как курс.

        Требует, чтобы ориентация пришла от звёздного датчика: зафиксировать
        курс по показаниям одного гироскопа нельзя, у него нет азимута.
        """
        if not self.star_fix_valid or not self.calibrated:
            self.locked = False
            return False
        x_axis, y_axis, z_axis = q_axes(self.attitude)
        self.course_dir, self.course_y, self.course_z = x_axis, y_axis, z_axis
        self.integral = (0.0, 0.0, 0.0)
        self.locked = True
        return True

    def arm(self) -> None:
        self.armed = True

    def disarm(self, _reason: str = "") -> None:
        self.armed = False

    # -- оценка ориентации --------------------------------------------------

    def update_attitude(self, dt: float, gyro) -> None:
        """Довернуть оценку ориентации по гироскопу.

        В полёте ориентация считается только гироскопом. Акселерометру здесь
        верить нельзя: он измеряет не гравитацию, а тягу двигателя, сопротивление
        воздуха и силы рулей — то есть перегрузку, а не «верх».
        """
        rate = v_sub(gyro, self.gyro_bias)
        self.attitude = q_integrate(self.attitude, rate, dt)

    def align_on_pad(self, dt: float, accel) -> bool:
        """Уточнить наклон по акселерометру, пока аппарат стоит на столе.

        На столе акселерометр показывает направление «вверх» — это единственное
        время, когда ему можно верить. Азимут он не даёт вообще.
        """
        magnitude = v_norm(accel)
        if magnitude < 1e-6:
            return False
        measured_up_body = v_scale(accel, 1.0 / magnitude)
        measured_up_inertial = q_rotate(self.attitude, measured_up_body)
        error = v_cross(measured_up_inertial, self.course_z)
        if v_norm(error) < 1e-9:
            return True
        correction = v_scale(error, self.tune.accel_trust * dt * 60.0)
        self.attitude = q_unit(q_mul(q_from_omega(correction, 1.0), self.attitude))
        return True

    # -- основной шаг -------------------------------------------------------

    def update(self, dt: float, gyro, q_dyn: float = 0.0, burning: bool = True) -> Command:
        cmd = self._cmd
        cmd.q_dyn = q_dyn

        if not self.armed:
            return cmd.neutral("не взведено")
        if not self.locked:
            return cmd.neutral("нет курса: звёздный датчик не дал ориентацию")
        if not self.calibrated:
            return cmd.neutral("гироскоп не откалиброван")

        self.update_attitude(dt, gyro)

        x_axis, y_axis, z_axis = q_axes(self.attitude)

        # Ошибка курса. Считать её как длину векторного произведения нельзя:
        # получится синус угла, и на 70° это даст 54° — защита не сработает
        # там, где должна. Берём настоящий угол
        misalign = v_cross(self.course_dir, x_axis)
        misalign_len = v_norm(misalign)
        cos_angle = clamp(v_dot(self.course_dir, x_axis), -1.0, 1.0)
        err_rad = math.atan2(misalign_len, cos_angle)
        cmd.err_deg = math.degrees(err_rad)
        # Крен вокруг оси аппарата относительно курса
        roll_err = math.atan2(v_dot(y_axis, self.course_z), v_dot(y_axis, self.course_y))
        cmd.roll_deg = math.degrees(roll_err)

        if not burning and not self.tune.hold_in_coast:
            # Разгон кончился — направление тяги уже задано, дальше аппарат
            # устойчив сам и летит по инерции. Рули в нейтраль: попытка
            # «держать ориентацию» на плановом участке только задирает нос
            return cmd.neutral("тяга кончилась — рули в нейтраль")

        if cmd.err_deg > self.tune.err_off_deg:
            return cmd.neutral(f"ошибка курса {cmd.err_deg:.1f}° — рули в нейтраль")

        # Точный вектор поворота, совмещающий ось аппарата с курсом:
        # ось вращения на угол. Для малых углов совпадает с векторным
        # произведением, для больших — нет
        if misalign_len > 1e-9:
            rot_inertial = v_scale(misalign, -err_rad / misalign_len)
        else:
            rot_inertial = (0.0, 0.0, 0.0)

        vehicle = self.vehicle
        authority = vehicle.authority(q_dyn, self.tune.q_min)
        cmd.authority = authority

        # Интегратор накапливает ошибку курса в инерциальных осях. Пока рули
        # упёрты в предел, копить нельзя: иначе после снятия насыщения аппарат
        # бросит в другую сторону (это называют накоплением интегратора)
        if not self._saturated:
            self.integral = v_add(self.integral, v_scale(rot_inertial, dt))
            limit_int = math.radians(self.tune.integral_max_deg)
            if v_norm(self.integral) > limit_int:
                self.integral = v_scale(v_unit(self.integral), limit_int)

        # Требуемый поворот с учётом накопленного: интеграл и снимает
        # остаточный увод оперением
        rotation = v_add(rot_inertial, v_scale(self.integral, self.tune.k_integral))
        rot_body = q_rotate_inv(self.attitude, rotation)
        band = math.radians(self.tune.deadband_deg)
        err_y = deadband(rot_body[1], band)
        err_z = deadband(rot_body[2], band)
        rate = v_sub(gyro, self.gyro_bias)

        # Нужное угловое ускорение: колебательное звено с заданной частотой
        # и демпфированием. Дальше — пересчёт в отклонение рулей
        wn = self.tune.omega_n
        damping = 2.0 * self.tune.zeta * wn
        acc_y = wn * wn * err_y - damping * rate[1]
        acc_z = wn * wn * err_z - damping * rate[2]

        # Знак по тангажу минус: пара рулей на оси +-y даёт момент по -y
        delta_z = acc_z * vehicle.inertia_z / authority
        delta_y = -acc_y * vehicle.inertia_y / authority

        limit = vehicle.delta_max
        cmd.saturated = abs(delta_z) > limit or abs(delta_y) > limit
        self._saturated = cmd.saturated
        cmd.ch_z = clamp(delta_z, -limit, limit) / limit
        cmd.ch_y = clamp(delta_y, -limit, limit) / limit
        cmd.active = True
        cmd.reason = "курс держится"
        return cmd
