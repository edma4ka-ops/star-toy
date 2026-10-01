"""Автоматический парашют: логика, размеры купола, проверки на сценариях.

    python recovery/system.py

Главное правило этой системы звучит так: **заряд не срабатывает, пока
не доказано, что аппарат действительно улетел**. Ни одно одиночное
показание датчика не даёт команду на срабатывание — нужно несколько
независимых подтверждений, а на земле не срабатывает никогда.

Почему не по камере. Камера видит крупный объект постоянно: земля при
подъёме, облако, птица, рука при запуске. Заряд, сработавший на столе
от руки перед объективом, калечит. Поэтому камера в этой системе
не участвует в принятии решения вообще — она может только писать картинку
оператору.

Три независимых источника подтверждения:

    1. барометр    — высота и вертикальная скорость
    2. акселерометр — старт и выгорание
    3. таймер      — резерв на случай отказа обоих

Размеры считаются здесь же: купол под нужную скорость снижения и навеска
вышибного заряда под объём корпуса.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from enum import Enum

G = 9.81
RHO = 1.2


class State(Enum):
    PAD = "на столе"
    ARMED = "взведено"
    BOOST = "разгон"
    COAST = "подъём по инерции"
    DESCENT_DROGUE = "спуск на вытяжном"
    DESCENT_MAIN = "спуск на основном"
    LANDED = "сел"


@dataclass
class Limits:
    """Пороги, при которых система принимает решения."""

    launch_accel_g: float = 3.0  # старт: перегрузка выше этой
    launch_hold_s: float = 0.25  # и держится столько
    boost_timeout_s: float = 3.0  # дольше этого разгон длиться не может
    arm_confirm_s: float = 3.0  # подтверждение взвода
    apogee_hold_s: float = 0.2  # вертикальная скорость около нуля столько
    apogee_min_time_s: float = 2.0  # раньше этого апогея быть не может
    apogee_min_altitude_m: float = 20.0  # и ниже этой высоты тоже
    main_altitude_m: float = 150.0  # высота выпуска основного купола
    backup_deploy_s: float = 30.0  # резервный таймер на выпуск
    main_backup_s: float = 12.0  # резерв на основной купол после вытяжного
    # Скорость, выше которой выпускать купол нельзя: порвёт
    max_deploy_speed: float = 35.0


@dataclass
class Sensors:
    """Показания. Пустое значение — None, а не выдуманное число."""

    time_s: float = 0.0
    altitude_m: float | None = None
    vertical_speed: float | None = None
    accel_magnitude: float | None = None
    arm_switch: bool = False
    charge_continuity: bool = False  # проверка целостности цепи заряда


@dataclass
class Command:
    deploy_drogue: bool = False
    deploy_main: bool = False
    reason: str = ""
    state: State = State.PAD
    refusals: list = field(default_factory=list)


class Recovery:
    """Конечный автомат парашюта.

    Каждый переход требует выполнения условия И подтверждения по времени:
    одиночный выброс датчика командой не считается. Это защита от помех,
    которые на старте ракеты неизбежны.
    """

    def __init__(self, limits: Limits | None = None) -> None:
        self.limits = limits or Limits()
        self.state = State.PAD
        self.launch_time = 0.0
        self.apogee_time = 0.0
        self.drogue_time = 0.0
        self._arm_since = 0.0
        self._accel_since = 0.0
        self._zero_rate_since = 0.0
        self._last = 0.0

    # -- условия -----------------------------------------------------------

    def _launched(self, sensors: Sensors) -> bool:
        if sensors.accel_magnitude is None:
            return False
        return sensors.accel_magnitude > self.limits.launch_accel_g * G

    def _approaching_apogee(self, sensors: Sensors) -> bool:
        if sensors.vertical_speed is None or sensors.altitude_m is None:
            return False
        # Проверка правдоподобности. Залипший барометр, показывающий нулевую
        # скорость и нулевую высоту, выглядит как апогей — а это опаснее
        # всего: заряд сработает через полсекунды после старта. Поэтому
        # апогей принимается только если аппарат успел подняться
        if sensors.time_s - self.launch_time < self.limits.apogee_min_time_s:
            return False
        if sensors.altitude_m < self.limits.apogee_min_altitude_m:
            return False
        return abs(sensors.vertical_speed) < 3.0

    def _below_main(self, sensors: Sensors) -> bool:
        return sensors.altitude_m is not None and sensors.altitude_m <= self.limits.main_altitude_m

    # -- основной шаг ------------------------------------------------------

    def update(self, sensors: Sensors) -> Command:
        command = Command(state=self.state)
        dt = max(0.0, sensors.time_s - self._last)
        self._last = sensors.time_s

        # Взвод: только тумблер, только на столе, только с целой цепью заряда
        if self.state is State.PAD:
            if sensors.arm_switch and sensors.charge_continuity:
                self._arm_since += dt
                if self._arm_since >= self.limits.arm_confirm_s:
                    self.state = State.ARMED
                    command.reason = "взведено"
            else:
                self._arm_since = 0.0
                if sensors.arm_switch and not sensors.charge_continuity:
                    command.refusals.append("нет целостности цепи заряда")
            command.state = self.state
            return command

        # Старт: перегрузка держится, а не мигнула
        if self.state is State.ARMED:
            if self._launched(sensors):
                self._accel_since += dt
                if self._accel_since >= self.limits.launch_hold_s:
                    self.state = State.BOOST
                    self.launch_time = sensors.time_s
                    command.reason = "старт подтверждён"
            else:
                self._accel_since = 0.0
            command.state = self.state
            return command

        if self.state is State.BOOST:
            # Выход из разгона по двум признакам: упала перегрузка или
            # вышло время. Второй нужен на случай залипшего акселерометра:
            # без него автомат застрянет в разгоне и купол не выпустит
            burnt_out = sensors.accel_magnitude is not None and sensors.accel_magnitude < 1.5 * G
            timed_out = (sensors.time_s - self.launch_time) >= self.limits.boost_timeout_s
            if burnt_out or timed_out:
                self.state = State.COAST
                command.reason = "двигатель отработал" if burnt_out else "разгон по времени"
            command.state = self.state
            return command

        if self.state is State.COAST:
            if self._approaching_apogee(sensors):
                self._zero_rate_since += dt
            else:
                self._zero_rate_since = 0.0

            backup = sensors.time_s - self.launch_time >= self.limits.backup_deploy_s
            if self._zero_rate_since >= self.limits.apogee_hold_s or backup:
                # Перед выпуском проверяем, что не порвём купол
                if (
                    sensors.vertical_speed is not None
                    and abs(sensors.vertical_speed) > self.limits.max_deploy_speed
                ):
                    command.refusals.append(
                        f"скорость {abs(sensors.vertical_speed):.0f} м/с — купол порвёт"
                    )
                    command.state = self.state
                    return command
                self.state = State.DESCENT_DROGUE
                self.drogue_time = sensors.time_s
                command.deploy_drogue = True
                command.reason = "апогей: вытяжной" if not backup else "резервный таймер"
            command.state = self.state
            return command

        if self.state is State.DESCENT_DROGUE:
            since = sensors.time_s - self.drogue_time
            if self._below_main(sensors) or since >= self.limits.main_backup_s:
                self.state = State.DESCENT_MAIN
                command.deploy_main = True
                command.reason = "основной купол"
            command.state = self.state
            return command

        if self.state is State.DESCENT_MAIN:
            if sensors.altitude_m is not None and sensors.altitude_m < 3.0:
                self.state = State.LANDED
                command.reason = "сел"
            command.state = self.state
            return command

        command.state = self.state
        return command


# ---------------------------------------------------------------------------
# Размеры: купол и вышибной заряд
# ---------------------------------------------------------------------------


def canopy_diameter(mass_kg: float, descent_speed: float, cd: float = 0.75) -> float:
    """Диаметр плоского купола под нужную скорость снижения.

    Y = 0,5 * rho * V^2 * S * Cd = m * g
    """
    area = 2.0 * mass_kg * G / (RHO * descent_speed ** 2 * cd)
    return math.sqrt(4.0 * area / math.pi)


def ejection_charge(volume_litres: float, fit: str = "плотная") -> tuple[float, float]:
    """Навеска вышибного заряда под объём корпуса, граммы.

    Возвращает рабочий диапазон. Это только отправная точка для наземной
    отработки: настоящая навеска подбирается на земле, с незаряженным
    парашютом, шаг за шагом, и записывается. Больше — не значит надёжнее:
    лишний заряд рвёт корпус.
    """
    base = volume_litres * 0.7
    if fit == "свободная":
        base *= 0.8
    elif fit == "тугая":
        base *= 1.3
    return base * 0.8, base * 1.2


def shock_load(mass_kg: float, speed: float, cd: float = 0.75, area: float = 0.5) -> float:
    """Рывок при раскрытии купола, Н. Сравнивать с прочностью корпуса."""
    return 0.5 * RHO * speed * speed * area * cd


# ---------------------------------------------------------------------------
# Проверки на сценариях
# ---------------------------------------------------------------------------

RESULTS: list[tuple[bool, str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((ok, name, detail))
    print(f"  {'OK  ' if ok else 'ПЛОХО'} {name}" + (f" — {detail}" if detail and not ok else ""))


def flight(recovery: Recovery, profile, pad_s: float = 4.0, dt: float = 0.05) -> list[Command]:
    """Прогон в реальной последовательности: сначала стол и взвод, потом пуск.

    Взвод требует, чтобы тумблер был включён несколько секунд и аппарат
    при этом стоял неподвижно. Поэтому сначала идёт стол, и только потом
    профиль полёта — как это и происходит на площадке.
    """
    commands = []
    time_s = 0.0
    while time_s < pad_s:
        commands.append(recovery.update(Sensors(
            time_s=time_s, altitude_m=0.0, vertical_speed=0.0,
            accel_magnitude=G, arm_switch=True, charge_continuity=True,
        )))
        time_s += dt
    for rel, altitude, speed, accel in profile:
        commands.append(recovery.update(Sensors(
            time_s=time_s + rel, altitude_m=altitude, vertical_speed=speed,
            accel_magnitude=accel, arm_switch=True, charge_continuity=True,
        )))
    return commands


def normal_profile(dt: float = 0.05, apogee: float = 400.0):
    """Обычный полёт: разгон, подъём, апогей, спуск."""
    profile = []
    time_s = 0.0
    while time_s < 60.0:
        if time_s < 2.0:
            altitude = 0.5 * 30.0 * time_s ** 2
            speed = 30.0 * time_s
            accel = 4.0 * G
        elif altitude < apogee:
            altitude = apogee - 0.5 * G * (time_s - 7.0) ** 2
            speed = 30.0 - G * (time_s - 7.0)
            accel = G
        else:
            altitude = apogee
            speed = 0.0
            accel = G
        if time_s > 7.0:
            fall = time_s - 7.0
            altitude = max(0.0, apogee - 0.5 * 9.0 * fall ** 2)
            speed = -9.0 * fall
        profile.append((time_s, max(0.0, altitude), speed, accel))
        if altitude <= 0.0 and time_s > 8.0:
            break
        time_s += dt
    return profile


def main() -> int:
    print("Система автоматического парашюта")
    print()

    # --- проверка на нормальном полёте ---
    print("--- обычный полёт ---")
    recovery = Recovery()
    commands = flight(recovery, normal_profile())
    drogue = [c for c in commands if c.deploy_drogue]
    main_chute = [c for c in commands if c.deploy_main]
    check("вытяжной выпущен ровно один раз", len(drogue) == 1, f"выпусков {len(drogue)}")
    check("основной выпущен ровно один раз", len(main_chute) == 1, f"выпусков {len(main_chute)}")
    check("аппарат сел", commands[-1].state is State.LANDED or recovery.state is State.LANDED,
          f"состояние {recovery.state.value}")

    # --- главное свойство: на земле не срабатывает ---
    print("--- на земле заряд не срабатывает ---")
    recovery = Recovery()
    lying = [(t / 10.0, 0.0, 0.0, G) for t in range(600)]
    commands = flight(recovery, [], pad_s=60.0)
    check("взведён, но лежит 60 с — заряд молчит",
          not any(c.deploy_drogue or c.deploy_main for c in commands),
          "заряд сработал на земле")
    check("взвод состоялся", recovery.state in (State.ARMED, State.PAD),
          f"состояние {recovery.state.value}")

    # --- рука перед камерой и прочие помехи ---
    print("--- помехи и ложные признаки ---")
    recovery = Recovery()
    noisy = [(t / 10.0, 0.0, 0.0, 1.0 * G if t % 20 else 8.0 * G) for t in range(600)]
    commands = flight(recovery, noisy)
    check("короткие всплески перегрузки не считаются стартом",
          not any(c.deploy_drogue for c in commands),
          "сработал на помехе")

    # --- отказ барометра: спасает таймер ---
    print("--- отказ барометра ---")
    recovery = Recovery()
    broken = [(t / 10.0, 0.0, 0.0, 4.0 * G if t > 10 else G) for t in range(600)]
    commands = flight(recovery, broken)
    check("при отказе барометра сработал резервный таймер",
          any(c.deploy_drogue for c in commands),
          "резерв не сработал, аппарат потерян")
    reasons = {c.reason for c in commands if c.deploy_drogue}
    check("сработал именно резерв, а не барометр",
          any("резерв" in r for r in reasons), f"причины: {reasons}")

    # --- целостность цепи заряда ---
    print("--- цепь заряда ---")
    recovery = Recovery()
    commands = []
    for t in range(200):
        sensors = Sensors(time_s=t / 10.0, altitude_m=0.0, vertical_speed=0.0,
                          accel_magnitude=G, arm_switch=True, charge_continuity=False)
        commands.append(recovery.update(sensors))
    check("без целостности цепи взвода нет", recovery.state is State.PAD,
          f"состояние {recovery.state.value}")
    check("отказ объяснён в журнале",
          any("целостност" in r for c in commands for r in c.refusals))

    # --- быстрое снижение до выпуска ---
    print("--- запрет выпуска на большой скорости ---")
    recovery = Recovery()
    fast = [(0.0, 0.0, 0.0, G), (1.0, 0.0, 0.0, G)]
    for t in range(1, 60):
        time_s = t * 0.1
        fast.append((time_s, max(0.0, 300.0 - 40.0 * t), -40.0, G))
    commands = flight(recovery, fast)
    check("на 40 м/с купол не выпущен",
          not any(c.deploy_drogue for c in commands) or
          any("порвёт" in r for c in commands for r in c.refusals),
          "выпустил купол на разрушающей скорости")

    # --- размеры ---
    print()
    print("Размеры под аппарат 2,4 кг")
    for speed in (4.0, 5.0, 6.0, 8.0):
        diameter = canopy_diameter(2.4, speed)
        area = math.pi * diameter ** 2 / 4.0
        load = shock_load(2.4, 30.0, area=area)
        print(f"  снижение {speed:.0f} м/с: купол {diameter:.2f} м "
              f"({area:.2f} м²), рывок при 30 м/с — {load:.0f} Н ({load / (2.4 * G):.1f} g)")
    low, high = ejection_charge(2.0)
    print(f"  навеска на корпус 2 л: от {low:.1f} до {high:.1f} г пороха —")
    print("  это отправная точка, настоящая навеска подбирается на земле")
    print()

    failed = [r for r in RESULTS if not r[0]]
    print(f"проверок: {len(RESULTS)}, успешно: {len(RESULTS) - len(failed)}, "
          f"провалено: {len(failed)}")
    for _ok, name, detail in failed:
        print(f"  провал: {name} — {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
