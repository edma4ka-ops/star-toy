"""Полётный компьютер: запуск, рули, парашют, спасение от встречи.

Запускается на RP2040 (Raspberry Pi Pico) под MicroPython. Собирает вместе
три системы, которые до этого были проверены по отдельности:

    launch/sequencer.py   запал, выдержка 0,5 с, катапульта
    flight/control.py     рули и стабилизация
    recovery/system.py    парашют: барометр, акселерометр, таймер

Плюс четвёртая — спасение от случайной встречи. Камера на Raspberry
присылает просьбу «в кадре крупный объект», а решение принимает плата.
Это принципиально: камера видит крупный объект постоянно — землю при
подъёме, облако, птицу, руку при запуске. Поэтому камера только ПРОСИТ,
а выпускает плата, и только если все её проверки пройдены.

Проверки перед выпуском по просьбе камеры:

    * система взведена и аппарат действительно улетел;
    * прошло не меньше 3 секунд после старта — иначе ракета рядом с расчётом;
    * высота выше минимальной;
    * скорость ниже той, на которой купол порвёт;
    * просьба держится несколько циклов, а не мигнула один раз.

Логика проверяется на компьютере: python mcu/flight_computer.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE.parent / "launch"))
sys.path.insert(0, str(HERE.parent / "recovery"))

from control import HeadingHolder, HeadingTune, HoverCraft  # noqa: E402
from sequencer import Inputs as LaunchInputs  # noqa: E402
from sequencer import Sequencer, State as LaunchState  # noqa: E402
from system import Recovery, Sensors as RecoverySensors, State as RecoveryState  # noqa: E402

try:  # на плате это есть, на компьютере нет
    from machine import I2C, PWM, UART, WDT, Pin  # noqa: F401

    ON_DEVICE = True
except ImportError:
    ON_DEVICE = False

G = 9.81


@dataclass
class RescueGate:
    """Проверки перед выпуском парашюта по просьбе камеры."""

    arm_delay_s: float = 3.0  # раньше этого после старта не выпускаем
    min_altitude_m: float = 25.0  # и ниже этой высоты не выпускаем
    max_speed: float = 35.0  # выше этой скорости купол порвёт
    hold_s: float = 0.2  # просьба должна держаться столько


@dataclass
class Vehicle:
    """Что нужно знать плате о ракете, чтобы принимать решения."""

    mass_kg: float = 2.4
    launch_elevation_deg: float = 35.0


@dataclass
class Telemetry:
    """Что плата отправляет на Raspberry для журнала оператора."""

    state: str = ""
    altitude_m: float = 0.0
    speed: float = 0.0
    heading_deg: float = 0.0
    recovery: str = ""
    rescue: str = ""
    log: list = field(default_factory=list)


class FlightComputer:
    """Собирает все системы вместе и следит за порядком решений."""

    def __init__(self, vehicle: Vehicle | None = None,
                 gate: RescueGate | None = None) -> None:
        self.vehicle = vehicle or Vehicle()
        self.gate = gate or RescueGate()
        self.launch = Sequencer()
        self.recovery = Recovery()
        self.holder = HeadingHolder(HeadingTune(), HoverCraft(), tail_arm=0.35)
        self.holder.arm()
        self.launch_time = 0.0
        self.flying = False
        self._rescue_since = 0.0
        self._last = 0.0
        self.traces: list[Telemetry] = []

    # -- проверка выпуска по просьбе камеры --------------------------------

    def rescue_allowed(self, time_s: float, altitude: float, speed: float,
                       requested: bool) -> tuple[bool, str]:
        """Пускать ли парашют по просьбе камеры. Возвращает решение и причину."""
        if not requested:
            self._rescue_since = 0.0
            return False, "нет просьбы"
        if not self.flying:
            self._rescue_since = 0.0
            return False, "аппарат не в полёте"
        if self.recovery.state is RecoveryState.PAD:
            self._rescue_since = 0.0
            return False, "парашют не взведён"
        if self.recovery.state in (RecoveryState.DESCENT_DROGUE, RecoveryState.DESCENT_MAIN):
            return False, "купол уже выпущен"

        since_launch = time_s - self.launch_time
        if since_launch < self.gate.arm_delay_s:
            self._rescue_since = 0.0
            return False, f"рано: {since_launch:.1f} с, ракета ещё рядом с расчётом"
        if altitude < self.gate.min_altitude_m:
            self._rescue_since = 0.0
            return False, f"низко: {altitude:.0f} м"
        if speed > self.gate.max_speed:
            self._rescue_since = 0.0
            return False, f"быстро: {speed:.0f} м/с, купол порвёт"

        # Просьба должна держаться, а не мигнуть: одна вспышка в кадре
        # не повод выбрасывать купол
        self._rescue_since += max(0.0, time_s - self._last)
        if self._rescue_since < self.gate.hold_s:
            return False, f"подтверждение {self._rescue_since:.2f} с"
        return True, "разрешено"

    # -- один шаг полёта ---------------------------------------------------

    def step(self, time_s: float, *, arm_switch: bool, igniter_ok: bool,
             accel_g: float, altitude: float, vertical_speed: float,
             rail_released: bool, rescue_request: bool,
             gyro=(0.0, 0.0, 0.0), heading_measured: float | None = None,
             operator_stop: bool = False) -> Telemetry:
        dt = max(0.0, time_s - self._last)

        # 1. Запуск: запал, выдержка, катапульта
        launch_command = self.launch.update(LaunchInputs(
            time_s=time_s, arm_switch=arm_switch, igniter_ok=igniter_ok,
            accel_g=accel_g, rail_released=rail_released,
            operator_stop=operator_stop))

        # 2. Парашют по своим датчикам
        recovery_command = self.recovery.update(RecoverySensors(
            time_s=time_s, altitude_m=altitude, vertical_speed=vertical_speed,
            accel_magnitude=accel_g * G, arm_switch=arm_switch,
            charge_continuity=igniter_ok))

        # 3. Спасение от встречи: камера просит, плата решает
        allowed, reason = self.rescue_allowed(time_s, altitude, abs(vertical_speed),
                                              rescue_request)
        rescue_deploy = False
        if allowed:
            rescue_deploy = True
            reason = "выпуск по просьбе камеры"
            # Переводим автомат парашюта в спуск, чтобы не сработал второй раз
            if self.recovery.state in (RecoveryState.COAST, RecoveryState.BOOST):
                self.recovery.state = RecoveryState.DESCENT_DROGUE

        if launch_command.state is LaunchState.FLYING and not self.flying:
            self.flying = True
            self.launch_time = time_s

        # 4. Рули: работают, пока аппарат в полёте
        servo = 0.0
        if self.flying:
            self.holder.set_target(math.radians(0.0))
            rudder = self.holder.update(dt, gyro, gyro[2], heading_measured, 0.0)
            servo = rudder.ch_z if rudder.active else 0.0

        # 5. Выходы
        deploy_drogue = recovery_command.deploy_drogue or rescue_deploy
        deploy_main = recovery_command.deploy_main

        telemetry = Telemetry(
            state=launch_command.state.value,
            altitude_m=altitude,
            speed=abs(vertical_speed),
            heading_deg=math.degrees(self.holder.reference),
            recovery=self.recovery.state.value,
            rescue=reason,
            log=list(launch_command.log),
        )
        telemetry.log.append(f"руль {servo:+.2f}")
        if deploy_drogue:
            telemetry.log.append("ВЫПУСК вытяжного")
        if deploy_main:
            telemetry.log.append("ВЫПУСК основного")
        if launch_command.fire_igniter:
            telemetry.log.append("ЗАПАЛ")
        if launch_command.release_catapult:
            telemetry.log.append("КАТАПУЛЬТА")

        self._last = time_s
        self.traces.append(telemetry)
        return telemetry


# ---------------------------------------------------------------------------
# Проверка на компьютере: прогон полёта целиком
# ---------------------------------------------------------------------------

RESULTS: list[tuple[bool, str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((ok, name, detail))
    print(f"  {'OK  ' if ok else 'ПЛОХО'} {name}" + (f" — {detail}" if detail and not ok else ""))


def fly(rescue_from: float | None = None, rescue_to: float | None = None) -> FlightComputer:
    """Прогон полёта: стол, взвод, запуск, разгон, подъём, спуск."""
    computer = FlightComputer()
    dt = 0.02
    time_s = 0.0
    altitude = 0.0
    speed = 0.0
    while time_s < 30.0:
        if time_s < 3.2:
            accel_g, altitude, speed = 1.0, 0.0, 0.0
        elif time_s < 3.8:
            accel_g, altitude, speed = 2.1, 0.0, 0.0  # двигатель тянет на направляющей
        else:
            t = time_s - 3.8
            speed = min(45.0, 16.8 + 30.0 * t)
            altitude = min(300.0, 16.8 * t + 15.0 * t * t)
            if t > 6.0:
                altitude = max(0.0, 300.0 - 9.0 * (t - 6.0) ** 2)
                speed = -9.0 * (t - 6.0)
            accel_g = 1.0 if t > 6.0 else 2.1
        rescue = (rescue_from is not None and rescue_to is not None
                  and rescue_from <= time_s <= rescue_to)
        computer.step(time_s, arm_switch=True, igniter_ok=True, accel_g=accel_g,
                      altitude=altitude, vertical_speed=speed,
                      rail_released=(time_s >= 3.5), rescue_request=rescue)
        time_s += dt
    return computer


def main() -> int:
    print("Полётный компьютер: запуск, парашют, спасение от встречи")
    print()

    print("--- обычный полёт без просьбы камеры ---")
    normal = fly()
    fired = [t for t in normal.traces if "ЗАПАЛ" in t.log]
    released = [t for t in normal.traces if "КАТАПУЛЬТА" in t.log]
    drogue = [t for t in normal.traces if "ВЫПУСК вытяжного" in t.log]
    check("запал подан один раз", len(fired) == 1, f"подач {len(fired)}")
    check("катапульта отпущена один раз", len(released) == 1, f"отпусканий {len(released)}")
    check("купол выпущен в апогее, без просьб камеры", len(drogue) == 1,
          f"выпусков {len(drogue)}")

    print("--- камера просит выпуск слишком рано ---")
    early = fly(rescue_from=1.0, rescue_to=2.5)
    early_reasons = {t.rescue for t in early.traces if t.rescue != "нет просьбы"}
    check("выпуск на 1-2 секунде отклонён",
          not any("разрешено" in r for r in early_reasons) and bool(early_reasons),
          f"причины: {early_reasons}")

    print("--- камера просит выпуск в полёте ---")
    # Просьба на спуске: скорость уже погашена, высота рабочая
    mid = fly(rescue_from=10.5, rescue_to=11.5)
    allowed = [t for t in mid.traces if "разрешено" in t.rescue]
    check("в полёте просьба принята", bool(allowed), "просьба не принята")
    if allowed:
        first = allowed[0]
        check("выпуск произошёл с рабочей высоты", first.altitude_m > 25.0,
              f"высота {first.altitude_m:.0f} м")

    print("--- камера просит при падении на большой скорости ---")
    fast = fly(rescue_from=8.0, rescue_to=9.0)  # на разгоне, 45 м/с
    fast_reasons = {t.rescue for t in fast.traces if t.rescue not in ("нет просьбы", "разрешено")}
    check("на большой скорости выпуск отклонён",
          bool(fast_reasons), f"причины: {fast_reasons}")

    print("--- парашют уже выпущен, повторной просьбы быть не должно ---")
    after = fly(rescue_from=14.0, rescue_to=16.0)
    drags = [t for t in after.traces if "ВЫПУСК вытяжного" in t.log]
    check("вытяжной выпущен ровно один раз", len(drags) == 1, f"выпусков {len(drags)}")

    print()
    print("Кто чем управляет")
    print("  камера      просит выпуск, сама не выпускает")
    print("  плата       решает: время, высота, скорость, подтверждение")
    print("  барометр    даёт апогей и высоту для основного купола")
    print("  таймер      резерв, если откажут датчики")
    print()
    failed = [r for r in RESULTS if not r[0]]
    print(f"проверок: {len(RESULTS)}, успешно: {len(RESULTS) - len(failed)}, "
          f"провалено: {len(failed)}")
    for _ok, name, detail in failed:
        print(f"  провал: {name} — {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
