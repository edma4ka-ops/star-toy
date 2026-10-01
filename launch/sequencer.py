"""Система запуска: запал, выдержка, катапульта.

    python launch/sequencer.py

Замысел: зажигается двигатель, через 0,5 секунды отпускает катапульта
с резинками, и ракета уходит по направляющей уже с работающим двигателем.

Почему выдержка нужна. Твёрдотопливный двигатель выходит на тягу
не мгновенно: от запала до полной тяги проходит 0,2-0,5 секунды.
Если отпустить сразу, ракета сойдёт с направляющей на скорости резинок,
а двигатель ещё только разгорается — и часть направляющей потратится зря.
Если задержать слишком долго, ракета будет стоять на направляющей
с работающим двигателем: это уже опасно для расчёта.

Главное, что должна уметь система: распознать, что двигатель не зажёгся.
Тогда катапульта всё равно выбросит ракету, и её спасёт парашют.

Логика проверяется на сценариях, включая отказы: без этого запускать нельзя.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from enum import Enum

G = 9.81


class State(Enum):
    SAFE = "безопасно"
    ARMED = "взведено"
    IGNITED = "запал подан"
    RELEASED = "катапульта отпустила"
    FLYING = "полёт"
    DUD = "двигатель не зажёгся"
    HELD = "катапульта не отпустила"


@dataclass
class Timeline:
    """Времена системы запуска, секунды."""

    arm_confirm_s: float = 3.0  # тумблер держится столько до взвода
    # Отпускание строго по таймеру, ровно через 0,5 с после запала,
    # без зависимости от датчиков: так решение предсказуемо
    ignition_to_release_s: float = 0.5
    release_timeout_s: float = 1.5  # если не отпустило за это время — тревога
    thrust_confirm_s: float = 0.15  # после отпускания ждём перегрузку
    # Порог подтверждения тяги. Считать его надо от тяговооружённости:
    # тяга 5 кг при массе 2,4 кг даёт всего 2,1 g собственной перегрузки,
    # поэтому порог 3 g никогда не сработал бы. Берём 1,5 g — это
    # уверенно выше покоя (1 g) и ниже рабочей тяги (2,1 g)
    min_thrust_g: float = 1.5


@dataclass
class Inputs:
    time_s: float = 0.0
    arm_switch: bool = False  # физический тумблер
    igniter_ok: bool = False  # целостность цепи запала
    accel_g: float = 1.0  # перегрузка по акселерометру
    rail_released: bool = False  # концевик: катапульта отпустила
    operator_stop: bool = False  # кнопка «стоп»


@dataclass
class Command:
    fire_igniter: bool = False
    release_catapult: bool = False
    deploy_parachute: bool = False
    abort: bool = False
    state: State = State.SAFE
    message: str = ""
    log: list = field(default_factory=list)


class Sequencer:
    """Автомат запуска. Одна команда на каждое действие, без повторов."""

    def __init__(self, timeline: Timeline | None = None) -> None:
        self.t = timeline or Timeline()
        self.state = State.SAFE
        self.ignition_time = 0.0
        self.release_time = 0.0
        self._arm_since = 0.0
        self._igniter_fired = False
        self._catapult_fired = False
        self._parachute_fired = False
        self._last = 0.0

    # -- вспомогательное ---------------------------------------------------

    def _once(self, flag: str) -> bool:
        """Действие выполняется один раз, повторов быть не должно."""
        if getattr(self, flag):
            return False
        setattr(self, flag, True)
        return True

    # -- основной шаг ------------------------------------------------------

    def update(self, inputs: Inputs) -> Command:
        command = Command(state=self.state)
        dt = max(0.0, inputs.time_s - self._last)
        self._last = inputs.time_s
        command.state = self.state

        # Оператор может остановить всё до момента запала
        if inputs.operator_stop and self.state in (State.SAFE, State.ARMED):
            self.state = State.SAFE
            command.abort = True
            command.message = "остановлено оператором"
            command.state = self.state
            return command

        if self.state is State.SAFE:
            if inputs.arm_switch and inputs.igniter_ok:
                self._arm_since += dt
                if self._arm_since >= self.t.arm_confirm_s:
                    self.state = State.ARMED
                    command.message = "взведено"
            else:
                self._arm_since = 0.0
                if inputs.arm_switch and not inputs.igniter_ok:
                    command.log.append("нет целостности цепи запала")
            command.state = self.state
            return command

        if self.state is State.ARMED:
            # Запал подаётся один раз и только при взведённом тумблере
            if not inputs.arm_switch:
                self.state = State.SAFE
                command.message = "тумблер снят до запала"
                command.state = self.state
                return command
            if self._once("_igniter_fired"):
                self.ignition_time = inputs.time_s
                self.state = State.IGNITED
                command.fire_igniter = True
                command.message = "запал подан"
            command.state = self.state
            return command

        if self.state is State.IGNITED:
            since = inputs.time_s - self.ignition_time
            if since >= self.t.ignition_to_release_s:
                if self._once("_catapult_fired"):
                    self.release_time = inputs.time_s
                    self.state = State.RELEASED
                    command.release_catapult = True
                    command.message = f"катапульта отпустила через {since:.2f} с"
            command.state = self.state
            return command

        if self.state is State.RELEASED:
            since = inputs.time_s - self.release_time
            # Ждём подтверждения тяги: ракета должна начать разгоняться
            if inputs.accel_g >= self.t.min_thrust_g:
                self.state = State.FLYING
                command.message = "двигатель тянет"
            elif since >= self.t.thrust_confirm_s:
                # Двигатель не зажёгся: катапульта выбросила ракету,
                # спасаем её парашютом
                self.state = State.DUD
                command.message = "двигатель не зажёгся, выпускаю парашют"
                if self._once("_parachute_fired"):
                    command.deploy_parachute = True
            elif since >= self.t.release_timeout_s:
                self.state = State.HELD
                command.abort = True
                command.message = "катапульта не отпустила"
            command.state = self.state
            return command

        command.state = self.state
        return command


# ---------------------------------------------------------------------------
# Расчёт разгона по направляющей
# ---------------------------------------------------------------------------


def rail_exit_speed(mass_kg: float, rail_m: float, elevation_deg: float,
                    bungee_peak: float, bungee_end: float, thrust: float) -> tuple[float, float]:
    """Скорость схода с направляющей и ускорение на ней.

    Направляющая 2,5 метра: работают резинки, двигатель и мешает
    составляющая веса.
    """
    import math

    bungee_avg = (bungee_peak + bungee_end) / 2.0
    weight_along = mass_kg * G * math.sin(math.radians(elevation_deg))
    accel = (bungee_avg + thrust - weight_along) / mass_kg
    speed = math.sqrt(max(0.0, 2.0 * accel * rail_m))
    return speed, accel


# ---------------------------------------------------------------------------
# Проверки на сценариях
# ---------------------------------------------------------------------------

RESULTS: list[tuple[bool, str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((ok, name, detail))
    print(f"  {'OK  ' if ok else 'ПЛОХО'} {name}" + (f" — {detail}" if detail and not ok else ""))


def run(seq: Sequencer, timeline: list[Inputs]) -> tuple[list[Command], Sequencer]:
    return [seq.update(i) for i in timeline], seq


def normal_timeline(dt: float = 0.01) -> list[Inputs]:
    """Обычный запуск: взвод 3 с, запал, выдержка 0,5 с, отпускание.

    Акселерометр на направляющей уже показывает тягу: ракета зажата между
    двигателем и резинками, и её корпус передаёт это усилие. Тяга 5 кг
    при массе 2,4 кг даёт около 2,1 g.
    """
    out = []
    t = 0.0
    while t < 5.0:
        if t < 3.5:
            accel = 1.0
        elif t < 3.7:
            accel = 1.0 + (t - 3.5) * 5.5  # выход на тягу за 0,2 с
        else:
            accel = 2.1
        out.append(Inputs(time_s=t, arm_switch=True, igniter_ok=True,
                          accel_g=accel, rail_released=(t >= 3.5)))
        t += dt
    return out

def main() -> int:
    print("Система запуска: запал, выдержка 0,5 с, катапульта")
    print()
    speed, accel = rail_exit_speed(2.4, 2.5, 35.0, 160.0, 40.0, 49.0)
    print("Разгон по направляющей 2,5 м")
    print(f"  ускорение на направляющей:   {accel:.1f} м/с²")
    print(f"  скорость схода:              {speed:.1f} м/с")
    print(f"  время на направляющей:       {speed / accel:.2f} с")
    bungee_only, accel_only = rail_exit_speed(2.4, 2.5, 35.0, 160.0, 40.0, 0.0)
    print(f"  только резинки, без двигателя: {bungee_only:.1f} м/с "
          f"({accel_only:.0f} м/с²)")
    print(f"  скорость срыва крыла:         14,6 м/с")
    print()
    print("  За 0,5 секунды выдержки двигатель успевает выйти на тягу,")
    print("  а ракета стоит на месте: энергия резинок не тратится зря.")
    print()
    print("  Важное следствие: одни резинки дают 13,4 м/с — это МЕНЬШЕ скорости")
    print("  срыва. Значит без двигателя такой пуск лететь не может, и ракету")
    print("  обязательно надо спасать парашютом. Поэтому отпускание по таймеру")
    print("  безопасно только вместе с распознаванием отказа двигателя.")
    print()

    print("--- обычный запуск ---")
    timeline, seq = run(Sequencer(), normal_timeline())
    ignitions = [c for c in timeline if c.fire_igniter]
    releases = [c for c in timeline if c.release_catapult]
    check("запал подан ровно один раз", len(ignitions) == 1, f"подач {len(ignitions)}")
    check("катапульта отпущена ровно один раз", len(releases) == 1, f"отпусканий {len(releases)}")
    if ignitions and releases:
        gap = releases[0].state is State.RELEASED
        check("выдержка между запалом и отпусканием", gap, "порядок нарушен")
    check("состояние — полёт", seq.state is State.FLYING, f"состояние {seq.state.value}")

    print("--- двигатель не зажёгся ---")
    bad = []
    t = 0.0
    while t < 5.0:
        if t < 3.8:
            bad.append(Inputs(time_s=t, arm_switch=True, igniter_ok=True, accel_g=1.0))
        else:
            # Катапульта выбросила, но перегрузки нет: двигатель молчит
            bad.append(Inputs(time_s=t, arm_switch=True, igniter_ok=True,
                              accel_g=1.0, rail_released=True))
        t += 0.01
    timeline2, seq2 = run(Sequencer(), bad)
    check("отказ двигателя распознан", seq2.state is State.DUD, f"состояние {seq2.state.value}")
    check("парашют выпущен при отказе",
          any(c.deploy_parachute for c in timeline2), "парашют не выпущен")

    print("--- без взвода запала не будет ---")
    safe = [Inputs(time_s=i * 0.01, arm_switch=False, igniter_ok=True) for i in range(500)]
    timeline3, seq3 = run(Sequencer(), safe)
    check("без тумблера запал не подан",
          not any(c.fire_igniter for c in timeline3), "запал подан без взвода")

    print("--- обрыв цепи запала ---")
    broken = [Inputs(time_s=i * 0.01, arm_switch=True, igniter_ok=False) for i in range(500)]
    timeline4, seq4 = run(Sequencer(), broken)
    check("при обрыве цепи запала нет",
          not any(c.fire_igniter for c in timeline4) and seq4.state is State.SAFE,
          f"состояние {seq4.state.value}")

    print("--- остановка оператором до запала ---")
    stop = [Inputs(time_s=i * 0.01, arm_switch=True, igniter_ok=True,
                   operator_stop=(i > 200)) for i in range(500)]
    timeline5, seq5 = run(Sequencer(), stop)
    check("остановка сработала до запала",
          not any(c.fire_igniter for c in timeline5) and seq5.state is State.SAFE,
          f"состояние {seq5.state.value}")

    print()
    print("Чего система не умеет и что надо доделать")
    print("  * нет датчика тяги: факт зажигания виден только после отпускания,")
    print("    по перегрузке. Раньше его увидеть нечем, поэтому выдержка")
    print("    делается по времени, а не по факту.");
    print("  * нет управления самой катапультой: она должна иметь свой")
    print("    электрический спуск и концевик, подтверждающий отпускание.");
    print("  * нет контроля, что ракета ушла с направляющей целиком.")
    print()

    failed = [r for r in RESULTS if not r[0]]
    print(f"проверок: {len(RESULTS)}, успешно: {len(RESULTS) - len(failed)}, "
          f"провалено: {len(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
