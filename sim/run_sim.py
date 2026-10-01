"""Прогон полёта в тренажёре: держит ли аппарат курс.

    python sim/run_sim.py

Печатает таблицу отклонений и рисует вид сверху в out/trajectory.svg.
Никаких внешних библиотек: только стандартный Python.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "flight"))
sys.path.insert(0, str(HERE))

from control import CourseHolder, Tune, Vehicle  # noqa: E402
from rocket import Conditions, Rocket, prepare_holder, simulate  # noqa: E402

OUT = HERE.parent / "out"


def scenario(
    name: str,
    control_on: bool,
    stars_ok: bool = True,
    wind: float = 3.0,
    gust: float = 3.0,
    gyro_bias_deg: float = 0.0,
    calibrate: bool = True,
    tune: Tune | None = None,
    note: str = "",
):
    # Один и тот же объект геометрии идёт и в модель полёта, и в регулятор
    vehicle = Vehicle()
    rocket = Rocket(vehicle=vehicle)
    bias = math.radians(gyro_bias_deg)
    cond = Conditions(
        wind=wind,
        gust=gust,
        gyro_bias=(0.0, 0.0, 0.0),
        gyro_noise=math.radians(0.05),
    )
    holder = CourseHolder(tune, vehicle) if control_on else None
    if control_on and holder is not None:
        # calibrate=True — гироскоп откалиброван при том же уходе, что и в полёте,
        # значит уход скомпенсирован. calibrate=False — калибровали по нулям,
        # а уход появился потом: так бывает, когда датчик прогревается
        cond.gyro_bias = (0.0, 0.0, bias if calibrate else 0.0)
        prepare_holder(rocket, cond, holder, stars_ok=stars_ok)
        cond.gyro_bias = (0.0, 0.0, bias)
    result = simulate(name, rocket, cond, control_on, holder, stars_ok)
    result.note = note
    return result


def main() -> int:
    runs = [
        scenario(
            "без управления",
            control_on=False,
            note="рули закреплены: аппарат сам уходит по ветру",
        ),
        scenario(
            "курс держится",
            control_on=True,
            note="курс зафиксирован по звезде, гироскоп откалиброван",
        ),
        scenario(
            "курс держится, но звёзд нет",
            control_on=True,
            stars_ok=False,
            note="звёздный датчик не дал ориентацию: управление отключено",
        ),
        scenario(
            "гироскоп откалиброван плохо",
            control_on=True,
            gyro_bias_deg=0.5,
            calibrate=False,
            note="уход 0,5 °/с остался в оценке: курс медленно уезжает",
        ),
    ]

    print(
        f"{'прогон':<32}{'высота':>9}{'снос':>9}{'падение':>10}"
        f"{'ошибка':>9}{'рули':>8}{'управление':>12}"
    )
    print("-" * 90)
    for r in runs:
        share = 100.0 * r.active_steps / max(r.total_steps, 1)
        print(
            f"{r.name:<32}{r.apogee:>8.1f} м{r.max_deviation:>8.1f} м"
            f"{r.impact_y:>9.1f} м{r.max_err_deg:>8.1f}°"
            f"{r.max_delta_deg:>7.1f}°{share:>11.0f}%"
        )
    print()
    for r in runs:
        if r.note:
            print(f"  {r.name}: {r.note}")

    base = runs[0]
    held = runs[1]
    print()
    print(f"Боковой снос без управления:      {base.max_deviation:>6.1f} м")
    print(f"Боковой снос с управлением:       {held.max_deviation:>6.1f} м")
    if base.max_deviation > 0:
        gain = base.max_deviation / max(held.max_deviation, 1e-9)
        print(f"Улучшение:                        {gain:>6.1f} раза")

    OUT.mkdir(exist_ok=True)
    svg = OUT / "trajectory.svg"
    svg.write_text(draw(runs), encoding="utf-8")
    print(f"\nВид сверху: {svg}")
    return 0


def draw(runs) -> str:
    """Простой вид сверху: дальность по горизонтали, боковой снос по вертикали."""
    width, height = 900, 460
    margin = 60
    max_x = max((max((s[1][0] for s in r.samples), default=1.0) for r in runs), default=1.0)
    max_y = max((max(abs(s[1][1]) for s in r.samples) for r in runs), default=1.0)
    max_y = max(max_y, 1.0)

    colors = ["#d64545", "#2f9e44", "#7a7a7a", "#1c7ed6"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfbfb"/>',
        f'<text x="{margin}" y="28" font-family="sans-serif" font-size="17" fill="#222">'
        "Вид сверху: чем ближе к нулю, тем ровнее курс</text>",
    ]

    # Сетка
    steps = 5
    for i in range(steps + 1):
        y = margin + (height - 2 * margin) * i / steps
        value = max_y * (1 - 2 * i / steps)
        parts.append(
            f'<line x1="{margin}" y1="{y:.1f}" x2="{width - margin}" y2="{y:.1f}" '
            'stroke="#e0e0e0"/>'
        )
        parts.append(
            f'<text x="{margin - 8}" y="{y + 4:.1f}" text-anchor="end" '
            f'font-family="sans-serif" font-size="12" fill="#666">{value:.1f}</text>'
        )
    zero = margin + (height - 2 * margin) / 2
    parts.append(
        f'<line x1="{margin}" y1="{zero:.1f}" x2="{width - margin}" y2="{zero:.1f}" '
        'stroke="#999" stroke-width="1.5"/>'
    )

    def px(x: float) -> float:
        return margin + (width - 2 * margin) * (x / max_x)

    def py(y: float) -> float:
        return zero - (height - 2 * margin) / 2 * (y / max_y)

    for index, r in enumerate(runs):
        points = " ".join(
            f"{px(s[1][0]):.1f},{py(s[1][1]):.1f}" for s in r.samples[::10]
        )
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="{colors[index % len(colors)]}" '
            'stroke-width="2.2"/>'
        )
        last = r.samples[-1][1] if r.samples else (0, 0, 0)
        parts.append(
            f'<circle cx="{px(last[0]):.1f}" cy="{py(last[1]):.1f}" r="4" '
            f'fill="{colors[index % len(colors)]}"/>'
        )
        parts.append(
            f'<text x="{px(last[0]) + 8:.1f}" y="{py(last[1]) - 6:.1f}" '
            f'font-family="sans-serif" font-size="13" fill="{colors[index % len(colors)]}">'
            f"{r.name}</text>"
        )

    parts.append(
        f'<text x="{width - margin}" y="{height - 18}" text-anchor="end" '
        'font-family="sans-serif" font-size="13" fill="#666">дальность, м</text>'
    )
    parts.append(
        f'<text x="{margin - 8}" y="{margin - 14}" text-anchor="end" '
        'font-family="sans-serif" font-size="13" fill="#666">снос, м</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


if __name__ == "__main__":
    sys.exit(main())
