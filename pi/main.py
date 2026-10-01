"""Главный цикл на Raspberry Pi Zero 2 W: солнце, GPS, команды рулям.

    python main.py

Что делает Pi:
  * снимает кадр и находит солнце (2-5 пикселей) — это абсолютный курсовой
    ориентир, он не уплывает, в отличие от гироскопа;
  * читает GPS: трек по земле и путевую скорость;
  * считает азимут солнца по времени и месту — это целевой курс;
  * считает, насколько развернуть нос против ветра, чтобы трек был прямым;
  * отправляет задание на микроконтроллер, который держит рули.

Чего Pi не делает: не управляет рулями напрямую. Под Linux тайминг
плавает на миллисекунды, а контуру нужен ровный шаг. Рули держит RP2040.

Если солнце не найдено или GPS молчит, программа не выдумывает данные,
а честно понижает режим и сообщает об этом — это видно в журнале.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from guidance import CourseGuidance, Tune, nose_azimuth_from_sun, sun_azimuth
from sun_vision import find_horizon, measure_sun

# --- настройки ---
WIDTH, HEIGHT = 640, 480
FOV_X_DEG, FOV_Y_DEG = 120.0, 90.0
SUN_THRESHOLD = 245
LOOP_HZ = 25.0
LINK_PORT = "/dev/ttyAMA0"
LINK_BAUD = 115200
LOG_EVERY_S = 1.0


@dataclass
class Sensors:
    """Показания датчиков. Пустое значение — None, а не выдуманное число."""

    sun_valid: bool = False
    sun_bearing_x: float = 0.0
    sun_bearing_y: float = 0.0
    sun_pixels: int = 0
    horizon_valid: bool = False
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    track_deg: float | None = None
    ground_speed: float | None = None
    latitude: float | None = None
    longitude: float | None = None
    utc_hour: float | None = None
    notes: list = field(default_factory=list)


class Camera:
    """Камера Raspberry через picamera2. Низкое разрешение — этого хватает."""

    def __init__(self) -> None:
        from picamera2 import Picamera2  # импорт здесь: на компьютере его нет

        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "YUV420"}
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(1.0)

    def luma(self):
        """Яркость кадра: берём канал Y, цвет для поиска солнца не нужен."""
        frame = self.camera.capture_array("main")
        return frame[:HEIGHT, :WIDTH].reshape(-1).tolist()


class Gps:
    """GPS по NMEA. Нужны только трек по земле, скорость и координаты."""

    def __init__(self, port: str, baud: int) -> None:
        import serial

        self.serial = serial.Serial(port, baud, timeout=0.1)
        self.buffer = b""

    @staticmethod
    def _parse_rmc(parts: list[str]) -> dict:
        def deg(value: str, hemisphere: str) -> float:
            if not value:
                return 0.0
            dot = value.index(".") if "." in value else len(value)
            degrees = float(value[: dot - 2])
            minutes = float(value[dot - 2:])
            result = degrees + minutes / 60.0
            return -result if hemisphere in ("S", "W") else result

        return {
            "track": float(parts[8]) if parts[8] else None,
            "speed_knots": float(parts[7]) if parts[7] else None,
            "latitude": deg(parts[3], parts[4]) if parts[3] else None,
            "longitude": deg(parts[5], parts[6]) if parts[5] else None,
            "utc_hour": (
                float(parts[1][:2]) + float(parts[1][2:4]) / 60.0
                + float(parts[1][4:6]) / 3600.0
                if parts[1]
                else None
            ),
        }

    def poll(self) -> dict | None:
        self.buffer += self.serial.read(4096)
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if not (text.startswith("$GNRMC") or text.startswith("$GPRMC")):
                continue
            parts = text.split(",")
            if len(parts) < 11 or parts[2] != "A":  # A — данные достоверны
                continue
            try:
                return self._parse_rmc(parts)
            except (ValueError, IndexError):
                continue
        return None


class Link:
    """Отправка задания на микроконтроллер."""

    def __init__(self, port: str, baud: int) -> None:
        import serial

        self.serial = serial.Serial(port, baud, timeout=0)

    def send(self, heading_deg: float, valid: bool) -> None:
        flag = "1" if valid else "0"
        self.serial.write(f"H {heading_deg:.1f} {flag}\n".encode("ascii"))


def read_sensors(camera, gps) -> Sensors:
    sensors = Sensors()

    fix = measure_sun(camera.luma(), WIDTH, HEIGHT, FOV_X_DEG, FOV_Y_DEG, SUN_THRESHOLD)
    if fix is not None and fix.valid():
        sensors.sun_valid = True
        sensors.sun_bearing_x = fix.bearing_x
        sensors.sun_bearing_y = fix.bearing_y
        sensors.sun_pixels = fix.pixels
    elif fix is not None:
        sensors.notes.append(f"солнце отбраковано: пикселей {fix.pixels}")

    gps_data = gps.poll() if gps is not None else None
    if gps_data:
        sensors.track_deg = gps_data["track"]
        if gps_data["speed_knots"] is not None:
            sensors.ground_speed = gps_data["speed_knots"] * 0.514444
        sensors.latitude = gps_data["latitude"]
        sensors.longitude = gps_data["longitude"]
        sensors.utc_hour = gps_data["utc_hour"]
    return sensors


def main() -> int:
    camera = None
    gps = None
    link = None
    try:
        camera = Camera()
    except Exception as error:  # noqa: BLE001 — на старте важно сказать, а не упасть
        print(f"камера недоступна: {error}")
    try:
        gps = Gps(LINK_PORT, 9600)
    except Exception as error:  # noqa: BLE001
        print(f"GPS недоступен: {error}")
    try:
        link = Link(LINK_PORT, LINK_BAUD)
    except Exception as error:  # noqa: BLE001
        print(f"связь с микроконтроллером недоступна: {error}")
        return 1

    guidance = CourseGuidance(Tune())
    period = 1.0 / LOOP_HZ
    last_log = 0.0
    print("курс по солнцу: поехали")

    while True:
        started = time.time()
        sensors = read_sensors(camera, gps)

        # Целевой курс — азимут солнца по времени и месту.
        if sensors.latitude is not None and sensors.utc_hour is not None:
            day = time.gmtime()
            azimuth, elevation = sun_azimuth(
                day.tm_year, day.tm_mon, day.tm_mday, sensors.utc_hour,
                sensors.latitude, sensors.longitude,
            )
            target_track = azimuth
            if elevation < 3.0:
                sensors.notes.append(f"солнце низко: {elevation:.1f}°")
        else:
            # Без места и времени азимут солнца неизвестен. Тогда целимся
            # по солнцу в кадре: держим его в центре, а ветер сносит
            target_track = guidance.heading_cmd
            sensors.notes.append("нет GPS: курс только по солнцу в кадре")

        command = guidance.update(period, target_track, sensors.track_deg,
                                  guidance.heading_cmd)
        link.send(command, sensors.sun_valid)

        now = time.time()
        if now - last_log >= LOG_EVERY_S:
            last_log = now
            crab = guidance.crab_angle()
            track_text = (
                f"{sensors.track_deg:.1f}°" if sensors.track_deg is not None else "нет"
            )
            print(
                f"курс {command:6.1f}° | трек {track_text:>7} | "
                f"снос {crab:+5.1f}° | солнце "
                + (f"{sensors.sun_pixels} px" if sensors.sun_valid else "нет")
                + (f" | {'; '.join(sensors.notes)}" if sensors.notes else "")
            )

        spent = time.time() - started
        time.sleep(max(0.0, period - spent))


if __name__ == "__main__":
    raise SystemExit(main())
