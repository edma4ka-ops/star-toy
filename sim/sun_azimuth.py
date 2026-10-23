# -*- coding: utf-8 -*-
"""Азимут и высота солнца для выставления направляющей перед пуском.

Никакого управления в полёте: считаем, где солнце, чтобы по нему
(или по тени) выставить направляющую на нужный пусковой азимут.

    python sim/sun_azimuth.py                     # Сочи, сегодня, сейчас
    python sim/sun_azimuth.py --lat 43.6 --lon 39.7 --date 2026-11-26 --time 12:30
    python sim/sun_azimuth.py --bearing 90        # куда смотреть тени от колышка

Алгоритм — упрощённый NOAA (точность порядка 0.1°), хватает для выставления
направляющей и планирования зоны приземления.
"""
import argparse
import math
from datetime import datetime, timezone

DEG = math.pi / 180.0


def sun_position(lat, lon, when_utc):
    """Возвращает (азимут от севера по часовой, высота над горизонтом) в градусах."""
    # день года и часы
    doy = when_utc.timetuple().tm_yday
    hour = when_utc.hour + when_utc.minute / 60.0 + when_utc.second / 3600.0

    # дробный год (радианы)
    g = 2.0 * math.pi / 365.0 * (doy - 1 + (hour - 12) / 24.0)
    # уравнение времени, минут
    eqt = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g)
                    - 0.014615 * math.cos(2 * g) - 0.040849 * math.sin(2 * g))
    # склонение солнца, радианы
    decl = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g)
            - 0.006758 * math.cos(2 * g) + 0.000907 * math.sin(2 * g)
            - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    # истинное солнечное время, минуты
    tst = hour * 60.0 + eqt + 4.0 * lon
    ha = (tst / 4.0 - 180.0) * DEG          # часовой угол

    lat_r = lat * DEG
    # зенитный угол
    cos_zen = (math.sin(lat_r) * math.sin(decl)
               + math.cos(lat_r) * math.cos(decl) * math.cos(ha))
    cos_zen = max(-1.0, min(1.0, cos_zen))
    zenith = math.acos(cos_zen)
    elevation = 90.0 - zenith / DEG

    # азимут: считаем от юга (положительный — к западу), затем переводим на север
    az_south = math.atan2(math.sin(ha),
                          math.cos(ha) * math.sin(lat_r) - math.tan(decl) * math.cos(lat_r))
    azimuth = (math.degrees(az_south) + 180.0) % 360.0
    return azimuth, elevation


def main():
    ap = argparse.ArgumentParser(description="Азимут солнца и выставление направляющей")
    ap.add_argument("--lat", type=float, default=43.5855, help="широта площадки, °")
    ap.add_argument("--lon", type=float, default=39.7231, help="долгота площадки, °")
    ap.add_argument("--date", default=datetime.now().strftime("%Y-%m-%d"))
    ap.add_argument("--time", default=datetime.now().strftime("%H:%M"), help="местное время, ЧЧ:ММ")
    ap.add_argument("--tz", type=float, default=3.0, help="смещение часового пояса от UTC, ч")
    ap.add_argument("--bearing", type=float, default=None, help="желаемый пусковой азимут, °")
    ap.add_argument("--pin", type=float, default=1.0, help="высота колышка для проверки тенью, м")
    args = ap.parse_args()

    dt_local = datetime.strptime(f"{args.date} {args.time}", "%Y-%m-%d %H:%M")
    dt_utc = dt_local.replace(tzinfo=timezone.utc) - \
        __import__("datetime").timedelta(hours=args.tz)

    az, el = sun_position(args.lat, args.lon, dt_utc)

    print(f"Площадка: {args.lat:.4f}°, {args.lon:.4f}°")
    print(f"Время:    {dt_local:%d.%m.%Y %H:%M} (UTC{args.tz:+.0f})")
    print(f"Солнце:   азимут {az:6.1f}°  высота {el:5.1f}°")
    if el < 0:
        print("Солнце ниже горизонта — тенью выставляться нечем, берите компас.")
        return

    shadow_len = args.pin / math.tan(el * DEG)
    shadow_az = (az + 180.0) % 360.0
    print(f"Тень колышка {args.pin:.2f} м: длина {shadow_len:5.2f} м, "
          f"направление {shadow_az:5.1f}° (от колышка)")

    if args.bearing is not None:
        diff = (args.bearing - shadow_az + 540.0) % 360.0 - 180.0
        print()
        print(f"Пусковой азимут {args.bearing:5.1f}°")
        print(f"Угол между осью направляющей и тенью: {diff:+.1f}°")
        print("Тень должна лежать справа от направляющей" if diff < 0
              else "Тень должна лежать слева от направляющей", f"на {abs(diff):.1f}°")

    # предупреждение о слепящем солнце в зоне пуска
    if el < 15:
        print("\nСолнце низко: тени длинные, но и слепит сильнее — учтите при выборе площадки.")
    print("\nПроверка: тень от вертикального колышка должна совпадать по направлению")
    print("с расчётной. Расхождение больше 5° — ошибка в координатах, времени или компасе.")


if __name__ == "__main__":
    main()
