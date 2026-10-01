#!/usr/bin/env bash
# Установка страниц игрушки «Звёздный курс» на сервер.
#
# Запускается на сервере скриптом deploy-site.ps1 с локальной машины.
# Кладёт страницы и архивы в /opt/star-toy и проверяет, что nginx их отдаёт.
#
# Отдельно от прибора намеренно: развёртывание прибора распаковывает свой
# архив в /opt/pooldesk и эту папку не трогает. Врезка location /toy/ живёт
# в конфиге nginx прибора, потому что он ставится из репозитория и переживает
# развёртывания.

set -euo pipefail

APP_DIR=/opt/star-toy
ARCHIVE=/tmp/star-toy-site.tar.gz
BASE_URL="http://127.0.0.1/toy/"

log() { printf '[toy] %s\n' "$*"; }

if [ ! -f "$ARCHIVE" ]; then
  log "ОШИБКА: архив $ARCHIVE не найден"
  exit 1
fi

log "распаковываю страницы в $APP_DIR"
mkdir -p "$APP_DIR"
# Старое содержимое убираем, чтобы не остались файлы от прошлых сборок
rm -rf "${APP_DIR:?}/"*
tar -xzf "$ARCHIVE" -C "$APP_DIR"

chown -R root:root "$APP_DIR"
find "$APP_DIR" -type d -exec chmod 755 {} \;
find "$APP_DIR" -type f -exec chmod 644 {} \;

log "файлов: $(find "$APP_DIR" -type f | wc -l), объём: $(du -sh "$APP_DIR" | cut -f1)"

# Врезка location /toy/ ставится конфигом прибора. Если её нет — говорим прямо,
# а не молча отдаём 404
if ! grep -q "alias /opt/star-toy/" /etc/nginx/sites-available/pooldesk.conf 2>/dev/null; then
  log "ОШИБКА: в конфиге nginx нет врезки location /toy/."
  log "Сначала разверните прибор: powershell -File deploy\\deploy.ps1"
  exit 1
fi

log "проверяю конфигурацию nginx"
if ! nginx -t 2>&1; then
  log "ОШИБКА: конфигурация nginx не принята"
  exit 1
fi
systemctl reload nginx

log "проверяю страницы"
fail=0
for path in "" "programs.html" "style.css" "downloads/star-toy-all.zip"; do
  code=$(curl -s -o /dev/null -w '%{http_code}' "${BASE_URL}${path}")
  size=$(curl -s -o /dev/null -w '%{size_download}' "${BASE_URL}${path}")
  log "  ${BASE_URL}${path} -> HTTP ${code}, ${size} байт"
  if [ "$code" != "200" ]; then
    fail=1
  fi
done

if [ "$fail" != "0" ]; then
  log "ОШИБКА: не все страницы отдаются"
  exit 1
fi

log "готово: http://194.87.200.210/toy/"
