#!/usr/bin/env bash
# Install serial-based stable names for the two candleLight adapters.
# Run as root. This does not configure bitrate or bring CAN up.
set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "请以 root 执行: $0" >&2
  exit 1
fi

install -d -m 0755 /etc/systemd/network

cat > /etc/systemd/network/10-nero-can-left.link <<'LINK'
[Match]
OriginalName=*
Property=ID_SERIAL_SHORT=004500404648571620303731

[Link]
NamePolicy=
Name=can_left
LINK

cat > /etc/systemd/network/10-nero-can-right.link <<'LINK'
[Match]
OriginalName=*
Property=ID_SERIAL_SHORT=002200384648570B20303731

[Link]
NamePolicy=
Name=can_right
LINK

chmod 0644 \
  /etc/systemd/network/10-nero-can-left.link \
  /etc/systemd/network/10-nero-can-right.link

udevadm control --reload-rules
echo '已安装稳定命名规则：'
echo '  004500404648571620303731 -> can_left'
echo '  002200384648570B20303731 -> can_right'
echo '设备重新枚举后生效。'
