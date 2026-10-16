#!/bin/bash
# setup_dds_buffer.sh — ★커널 UDP 수신 버퍼 상한을 올린다★ [2026-10-16, nxde]
#
#   sudo bash gold_ws/src/nxde/tools/setup_dds_buffer.sh          적용 (기계마다 한 번)
#   bash gold_ws/src/nxde/tools/setup_dds_buffer.sh --check       지금 값만 본다
#
# 왜 : CycloneDDS 가 1080p 영상(4.1 MB)을 받으려면 수신 버퍼가 커야 하는데, 우분투 기본
#      net.core.rmem_max 는 212992(208 KB)다. 그러면 카메라 노드가 프레임을 받았다 못 받았다
#      한다(traffic_light '⏳ /image_raw 수신 대기', 인지 디버그 mp4 가 안 생김).
#      값은 ROS 2 공식 문서(DDS tuning)의 CycloneDDS 권장값이다. 이것은 '상한' 이라 실제
#      메모리는 요청한 소켓만 쓴다 — 카메라 노드는 nxde/config/cyclonedds.xml 로 32 MB 를 요청한다.
#
# 무엇을 : /etc/sysctl.d/60-gold-dds.conf 를 쓰고 바로 적용한다(재부팅 뒤에도 남는다).
# 되돌리기 : sudo rm /etc/sysctl.d/60-gold-dds.conf && sudo sysctl -w net.core.rmem_max=212992
set -e
CONF=/etc/sysctl.d/60-gold-dds.conf
WANT=2147483647
NOW=$(sysctl -n net.core.rmem_max)
echo "지금 net.core.rmem_max = $NOW  (권장 $WANT)"
[ "${1:-}" = "--check" ] && exit 0
if [ "$(id -u)" -ne 0 ]; then
  echo "⛔ root 권한이 필요하다 — sudo bash $0" >&2
  exit 1
fi
cat > "$CONF" <<EOF
# gold (kasa 금색차) — CycloneDDS 가 1080p 영상을 받을 수 있게 [nxde/tools/setup_dds_buffer.sh]
net.core.rmem_max = $WANT
EOF
sysctl -p "$CONF"
echo "✅ 적용했다 — 이제 net.core.rmem_max = $(sysctl -n net.core.rmem_max)"
echo "   ★이미 떠 있는 ROS 노드에는 효력이 없다★ — 런치를 다시 띄울 것"
