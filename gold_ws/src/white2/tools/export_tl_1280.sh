#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════
#  신호등 1280 엔진 만들기 — ★실차 PC 에서 한 번 돌린다★  [2026-10-05]
# ══════════════════════════════════════════════════════════════════════════
#  traffic_light 의 기본 가중치(TL_WEIGHTS)가 weights_1280/best.engine 이 됐다.
#  TensorRT 엔진은 ★빌드한 GPU·드라이버에 묶이므로★ 노트북에서 만든 것을 복사해
#  오면 안 되고, 그 기계에서 같은 best.pt 를 imgsz 1280 으로 export 해야 한다.
#
#  안 돌려도 차는 선다 — traffic_light 가 1280 엔진이 없으면 종전 640 엔진으로
#  내려가고 경고를 찍는다. 다만 그때는 원거리 신호 검출률이 절반이다(36% vs 68%).
#
#  쓰는 법 :  bash tools/export_tl_1280.sh [가중치 폴더]
#     기본 폴더 = /home/mad1/runs/detect/combined_light  (traffic_light.py 의 TL_WEIGHTS)
#  ★traffic_light 를 돌리는 그 파이썬(ultralytics·tensorrt 가 있는 환경)에서 실행할 것★
#  10~20분 걸린다(TensorRT 빌드). 기존 weights/ 는 건드리지 않는다.
set -euo pipefail
D="${1:-/home/mad1/runs/detect/combined_light}"
SRC="$D/weights/best.pt"
OUT="$D/weights_1280"

[ -f "$SRC" ] || { echo "❌ $SRC 가 없다 — 가중치 폴더를 인자로 줄 것"; exit 1; }
python3 -c "import ultralytics, tensorrt" 2>/dev/null \
  || { echo "❌ 이 python3 에 ultralytics/tensorrt 가 없다 — traffic_light 가 쓰는 환경을 켜고 다시"; exit 1; }

if [ -f "$OUT/best.engine" ]; then
  echo "ℹ️  $OUT/best.engine 이 이미 있다 — 검사만 한다 (다시 만들려면 지우고 돌릴 것)"
else
  mkdir -p "$OUT"
  cp "$SRC" "$OUT/best.pt"
  echo "⏳ TensorRT export (imgsz 1280, FP32 — 종전 640 엔진과 같은 정밀도) …"
  (cd "$OUT" && python3 -c "from ultralytics import YOLO; YOLO('best.pt').export(format='engine', imgsz=1280, device=0)")
fi

# ── 검사: 메타데이터의 고정 입력이 1280 인가 + 실제로 한 장 추론이 되는가 ──
python3 - "$OUT/best.engine" <<'EOF'
import json, sys, numpy as np
from ultralytics import YOLO
p = sys.argv[1]
with open(p, 'rb') as f:
    n = int.from_bytes(f.read(4), 'little'); meta = json.loads(f.read(n).decode())
sz = meta.get('imgsz'); names = meta.get('names')
print(f"  엔진 고정 입력 = {sz}   클래스 = {names}")
assert (sz[0] if isinstance(sz, list) else sz) == 1280, "imgsz 가 1280 이 아니다"
m = YOLO(p, task='detect')
m.predict(np.zeros((560, 640, 3), np.uint8), imgsz=1280, device=0, verbose=False)
print("✅ 1280 엔진 OK — colcon build 뒤 one_launch 를 띄우면 traffic_light 기동 로그에")
print("   '1280 신호등 엔진이 없다' 경고가 ★안★ 나와야 한다")
EOF
