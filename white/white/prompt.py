#!/usr/bin/env python3
"""
prompt.py - 사용자 명령 인터페이스 노드 (CLI)
기능: 수집(매핑), 주행, 저장된 맵 목록 확인, GPS/IMU 단독 비교주행,
      카메라 헤딩보정 OFF 비교주행, 프로그램 종료 기능 제공

실행: ros2 run white prompt   (one_launch.py 와 별도 터미널)

═══════════════════════════════════════════════════════════════════════════════
 ★★ [2026-08-04] 주행모드(B보드 D5) 강제 ★★
═══════════════════════════════════════════════════════════════════════════════
   · 수집(1)   : ★수동조종 모드에서만★ 가능하다. 사람이 페달·핸들로 차를 몰고 그
                 실계측(페달 환산펄스 / 실 주행펄스 / 실측 조향각)을 기록하기 때문이다.
                 자율주행 모드면 안내만 띄우고 메뉴로 돌아간다.
   · 주행(2,5~8): ★자율주행 모드에서만★ 가능하다. 수동조종 모드에서는 arduino 노드가
                 /cmd_vel_raw 를 아예 무시하고(조향은 힘빼기 'x') 페달값만 넘기므로,
                 명령을 내려도 차가 경로를 따라가지 않는다 — 조용히 실패하는 대신 막는다.
   모드는 물리 스위치(B보드 D5)가 유일한 권한이다. 이 화면에서 바꿀 수단은 없다.
   상태는 nxde arduino 노드가 발행하는 /vehicle_mode 로 실시간 표시된다.

 ★ E-stop 감지 ★ /estop 이 걸린 동안에는 수집·주행을 시작할 수 없고, 메뉴 상단에
   경고가 표시된다. 실제 정지는 아두이노(13번 핀)와 B보드 리니어 2단이 이미 수행한다 —
   여기 표시는 "왜 차가 안 가는지"를 알려주는 용도다.

 ★ 무선 컨트롤러는 더 이상 쓰지 않는다 ★ 수동조종은 차량의 D5 스위치 + 실제 페달·핸들이다.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import String, Bool
import threading
import time
import os
import sys

# [v1.1] GPS/IMU 비교주행 노브 — gps_imu_node의 fusion_mode 파라미터를 전환한다.
# gps_imu.py의 FUSION_MODE 프리셋과 이름을 맞춤.
FUSION_MODE_GPS_ONLY = "gps_course_only"   # 위치+헤딩 전부 GPS(코스헤딩) 단독
FUSION_MODE_IMU_ONLY = "dr_only"           # lock 이후 엔코더+자이로 DR 단독
FUSION_MODE_NORMAL   = "fused"             # 기본 GPS+IMU 융합

class PromptNode(Node):
    def __init__(self):
        super().__init__('prompt_node')
        self.map_pub = self.create_publisher(Bool, '/mapping_cmd', 10)
        self.drive_pub = self.create_publisher(String, '/drive_cmd', 10)
        self.state_pub = self.create_publisher(Bool, '/control_state', 10)
        # mapping.py가 저장하는 폴더와 동일해야 한다(주석 참조).
        self.data_dir = os.path.expanduser("~/white_ws/white_ws/gps_data")
        # 🔧 BUG7 수정: 워커스레드에서 메인루프 종료 신호를 전달하는 이벤트
        self._shutdown_event = threading.Event()
        # [v1.1] gps_imu_node 파라미터 전환용 서비스 클라이언트(비교주행 노브)
        self._fusion_cli = self.create_client(SetParameters, '/gps_imu_node/set_parameters')
        # [추측항법] 시작점(초기 헤딩) 고정 여부 추적 — gps_imu 의 /gps_status "[헤딩고정]" 플래그.
        self._heading_locked   = False
        self._await_lock_notice = False   # DR 주행 시작 후 고정되는 순간 1회 안내
        self.create_subscription(String, '/gps_status', self._cb_gps_status, 10)

        # ── ★주행모드 / E-stop / 보드 연결★ (nxde arduino 노드가 발행) ──
        #   auto_mode : True 자율주행 / False 수동조종 / None 아직 미수신
        #     ※ None 을 '자율'로 가정하지 않는다 — 모드를 모르는 상태에서 주행을 시작하면
        #       수동조종 중인 차에 자율 명령을 쏘는 셈이 되므로, 모르면 막는다.
        self._auto_mode = None
        self._estop = False
        self._board_status = None       # "A:1,B:1,ESTOP:0,MODE:1" 원문
        self.create_subscription(Bool, '/vehicle_mode', self._cb_vehicle_mode, 10)
        self.create_subscription(Bool, '/estop', self._cb_estop, 10)
        self.create_subscription(String, '/board_status', self._cb_board_status, 10)

    # ── [v1.2] 현재 카메라 모드 감지 ────────────────────────────────────
    # use_camera 는 launch 인자라서(one_launch.py) driving_cam(/cmd_vel_drive
    # → camera_judgment 게이트 경유) / driving_nocam(/cmd_vel_raw 직결) 중
    # 하나만 뜬다. prompt 는 그걸 알 방법이 없어 두 경우 메뉴가 똑같이 보였다
    # → "카메라 켜고 주행하는 줄 알았는데 nocam" 을 막기 위해 메뉴에 표시한다.
    def camera_mode_str(self) -> str:
        try:
            names = self.get_node_names()
        except Exception:
            return "❓ 카메라 모드 확인 불가"
        if 'camera_judgment' in names:
            return "🎥 카메라 ON (판단 게이트 경유)"
        return "📡 GPS 단독 (카메라 OFF)"

    # ══════════════════════════════════════════════════════════════════
    #  주행모드(B보드 D5) / E-stop 상태
    # ══════════════════════════════════════════════════════════════════
    def _cb_vehicle_mode(self, msg: Bool):
        new_mode = bool(msg.data)
        if new_mode != self._auto_mode:
            # 모드가 바뀌는 순간을 알려준다 — 메뉴를 다시 그리기 전에도 보이도록.
            print(f"\n🔀 주행모드 전환: {'자율주행' if new_mode else '수동조종'} "
                  f"(B보드 D5 스위치)\n")
        self._auto_mode = new_mode

    def _cb_estop(self, msg: Bool):
        new_estop = bool(msg.data)
        if new_estop and not self._estop:
            print("\n🚨🚨 E-STOP 발동 — 차량 정지 (B보드가 리니어 2단 체결) 🚨🚨\n")
        elif self._estop and not new_estop:
            print("\n✅ E-stop 해제 — 리니어 0단 복귀, 정상 동작 재개\n")
        self._estop = new_estop

    def _cb_board_status(self, msg: String):
        self._board_status = msg.data

    def mode_str(self) -> str:
        if self._auto_mode is None:
            return "❓ 주행모드 확인 불가 (nxde arduino / B보드 연결 확인)"
        return "🤖 자율주행 모드" if self._auto_mode else "🕹️ 수동조종 모드 (페달·핸들)"

    def board_str(self) -> str:
        if not self._board_status:
            return "🔌 보드 상태 미수신 — arduino 노드 확인 (`ros2 run nxde check`)"
        flags = {}
        for field in self._board_status.split(','):
            if ':' in field:
                k, v = field.split(':', 1)
                flags[k.strip()] = v.strip()
        a = "A✅" if flags.get('A') == '1' else "A❌"
        b = "B✅" if flags.get('B') == '1' else "B❌"
        return f"🔌 아두이노 {a} {b}"

    # ── 수집/주행 진입 가능 여부 판정 (공통 게이트) ─────────────────────
    def _require_mode(self, want_auto: bool, action: str) -> bool:
        """want_auto 가 요구 모드다. 진입 가능하면 True, 아니면 안내 후 False.

        ★ E-stop 이 최우선 ★ 발동 중에는 어떤 명령도 아두이노에서 무시되므로, 시작해도
        조용히 아무 일도 일어나지 않는다. 그 혼란을 막기 위해 먼저 막는다."""
        if self._estop:
            print("\n🚨 E-stop 발동 중입니다 — 해제한 뒤 다시 시도하세요.")
            print("   (E-stop 스위치를 원위치하면 500ms 뒤 자동 해제되고, "
                  "B보드가 브레이크를 0단으로 되돌립니다.)\n")
            return False

        if self._auto_mode is None:
            print(f"\n⚠️ 주행모드를 알 수 없어 {action}을 시작할 수 없습니다.")
            print("   B보드(조향·제동) 연결을 확인하세요:")
            print("     · `ros2 launch white one_launch.py` 가 떠 있는지 "
                  "(그 런치가 arduino 노드를 함께 띄웁니다)")
            print("     · 연결 자체가 의심되면: `ros2 run nxde check`")
            print(f"     · 현재 보드 상태: {self.board_str()}")
            print("     · `ros2 topic echo /vehicle_mode` 로 수신 여부 확인\n")
            return False

        if self._auto_mode == want_auto:
            return True

        need = "자율주행" if want_auto else "수동조종"
        now = "자율주행" if self._auto_mode else "수동조종"
        print(f"\n⚠️ {action}은 ★{need} 모드★ 에서만 할 수 있습니다. "
              f"현재는 {now} 모드입니다.")
        print(f"   👉 차량의 주행모드 스위치(B보드 D5)를 ★{need}★ 으로 전환하세요.")
        if want_auto:
            print("      (수동조종 모드에서는 arduino 노드가 경로추종 명령을 무시하고 "
                  "조향을 힘빼기로 두므로, 명령을 내려도 차가 경로를 따라가지 않습니다.)")
        else:
            print("      (수집은 사람이 페달·핸들로 몰 때의 실계측을 기록합니다 — "
                  "자율주행 모드에서는 그 값이 사람의 조작이 아닙니다.)")
        print("   전환하면 이 화면 상단 표시가 바뀝니다. 그 뒤 다시 선택하세요.\n")
        return False

    # ── [추측항법] /gps_status 의 헤딩고정 플래그 추적 ──────────────────
    def _cb_gps_status(self, msg: String):
        locked_now = "[헤딩고정]" in msg.data
        # DR 주행 중 고정되는 '순간'을 한 번만 안내(시작점 확정 = GPS 두절 전환 시점)
        if locked_now and not self._heading_locked and self._await_lock_notice:
            print("\n🔒 시작점 확정(초기 헤딩 고정) → 이후 GPS 두절, 엔코더+IMU+카메라 DR 주행\n")
            self._await_lock_notice = False
        self._heading_locked = locked_now

    def get_input(self):
        print("\n===================================================")
        if self._estop:
            print(" 🚨🚨  E-STOP 발동 중 — 수집·주행 불가  🚨🚨")
            print("---------------------------------------------------")
        print(f" 주행모드: {self.mode_str()}")
        print(f" 카메라  : {self.camera_mode_str()}")
        print(f" 하드웨어: {self.board_str()}")
        print("---------------------------------------------------")
        print(" 1. 수집(매핑) 시작 (Enter로 종료/저장)   [수동조종 모드]")
        print(" 2. 경로 주행 시작 (Enter로 정지)         [자율주행 모드]")
        print(" 3. 저장된 경로 목록 확인")
        print(" 5. [비교실험] GPS 단독 주행 (종료 후 자동 fused 복귀)      [자율주행 모드]")
        print(" 6. [비교실험] IMU 단독(DR) 주행 (종료 후 자동 fused 복귀)  [자율주행 모드]")
        print(" 7. [비교실험] 카메라 헤딩보정 OFF 주행 (종료 후 ON 복귀)   [자율주행 모드]")
        print(" 8. [추측항법] DR+카메라 주행 — 시작점 GPS 확정 후 GPS 두절 [자율주행 모드]")
        print(" 4. 터미널 종료 (Exit)")
        print("===================================================")
        return input("메뉴 선택 (1/2/3/4/5/6/7/8): ").strip()

    # ── [v1.1] gps_imu_node fusion_mode 파라미터 전환 ────────────────────
    # 주의: 워커스레드에서 호출되지만 rclpy.spin은 메인스레드가 돌리고 있으므로
    # 여기서는 call_async 후 future.done()만 폴링한다(직접 spin 호출 금지 —
    # 메인스레드의 spin_once와 동시에 같은 노드를 spin하면 충돌 위험).
    def set_fusion_mode(self, mode: str, timeout_s: float = 3.0) -> bool:
        return self._set_gps_imu_param(
            Parameter(name='fusion_mode',
                      value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                           string_value=mode)),
            label=f"fusion_mode={mode}", timeout_s=timeout_s)

    # ── [v1.2] 카메라 헤딩보정 ON/OFF (gps_imu_node.cam_head_enable) ──────
    def set_cam_head_enable(self, enable: bool, timeout_s: float = 3.0) -> bool:
        return self._set_gps_imu_param(
            Parameter(name='cam_head_enable',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=bool(enable))),
            label=f"cam_head_enable={enable}", timeout_s=timeout_s)

    def _set_gps_imu_param(self, param: Parameter, label: str,
                           timeout_s: float = 3.0) -> bool:
        if not self._fusion_cli.wait_for_service(timeout_sec=timeout_s):
            print(f"⚠️ gps_imu_node 파라미터 서비스 연결 실패 — {label} 전환 안 됨")
            return False
        req = SetParameters.Request()
        req.parameters = [param]
        future = self._fusion_cli.call_async(req)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_s:
                print(f"⚠️ {label} 전환 응답 타임아웃")
                return False
            time.sleep(0.05)
        result = future.result()
        ok = result is not None and all(r.successful for r in result.results)
        if ok:
            print(f"✅ gps_imu_node {label}")
        else:
            print(f"⚠️ {label} 설정 실패(gps_imu 로그 확인)")
        return ok

    def list_routes(self):
        if not os.path.exists(self.data_dir):
            print(f"⚠️ {self.data_dir} 폴더가 없습니다.")
            return []
        
        files = sorted([f for f in os.listdir(self.data_dir) if f.endswith('.csv')])
        if not files:
            print("⚠️ 저장된 경로(.csv) 파일이 없습니다.")
            return []
        
        print("\n📂 [저장된 경로 파일 목록]")
        for i, f in enumerate(files):
            print(f" {i+1}. {f}")
        return files

    # ── 수집(매핑) 흐름 (메뉴 '1') — ★수동조종 모드 전용★ ──────────────────
    def _collect_flow(self):
        """사람이 페달·핸들로 차를 몰고, 그 실계측을 mapping 노드가 CSV 로 기록한다.

        기록되는 수동조종 실계측 3종 (전부 nxde arduino 노드가 발행) :
          ① /drive_pulse_cmd       페달 raw → 환산된 주행 목표펄스 (0~15)
          ② /encoder               실제로 돈 주행 펄스 (좌+우 합) + m/s 환산
          ③ /steer_angle_measured  DC 조향모터 가변저항 실측 각도 [deg]
        무선 컨트롤러는 쓰지 않는다 — 차량의 D5 스위치와 실제 페달·핸들이 조작 수단이다."""
        if not self._require_mode(want_auto=False, action="수집(매핑)"):
            return

        # ★ 자율 주행권한을 확실히 내려둔다 ★ 직전 주행이 비정상 종료되어
        #   /control_state=True 가 남아 있으면, 수동조종으로 되돌아온 순간 arduino 노드가
        #   자율 경로로 오해할 여지를 없앤다(수동 분기가 우선이라 실제 위험은 없지만,
        #   기록에 auto 흔적이 남지 않도록 상태를 깨끗하게 만든다).
        self.state_pub.publish(Bool(data=False))

        print("\n🗺️ 수집(매핑)을 시작합니다 — ★페달과 핸들로 직접 주행하세요★")
        print("   · 조향 DC모터는 힘이 빠져 있어 핸들이 손으로 돌아갑니다.")
        print("   · 브레이크는 수동조종 진입 시 2단으로 잠겨 있고, 페달을 밟으면 풀립니다.")
        print("   · 기록: 페달 환산펄스 / 실 주행펄스 / 실측 조향각 + GPS·IMU·차선")
        self.map_pub.publish(Bool(data=True))

        input("🛑 수집을 종료하고 저장하려면 [Enter] 키를 누르세요...\n")

        self.map_pub.publish(Bool(data=False))
        print("✅ 수집 종료 및 저장 완료! (mapping 터미널의 '수집 유효성' 로그를 확인하세요)")
        if self._auto_mode:
            print("⚠️ 수집 도중 자율주행 모드로 전환되었습니다 — 그 구간 행은 "
                  "auto_mode=1 로 기록되어 있으니 분석에서 걸러 쓰세요.")

    # ── [v1.1] 경로 주행 흐름 (기존 메뉴 '2') — 비교주행에서도 재사용 ────────
    #   ★ 모드 게이트는 여기 한 곳에 둔다 ★ 2·5·6·7·8 이 전부 이 함수(또는
    #     _drive_flow_with_lock_notice)를 타므로, 메뉴마다 검사를 흩뿌리지 않는다.
    def _drive_flow(self):
        if not self._require_mode(want_auto=True, action="경로 주행"):
            return
        files = self.list_routes()
        if not files:
            return

        try:
            f_idx = int(input("\n주행할 파일 번호: ")) - 1
            if 0 <= f_idx < len(files):
                target_file = files[f_idx]

                # 🌟 [수정 포인트 2] 주행 시작 시 모터 제어 권한(True) 부여
                self.state_pub.publish(Bool(data=True))
                self.drive_pub.publish(String(data=target_file))
                print(f"\n🚀 자율주행 시작: {target_file}")

                input("🛑 주행을 긴급 정지하려면 [Enter] 키를 누르세요...\n")

                # 🌟 [수정 포인트 3] 긴급 정지 시 모터 제어 권한(False) 즉각 박탈
                self.state_pub.publish(Bool(data=False))
                self.drive_pub.publish(String(data="STOP"))
                print("🛑 긴급 정지 명령 및 모터 차단 완료!")
            else:
                print("⚠️ 잘못된 번호입니다.")
        except ValueError:
            print("⚠️ 숫자를 입력하세요.")

    # ── [v1.1] GPS 단독 / IMU 단독 비교주행 ─────────────────────────────
    # gps_imu_node의 fusion_mode 파라미터를 전환한 뒤 동일한 주행 흐름을 태우고,
    # 주행 종료(또는 실패) 후 반드시 fused로 복귀시켜 다음 일반 주행에 영향이
    # 남지 않도록 한다.
    def _compare_drive_flow(self, mode: str, label: str):
        # 모드를 먼저 확인한다 — 통과하지 못할 주행을 위해 fusion_mode 를 건드리지 않는다.
        if not self._require_mode(want_auto=True, action=f"{label} 비교주행"):
            return
        print(f"\n🔬 [{label} 비교주행] gps_imu_node fusion_mode → {mode} 전환 시도...")
        if not self.set_fusion_mode(mode):
            print(f"⚠️ {label} 모드 전환 실패 — 안전을 위해 이번 비교주행은 취소합니다.")
            return
        print(f"   (참고: gps_imu 터미널 로그의 '[GPS-IMU ... 융합설정] mode={mode}' "
              f"확인 후 주행 시작 권장)")
        try:
            self._drive_flow()
        finally:
            reverted = self.set_fusion_mode(FUSION_MODE_NORMAL)
            if reverted:
                print(f"↩️  fusion_mode → {FUSION_MODE_NORMAL} 복귀 완료")
            else:
                print(f"🚨 fusion_mode fused 복귀 실패! 다음 주행 전 수동 확인 필요: "
                      f"ros2 param set /gps_imu_node fusion_mode {FUSION_MODE_NORMAL}")

    # ── [v1.2] 카메라 헤딩보정 OFF 비교주행 ─────────────────────────────
    # gps_imu 의 CAM_HEAD_ENABLE 을 끄고 같은 경로를 주행해, 카메라 헤딩보정이
    # 실제로 도움이 되는지(그리고 CAM_HEAD_SIGN 부호가 맞는지) A/B 로 본다.
    # 5/6 과 같은 이유로 finally 복귀가 필수 — OFF 인 채 다음 주행에 넘어가면
    # 평소 주행이 조용히 다른 설정으로 돌게 된다.
    def _cam_head_off_drive_flow(self):
        if not self._require_mode(want_auto=True, action="카메라 헤딩보정 OFF 비교주행"):
            return
        if 'camera_judgment' not in self.get_node_names():
            print("⚠️ 카메라 노드가 없습니다(use_camera:=false로 기동된 듯). "
                  "카메라 헤딩보정은 원래 꺼진 상태라 비교 의미가 없습니다.")
            return
        print("\n🔬 [카메라 헤딩보정 OFF 비교주행] cam_head_enable → False 전환 시도...")
        if not self.set_cam_head_enable(False):
            print("⚠️ 전환 실패 — 안전을 위해 이번 비교주행은 취소합니다.")
            return
        print("   (참고: gps_imu 터미널의 '[카메라 융합] CAM_HEAD_ENABLE 전환' 로그 확인 후 시작 권장)")
        try:
            self._drive_flow()
        finally:
            if self.set_cam_head_enable(True):
                print("↩️  cam_head_enable → True 복귀 완료")
            else:
                print("🚨 cam_head_enable 복귀 실패! 다음 주행 전 수동 확인 필요: "
                      "ros2 param set /gps_imu_node cam_head_enable true")

    # ── [추측항법] DR+카메라 주행 — 시작점 GPS 확정 후 GPS 두절 ─────────────
    # dr_only(α=0) 규약: 초기 헤딩 고정 전에는 GPS 로 원점·초기헤딩(=시작점)을 확립하고,
    # 고정된 순간부터 순수 DR(엔코더+자이로) 로 전환한다(GPS 미개입). 여기에 카메라
    # 헤딩보정을 강제로 켜서, DR 중 쌓이는 IMU 헤딩 드리프트를 차선으로 잡는다.
    # → "GPS 로 시작점만 잡고, 이후 GPS 두절, 엔코더+IMU+카메라 로만 주행".
    def _dr_camera_drive_flow(self):
        if not self._require_mode(want_auto=True, action="추측항법(DR+카메라) 주행"):
            return
        if 'camera_judgment' not in self.get_node_names():
            print("⚠️ 카메라 노드가 없습니다(use_camera:=false 로 기동된 듯). "
                  "이 모드는 카메라 헤딩보정이 필수라 취소합니다.")
            return
        print("\n🧭 [추측항법(DR+카메라) 주행]")
        print("   카메라 헤딩보정 ON + fusion_mode → dr_only(α=0) 전환 시도...")
        if not self.set_cam_head_enable(True):
            print("⚠️ 카메라 헤딩보정 ON 실패 — 안전을 위해 취소합니다.")
            return
        if not self.set_fusion_mode(FUSION_MODE_IMU_ONLY):
            print("⚠️ dr_only 전환 실패 — 안전을 위해 취소합니다.")
            # 카메라만 켜졌으니 원복은 불필요(ON 이 기본값)
            return
        print("   시작점(원점·초기 헤딩)은 GPS 로 확정되고, 고정된 뒤 GPS 는 두절됩니다.")
        print("   이후 헤딩은 IMU 자이로 + 카메라 차선보정, 위치는 엔코더 DR 로만 갑니다.")
        try:
            self._drive_flow_with_lock_notice()
        finally:
            if self.set_fusion_mode(FUSION_MODE_NORMAL):
                print(f"↩️  fusion_mode → {FUSION_MODE_NORMAL} 복귀 완료")
            else:
                print(f"🚨 fusion_mode fused 복귀 실패! 다음 주행 전 수동 확인 필요: "
                      f"ros2 param set /gps_imu_node fusion_mode {FUSION_MODE_NORMAL}")

    # 기존 _drive_flow 와 동일하되, 시작점(헤딩) 고정 순간을 콜백으로 1회 안내한다.
    def _drive_flow_with_lock_notice(self):
        if not self._require_mode(want_auto=True, action="추측항법 주행"):
            return
        files = self.list_routes()
        if not files:
            return
        try:
            f_idx = int(input("\n주행할 파일 번호: ")) - 1
        except ValueError:
            print("⚠️ 숫자를 입력하세요.")
            return
        if not (0 <= f_idx < len(files)):
            print("⚠️ 잘못된 번호입니다.")
            return
        target_file = files[f_idx]

        if self._heading_locked:
            print("✅ 이미 헤딩 고정 상태(시작점 확정) — 바로 DR 주행합니다.")
        else:
            print("🚀 GPS 로 시작점(초기 헤딩) 확정 중... 차량이 1~2m 전진하면 고정됩니다.")
            self._await_lock_notice = True

        self.state_pub.publish(Bool(data=True))
        self.drive_pub.publish(String(data=target_file))
        print(f"🚀 자율주행 시작: {target_file}")

        input("🛑 주행을 긴급 정지하려면 [Enter] 키를 누르세요...\n")

        self._await_lock_notice = False
        self.state_pub.publish(Bool(data=False))
        self.drive_pub.publish(String(data="STOP"))
        print("🛑 긴급 정지 명령 및 모터 차단 완료!")

    def run(self):
        while rclpy.ok():
            choice = self.get_input()

            if choice == '1':
                self._collect_flow()

            elif choice == '2':
                self._drive_flow()

            elif choice == '3':
                self.list_routes()

            elif choice == '5':
                self._compare_drive_flow(FUSION_MODE_GPS_ONLY, "GPS 단독")

            elif choice == '6':
                self._compare_drive_flow(FUSION_MODE_IMU_ONLY, "IMU 단독(DR)")

            elif choice == '7':
                self._cam_head_off_drive_flow()

            elif choice == '8':
                self._dr_camera_drive_flow()

            elif choice == '4':
                print("\n👋 터미널을 종료합니다.")
                # 🔧 BUG7 수정: 워커스레드에서 직접 rclpy.shutdown()+sys.exit() 호출하면
                # spin() 중인 메인스레드와 충돌 → 이벤트로 신호만 보내고 메인이 처리
                self._shutdown_event.set()
                return

            else:
                print("⚠️ 1~8 사이의 숫자를 입력해주세요.")

def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()
    
    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()

    try:
        # 🔧 BUG7 수정: spin 대신 주기적으로 shutdown_event를 확인하며 spin_once
        while rclpy.ok() and not node._shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.state_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()