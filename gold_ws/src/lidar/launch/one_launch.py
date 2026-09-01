#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""one_launch.py — lidar 통합 런치 [AEB 수동 / 라바콘 헤딩홀드 자율]

    ros2 launch lidar one_launch.py                         # ★기본★ AEB 수동시험
    ros2 launch lidar one_launch.py use_rviz:=true
    ros2 launch lidar one_launch.py manual_pwm_max:=90      # 페달 상한을 4펄스급으로
    ros2 launch lidar one_launch.py cone_drive:=true        # 라바콘 복도 잠금 → 스스로 직진
    ros2 launch lidar one_launch.py cone_drive:=true drive:=false use_rviz:=true  # 책상

════════════════════════════════════════════════════════════════════════════════
 무엇을 시험하는 런치인가
════════════════════════════════════════════════════════════════════════════════
  ★기본은 주행을 사람이 페달로 한다★ (B보드 D5 = 수동조종). 자율주행 노드를
  띄우지 않는다 — 차를 스스로 출발시키는 것이 없다. AEB 만 시험한다.
  ★cone_drive:=true 면 예외★ 양옆 라바콘으로 복도를 잠그고 스스로 직진한다.
  그 페달은 ★white1 과 같은 직접 PWM 16~255★ 로 A보드에 내려간다. 끝까지 밟으면
  PWM 255(프로토콜 상한)가 나간다. 펄스 PID 는 펌웨어 PWM_MAX=170 에 묶여 풀
  엑셀이어도 최대 듀티가 안 나왔다.
  ★[2026-08-26] 페달 0점은 throttle_raw_min=220 이다★ 휴지 raw 196~208 은 지령 0.
  ★[2026-08-27] 페달 100점은 throttle_raw_max=950 이다★ 옛 800 은 살짝 밟아도
  개도 1.0 → /drive_pulse_cmd=15 가 되어 엑셀로 속도를 나눌 수 없었다.
  그 상태에서 라이다가 ★상시 감시★ 하다가 전방 장애물을 확정하면, 차가 스스로
  구동을 끊고 리니어를 물어 선다. 그 사슬 하나만 시험하는 런치다.

  속도를 더 묶으려면 manual_pwm_max 를 낮춘다 (예: 90 ≈ 4펄스 ≈ 12.7 km/h).
  목표펄스 PID 가 필요하면 manual_use_pwm:=false 로 되돌린다(풀 엑셀 PWM 255 는
  그때 나오지 않는다 — 보드 PWM_MAX=170).

     ouster 드라이버 ─/ouster/points─▶ cone_lidar_node ─/…/stop_signal─▶
       pedal_drive_node ─/aeb_stop─▶ arduino ─▶ A보드 구동차단 + B보드 리니어 2단
                              └────────────▶ sound  (경고음 반복)

  ★기본(cone_drive:=false) 은 drive_lidar_node 를 띄우지 않는다★ 그쪽은 라바콘
  복도를 스스로 출발해 7펄스로 달리는 자율주행 노드다(런치 = 출발). 비상정지만
  보려면 차를 움직이는 주체가 사람이어야 한다. 그래서 기본의 '구동 담당'은
  pedal_drive_node 이고, 그 노드는 ★/cmd_vel_raw·/control_state·/brake_level 을
  하나도 발행하지 않는다★.

  ★cone_drive:=true 는 라바콘 헤딩홀드 자율이다★
     정지 관측(기본 2.5초) → 양옆 라바콘으로 복도 직선/헤딩 잠금 → 7펄스 직진.
     헤딩은 외장 iAHRS(/imu 쿼터니언). 전방 AEB 는 그대로 산다.
     D5 는 ★자율주행★, E-STOP 풀림. 세우는 수단: Ctrl-C · E-STOP · AEB.

════════════════════════════════════════════════════════════════════════════════
 띄우는 것
════════════════════════════════════════════════════════════════════════════════
    nxde/sound            음성 안내 (구독 전용)      ← ★제일 먼저★ use_sound
    nxde/arduino          A/B 2보드 시리얼 브리지                use_arduino
    ouster.launch.py      OS1-32 드라이버 (통째로 include)
    cone_lidar_node       가상범퍼 AEB 판정 (aeb.launch.py 를 통째로 include)
    rviz2                 ROI 표시                               use_rviz
    lidar/pedal_drive_node ★확정·래치 → /aeb_stop★ (속도는 판정하지 않는다)
    white1/iahrs          외장 iAHRS → /imu                       cone_drive
    lidar/drive_lidar_node 라바콘 복도 잠금 + 헤딩홀드 직진        cone_drive
    white1/hud            차량 상면도 HUD (구독 전용)              use_hud

  ※ ouster·cone_lidar·rviz 는 ★aeb.launch.py 를 그대로 include★ 한다. 같은 노드를
    두 곳에서 선언하면 파라미터가 갈라진다 — 조각의 주인을 그 파일에 남겨 둔다.

════════════════════════════════════════════════════════════════════════════════
 음성 안내 (white1 one_launch.py 와 같은 규칙 + AEB 두 개)
════════════════════════════════════════════════════════════════════════════════
    one_launch_1   런치한 직후 (sound 노드가 뜨는 순간)
    one_launch_2   /board_status 가 A:1,B:1 이 되는 순간 = ★아두이노 연결됨★
    estop          ★/aeb_stop 이 True 인 동안 반복재생★  [2026-08-25 신설]
    estop_re       /aeb_stop 이 False 로 떨어진 순간 (해제음)
  음원은 white1 이 소유한다(white1/sound/*.mp3). 이 런치는 그 경로를 찾아 넘긴다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 시험 절차 ★★
════════════════════════════════════════════════════════════════════════════════
  ① 차 앞을 비우고 런치한다. one_launch_1 이 들리고, 보드가 붙으면 one_launch_2.
  ② D5 스위치는 ★수동조종★ 에 둔다 (이 시험의 기본).
  ③ 로그의 "감시 중 — 전방 inf" 를 확인한다. 여기까지가 인지 확인이다.
     ★차 앞 3 m 에 사람이 서서 전방 거리가 3 m 로 나오는지 먼저 볼 것★
     (안 나오면 flip_lidar_xy — aeb.launch.py 헤더 참고. 인지가 먼저다)
  ④ 페달로 천천히 출발한다. 전방에 장애물이 들어오면
       · 로그   "🛑 ★AEB 정지★ 전방 x.xx m"
       · 소리   경고음이 ★해제될 때까지 반복★
       · 차     구동이 끊기고 리니어가 물린다 (페달을 밟고 있어도)
  ⑤ 앞을 비우면 release_clear_s(1.5초) 뒤에 풀리고 해제음이 난다. 다시 페달로 간다.
  ⑥ 로그에 "★AEB 가 못 세우는 속도★" 가 뜨면 그 속도에서는 ★정말로 못 선다★.
     페달을 조금 놓거나 manual_pwm_max 를 낮춰서 다시 한다.
  ⚠️ ★사람으로 시험할 때는 라바콘·박스를 먼저 쓸 것★ roi_agl_max=0.75 라 사람
     상체는 슬랩 위에 있고, 서 있는 자세·거리에 따라 검출 점수가 달라진다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 라바콘 헤딩홀드 (cone_drive:=true) ★★
════════════════════════════════════════════════════════════════════════════════
  ① 차 앞을 비우고, 양옆에 라바콘 열을 둔다 (한쪽 최소 2개, 전후 간격 ≳ 3 m,
     차 중심에서 옆으로 1~4 m). 런치 = 출발이므로 사람이 앞에 서지 말 것.
  ② D5 스위치는 ★자율주행★, E-STOP 은 풀어 둔다.
  ③ 로그에 이 줄이 나와야 한다:
       헤딩 IMU = '/imu'  imu_use_orientation=true
       🛑 정지 관측 2.5s — 양쪽 라바콘 누적 중
       🔒 복도 잠금: heading=… L=… R=… 헤딩=AHRS 쿼터니언
       AHRS 헤딩 잠금 yaw0=…
  ④ 잠기면 1펄스로 차 코를 헤딩에 맞춘다. 맞으면 조향을 0에 고정하고
     7펄스(≈22.3 km/h)로 직진만 한다. 전방 장애물이 확정되면 AEB.
  ⑤ 6초 안에 양옆이 안 잡히면 ❌ 관측 실패 — 정지 유지. 그때는 콘 간격·
     flip_lidar_xy·lane_band(1~4 m)를 본다. 한쪽만으로 가려면
     require_dual_side:=false.

════════════════════════════════════════════════════════════════════════════════
 ★★ 빌드 — 이 패키지에 --symlink-install 을 쓰지 않는다 ★★
════════════════════════════════════════════════════════════════════════════════
    colcon build --packages-select lidar \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    source ~/gold/gold_ws/install/setup.bash
  (C++ 이 들어 있어 심볼릭 설치본이 build/ 를 가리키게 되고, 그 상태로 build/ 를
   지우면 install/ 이 조용히 깨진다)
"""

import os
from glob import glob

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# 파이썬 stdout 버퍼링을 끄지 않으면 노드 로그가 뭉쳐서 늦게 나온다
NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}


def _sound_dir():
    """음성 안내 mp3 폴더. ★음원의 주인은 white1 이다★ (그쪽 paths.py 헤더 참고)

    ① white1 의 share — setup.py 가 mp3 를 복사해 두므로 ★심볼릭 설치 여부와
       무관하게★ 있다. 이 패키지는 --symlink-install 을 쓰지 않기로 했으므로
       소스트리 되찾기(realpath)에 의존하지 않는 이쪽을 먼저 본다.
    ② white1 소스트리 — white1 을 --symlink-install 로 빌드한 경우.
    ③ 못 찾으면 빈 문자열 → sound 노드가 자기 기본값(<nxde>/sound)을 본다.
       그쪽은 비어 있어서 '음원 없음' 경고만 한 번 나고 노드는 계속 돈다.
    ※ .gitignore 가 *.mp3 를 막으므로 새로 clone 하면 음원이 아예 없다. 그때는
      `ros2 run nxde tts` 로 다시 만든다 — 없어도 시험 자체는 된다.
    """
    try:
        share = os.path.join(get_package_share_directory('white1'), 'sound')
        if glob(os.path.join(share, '*.mp3')):
            return share
    except Exception:                      # noqa: BLE001 — white1 이 없어도 진행
        pass
    try:
        from white1 import paths          # noqa: PLC0415 — 있을 때만 쓴다
        src = paths.sound_dir()
        if glob(os.path.join(src, '*.mp3')):
            return src
    except Exception:                      # noqa: BLE001
        pass
    return ''


def _cone_params(path):
    """cone_lidar.yaml 의 ros__parameters 를 dict 로 읽는다. 실패하면 빈 dict.

    ★왜 런치가 YAML 을 직접 읽는가★ 감지 기하(거리·코리도·확정 프레임)는
    cone_lidar_node 가 이 파일을 파라미터로 받아 쓰지만(aeb.launch.py), 그 값이
    화면에는 한 줄도 안 나온다. AEB 시험에서 제일 먼저 알아야 하는 것이
    ★지금 몇 미터로 보고 있는가★ 인데, YAML 을 따로 열어 봐야 알 수 있었다.
    그래서 런치가 같은 파일을 읽어 ★기동 화면에 그대로 남긴다★ (출력 전용 —
    이 값으로 무엇을 계산하거나 제한하지 않는다).

    읽기에 실패해도 런치를 죽이지 않는다 — 인지 자체는 cone_lidar_node 가 같은
    파일을 직접 읽으므로 정상 동작하고, 화면에 그 사실만 남긴다.
    """
    try:
        with open(path, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        params = (doc.get('cone_lidar_node') or {}).get('ros__parameters') or {}
        if not params:
            print(f" ⚠️  {path} 에 cone_lidar_node/ros__parameters 가 없습니다 "
                  f"— 감지 설정을 화면에 보여줄 수 없습니다(동작에는 영향 없음)")
        return params
    except Exception as exc:               # noqa: BLE001 — 런치를 죽이지 않는다
        print(f" ⚠️  {path} 를 읽지 못했습니다 ({exc}) "
              f"— 감지 설정을 화면에 보여줄 수 없습니다(동작에는 영향 없음)")
        return {}


def _pedal_drive_node(context, *_args, **_kwargs):
    """[구동 담당] pedal_drive_node — ★cone_lidar.yaml 의 감지 설정을 화면에 남긴다★

    OpaqueFunction 으로 감싼 이유는 하나다: cone_params_file 이
    LaunchConfiguration 이라 ★런치 시점에야 실제 경로가 정해진다★
    (`cone_params_file:=...` 로 다른 YAML 을 줄 수 있다). 그래서 '지금
    cone_lidar_node 가 실제로 쓰는 그 파일' 의 값을 그대로 찍을 수 있다.

    ★[2026-08-25] 여기서 읽은 값을 pedal_drive_node 에 넘기지 않는다★
      한때 감지 거리·앞범퍼로 '설 수 있는 속도 상한' 을 계산해 넘겼는데,
      그 숫자가 로그에 남는 것 자체가 '무언가 걸려 있다' 로 읽혔다. 실제로
      제한한 적은 없지만 ★없는 상한을 읽게 만드는 로그는 없는 편이 낫다★.
      → 지금은 순수 출력이다. 감지 기하는 cone_lidar_node 가 같은 파일을 직접
        읽어 쓰고(aeb.launch.py), 우리는 '무슨 값으로 도는지' 만 화면에 남긴다.

    ★구동 지령을 만들지 않는다★ (/cmd_vel_raw·/control_state·/brake_level 을
    발행하지 않는다 — 그 노드 헤더의 '발행하지 않는 것' 참고). 주행은 사람의
    페달이고, 이 노드는 그 위에 AEB 하나만 얹는다.
    """
    cfg = LaunchConfiguration('cone_params_file').perform(context)
    p = _cone_params(cfg)

    print(f" 📄 cone_lidar 감지 설정 : {cfg}")
    for key, label in (('roi_x_min',              '감지 하한        '),
                       ('stop_distance_threshold','감지 상한        '),
                       ('vehicle_front_m',        '앞범퍼(원점→앞끝)'),
                       ('path_corridor_half_width', '코리도 반폭    '),
                       ('confirm_frames',         '확정 프레임      '),
                       ('blind_latch_hold_s',     '사각지대 래치[s] ')):
        if key in p:
            print(f"      {label} {key} = {p[key]}")
    print("      ※ 이 값들은 cone_lidar_node 가 직접 읽어 쓴다 — 고칠 곳은 이 YAML "
          "한 곳이고 재빌드는 필요 없다(노드 재시작만).")
    print("      ※ 페달 구동 = 직접 PWM 16~255 (white1 과 동일). "
          "끝까지 밟으면 PWM 255. 묶으려면 manual_pwm_max.")
    print()

    return [Node(
        package='lidar', executable='pedal_drive_node', name='pedal_drive_node',
        output='screen', additional_env=NODE_ENV,
        parameters=[{
            'stop_signal_topic':       '/cone_lidar_node/stop_signal',
            'obstacle_distance_topic': '/cone_lidar_node/obstacle_distance',
            'aeb_stop_topic':          '/aeb_stop',
            'engage_frames':           LaunchConfiguration('aeb_engage_frames'),
            'min_engage_s':            LaunchConfiguration('aeb_min_engage_s'),
            'release_clear_s':         LaunchConfiguration('aeb_release_clear_s'),
            'signal_stale_s':          LaunchConfiguration('aeb_stale_s'),
        }],
    )]


def generate_launch_description():
    pkg = get_package_share_directory('lidar')
    snd = _sound_dir()

    imu_dev = '/dev/imu'
    try:
        from white1 import ports as wports  # noqa: PLC0415
        imu_dev = wports.resolve_device(
            wports.SYMLINK_IMU, wports.IMU_VIDPID,
            log=lambda m: print(f"    [IMU] {m}"))
    except Exception as exc:  # noqa: BLE001
        print(f"    [IMU] white1.ports 를 못 썼습니다 ({exc}) — {imu_dev} 로 시도")

    print("\n=====================================================")
    print(" 🛑 lidar one_launch")
    print("    기본          = AEB 수동시험 (D5 수동, 페달 PWM 16~255)")
    print("    cone_drive:=true = 라바콘 복도 잠금 → 7펄스 직진 (D5 자율)")
    print("    페달 끝까지 = PWM 255 / 0점 = throttle_raw_min (기본 220)")
    print(f"    음원 = {snd or '(못 찾음 — 안내음 없이 돕니다)'}")
    print("    라이다는 유선 LAN 이다 — 안 뜨면 먼저:")
    print("        ip -br addr show eno1   /   ping -c2 192.168.6.11")
    print("=====================================================\n")

    cone_on = PythonExpression(
        ["'", LaunchConfiguration('cone_drive'), "' == 'true'"])
    iahrs_on = PythonExpression([
        "'", LaunchConfiguration('cone_drive'), "' == 'true' and '",
        LaunchConfiguration('use_iahrs'), "' == 'true'",
    ])

    args = [
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='nxde arduino(A/B 2보드)를 함께 띄울지. false 면 별 터미널에서 '
                        '직접 띄운다 — ★그때는 aeb_brake_level 을 손으로 넘겨야 한다★ '
                        '(안 주면 기본 0 = 비상정지가 리니어를 물지 않는다)'),
        DeclareLaunchArgument(
            'use_sound', default_value='true',
            description='nxde sound(음성 안내). 구독만 하므로 제어에는 영향이 없다'),
        DeclareLaunchArgument(
            'use_hud', default_value='true',
            description='white1 hud 계기판(상면도 + 게이지). 구독만 하므로 제어에 '
                        '영향이 없다. DISPLAY 없는 SSH 면 false'),
        DeclareLaunchArgument(
            'use_ouster', default_value='true',
            description='라이다 드라이버를 함께 띄울지. false 면 rosbag 재생·별 터미널'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='RViz 로 ROI·검출점을 보면서 시험'),

        # ★'params_file' 이라는 이름을 여기서 선언하지 않는다★ IncludeLaunchDescription
        #   은 부모의 LaunchConfiguration 을 자식에게 물려주므로, 그 이름을 쓰면
        #   ouster 드라이버가 남의 YAML 을 자기 파라미터로 읽고 죽는다
        #   (aeb.launch.py 헤더에 실제 사고 기록이 있다). 이름을 갈라 두는 것이 방어다.
        DeclareLaunchArgument(
            'cone_params_file',
            default_value=os.path.join(pkg, 'config', 'cone_lidar.yaml'),
            description='cone_lidar_node 파라미터 YAML. ★감지 거리·ROI 는 여기서 '
                        '고친다★ (재빌드 불필요, 노드 재시작만)'),

        # ═══════════════════════════════════════════════════════════════
        #  아두이노 — ★수동조종 페달 PWM 구간과 AEB 제동단계★
        # ═══════════════════════════════════════════════════════════════
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='A/B 보드 공통 시리얼 보드레이트'),
        DeclareLaunchArgument(
            'steer_invert', default_value='false',
            description='조향 부호 반전. ★기본 false★ — 이 시험에서 ROS 는 조향을 '
                        '내지 않으므로(수동조종은 힘빼기) 사실상 쓰이지 않는다'),
        DeclareLaunchArgument(
            'stop_brake_level', default_value='0',
            description='/control_state=False 일 때 걸 브레이크 단계. ★0 을 유지할 것★ '
                        '— 이 런치에서 리니어를 물어야 하는 것은 AEB 뿐이다'),
        # ★★ [2026-08-27] 페달은 white1 과 같은 직접 PWM 이다 ★★
        #   목표펄스 PID 는 펌웨어 PWM_MAX=170 에 묶여, 엑셀을 끝까지 밟아도
        #   듀티가 프로토콜 상한(255)까지 안 올라갔다. 직접 PWM 은 그 캡을
        #   무시하고 받은 값을 그대로 낸다 → 풀 엑셀 = PWM 255.
        #   개루프라 살짝 밟아도 듀티가 유지된다. 시험 속도를 묶으려면
        #   manual_pwm_max 를 낮춘다(90 ≈ 4펄스). PID 가 필요하면
        #   manual_use_pwm:=false (그때 풀 엑셀 PWM 255 는 나오지 않는다).
        DeclareLaunchArgument(
            'manual_use_pwm', default_value='true',
            description='수동조종 페달을 A보드 직접 PWM 으로 보낼지. ★true = 16~255★ '
                        '(풀 엑셀 = PWM 255). false 면 목표펄스 PID, 펌웨어 PWM_MAX=170 '
                        '에 묶여 최대 듀티가 안 나온다'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='페달 최대치가 대응할 목표펄스. ★라벨(/drive_pulse_cmd)용★ '
                        'manual_use_pwm:=true 이면 차를 굴리지 않는다. PWM 경로를 끄면 '
                        '이 값이 실제 상한(펌웨어 PWM_MAX=170)'),
        DeclareLaunchArgument(
            'manual_pwm_min', default_value='16',
            description='페달을 살짝 밟았을 때의 PWM. ★16 = A보드 프로토콜 하한★'),
        DeclareLaunchArgument(
            'manual_pwm_max', default_value='255',
            description='★페달을 끝까지 밟았을 때의 PWM = 수동조종 최고속★ '
                        '기본 255 = A보드 프로토콜 상한. 직접 PWM 은 펌웨어 '
                        'PWM_MAX=170 을 무시한다. 시험 속도를 묶으려면 낮춘다 '
                        '(예: 90 ≈ 4펄스 ≈ 12.7 km/h)'),
        DeclareLaunchArgument(
            'throttle_raw_min', default_value='220',
            description='페달을 놓은 것으로 볼 A0 최댓값. 이 이하 → 지령 0. '
                        '★[2026-08-26] 177→220★ 실차 휴지가 196~208 이라 옛 177 은 '
                        '지령이 나가 엑셀을 안 밟아도 저속으로 갔다. '
                        '휴지가 더 올라가면 이 값을 더 올린다'),
        DeclareLaunchArgument(
            'throttle_raw_max', default_value='950',
            description='페달을 끝까지 밟았을 때 A0. ★[2026-08-27] 800→950★ '
                        '실측 풀 행정이 946. 옛 800 은 살짝 밟아도 /drive_pulse_cmd=15'),
        DeclareLaunchArgument(
            'throttle_gamma', default_value='1.4',
            description='페달 개도 곡선. 1=선형, 키울수록 초반이 완만(저속 정밀). '
                        '기본 1.4 — 살짝 밟으면 저속, 끝까지 밟으면 15/PWM255'),

        # ═══════════════════════════════════════════════════════════════
        #  AEB — ★제동 단계는 arduino 가, 판단은 pedal_drive_node 가 소유한다★
        # ═══════════════════════════════════════════════════════════════
        DeclareLaunchArgument(
            'aeb_brake_level', default_value='2',
            description='/aeb_stop 이 True 일 때 arduino 가 물 브레이크 단계. '
                        '★2(풀브레이킹) 를 권한다★ — 1단은 물린 뒤 첫 0.55초 동안 '
                        '제동력이 0 이고(행정 램프) 2펄스에서도 정지거리가 1.2 m 다. '
                        '★0 을 주면 비상정지가 리니어를 물지 않는다★(구동차단만 = '
                        '코스트 0.41 m/s²). arduino.py 자체 기본값은 0(기능 꺼짐)이고, '
                        '이 런치가 켜는 유일한 곳이다'),
        DeclareLaunchArgument(
            'aeb_stale_s', default_value='1.0',
            description='/aeb_stop 신선도 [s]. 판단 노드가 20Hz 로 내므로 1.0 은 '
                        '20틱이 빠진 것이다. 끊기면 ★제동을 푼다★(fail-open) — '
                        '그 상태는 AEB 없는 수동조종 = 원래 상태이고, 사람이 '
                        '예상하지 못한 정지가 뒤차·경사에서 더 위험하다'),
        DeclareLaunchArgument(
            'aeb_engage_frames', default_value='1',
            description='확정에 필요한 연속 프레임(20Hz 기준). cone_lidar 가 이미 '
                        'confirm_frames=2 로 확정한 뒤라 기본 1 이다. ★올리면 그만큼 '
                        '제동이 늦는다★ (1프레임 = 0.05초 = 2펄스에서 9 cm)'),
        DeclareLaunchArgument(
            'aeb_min_engage_s', default_value='1.0',
            description='한 번 물면 최소 유지 [s]. 리니어 1단 행정이 0.54초라 '
                        '그보다 짧게 풀면 제동력은 안 나오고 기구만 왕복한다'),
        DeclareLaunchArgument(
            'aeb_release_clear_s', default_value='1.5',
            description="'비었다'가 이만큼 이어져야 해제 [s]. cone_lidar 의 "
                        'blind_zone_latch 가 사각지대를 이미 붙잡고 있으므로 '
                        '그쪽 blind_latch_hold_s(3.0)보다 짧아도 된다'),

        # ═══════════════════════════════════════════════════════════════
        #  라바콘 헤딩홀드 자율 — 기본 off. 켜면 런치 = 출발.
        # ═══════════════════════════════════════════════════════════════
        DeclareLaunchArgument(
            'cone_drive', default_value='false',
            description='true 면 drive_lidar_node + 외장 iAHRS 를 띄운다. '
                        '정지 관측 후 양옆 라바콘으로 복도를 잠그고 7펄스 직진. '
                        '★런치 = 출발★ D5 자율 · E-STOP 해제 · 차 앞을 비울 것'),
        DeclareLaunchArgument(
            'drive', default_value='true',
            description='cone_drive 일 때 D5·E-STOP 게이트. false 는 책상 시험용 '
                        '(지령만 계산). 차에 연결한 채로 쓰지 말 것'),
        DeclareLaunchArgument(
            'linear_speed', default_value='6.188',
            description='cone_drive 순항속도 [m/s]. 정수 펄스로 반올림 '
                        '(1펄스=0.884 / ★7펄스=6.188 기본 ≈ 22.3 km/h★)'),
        DeclareLaunchArgument(
            'survey_duration', default_value='2.5',
            description='cone_drive 정지 관측 [s]. 이 시간이 지나면 잠금·출발'),
        DeclareLaunchArgument(
            'use_iahrs', default_value='true',
            description='cone_drive 일 때 white1 iahrs(/imu) 를 띄울지. 끄면 '
                        'imu_topic 을 /ouster/imu 로 바꿔 라이다 자이로를 쓴다'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='iAHRS 출력주기[ms]. 50 = 20Hz'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='헤딩 IMU 토픽. 기본 /imu = 외장 iAHRS'),

    ]

    # ═══════════════════════════════════════════════════════════════════
    #  [안내] 음성 — ★제일 먼저 띄운다★ '런치했다'는 안내가 하드웨어 탐색보다
    #   늦으면 의미가 없다. respawn 을 걸지 않는다(되살아날 때마다 시작 안내가
    #   다시 나오는 편이 조용히 없는 것보다 헷갈린다 — white1 과 같은 판단).
    # ═══════════════════════════════════════════════════════════════════
    sound = Node(
        package='nxde', executable='sound', name='sound_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'sound_dir': snd}],
        condition=IfCondition(LaunchConfiguration('use_sound')),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B — ★이 시험의 제동 실행자★
    #    cone_drive 가 iAHRS 를 띄우므로 IMU 경로를 exclude 한다. 없어도
    #    arduino 는 GPS/IMU VID/PID 를 스스로 건너뛴다(그쪽 헤더 2026-08-05).
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='nxde', executable='arduino', name='arduino', output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':             LaunchConfiguration('baud'),
            'steer_invert':     LaunchConfiguration('steer_invert'),
            'stop_brake_level': LaunchConfiguration('stop_brake_level'),
            'manual_use_pwm':   LaunchConfiguration('manual_use_pwm'),
            'manual_pulse_max': LaunchConfiguration('manual_pulse_max'),
            'manual_pwm_min':   LaunchConfiguration('manual_pwm_min'),
            'manual_pwm_max':   LaunchConfiguration('manual_pwm_max'),
            'throttle_raw_min': LaunchConfiguration('throttle_raw_min'),
            'throttle_raw_max': LaunchConfiguration('throttle_raw_max'),
            'throttle_gamma':   LaunchConfiguration('throttle_gamma'),
            # ★여기가 비상정지를 '켜는' 유일한 지점이다★ arduino.py 기본은 0(꺼짐)
            'aeb_brake_level':  LaunchConfiguration('aeb_brake_level'),
            'aeb_stale_s':      LaunchConfiguration('aeb_stale_s'),
            'aeb_topic':        '/aeb_stop',
            'exclude_ports':    [imu_dev],
        }],
        condition=IfCondition(LaunchConfiguration('use_arduino')),
    )

    iahrs = Node(
        package='white1', executable='iahrs', name='iahrs_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=2.0,
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': 115200,
            'send_tf': True,
            'rescan': True,
            'sync_period_ms': LaunchConfiguration('imu_sync_period_ms'),
        }],
        condition=IfCondition(iahrs_on),
    )

    drive_lidar = Node(
        package='lidar', executable='drive_lidar_node',
        name='drive_lidar_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[
            os.path.join(pkg, 'config', 'drive_lidar.yaml'),
            {
                'linear_speed': LaunchConfiguration('linear_speed'),
                'survey_duration': LaunchConfiguration('survey_duration'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'imu_use_orientation': True,
                'kasa.max_pulse': 7,
                'kasa.require_auto_mode': LaunchConfiguration('drive'),
                'kasa.require_estop_clear': LaunchConfiguration('drive'),
            },
        ],
        condition=IfCondition(cone_on),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [인지] ouster 드라이버 + cone_lidar_node + (rviz)
    #    ★aeb.launch.py 를 통째로 include 한다★ 그 파일이 조각의 주인이다 —
    #    같은 노드를 여기서 다시 선언하면 파라미터가 두 곳으로 갈라진다.
    #    (그 파일이 ouster.launch.py 를 다시 include 한다 = '기존 구성품' 그대로)
    # ═══════════════════════════════════════════════════════════════════
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'aeb.launch.py')),
        launch_arguments={
            'use_ouster':       LaunchConfiguration('use_ouster'),
            'use_rviz':         LaunchConfiguration('use_rviz'),
            'cone_params_file': LaunchConfiguration('cone_params_file'),
        }.items(),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [구동 담당] pedal_drive_node
    #    ★cone_lidar.yaml 을 읽어서 파라미터에 반영한다★ — _pedal_drive_node()
    #    안에 그 이유와 우선순위(런치 인자 > YAML > 기본값)를 적었다. YAML 경로가
    #    LaunchConfiguration 이라 런치 시점에 풀어야 하므로 OpaqueFunction 이다.
    # ═══════════════════════════════════════════════════════════════════
    pedal_drive = OpaqueFunction(function=_pedal_drive_node)

    # 구독 전용 계기판. white1 패키지 노드. 창을 닫아도 시험은 계속된다.
    hud = Node(
        package='white1', executable='hud', name='hud_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'show_camera': False,
            'lidar_range_min': 2.0,
            'lidar_range_max': 15.0,
            'vehicle_front_m': 1.2,
            'corridor_half_m': 0.8,
        }],
        condition=IfCondition(LaunchConfiguration('use_hud')),
    )

    return LaunchDescription(args + [
        sound,        # 안내가 제일 먼저
        arduino,      # 그다음 하드웨어
        iahrs,        # cone_drive 일 때만 (외장 iAHRS → /imu)
        perception,   # 라이다 + AEB 판정 (+ rviz)
        pedal_drive,  # 확정 → /aeb_stop (+ YAML 감지 설정을 화면에 표시)
        drive_lidar,  # cone_drive 일 때만 (복도 잠금 → /cmd_vel_raw)
        hud,          # 상면도 HUD (구독 전용)
    ])
