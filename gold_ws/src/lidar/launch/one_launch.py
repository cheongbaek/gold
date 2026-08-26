#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""one_launch.py — ★비상정지(AEB) 시험 통합 런치★ [lidar / 2026-08-25]

    ros2 launch lidar one_launch.py
    ros2 launch lidar one_launch.py use_rviz:=true        # ROI 를 눈으로 보면서
    ros2 launch lidar one_launch.py manual_pwm_max:=70    # 페달 상한을 2펄스로 묶어서

════════════════════════════════════════════════════════════════════════════════
 무엇을 시험하는 런치인가
════════════════════════════════════════════════════════════════════════════════
  ★주행은 사람이 페달로 한다★ (B보드 D5 = 수동조종). 자율주행 노드를 하나도
  띄우지 않는다 — 차를 스스로 출발시키는 것이 아무것도 없다.
  그 페달은 ★직접 PWM 16~255 전 구간★ 으로 A보드에 내려간다(arduino.py 가 개도량을
  비례 대응시킨다 / 2026-08-25). ★고정 2펄스 같은 상한이 없다 — 밟은 만큼 나간다★.
  그 상태에서 라이다가 ★상시 감시★ 하다가 전방 장애물을 확정하면, 차가 스스로
  구동을 끊고 리니어를 물어 선다. 그 사슬 하나만 시험하는 런치다.

  ⚠️ 그 대가로 ★AEB 가 못 세우는 속도★ 가 열려 있다. 소프트웨어는 그것을 판정하지도
     경고하지도 않는다 — ★밟지 않는 것이 사람의 몫이다★. 속도를 실제로 묶어야 하면
     manual_pwm_max 를 낮추는 것이 유일한 방법이다 (예: 70 ≈ 2펄스, 90 ≈ 4펄스).

     ouster 드라이버 ─/ouster/points─▶ cone_lidar_node ─/…/stop_signal─▶
       pedal_drive_node ─/aeb_stop─▶ arduino ─▶ A보드 구동차단 + B보드 리니어 2단
                              └────────────▶ sound  (경고음 반복)

  ★drive_lidar_node 는 띄우지 않는다★ 그쪽은 라바콘 복도를 스스로 출발해 2펄스로
  달리는 자율주행 노드다(런치 = 출발). 비상정지만 보려면 차를 움직이는 주체가
  사람이어야 한다. 그래서 이 런치의 '구동 담당'은 pedal_drive_node 이고, 그 노드는
  ★/cmd_vel_raw·/control_state·/brake_level 을 하나도 발행하지 않는다★.

════════════════════════════════════════════════════════════════════════════════
 띄우는 것
════════════════════════════════════════════════════════════════════════════════
    nxde/sound            음성 안내 (구독 전용)      ← ★제일 먼저★ use_sound
    nxde/arduino          A/B 2보드 시리얼 브리지                use_arduino
    ouster.launch.py      OS1-32 드라이버 (통째로 include)
    cone_lidar_node       가상범퍼 AEB 판정 (aeb.launch.py 를 통째로 include)
    rviz2                 ROI 표시                               use_rviz
    lidar/pedal_drive_node ★확정·래치 → /aeb_stop★ (속도는 판정하지 않는다)

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
from launch.substitutions import LaunchConfiguration
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
    print("      ※ ★속도 상한은 어디에도 없다★ 페달이 곧 속도다 "
          "(묶으려면 manual_pwm_max).")
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

    print("\n=====================================================")
    print(" 🛑 lidar one_launch — ★비상정지(AEB) 시험★")
    print("    주행 = 사람의 페달 (D5 수동조종) / 자율주행 노드 없음")
    print(f"    음원 = {snd or '(못 찾음 — 안내음 없이 돕니다)'}")
    print("    라이다는 유선 LAN 이다 — 안 뜨면 먼저:")
    print("        ip -br addr show eno1   /   ping -c2 192.168.6.11")
    print("=====================================================\n")

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
        # ★★ [2026-08-25 개정] 페달을 프로토콜 전 구간(16~255)으로 쓴다 ★★
        #   arduino.py 가 페달 개도량을 A보드 직접 PWM(개루프)으로 내려보낸다.
        #   FF 표(kasa_0730_A.ino ffPwmTable) 기준 대략치 :
        #       PWM  60 ≈  1펄스 ( 3.2 km/h)     PWM 150 ≈ 16펄스( 51 km/h)
        #       PWM  70 ≈  2펄스 ( 6.4 km/h)     PWM 170 ≈ 24펄스( 76 km/h)
        #       PWM  90 ≈  4펄스 (12.7 km/h)     PWM 255 = 표 밖 (전개)
        #   ※ 개루프라 경사·하중에 따라 실제 속도가 달라진다 — 표는 평지 근사다.
        #
        #   ★종전 '2펄스(PWM 70) 제한' 을 걷어냈다★ 그리고 그 자리에 다른 상한을
        #   들여놓지도 않았다 — 한때 pedal_drive_node 가 '설 수 있는 속도' 를 계산해
        #   로그에 찍었는데, 그 숫자가 남아 있는 것 자체가 '무언가 걸려 있다' 로
        #   읽혔다. ★지금은 소프트웨어가 속도를 판정하는 곳이 한 군데도 없다★.
        #   사람 발이 곧 스로틀이고, 프로토콜 전 구간이 열려 있다.
        #
        #   ⚠️⚠️ 그 대가는 ★AEB 가 못 세우는 속도까지 갈 수 있다★ 는 것이다 ⚠️⚠️
        #     감지 거리(cone_lidar.yaml)와 리니어 2단 성능이 정하는 물리적 한계는
        #     그대로 있다. 다만 아무도 그것을 감시하지 않는다 — ★밟지 않는 것이
        #     사람의 몫이다★. 처음 시험은 라바콘·박스로, 살짝만 밟아서 시작할 것.
        #     실제로 묶어야 하면 manual_pwm_max 를 낮춘다(그것이 유일한 지점이다).
        DeclareLaunchArgument(
            'manual_pwm_min', default_value='16',
            description='페달을 살짝 밟았을 때의 PWM. ★16 = A보드 프로토콜 하한★ '
                        '(그 아래는 펌웨어가 펄스로 읽어버린다). 순수 비례라 페달 '
                        '초반 1/3 쯤은 유격이 된다 — 바퀴가 실제로 도는 지점이 '
                        'PWM 60 부근이기 때문이다. 그 유격이 거슬리면 60 으로 올려라'),
        DeclareLaunchArgument(
            'manual_pwm_max', default_value='255',
            description='★페달을 끝까지 밟았을 때의 PWM = 이 시험의 최고속★ '
                        '기본 255 = A보드 프로토콜 상한(전개). 직접 PWM 은 펌웨어의 '
                        '무보호 경로라 펄스모드 상한(PWM_MAX=170)도 무시한다. '
                        '⚠️ 소프트웨어가 속도를 판정하는 곳은 없다 — 너무 빠르면 '
                        'AEB 가 감지 거리 안에 못 세운다는 물리적 사실만 남는다. '
                        '속도를 실제로 묶고 싶으면 ★이 값을 낮추는 것이 유일한 '
                        '지점이다★ (예: 70 ≈ 2펄스, 90 ≈ 4펄스)'),

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
    #    GPS/IMU 를 띄우지 않으므로 exclude_ports 를 넘기지 않는다. arduino 는
    #    자기가 GPS/IMU VID/PID 를 보고 그 포트를 건너뛴다(그쪽 헤더 2026-08-05).
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='nxde', executable='arduino', name='arduino', output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':             LaunchConfiguration('baud'),
            'steer_invert':     LaunchConfiguration('steer_invert'),
            'stop_brake_level': LaunchConfiguration('stop_brake_level'),
            'manual_pwm_min':   LaunchConfiguration('manual_pwm_min'),
            'manual_pwm_max':   LaunchConfiguration('manual_pwm_max'),
            # ★여기가 비상정지를 '켜는' 유일한 지점이다★ arduino.py 기본은 0(꺼짐)
            'aeb_brake_level':  LaunchConfiguration('aeb_brake_level'),
            'aeb_stale_s':      LaunchConfiguration('aeb_stale_s'),
            'aeb_topic':        '/aeb_stop',
        }],
        condition=IfCondition(LaunchConfiguration('use_arduino')),
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

    return LaunchDescription(args + [
        sound,        # 안내가 제일 먼저
        arduino,      # 그다음 하드웨어
        perception,   # 라이다 + AEB 판정 (+ rviz)
        pedal_drive,  # 확정 → /aeb_stop (+ YAML 감지 설정을 화면에 표시)
    ])
