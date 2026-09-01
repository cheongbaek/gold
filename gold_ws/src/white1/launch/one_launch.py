#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white1 통합 런치 (GPS + IMU + 아두이노 + 자율주행)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 one_launch.py

띄우는 것 (카메라 없음):
    nxde/arduino          A/B 2보드 시리얼 브리지  ★수정 없이 그대로 쓴다★
    white1/iahrs        6축 IMU 드라이버 → /imu
    white1/speed        /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS 수신기 → /fix
    white1/gps          ★/fix + /imu → /gps_fused★ RTK Fixed/Float 판정 +
                          5Hz fix 사이 공백을 IMU 로 메운 20Hz 가상좌표
    white1/driving      ★헤딩 + 모드스위치 상태기계 + 경로추종★ (위치는 gps 가 준다)
    white1/mapping      ★/fix 원값만★ 보고 경로 수집 (gps 후처리를 거치지 않는다)
    white1/record       자율주행 구간 토픽 → CSV
    usb_cam             카메라 → /image_raw                      (use_camera)
    white1/traffic_light 신호등 인지 — 빨간불이면 리니어 2단      (use_camera)
        ★DRIVE_RUN 중에만 개입한다★ 빨간불이 사라지거나 초록불이면 즉시 풀고,
        driving 이 계속 내던 목표펄스가 그대로 통해 스스로 재출발한다.
        카메라를 안 꽂았으면 use_camera:=false (usb_cam 이 respawn 루프를 돈다)
    white1/hud          차량 상면도 HUD (구독 전용)               (use_hud)
    ouster_ros/os_driver  OS1-32 라이다 → /ouster/points           (use_lidar)
    mppi_local_planner  ★라바콘 회피 — CSV terrain 열이 'L' 인 구간만★ (use_lidar)

════════════════════════════════════════════════════════════════════════════════
 ★라이다 구간 이양 [2026-09-01]★
════════════════════════════════════════════════════════════════════════════════
    매핑 CSV 의 미사용 열 terrain 에 사람이 손으로 'L' 을 적어 둔 구간에서는
    driving 이 조종권을 놓고 mppi_local_planner 가 라바콘을 피하며 몬다.
    그 밖의 값(빈 칸·'0')은 전부 GPS 추종이다 — ★기존 CSV 가 그대로 돈다★.

        driving ──/lidar_permit──▶ mppi        "이 구간은 네가 몰아라"
        driving ◀──/lidar_active── mppi        "나 살아 있다" (매 틱·신선도가 생존)

    설계 근거는 white1/white1/driving.py 헤더의 '라이다 구간 이양' 절에 있다.

    ★라이다는 USB 가 아니라 유선 LAN 이다★ eno1 이 192.168.6.100/24 로 올라와
    있어야 하고 센서는 192.168.6.11 이다. 안 붙으면:
        ip -br addr show eno1  ·  ping -c2 192.168.6.11

    ★라이다 없이 종전처럼 돌리려면 use_lidar:=false★ 그때 L 구간이 있는 CSV 를
    주행하면 driving 이 그 구간에서 ★정지한다★ (아무도 몰지 않는 상태로 달리지
    않는다 — /lidar_active 가 오지 않는 것을 보고 판단한다).

    ★AEB(lidar cone_lidar_node)는 띄우지 않는다★ 회피는 mppi 가 자기 코스트맵으로
    판단하고, 못 피하면 스스로 리니어 2단을 문다. 그 위에 가상범퍼를 겹치면
    라바콘 사이를 지나갈 때 AEB 가 먼저 세운다.

CLI 는 따로 띄운다 (별 터미널):
    ros2 run white1 prompt
    ros2 run white1 hud         # 차량 상면도 HUD (구독 전용, 제어 안 함)

════════════════════════════════════════════════════════════════════════════════
 조작 [2026-08-11] prompt 의 1)매핑 / 2)주행 메뉴로 시작한다
════════════════════════════════════════════════════════════════════════════════
    매핑 : prompt 에서 1 선택 — 스위치가 수동조종이면 즉시, 자율주행이면 내릴 때까지 대기
    주행 : prompt 에서 2 선택(+경로) — 스위치가 자율주행이면 즉시, 수동조종이면 올릴 때까지 대기

    ★B보드 D5 스위치는 더 이상 시작을 트리거하지 않는다★ 진행 중인 것이 스위치
    위치와 안 맞게 되면 취소하고, 도착·저장 시점을 정리하는 역할만 한다 —
    driving.py 헤더의 on_mode_edge 참고.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white1 import camera_launch, paths, ports


# 파이썬 stdout 버퍼링을 끄지 않으면 노드 로그가 뭉쳐서 늦게 나온다
NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
RESPAWN_DELAY = 2.0


def _lidar_actions(context, *_a, **_kw):
    """라이다 노드 조각. ★use_lidar 가 실제로 true 일 때만 패키지를 찾는다★

    OpaqueFunction 으로 감싼 이유는 하나다 — get_package_share_directory 는 런치
    ★파싱 시점★ 에 실행되므로, 그냥 두면 lidar·mppi_local_planner 를 빌드하지 않은
    사람이 use_lidar:=false 로도 이 런치를 띄울 수 없다. 인자가 확정된 뒤에 찾는다.
    """
    from ament_index_python.packages import get_package_share_directory

    if LaunchConfiguration('use_lidar').perform(context).lower() \
            not in ('true', '1', 'yes', 'on'):
        return []

    try:
        lidar_share = get_package_share_directory('lidar')
        mppi_share = get_package_share_directory('mppi_local_planner')
    except Exception as exc:            # noqa: BLE001 — 파서에도 이유를 남긴다
        raise RuntimeError(
            "use_lidar:=true 인데 lidar / mppi_local_planner 패키지를 찾을 수 없습니다.\n"
            "  colcon build --packages-select ouster_sensor_msgs ouster_ros \\\n"
            "      lidar mppi_local_planner --cmake-args -DCMAKE_BUILD_TYPE=Release\n"
            "  ★--symlink-install 을 쓰지 말 것★ (C++ 패키지 — lidar/README.md 참고)\n"
            "  source ~/gold/gold_ws/install/setup.bash\n"
            "라이다 없이 GPS 추종만 하려면 use_lidar:=false"
        ) from exc

    # ── OS1-32 드라이버 ── lidar/launch/ouster.launch.py 가 조각을 소유한다.
    #   ★그쪽 런치를 그대로 include 한다★ 0.13.x 고정 이유·ouster_ns·params 규약이
    #   전부 거기 있어서, 여기 다시 적으면 두 곳이 갈린다.
    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'ouster.launch.py')),
        launch_arguments={'viz': 'false'}.items(),
    )

    # ── MPPI 회피 ──
    #   ★lidar / mppi 의 one_launch.py 를 include 하지 않는다★ 그 둘은 각자
    #   arduino·sound·hud 를 함께 띄운다 — 여기서 include 하면 arduino 가 두 개가
    #   되어 같은 시리얼 포트를 다툰다. 노드만 직접 선언한다.
    #   ★drive_gps_node·drive_lidar_node 도 띄우지 않는다★ 앞은 driving.py 와 기능이
    #   정면으로 겹치고(lidar/README.md), 뒤는 라바콘 '사이를 지나는' 코리도 주행이라
    #   '세워진 라바콘을 피하는' 이 용도가 아니다.
    mppi = Node(
        package='mppi_local_planner',
        executable='mppi_local_planner_node',
        name='mppi_local_planner_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[
            os.path.join(mppi_share, 'config', 'params.yaml'),
            {
                # ★반드시 true★ false 면 이 노드가 GPS 추종 구간에서도 /cmd_vel_raw 를
                #   내며 driving.py 와 20Hz 로 서로를 덮는다. 런치 인자로도 열지 않는다.
                'handover.require_permit': True,
                # 외장 iAHRS. /ouster/imu 는 자이로만이라 드리프트가 크다.
                'imu_topic': '/imu',
                'imu_use_orientation': True,
                'flip_lidar_xy': LaunchConfiguration('flip_lidar_xy'),
                'mppi.desired_speed': LaunchConfiguration('lidar_speed'),
                'kasa.max_pulse': LaunchConfiguration('lidar_pulse'),
                # 실차 게이트는 항상 켠다 — D5 수동조종·E-STOP 중에는 안 움직인다.
                'kasa.require_auto_mode': True,
                'kasa.require_estop_clear': True,
            },
        ],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_mppi', output='screen',
        arguments=['-d', os.path.join(mppi_share, 'config', 'mppi.rviz')],
        condition=IfCondition(LaunchConfiguration('use_lidar_rviz')),
    )

    return [ouster, mppi, rviz]


def generate_launch_description():
    package_name = 'white1'

    print("\n=====================================================")
    print(" 🔌 하드웨어 장치 경로 확인 (GPS / IMU)")
    print("    아두이노 A/B 는 arduino 노드가 텔레메트리 접두어로 자체 식별합니다.")
    print("    ※ 미리 확인하려면: ros2 run nxde check")

    # 못 찾아도 실패가 아니다 — resolve_device 는 udev 링크 → VID/PID → (없으면)
    # 링크 경로를 그대로 돌려준다. 나중에 꽂으면 자체 재연결·respawn 이 붙는다.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)
    print("=====================================================\n")

    exclude_for_arduino = [gps_dev, imu_dev]

    # 카메라(신호등 인지) — 조각은 white1/camera_launch.py 가 소유한다
    cam_dev, cam_format = camera_launch.banner(ports)

    use_arduino = LaunchConfiguration('use_arduino')
    use_record  = LaunchConfiguration('use_record')
    use_mapping = LaunchConfiguration('use_mapping')
    use_sound   = LaunchConfiguration('use_sound')
    use_hud     = LaunchConfiguration('use_hud')

    args = [
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='nxde 의 arduino 노드(A/B 2보드)를 함께 띄울지. false 면 별 '
                        '터미널에서 `ros2 run nxde arduino` 로 직접 띄운다'),
        DeclareLaunchArgument(
            'use_record', default_value='true',
            description='record 노드(주행 CSV 기록). 자율주행 구간에서만 파일이 생긴다'),
        DeclareLaunchArgument(
            'use_mapping', default_value='true',
            description='mapping 노드(경로 수집). prompt 에서 1)매핑을 시작해야 동작한다'),
        DeclareLaunchArgument(
            'use_sound', default_value='true',
            description='nxde 의 sound 노드(음성 안내). 뜨는 즉시 one_launch_1 이 나오고, '
                        'A·B 보드 연결·매핑/주행 시작·도착·E-stop 을 토픽으로 보고 안내한다. '
                        '구독만 하므로 제어에는 영향이 없다 — 조용히 쓰려면 false'),
        DeclareLaunchArgument(
            'use_hud', default_value='true',
            description='white1 hud 계기판(상면도 + 게이지). 구독만 하므로 제어에 '
                        '영향이 없다. DISPLAY 없는 SSH 면 false'),
        DeclareLaunchArgument(
            'gps_port', default_value=gps_dev,
            description='GPS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로 override'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='IMU 출력주기[ms]. 기본 50 = 20Hz (driving 제어주기와 동일)'),
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='A/B 보드 공통 시리얼 보드레이트'),
        DeclareLaunchArgument(
            'steer_invert', default_value='false',
            description='조향 부호 반전. ★기본 false★ — ROS 토픽과 B보드가 같은 규약'
                        '(− 좌 / + 우)이다. 배선을 뒤집었을 때만 true'),
        DeclareLaunchArgument(
            'stop_brake_level', default_value='0',
            description='/control_state=False 일 때 arduino 가 걸 브레이크 단계. '
                        '★0 을 권한다★ — 정지 시 리니어는 driving 이 직접 지시한다'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='수동조종에서 페달 최대치가 대응할 펄스. '
                        '★[2026-08-25] 이 값은 더 이상 차를 굴리지 않는다★ — '
                        '/drive_pulse_cmd 라벨(mapping 수집 라벨 ①)을 종전 0~15 '
                        '스케일로 유지하는 용도만 남았다. 실제 속도는 아래 '
                        'manual_pwm_max 가 정한다 — 속도를 낮추려고 이 값을 '
                        '건드리면 ★라벨만 줄고 차는 그대로 나간다★'),
        DeclareLaunchArgument(
            'manual_use_pwm', default_value='true',
            description='수동조종 페달을 A보드 직접 PWM 으로 보낼지. ★true = lidar 와 동일★ '
                        '풀 엑셀 = PWM 255. false 면 목표펄스 15 를 A보드에 보내고 '
                        '펌웨어 PID 가 PWM_MAX=170 에 묶어 lidar 보다 현저히 느리다'),

        # ── ★[2026-08-25] 수동조종 페달 = A보드 직접 PWM★ ──
        #   arduino.py 가 페달 개도량을 manual_pwm_min~max 에 비례 대응시켜
        #   A보드의 직접 PWM 경로("<pwm>,<pwm>")로 내려보낸다. 종전에는 목표펄스
        #   (0~15)를 보내 보드의 PID 가 맞추게 했는데, 그 사이에 PID·기동
        #   블랭킹·코스트가 끼어 '밟은 만큼 나가지' 않았다(arduino.py (2) 분기).
        #
        #   ★프로토콜 전 구간 16~255 를 쓴다★ 이 런치의 수동조종은 매핑 절차
        #   (사람이 페달로 곧게 굴려 초기 헤딩을 잡는 것)에 쓰이는데, 거기서
        #   속도를 소프트웨어가 잘라 둘 이유가 없다 — 사람이 밟는 만큼이 맞다.
        #
        #   ★이 스택에 AEB 가 없는 것은 의도된 것이다★ (2026-08-25 확인)
        #     lidar one_launch.py 와 달리 수동조종 중 전방을 보는 것이 없다
        #     (신호등 카메라는 자율주행 분기에서만 일한다). 그쪽은 AEB 시험용
        #     런치고, 이쪽은 자율주행 스택이다 — 수동조종은 매핑 절차를 위해
        #     지나가는 상태이지 이 런치가 시험하려는 대상이 아니다.
        #     → 수동조종에서 차를 세우는 것은 ★사람의 발뿐★ 이고, PWM 255 는
        #       '사람이 낼 수 있는 최고속' 이라는 뜻 그대로다.
        #     매핑처럼 천천히 굴려야 하는 절차에서는 낮춰 두는 편이 편하다
        #     (예: manual_pwm_max:=90 ≈ 4펄스 ≈ 12.7 km/h — drive_pulse 와 같은 속도).
        DeclareLaunchArgument(
            'manual_pwm_min', default_value='16',
            description='페달을 살짝 밟았을 때의 PWM. ★16 = A보드 프로토콜 하한★ '
                        '(그 아래는 펌웨어가 펄스로 읽어버린다). 순수 비례라 페달 '
                        '초반 1/3 쯤은 유격이 된다 — 바퀴가 실제로 도는 지점이 '
                        'PWM 60 부근이기 때문이다. 그 유격이 거슬리면 60 으로 올려라'),
        DeclareLaunchArgument(
            'manual_pwm_max', default_value='255',
            description='★페달을 끝까지 밟았을 때의 PWM = 수동조종 최고속★ '
                        '기본 255 = A보드 프로토콜 상한(전개). 직접 PWM 은 펌웨어의 '
                        '무보호 경로라 펄스모드 상한(PWM_MAX=170)도 무시한다. '
                        'FF 표 대략치: 60≈1펄스 / 70≈2 / 90≈4 / 150≈16펄스(51km/h). '
                        '※ 이 스택에 AEB 가 없는 것은 의도된 것이다 — 수동조종에서 '
                        '세우는 것은 사람 발뿐이다(위 주석)'),
        DeclareLaunchArgument(
            'throttle_raw_min', default_value='220',
            description='페달을 놓은 것으로 볼 A0 최댓값. 이 이하 → 지령 0. '
                        '실차 휴지 196~208'),
        DeclareLaunchArgument(
            'throttle_raw_max', default_value='950',
            description='페달을 끝까지 밟았을 때 A0. lidar 와 동일(실측 풀 행정 ≈946)'),
        DeclareLaunchArgument(
            'throttle_gamma', default_value='1.4',
            description='페달 개도 곡선. 1=선형. lidar 와 동일 — 살짝 밟으면 저속, '
                        '끝까지 밟으면 PWM 255'),

        # ── 주행 튜닝 (driving.py 상단 상수의 런치 override) ──
        DeclareLaunchArgument(
            'drive_pulse', default_value='4',
            description='★주행 고정 속도[펄스]★ 4 ≈ 12.7 km/h (1펄스 ≈ 3.18 km/h). '
                        '[2026-08-11] driving.py 의 MAX_PULSE_LIMIT(4)로 잘린다 — '
                        '이보다 크게 넣어도 4 로 내려간다'),
        DeclareLaunchArgument(
            'heading_pulse', default_value='3',
            description='헤딩 초기화 중 속도[펄스] 3 ≈ 9.5 km/h'),
        # ★[2026-08-11] steer_kp 가 사라졌다★ 순수추종(Pure Pursuit)으로 바뀌어
        #   비례게인이 없다. 조향 세기는 lfd_omega_n 으로 만진다.
        DeclareLaunchArgument(
            'lfd_omega_n', default_value='0.97',
            description='순수추종 목표 고유진동수[rad/s] → LFD = v·√2/ω_n. '
                        '★낮추면 LFD 가 길어져 조향이 완만해진다★ (사행·발산 시 낮춘다). '
                        '구 white 로스백 실측 발산임계가 1.2 이므로 그 위로 올리지 말 것'),
        DeclareLaunchArgument(
            'lfd_min_m', default_value='2.3',
            description='LFD 하한[m]. ★최소회전반경(1.49m)보다 넉넉히 커야 한다★ — '
                        '낮추면 목표점이 회전반경 안으로 들어와 제자리를 돈다'),
        DeclareLaunchArgument(
            'wheelbase_m', default_value='1.25',
            description='축거[m] 실측 1250mm. 순수추종 조향식의 L'),
        # ★[2026-08-11] 조향 전달계 실측 보정★ B보드의 ±40° 는 가변저항 행정 이름일
        #   뿐 도로휠각이 아니다(실측 링키지비 1.75). driving.py 상단 주석 참고.
        DeclareLaunchArgument(
            'steer_plant_gain', default_value='1.26',
            description='pot 지령 / 도로휠각. 126표본 최소자승 실측(1.75)을 '
                        '[2026-08-12] 가변저항 하드리밋 재측정에 맞춰 ×0.7185 '
                        '재환산한 값. ★코너를 여전히 크게 돌면 올린다(더 꺾는다)★'),
        DeclareLaunchArgument(
            'steer_understeer', default_value='5.17',
            description='언더스티어 계수 [deg/(m/s²)]. 같은 반경이라도 속도가 오르면 '
                        '더 꺾어야 하는 양. 고속 코너에서 부족하면 올린다'),
        DeclareLaunchArgument(
            'cte_ki', default_value='0.30',
            description='CTE 적분 게인 [deg(도로휠)/(m·s)]. ★크게 잡지 말 것★ — '
                        '순수추종이 못 지우는 정상상태 측방편향(실측 +0.13~0.27m)을 '
                        '천천히 지우는 용도다. 0 이면 적분항을 끈다. 기여는 '
                        'driving.py 의 CTE_I_MAX_DEG(2.5°)로 한 번 더 잘린다'),
        # ── 종점 접근 [2026-08-12 도입 → 2026-08-19 개편] ── driving.py '종점 접근' 절
        DeclareLaunchArgument(
            'goal_brake_m', default_value='20.0',
            description='★종점 감시 창[m]★ (2026-08-19 로 뜻이 바뀌었다 — 종전에는 '
                        '"이 거리에서 2단을 문다"였고 5.0 이었다). 지금은 이 창 안에서 '
                        '★속도로 계산한 지점★ 에 리니어 1단을 문다 — 4펄스면 15.8m 가 '
                        '필요하므로 5.0 으로는 좁다. ★0 으로 주면 이 단계 자체가 '
                        '꺼진다★(도착 시에만 리니어를 무는 종전 거동). 5.0 을 주면 '
                        '사실상 2026-08-12 의 거동(5m 에서 2단 백스톱)으로 돌아간다. '
                        '남은 호길이와 직선거리가 둘 다 이 안일 때만 걸린다 — 순환 '
                        '코스 출발점 오작동 방지'),
        DeclareLaunchArgument(
            'goal_brake1_ms2', default_value='1.30',
            description='★종점 1단 제동의 가정 감속도 [m/s²]★ 체결 지점이 이 값으로 '
                        '정해진다(작을수록 일찍 문다). 기본 1.30 은 2026-08-19 실차의 '
                        '구동차단 1단 실측이다. ⚠️ 올리면 늦게 물어 종점을 지나칠 수 '
                        '있고, 내리면 일찍 서서 크립 구간만 길어진다 — ★모르면 내리는 '
                        '쪽★. 로그의 goal_phase=1 구간 gps_kmh 기울기가 실측값이다'),
        DeclareLaunchArgument(
            'goal_brake2_backstop', default_value='true',
            description='1단으로는 못 세운다고 계산되면 ★리니어 2단으로 올린다★. '
                        '이것이 종점 통과를 막는 마지막 방벽이다(2026-08-12 의 5m 2단 '
                        '접근제동이 필요할 때만 되살아나는 것과 같다). ★끄지 말 것★ — '
                        '끄면 1단이 안 들을 때 차가 종점을 그대로 지나간다'),
        DeclareLaunchArgument(
            'goal_creep_kmh', default_value='4.0',
            description='접근제동 중 GPS 속도가 이 밑으로 내려오고 종점이 1.2m 넘게 '
                        '남았으면 리니어를 풀고 1펄스 크립으로 마저 간다 — ★정지한 차를 '
                        '다시 떼어내는 것이 제일 어렵기 때문에★ 구르는 채로 넘긴다. '
                        '완전히 선 경우에는 크립 재출발 킥이 떼어낸다'),
        DeclareLaunchArgument(
            'wp_reach_m', default_value='0.9',
            description='마지막 WP 도착 허용반경[m]. ★[2026-08-11] 0.2 → 0.9★ — '
                        'GPS 가 5Hz 라 0.2m 창은 접근속도에서 통째로 건너뛴다(실측 '
                        '최근접 0.24·0.38m 로 두 주행 다 도착 판정 실패). 반경과 '
                        '별개로 마지막 WP 를 지나쳤는지도 함께 본다 — driving.py '
                        'run_follow() 의 종점 판정 참고'),
        DeclareLaunchArgument(
            'require_rtk', default_value='true',
            description='헤딩 초기화·코스 융합에 품질 문턱을 걸지. ★true 권장★ — '
                        'false 면 품질을 아예 안 본다(SPS σ4m 도 통과). 문턱의 높이는 '
                        '아래 min_quality 가 정한다'),
        DeclareLaunchArgument(
            'min_quality', default_value='2',
            description='★헤딩 초기화·코스 융합을 허용할 최저 GPS 품질★ '
                        '1=SPS 2=DGPS 3=RTK_FLOAT 4=RTK_FIXED. 기본 2(DGPS) — '
                        '2026-08-18 실차에서 DGPS 헤딩 오차가 RTK Fixed 와 같은 급'
                        '(−0.8~−4.4° vs +2.0°)임을 로그 재생으로 확인해 내렸다. '
                        '⚠️ 1(SPS)로 내리지 말 것 — HeadingEstimator 가 5m 를 가면 '
                        '정확도 미달이어도 강제확정하므로 엉뚱한 헤딩이 박힌다. '
                        '4 로 올리면 Fixed 가 아닐 때 아예 출발하지 않는다(안전하지만 '
                        '조용한 실패 — 로그의 "헤딩 표본이 쌓이지 않는다" 경고를 볼 것)'),

        # ── 저장 위치 (비우면 white1/paths.py 규칙) ──
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='경로(맵) CSV 폴더. 비우면 소스트리의 white1/gps_data/'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='주행 기록 CSV 폴더. 비우면 소스트리의 white1/ros2bag/'),
        # ★[2026-08-14] 음원이 white1/sound 로 옮겨 왔다★ nxde 의 sound 노드는
        #   기본적으로 자기 패키지(<nxde>/sound)를 보므로, 여기서 경로를 넘겨 준다.
        DeclareLaunchArgument(
            'sound_dir', default_value=paths.sound_dir(),
            description='음성 안내 mp3 폴더. 기본은 소스트리의 white1/sound/ — '
                        '안내 문구가 전부 white1 의 사건이라 음원의 주인도 white1 이다'),
    ] + camera_launch.declare_args(cam_dev)     # 카메라·신호등 인자

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B — nxde 패키지, ★수정 없이 사용★
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='nxde',
        executable='arduino',
        name='arduino',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':             LaunchConfiguration('baud'),
            'steer_invert':     LaunchConfiguration('steer_invert'),
            'stop_brake_level': LaunchConfiguration('stop_brake_level'),
            'manual_pulse_max': LaunchConfiguration('manual_pulse_max'),
            # ★[2026-08-27] 페달 = lidar 와 같은 직접 PWM★
            #   /drive_pulse_cmd=15 는 라벨이다. 실제 구동은 /drive_pwm_cmd (풀=255).
            #   이 네 값을 안 넘기면 A보드가 목표펄스 15(PID, PWM_MAX=170)로 가서
            #   lidar 풀스로틀보다 현저히 느리다.
            'manual_use_pwm':   LaunchConfiguration('manual_use_pwm'),
            'manual_pwm_min':   LaunchConfiguration('manual_pwm_min'),
            'manual_pwm_max':   LaunchConfiguration('manual_pwm_max'),
            'throttle_raw_min': LaunchConfiguration('throttle_raw_min'),
            'throttle_raw_max': LaunchConfiguration('throttle_raw_max'),
            'throttle_gamma':   LaunchConfiguration('throttle_gamma'),
            'exclude_ports':    exclude_for_arduino,
        }],
        condition=IfCondition(use_arduino),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] iAHRS IMU → /imu. 자체 2초 재연결 + VID/PID 재탐색.
    # ═══════════════════════════════════════════════════════════════════
    iahrs = Node(
        package=package_name,
        executable='iahrs',
        name='iahrs_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'port':           LaunchConfiguration('imu_port'),
            'baud':           115200,
            'send_tf':        True,
            'rescan':         True,
            'sync_period_ms': LaunchConfiguration('imu_sync_period_ms'),
            'exclude_ports':  [gps_dev],
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [계측] /imu 적분 속도계 → /speed [km/h]  [2026-08-12 신설]
    #    driving 의 ★저속 펄스 보정★ 이 이 값을 쓴다. 안 떠 있어도 driving 은
    #    그대로 돌고, 보정이 엔코더로 내려갈 뿐이다(measured_pulse).
    # ═══════════════════════════════════════════════════════════════════
    speed = Node(
        package=package_name,
        executable='speed',
        name='speed_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] u-blox GPS → /fix. 외부 패키지라 respawn 에 의존한다.
    #    ★RTCM 보정을 넣지 않는다★ RTK 여부는 수신기가 만들고 GGA quality 로 드러난다.
    # ═══════════════════════════════════════════════════════════════════
    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{'port': LaunchConfiguration('gps_port'), 'baud': 115200}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [GPS 후처리] gps — /fix + /imu → /gps_fused   ★[2026-08-18] 신설★
    #    ① RTK Fixed / Float 판정 : nmea_navsat_driver 는 GGA q4(Fixed, 2cm)와
    #       q5(Float, 수 m)를 ★같은 status.status=2 로★ 내보낸다. 두 값을 가르는
    #       것은 position_covariance 이고 그 판정을 이 노드가 한다(gps.py 헤더 ①절).
    #    ② 5Hz fix 사이 공백을 IMU 로 메워 20Hz 가상좌표를 만든다 — 4.42m/s 에서
    #       마지막 틱의 좌표는 최대 0.88m 낡아 있었다.
    #    ★없으면 차가 서 있는다★ driving 이 /gps_fused 를 못 받아 GPS 두절로 판단한다
    #    (안전한 실패). driving 이 그 경우를 구별해서 경고한다.
    #    ★매핑에는 영향이 없다★ mapping 은 /fix 원값을 직접 받는다.
    # ═══════════════════════════════════════════════════════════════════
    gps_post = Node(
        package=package_name,
        executable='gps',
        name='gps_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{'min_quality': LaunchConfiguration('min_quality')}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [자율주행] driving — 헤딩·상태기계·추종·브레이크를 맡는다
    #    (위치는 gps 노드가 만든다 — [2026-08-18])
    # ═══════════════════════════════════════════════════════════════════
    driving = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'data_dir':      LaunchConfiguration('data_dir'),
            'drive_pulse':   LaunchConfiguration('drive_pulse'),
            'heading_pulse': LaunchConfiguration('heading_pulse'),
            'lfd_omega_n':   LaunchConfiguration('lfd_omega_n'),
            'lfd_min_m':     LaunchConfiguration('lfd_min_m'),
            'wheelbase_m':   LaunchConfiguration('wheelbase_m'),
            'steer_plant_gain': LaunchConfiguration('steer_plant_gain'),
            'steer_understeer': LaunchConfiguration('steer_understeer'),
            'cte_ki':        LaunchConfiguration('cte_ki'),
            'goal_brake_m':   LaunchConfiguration('goal_brake_m'),
            'goal_creep_kmh': LaunchConfiguration('goal_creep_kmh'),
            'goal_brake1_ms2': LaunchConfiguration('goal_brake1_ms2'),
            'goal_brake2_backstop': LaunchConfiguration('goal_brake2_backstop'),
            'wp_reach_m':    LaunchConfiguration('wp_reach_m'),
            'require_rtk':   LaunchConfiguration('require_rtk'),
        }],
    )

    mapping = Node(
        package=package_name,
        executable='mapping',
        name='mapping_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': LaunchConfiguration('data_dir')}],
        condition=IfCondition(use_mapping),
    )

    # 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음).
    record = Node(
        package=package_name,
        executable='record',
        name='record_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'output_dir': LaunchConfiguration('record_dir')}],
        condition=IfCondition(use_record),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [안내] 음성 — nxde 패키지, 구독 전용
    #    ★respawn 을 걸지 않는다★ 되살아날 때마다 시작 안내(one_launch_1)가 다시
    #    나오는 편이 조용히 없는 것보다 헷갈린다. 죽어도 주행에는 영향이 없다.
    # ═══════════════════════════════════════════════════════════════════
    sound = Node(
        package='nxde',
        executable='sound',
        name='sound_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'sound_dir': LaunchConfiguration('sound_dir')}],
        condition=IfCondition(use_sound),
    )

    # 구독 전용 계기판. respawn 없음 — 창을 닫으면 끝이고 주행은 그대로다.
    hud = Node(
        package=package_name,
        executable='hud',
        name='hud_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'show_camera': LaunchConfiguration('use_camera'),
            'data_dir': LaunchConfiguration('data_dir'),
        }],
        condition=IfCondition(use_hud),
    )

    args += [
        # ══════════════════════════════════════════════════════════════════
        #  라이다 구간 이양 [2026-09-01] — 위 헤더의 '라이다 구간 이양' 절
        # ══════════════════════════════════════════════════════════════════
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='OS1-32 드라이버 + mppi 회피 노드를 함께 띄울지. '
                        '★라이다는 USB 가 아니라 유선 LAN 이다★ (eno1 192.168.6.100). '
                        'false 면 종전처럼 GPS 추종만 돈다 — 그때 CSV 의 L 구간을 '
                        '만나면 driving 이 정지한다(아무도 몰지 않는 채로 달리지 않는다)'),
        DeclareLaunchArgument(
            'use_lidar_rviz', default_value='false',
            description='mppi 코스트맵·롤아웃을 RViz 로 볼지 (책상 시험용)'),
        DeclareLaunchArgument(
            'flip_lidar_xy', default_value='true',
            description='라이다 xy 180° 반전. lidar cone_lidar.yaml 과 같은 값이다. '
                        '차 앞 3m 에 사람이 서서 코스트맵 전방에 점이 생기면 맞다'),
        DeclareLaunchArgument(
            'lidar_speed', default_value='1.768',
            description='L 구간 순항속도 [m/s]. 1.768 = 2펄스 ≈ 6.4 km/h. '
                        '★정지 재출발에서 4펄스는 피한다★ (A보드 재가속 함정 — '
                        'lidar/include/lidar/kasa_units.hpp 2절)'),
        DeclareLaunchArgument(
            'lidar_pulse', default_value='2',
            description='L 구간 펄스 상한. lidar_speed 와 짝을 맞춰 둘 것'),
    ]

    return LaunchDescription(args + [
        # 음성 먼저 — '런치했다'는 안내가 하드웨어 탐색보다 늦으면 의미가 없다
        sound,
        # 하드웨어
        arduino,
        iahrs,
        speed,
        gps,
        # GPS 후처리 — driving 이 이 노드의 출력에 의존한다(먼저 띄운다)
        gps_post,
        # 자율주행
        driving,
        mapping,
        record,
        hud,
        # 라이다(드라이버 + mppi 회피) — use_lidar 가 true 일 때만 실제로 만든다.
        #   ★driving 뒤에 둔다★ mppi 는 /lidar_permit 을 기다리는 쪽이고, 그 발행자가
        #   driving 이다. 순서가 동작을 바꾸지는 않지만(둘 다 신선도로 판단한다)
        #   로그를 읽을 때 누가 누구를 기다리는지가 순서로 드러난다.
        OpaqueFunction(function=_lidar_actions),
    ] + camera_launch.actions(package_name, cam_format, NODE_ENV, RESPAWN_DELAY))
