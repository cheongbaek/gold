#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
braketest.launch.py ― ★제동검차 전용 런치★ [white2 / 2026-09-12 개편]
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white2 braketest.launch.py
    ros2 launch white2 braketest.launch.py goal_lat:=37.1234567 goal_lon:=127.1234567
    ros2 launch white2 braketest.launch.py route:=subcourse.csv  # ★그 CSV 의 마지막 점으로★
    ros2 launch white2 braketest.launch.py pick_route:=false     # ★선택 창 없이★ 상단 상수로
    ros2 launch white2 braketest.launch.py nolidar:=true         # ★라이다 미사용★ S 지점 / 끝점에서만 정지

  ★출발 조건★ [2026-10-16 점검] — 셋이 다 서야 출발한다(그 전에는 펄스 0 으로 기다리며 이유를 찍는다)
    ① D5 = 자율주행   ② E-STOP 해제 — 체결돼 있으면 ★풀고 1.5 s 뒤★ 출발(B보드가 리니어를 빼는 시간)
    ③ 라이다 사슬이 살아 있음 — nolidar:=true 면 생략.  (그 밖에 GPS 품질 · 목표 거리 20~400 m)

  ★[2026-10-16] 런치를 띄우면 먼저 '경로 CSV 선택' 창이 뜬다★ — 고른 CSV 의 ★마지막 점★
  이 목표 좌표가 된다(경로 모양은 따라가지 않는다 — 출발 위치에서 그 점까지 직선).
  파일 이름은 보지 않는다 : ★.csv 면 어느 폴더의 무엇이든★ 고를 수 있고, 위도·경도 열
  (latitude/longitude 또는 lat/lon)이 없으면 이유를 보여 주고 창을 다시 띄운다.
  ★취소하면 아무 노드도 띄우지 않고 런치가 내려간다★ (하드웨어는 고른 뒤에 뜬다).
  창이 뜨지 않는 경우 — route:= 나 goal_lat/goal_lon 을 줬을 때(그쪽이 이긴다),
  pick_route:=false, 화면이 없는 SSH(경고를 찍고 braketest.py 상단 ROUTE·GOAL 로 간다).
    ros2 launch white2 braketest.launch.py drive_pulse:=8
    ros2 launch white2 braketest.launch.py launch_pulse:=12       # ★출발만 낮춘다★
    ros2 launch white2 braketest.launch.py lateral_offset_m:=0.0  # ★선을 안 옮긴다★
    ros2 launch white2 braketest.launch.py t_preview:=1.2       # ★좁은 차로★
    ros2 launch white2 braketest.launch.py drive_pwm:=140       # ★직접 PWM★

  ★[2026-09-12] 경로 CSV 추종을 버렸다★ 기준은 ★목표 좌표 한 점★ 이고, 주행은
  '출발 위치 → 그 점' 을 잇는 ★직선 한 줄★ 을 프리뷰로 추종한다. 목표 좌표의
  단일 소유자는 ★braketest.py 상단의 GOAL_LAT / GOAL_LON★ 이다(런치 인자는 그것을
  덮는 임시 수단). 근거 전문은 braketest.py 헤더.

  띄우는 것 (one_launch.py 에서 ★측정에 필요한 것만★ 남겼다):
      lidar/aeb.launch.py   ★라이다 정지 시스템 그대로★ (통째로 include)
                            ※ [2026-09-10] braketest 는 이 사슬이 ★실제로 프레임을
                              흘리기 시작한 뒤에★ 출발한다(require_lidar)
                            = ouster.launch.py(OS1-32 드라이버) + cone_lidar_node
      lidar/pedal_drive_node ★AEB 확정·래치★ stop_signal → /aeb_stop
                            (구동은 발행하지 않는다 — 그 파일 헤더 참고)
      → 정지 사슬이 lidar/one_launch.py 와 ★완전히 같다★. braketest 는 끼어들지
        않고 GPS + 지정속도 주행만 한다
      nxde/arduino          A/B 2보드 시리얼 브리지  ★없으면 아무것도 안 움직인다★
      white2/iahrs          6축 IMU → /imu
      white2/speed          /imu 적분 속도계 → /speed
      nmea_navsat_driver    GPS 수신기 → /fix
      white2/gps            /fix + /imu → /gps_fused (RTK 판정 + 20Hz 보간)
      white2/braketest      ★이 시험의 본체★ (driving 대신 들어간다)
      white2/record         토픽 → CSV (ros2bag/) — 나중에 그래프로 되짚는다

  ★띄우지 않는 것과 그 이유★
      driving   — 이 시험의 반대말이다. 코너 감속·종점 접근제동·CTE 적분·라이다
                  이양이 전부 '제동거리를 재는 일'에 개입한다.
      mapping   — 시험 중에 경로를 쓸 이유가 없다.
      prompt    — ★런치 = 출발★ 이다(아래). 고를 것이 없다.
      hud       — 띄워도 되지만 기본은 끈다(use_hud:=true 로 켤 수 있다).
      drive_lidar_node · drive_gps_node
                — lidar 패키지의 ★주행★ 노드들이다. /cmd_vel_raw 발행자가 겹친다.
                  (pedal_drive_node 는 이름과 달리 구동을 안 내므로 함께 띄운다)
      카메라·신호등 — 제동거리와 무관하고, /cmd_vel_raw 발행자만 늘린다.

════════════════════════════════════════════════════════════════════════════════
 ⚠️ 실행 절차 — ★런치 = 출발★ 이다
════════════════════════════════════════════════════════════════════════════════
  0. ★라이다가 붙었는지 먼저 본다★ — 유선 LAN 이다(USB 아님).
        ip -br addr show eno1     # 192.168.6.100/24 여야 한다
        ping -c2 192.168.6.11
     인지만 따로 확인하려면 : ros2 launch lidar aeb.launch.py use_rviz:=true
        차 앞 3 m 에 사람 → /cone_lidar_node/obstacle_distance 가 3.0 이면 맞다
  1. `ros2 run nxde check` 로 A/B보드·GPS·IMU 연결을 먼저 확인한다.
  2. ★차 앞을 비운다.★ 기본 10펄스(31.8 km/h)에서 트리거 지점 뒤로 13~20 m 를
     더 간다(braketest.py 헤더의 안전절 계산).
     ★목표 좌표(또는 장애물) 뒤로 최소 30 m 가 필요하다.★
  3. D5 스위치를 ★자율주행★ 으로, E-STOP 을 해제한다.
  4. ★차를 목표 좌표를 일직선으로 바라보는 자리에 세운다.★ 이것이 이 시험의
     유일한 전제다 — 출발 방위를 그 선에서 빌리기 때문이다. 전제가 틀리면
     ★출발 1초 뒤 GPS 코스가 알려 주고 차가 스스로 선다★(조준 오차 8° 한계).
  5. 런치를 띄운다. GPS 품질이 서고 라이다가 살아나면 ★그 자리에서 출발한다★
     (헤딩 초기화 구간이 없다).
     → 확인하고 출발시키고 싶으면 `auto_start:=false` 로 띄우고,
       `ros2 topic pub -1 /braketest_go std_msgs/msg/Bool '{data: true}'`
  6. 달리는 동안 ★먼저 오는 것★ 에서 선다 — 셋 다 리니어 2단
     ① 라이다 장애물 → arduino AEB + braketest /brake_level 2단
     ② ★S 지점 통과★ [2026-10-16] → 고른 CSV 의 terrain 열 S/s ★첫 행★ 을 지나는 즉시
        (S 가 없으면 이 줄은 없다 — 종전대로 ① 또는 ③)
     ③ ★목표 좌표(끝점) 도달★ → FAIL-SAFE
  7. ★[2026-10-16] 선 뒤에는 런치를 끌 때까지 리니어 2단을 유지한다★ 결과 표를 찍고
     기다린다(10초마다 안내). ★Ctrl-C 로 런치를 끄는 순간 리니어 0단★ — nxde/arduino 의
     종료 정리가 B보드에 'x,0' 을 직접 쓴다. (종전 : 스스로 풀고 1.5 s 뒤 내려갔다)

  ⚠️ ★목표 좌표 뒤로 최소 30 m 가 비어 있어야 한다★ — 목표 좌표는 "여기서 서라"
     가 아니라 ★"여기서 밟기 시작하라"★ 다. 31.8 km/h 에서 그 뒤로 13~20 m 를 더 간다.
  ⚠️ 라이다 ROI 는 전방 2.0~6.0 m 인데 10펄스 2단 정지거리가 12.9~20.4 m 다 —
     ★보고 나서 서기에는 원리적으로 부족하다.★ 이 시험은 '얼마나 못 서는가' 를
     재는 것이다. 사람·부술 수 있는 물건을 장애물로 쓰지 말 것.

  언제든 세우는 방법 : E-STOP(하드웨어) · D5 를 수동조종으로 · Ctrl-C
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white2 import paths, ports
from white2.braketest import pick_csv

PACKAGE = 'white2'
NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_COLORIZED_OUTPUT': '1'}
RESPAWN_DELAY = 2.0


def _pick_route(context, cfg, num):
    """★경로 CSV 선택 창★ [2026-10-16] → (route 파라미터 값, 그만둘 사유 or None).

    노드를 하나도 띄우기 ★전에★ 부른다 — 고르기 전에 arduino 가 붙지 않고, 취소하면
    아무것도 움직이지 않는다. 고른 파일은 ★절대경로★ 로 넘긴다(gps_data 밖이어도 된다).
    """
    route = str(cfg('route').perform(context)).strip()
    pick = str(cfg('pick_route').perform(context)).strip().lower() in ('true', '1', 'yes')
    goal_arg = (num('goal_lat') != 0.0 and num('goal_lon') != 0.0)
    if route or goal_arg or not pick:
        return route, None
    start = paths.data_dir(str(cfg('data_dir').perform(context)).strip())
    try:
        got = pick_csv(
            start, title='제동검차 — 목표 경로 CSV 선택 (★마지막 점 = 목표★)',
            log=print)
    except RuntimeError as e:
        print(f"    [경로] ⚠️ 선택 창을 못 띄운다({e}) — braketest.py 상단 "
              f"ROUTE·GOAL_LAT/GOAL_LON 으로 간다. 고르려면 route:=<파일>.csv")
        return '', None
    if got is None:
        return '', '경로 CSV 선택을 취소했다 — 출발하지 않는다'
    path, (lat, lon), n, stop = got
    print(f"    [경로] ✅ {path}")
    print(f"    [경로]    좌표 {n}행 · ★목표 = 마지막 점 ({lat:.7f}, {lon:.7f})★ — "
          f"차를 이 점을 일직선으로 바라보게 세울 것")
    if stop is None:
        print("    [경로]    S 표시 없음 — 라이다 장애물 / 끝점 도달에서 선다")
    else:
        print(f"    [경로]    ★S 지점 = {stop[2]}행 ({stop[0]:.7f}, {stop[1]:.7f}) — 지나는 즉시 2단★"
              + (f"  (S {stop[3]}행 중 첫 행)" if stop[3] > 1 else ""))
    return path, None


def _setup(context, *args, **kwargs):
    """장치 경로를 확정한 뒤 노드를 만든다 (one_launch.py 와 같은 방식)."""
    def cfg(name):
        return LaunchConfiguration(name)

    def num(name, default=0.0):
        """★좌표는 여기서 실수로 풀어서 넘긴다★

        LaunchConfiguration 을 그대로 파라미터에 실으면 launch_ros 가 문자열을
        보고 형을 짐작한다 — 사용자가 `goal_lat:=37` 처럼 소수점 없이 적으면 int
        로 짐작해 ★형 불일치로 노드가 뜨지 않는다.★ 좌표는 손으로 적는 값이라
        그 사고가 가장 나기 쉬운 자리다. OpaqueFunction 안이라 context 가 있으므로
        여기서 float 로 풀어 ★파이썬 실수★ 로 넘긴다.
        """
        try:
            return float(str(cfg(name).perform(context)).strip() or default)
        except (ValueError, TypeError):
            print(f"    [braketest] ⚠️ {name} 값을 실수로 읽지 못했다 — {default} 로 둔다")
            return float(default)

    #  ★경로 CSV 선택 창★ — 장치를 찾기 전에 고른다(위 _pick_route).
    route, quit_why = _pick_route(context, cfg, num)
    if quit_why:
        print(f"    [경로] ⏹️ {quit_why}")
        return [EmitEvent(event=Shutdown(reason=quit_why))]

    #  ★[2026-10-16] nolidar★ true = 라이다 정지 사슬(aeb.launch.py + pedal_drive_node)을
    #  띄우지 않고, braketest 도 라이다를 기다리지 않는다. 정지는 S 지점 / 끝점뿐이다.
    use_lidar = str(cfg('nolidar').perform(context)).strip().lower() not in ('true', '1', 'yes')
    print("    [라이다] " + ("사용 — 라이다 사슬이 살아난 뒤에 출발한다" if use_lidar else
                           "★미사용 (nolidar:=true)★ — 장애물 보호 없음. S 지점 / 끝점에서만 선다"))

    #  udev 링크 → VID/PID → (없으면) 링크 경로 그대로. one_launch.py 와 같은 방식이다.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B
    # ═══════════════════════════════════════════════════════════════════
    #  ★auto_direct_pwm★ 여기서만 연다. drive_pwm 이 0 이면 arduino 는 종전과
    #  완전히 같게 동작한다(0~15 는 어차피 펄스 모드다). 16~255 를 실었을 때만
    #  콤마 2값으로 나가고, 그때는 A보드에서 PID·슬루·폭주감지·기동블랭킹이
    #  전부 빠진다 — braketest.py 헤더의 '속도 지령' 절 참고.
    arduino = Node(
        package='nxde', executable='arduino', name='arduino',
        output='screen', additional_env=NODE_ENV,
        parameters=[{
            'baud': 115200,
            'auto_direct_pwm': True,
            # ★비상정지를 '켜는' 유일한 지점★ arduino 기본 aeb_brake_level=0 = 꺼짐.
            #   lidar/one_launch.py 와 ★같은 값★ 을 넘긴다(2 = 풀브레이킹 / 1.0s).
            #   이 셋이 없으면 /aeb_stop 이 아무리 서도 arduino 는 무시한다.
            'aeb_brake_level': cfg('aeb_brake_level'),
            'aeb_stale_s':     cfg('aeb_stale_s'),
            'aeb_topic':       '/aeb_stop',
            'exclude_ports': [gps_dev, imu_dev],
        }],
    )

    iahrs = Node(
        package=PACKAGE, executable='iahrs', name='iahrs_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'port': imu_dev, 'baud': 115200, 'send_tf': True, 'rescan': True,
            'sync_period_ms': 50, 'exclude_ports': [gps_dev],
        }],
    )

    speed = Node(
        package=PACKAGE, executable='speed', name='speed_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
    )

    nmea = Node(
        package='nmea_navsat_driver', executable='nmea_serial_driver',
        name='nmea_serial_driver', output='screen',
        additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'port': gps_dev, 'baud': 115200}],
    )

    gps = Node(
        package=PACKAGE, executable='gps', name='gps_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'min_quality': cfg('min_quality')}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [본체] 브레이크 시험 — driving 대신 들어간다
    # ═══════════════════════════════════════════════════════════════════
    braketest = Node(
        package=PACKAGE, executable='braketest', name='braketest_node',
        output='screen', additional_env=NODE_ENV,
        parameters=[{
            'data_dir':      cfg('data_dir'),
            'route':         route,        # ★선택 창이 고른 절대경로 (또는 route:=)★
            #  ★목표 좌표★ 비우면(0) braketest.py 상단 GOAL_LAT/GOAL_LON 을 쓴다
            'goal_lat':      num('goal_lat'),
            'goal_lon':      num('goal_lon'),
            'drive_pulse':   cfg('drive_pulse'),
            'drive_pwm':     cfg('drive_pwm'),
            'launch_pulse':       cfg('launch_pulse'),
            'cruise_switch_kmh':  cfg('cruise_switch_kmh'),
            'launch_max_s':       cfg('launch_max_s'),
            'lateral_offset_m':   cfg('lateral_offset_m'),
            'cte_abort_m':   cfg('cte_abort_m'),
            'steer_limit_deg': cfg('steer_limit_deg'),
            't_preview':     cfg('t_preview'),
            'ay_max':        cfg('ay_max'),
            'steer_trim_deg': cfg('steer_trim_deg'),
            #  ★라이다가 살아난 뒤에 출발한다★ [2026-09-10]
            #  nolidar 의 반대를 물려준다 — 라이다를 안 띄우는 구성에서
            #  영영 기다리는 일이 없게 한다.
            'require_lidar':   use_lidar,     # ★nolidar:=true 면 기다리지 않는다★
            'auto_start':    cfg('auto_start'),
        }],
    )

    record = Node(
        package=PACKAGE, executable='record', name='record_node',
        output='screen', additional_env=NODE_ENV,
        parameters=[{'record_dir': cfg('record_dir')}],
        condition=IfCondition(cfg('use_record')),
    )

    hud = Node(
        package=PACKAGE, executable='hud', name='hud_node',
        output='screen', additional_env=NODE_ENV,
        condition=IfCondition(cfg('use_hud')),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [인지] 라이다 정지 시스템 — ★lidar/aeb.launch.py 를 통째로 include★
    # ═══════════════════════════════════════════════════════════════════
    #  = ouster.launch.py(OS1-32 드라이버) + cone_lidar_node
    #  ★그 조각을 그대로 쓴다★ lidar/one_launch.py 도 같은 것을 include 한다 —
    #  여기서 노드를 다시 선언하면 감지 파라미터의 소유자가 둘이 되어, 한쪽만
    #  고치는 사고가 난다(cone_lidar.yaml 이 유일한 소유자여야 한다).
    #
    #  ⚠️ ★인자 이름이 params_file 이면 안 된다★ IncludeLaunchDescription 은 부모의
    #  LaunchConfiguration 을 자식에게 물려준다 — ouster.launch.py 의 같은 이름
    #  인자를 덮어써서 드라이버가 cone_lidar.yaml 을 읽고 죽는다(2026-08-25 실제).
    #  aeb.launch.py 가 그래서 cone_params_file 로 이름을 갈라 두었고, 우리는
    #  ★그 인자를 아예 넘기지 않아★ 자식 기본값을 쓰게 둔다.
    lidar_pkg = get_package_share_directory('lidar')
    aeb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'aeb.launch.py')),
        condition=IfCondition(str(use_lidar).lower()),
        launch_arguments={'use_rviz': cfg('use_lidar_rviz')}.items(),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [AEB 확정·래치] pedal_drive_node — ★lidar/one_launch.py 와 같은 파라미터★
    # ═══════════════════════════════════════════════════════════════════
    #  cone_lidar_node ─stop_signal─▶ 이 노드 ─/aeb_stop─▶ arduino (구동차단+리니어)
    #  ★braketest 는 이 사슬에 끼어들지 않는다★ 반응속도가 lidar one_launch.py 와
    #  같아야 하므로(사용자 지시), 판정·래치·제동을 전부 그쪽 노드에 맡긴다.
    #
    #  ※ 이름이 'pedal_drive' 지만 ★구동을 발행하지 않는다★ —
    #    /cmd_vel_raw·/control_state·/brake_level 을 하나도 내지 않고 /aeb_stop 만
    #    낸다(그 파일 헤더 '발행하지 않는 것' 절). 그래서 braketest 의 주행 지령과
    #    겹치지 않는다.
    pedal_aeb = Node(
        package='lidar', executable='pedal_drive_node', name='pedal_drive_node',
        output='screen', additional_env=NODE_ENV,
        condition=IfCondition(str(use_lidar).lower()),
        parameters=[{
            'stop_signal_topic':       '/cone_lidar_node/stop_signal',
            'obstacle_distance_topic': '/cone_lidar_node/obstacle_distance',
            'aeb_stop_topic':          '/aeb_stop',
            'engage_frames':           cfg('aeb_engage_frames'),
            'min_engage_s':            cfg('aeb_min_engage_s'),
            'release_clear_s':         cfg('aeb_release_clear_s'),
            'signal_stale_s':          cfg('aeb_stale_s'),
        }],
    )

    # ★braketest 가 끝나면 런치 전체를 내린다★ [2026-10-16] 이제 이것이 타는 것은 목표를
    #   못 정해 ★시작도 못 했을 때★ 뿐이다 — 섰을 때는 사람이 끌 때까지 2단을 유지한다.
    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=braketest,
                      on_exit=[EmitEvent(event=Shutdown(
                          reason='braketest 완료 — 런치를 내린다'))]))

    return [aeb, pedal_aeb, arduino, iahrs, speed, nmea, gps, braketest,
            record, hud, shutdown_on_done]


def generate_launch_description():
    return LaunchDescription([
        # ══════════════════════════════════════════════════════════════════
        #  ★★ 목표 좌표 — 이 시험의 기준선은 '출발 위치 → 이 점' 이다 ★★
        # ══════════════════════════════════════════════════════════════════
        #  ★기본은 braketest.py 상단 상수다★ 여기 인자는 그것을 덮는 임시 수단이고,
        #  값의 단일 소유자는 파일 상단 GOAL_LAT / GOAL_LON 이다(사용자 지시).
        DeclareLaunchArgument(
            'goal_lat', default_value='0.0',
            description='★목표 위도★ 0 이면 braketest.py 상단 GOAL_LAT 를 쓴다'),
        DeclareLaunchArgument(
            'goal_lon', default_value='0.0',
            description='★목표 경도★ 0 이면 braketest.py 상단 GOAL_LON 을 쓴다'),
        DeclareLaunchArgument(
            'route', default_value='',
            description='★경로 CSV★ 이 CSV 의 ★마지막 점★ 을 목표로 쓴다 — '
                        'braketest.py 상단 ROUTE·GOAL_LAT/GOAL_LON 을 이긴다(goal_lat/'
                        'goal_lon 인자만 이보다 앞). 이름만 주면 gps_data/ 에서 찾는다. '
                        '기록 파일명도 이 이름이 된다. 못 읽으면 출발하지 않는다. '
                        '경로를 추종하지는 않는다 — 이 노드는 ★직선 한 줄★ 만 본다'),
        DeclareLaunchArgument(
            'pick_route', default_value='true',
            description='★런치 때 경로 CSV 선택 창을 띄운다★ 고른 CSV 의 마지막 점이 목표. '
                        '파일 이름 무관 — .csv 면 무엇이든. 취소하면 출발하지 않는다. '
                        'route:= 나 goal_lat/goal_lon 을 주면 창을 띄우지 않는다. '
                        'false 면 braketest.py 상단 ROUTE·GOAL_LAT/GOAL_LON 을 쓴다'),
        DeclareLaunchArgument(
            'drive_pulse', default_value='12',
            description='★순항 목표펄스 0~15★ 기본 12 [2026-10-17] — 통합 보드(kasa_1017)는 지령보다 '
                        '낮게 돈다(지령 9 → 실측 ≈7.3, 23 km/h). 12 면 ≈30.5 km/h 로 본다. '
                        'braketest.py 상단 DRIVE_PULSE 와 같은 값이며 여기가 이긴다. '
                        '★출발은 launch_pulse 로 민다★ (2단 속도 루틴)'),
        #  ★[2026-09-13] 2단 속도 루틴 — 제동검차는 30 km/h 를 넘겨야 한다★
        #   출발만 15펄스로 밀어 30 km/h 도달을 앞당기고, GPS 로 그 속도를 확인하는
        #   순간 순항(drive_pulse)으로 내린 뒤 제동한다. 한 번 내려오면 래치된다.
        DeclareLaunchArgument(
            'launch_pulse', default_value='15',
            description='★출발 구간 목표펄스★ 기본 15 (A보드 TARGET_MAX 와 같다). '
                        'drive_pulse 이하로 주면 2단 루틴이 꺼진다(단일 속도 주행)'),
        DeclareLaunchArgument(
            'cruise_switch_kmh', default_value='33.0',
            description='★GPS 속도★ 가 이 값을 넘으면 순항으로 내린다. '
                        '[2026-10-17] 30 → 33 — 통과선 30 km/h 위에서 내려와 순항이 30 위에 머물게'),
        DeclareLaunchArgument(
            'launch_max_s', default_value='12.0',
            description='출발 구간 시간 상한[s] — GPS·엔코더가 둘 다 안 걸려도 '
                        '이 시간이 지나면 순항으로 내린다(내리는 방향 방벽)'),
        #  ★[2026-09-13 저녁] 기준선을 옆으로 평행이동한다★
        DeclareLaunchArgument(
            'lateral_offset_m', default_value='0.20',
            description='S→G 직선을 ★진행방향 기준 왼쪽★ 으로 이만큼 평행이동한다 '
                        '[m]. + = 왼쪽 / − = 오른쪽 / 0 = 옮기지 않는다. '
                        '평행이동이라 진행거리·목표 도달 판정은 바뀌지 않는다'),
        DeclareLaunchArgument(
            'drive_pwm', default_value='0',
            description='0 이 아니면 ★이쪽이 이긴다★ — A보드 직접 PWM 16~255. '
                        'PID·슬루·폭주감지·기동블랭킹이 전부 빠지는 무보호 경로다. '
                        '구동계 개입 없는 순수 관성 상태로 제동에 들어가고 싶을 때만'),

        # ══════════════════════════════════════════════════════════════════
        #  ★★ 조향 — 튜닝 손잡이는 t_preview 와 ay_max 둘뿐이다 ★★
        # ══════════════════════════════════════════════════════════════════
        DeclareLaunchArgument(
            't_preview', default_value='1.5',
            description='★프리뷰 시간 [s]★ = 이득을 정하는 유일한 값. '
                        'L_p = t_preview × v (8.84 m/s 에서 13.3 m, ω_n 0.94 rad/s). '
                        '★줄이면 정밀하지만 위상여유를 먹고, 키우면 매끄럽지만 '
                        '벌어진 채로 간다★ — 차로가 좁으면 1.2 가 첫 손잡이다'),
        DeclareLaunchArgument(
            'ay_max', default_value='2.0',
            description='★횡가속도 상한 [m/s²]★ pot 상한이 여기서 유도된다 '
                        '(8.84 m/s 에서 pot ≈12.6°). 속도가 달라져도 뜻이 변하지 '
                        '않는다. 필터된 실측 횡가속이 이 값의 1.5배를 넘으면 중단'),
        DeclareLaunchArgument(
            'steer_limit_deg', default_value='12.0',
            description='★pot 지령의 절대 뚜껑 [deg]★ ay_max 유도값이 저속에서 '
                        '커지는 것을 자른다. [2026-09-12] 5.0 → 12.0 : 5° 는 실측 '
                        '전달비로 환산하면 도로휠 0.72°(횡가속 0.79) 뿐이라 ★항상 '
                        '포화★ 했고, 포화가 곧 계전기 진동이었다. 대신 전달비를 '
                        '주행 중에 재서 ★조이는 쪽으로만★ 자동 보정한다'),
        DeclareLaunchArgument(
            'steer_trim_deg', default_value='0.0',
            description='★조향 영점 트림 [pot deg]★ + 면 우. 직진인데 한쪽으로 '
                        '계속 밀리면 지난 로그 steer_measured_deg 의 평균을 부호 '
                        '반대로 적는다. B보드 시리얼 `a` 로 영점을 다시 잡는 것이 정공법'),
        DeclareLaunchArgument(
            'cte_abort_m', default_value='3.0',
            description='기준선에서 이만큼 벗어나면 스스로 2단을 물고 시험을 접는다'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='true = ★런치 = 출발★ 준비되는 대로 스스로 굴러간다. '
                        'false 면 /braketest_go 에 true 가 올 때까지 기다린다'),
        DeclareLaunchArgument(
            'nolidar', default_value='false',
            description='★true = 라이다 미사용★ [2026-10-16] 라이다 정지 시스템(ouster 드라이버 '
                        '+ cone_lidar_node + pedal_drive_node)을 띄우지 않고, 라이다를 기다리지 '
                        '않고 출발한다. 정지는 S 지점 / 끝점 도달뿐(장애물 보호 없음). '
                        'false(기본) = 라이다 사용 — 라이다가 살아난 뒤에 출발한다. '
                        '(종전 use_lidar 를 대신한다)'),
        DeclareLaunchArgument(
            'use_lidar_rviz', default_value='false',
            description='RViz 로 라이다 ROI 를 보면서 돌린다'),
        #  ★lidar/one_launch.py 와 같은 기본값★ — 반응속도를 그쪽과 맞추는 것이
        #  이 런치의 요구사항이므로, 값을 새로 발명하지 않고 그대로 가져온다.
        DeclareLaunchArgument(
            'aeb_brake_level', default_value='2',
            description='AEB 가 물 리니어 단수. ★0 이면 비상정지가 통째로 꺼진다★'),
        DeclareLaunchArgument(
            'aeb_stale_s', default_value='1.0',
            description='/aeb_stop · stop_signal 이 이보다 낡으면 판단자 사망으로 본다'),
        DeclareLaunchArgument(
            'aeb_engage_frames', default_value='1',
            description='pedal_drive_node 의 확정 프레임 (cone_lidar confirm_frames 뒤 한 번 더)'),
        DeclareLaunchArgument(
            'aeb_min_engage_s', default_value='1.0',
            description='한 번 물면 최소 이만큼 유지 — 리니어 왕복 방지'),
        DeclareLaunchArgument(
            'aeb_release_clear_s', default_value='1.5',
            description="'비었다' 가 이만큼 이어져야 해제 (사각지대 통과 방어)"),
        DeclareLaunchArgument(
            'min_quality', default_value='2',
            description='gps 노드 품질 문턱 (1=SPS 2=DGPS 3=FLOAT 4=FIXED)'),
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='경로 CSV 폴더. 비우면 소스트리의 white2/gps_data/'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='기록 CSV 폴더. 비우면 소스트리의 white2/ros2bag/'),
        DeclareLaunchArgument('use_record', default_value='true',
                              description='주행 기록 CSV 를 남긴다'),
        DeclareLaunchArgument('use_hud', default_value='false',
                              description='상면도 HUD 를 함께 띄운다(구독 전용)'),
        OpaqueFunction(function=_setup),
    ])
