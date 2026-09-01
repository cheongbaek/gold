from setuptools import setup
import os
from glob import glob

package_name = 'white1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            sorted(set(glob('launch/*.launch.py')) | set(glob('launch/*.py')))),
        # ★[2026-08-19] 카메라 캘리브레이션 — camera_model.py 가 share 에서 읽는다★
        #   어안 왜곡보정 계수의 단일 소유자다. 재캘리브하면 이 yaml 만 바꾼다.
        (os.path.join('share', package_name, 'calibration'),
            glob('calibration/*.yaml')),
        # ★[2026-08-14] 음성 안내 음원 — nxde/sound 에서 옮겨 왔다★
        #   .gitignore 가 *.mp3 를 막으므로 새로 clone 하면 이 폴더는 비어 있다
        #   (glob 이 빈 목록이 되어 설치도 조용히 건너뛴다). sound 노드는 '음원 없음'
        #   경고만 한 번 내고 계속 돈다 — 다시 만들려면 `ros2 run nxde tts`.
        (os.path.join('share', package_name, 'sound'), glob('sound/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='yellsote9@gmail.com',
    description='kasa GPS+IMU 최소 추종 자율주행 스택 (카메라 없음)',
    license='TODO',
    entry_points={
        'console_scripts': [
            # ── 센서 드라이버 ──
            #   GPS 수신기 드라이버(nmea_serial_driver)는 외부 패키지라 여기 없고
            #   one_launch.py 가 띄운다. 아래 gps 는 그 /fix 를 ★후처리★ 하는 노드다.
            'iahrs   = white1.iahrs:main',      # 6축 IMU → /imu (순수 드라이버)
            'speed   = white1.speed:main',      # /imu 적분 → /speed [km/h]
            # ★[2026-08-18] gps 신설★ /fix + /imu → /gps_fused
            #   ① RTK Fixed / Float 판정(status.status 로는 구별 불가 — 그쪽 헤더 ①절)
            #   ② 5Hz fix 사이 공백을 IMU 로 메워 20Hz 가상좌표
            #   ★매핑은 이 노드를 거치지 않는다★ mapping 은 /fix 원값을 직접 받는다.
            'gps     = white1.gps:main',
            # ── 주행 ──
            #   ★driving 이 헤딩·상태기계·추종을 맡는다★ 구 white 의 gps_imu 노드는 없고,
            #   위치는 [2026-08-18] 부터 gps 노드가 만든다.
            'driving = white1.driving:main',
            'mapping = white1.mapping:main',    # /fix 만 보고 경로 수집
            'prompt  = white1.prompt:main',     # CLI (경로 선택·상태 표시)
            #  ★[2026-08-14] prompt_g(tkinter GUI)를 삭제했다★ 화면이 둘이면 각자
            #   다른 대기 상태를 들고 있게 되고, 안전 게이트도 두 곳에 두게 된다.
            #   화면은 prompt(CLI) 하나로 간다.
            'record  = white1.record:main',     # 주행 구간 토픽 → CSV
            #  ★구독 전용 계기판★ F1 상면도 + 게이지. 제어 토픽은 발행하지 않는다.
            #    ros2 run white1 hud     (이미 떠 있는 스택 위에 얹는다)
            'hud     = white1.hud:main',
            # ── 카메라 ──
            #   ★[2026-08-14] 신호등 인지·정지★ 빨간불이면 리니어 2단, 사라지거나
            #   초록불이면 해제. 개입 허락은 DRIVE_RUN 또는 master 의 체크박스.
            'traffic_light = white1.traffic_light:main',
        ],
    },
)
