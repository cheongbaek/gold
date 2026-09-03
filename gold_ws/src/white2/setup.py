from setuptools import setup
import os
from glob import glob

package_name = 'white2'

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
        # 카메라 캘리브레이션(camera_info) — one_launch.py 가 share 경로로 읽는다.
        (os.path.join('share', package_name, 'calibration'),
            glob('calibration/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='domi',
    maintainer_email='domi@todo.todo',
    description='white 패키지를 1/5카(헤네스 브룬 T870) 원본 규약으로 되돌린 판 '
                '(속도 m/s 직결·단일 아두이노 motor 노드·조향 +좌/-우 규약).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── 하드웨어 (GPS·IMU·카메라·아두이노 전부 white2 소관) ──
            'iahrs          = white2.iahrs:main',
            # [white2] motor 노드를 되살렸다 — 1/5카의 단일 아두이노 Mega
            #   (C/S 프레임, 300틱 엔코더) 전용 브리지다. kasa 의 A/B 2보드·nxde 패키지는
            #   이 차량에 맞지 않으므로 쓰지 않는다.
            'motor          = white2.motor:main',
            # ── 자율주행 ──
            'gps_imu        = white2.gps_imu:main',
            'mapping        = white2.mapping:main',
            'driving        = white2.driving:main',
            'prompt         = white2.prompt:main',
            'sensor_monitor = white2.sensor_monitor:main',
            # ── 기록 ──
            #   자율주행 모드 + prompt 주행 구간만 골라 토픽을 CSV 로 남긴다.
            #   one_launch.py 가 함께 띄운다(use_record:=false 로 끌 수 있다).
            #   저장 위치: <white2 패키지>/ros2bag/rec_<날짜>_<시각>_<경로명>/
            'record         = white2.record:main',
            # ── 카메라 융합 ──
            'perception       = white2.perception:main',        # 인지(차선 polyfit + 신호등)
            'camera_judgment  = white2.camera_judgment:main',   # /lane_metrics 브리지 + 신호등 게이트
        ],
    },
)
