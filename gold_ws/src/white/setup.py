from setuptools import setup
import os
from glob import glob

package_name = 'white'

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
        #   ★[2026-08-05] 소유권이 white 로 돌아왔다★ 2026-08-04 에는 usb_cam 을
        #   nxde 의 g.launch.py 가 띄우면서 nxde/calibration 으로 옮겨갔는데, 카메라가
        #   다시 이 런치 소관이 되었으므로 되돌린다(nxde 쪽 사본은 삭제됐다).
        (os.path.join('share', package_name, 'calibration'),
            glob('calibration/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='domi',
    maintainer_email='domi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── 하드웨어 (GPS·IMU·카메라는 white 소관 — 2026-08-05 환원) ──
            #   ★iahrs 가 되살아났다★ 2026-08-04 에 nxde 로 옮겨갔던 노드를 되가져왔다.
            #   GPS(nmea_serial_driver)·카메라(usb_cam)는 외부 패키지라 여기 없고
            #   one_launch.py 가 직접 띄운다. 아두이노는 nxde 의 arduino 노드가 담당한다.
            'iahrs          = white.iahrs:main',
            # ★ [kasa 이식] motor 노드는 제거했다 ★
            #   white 차량의 단일 아두이노(C/S 프레임, 300틱 엔코더) 전용이라 kasa 의
            #   A/B 2보드에 쓸 수 없다. 그 역할은 nxde 패키지의 arduino 노드가 대신하며,
            #   one_launch.py 가 함께 띄운다(단독으로는 `ros2 run nxde arduino`).
            #   white/motor.py 파일 자체는 프로토콜 참고용으로 남겨 두었지만 실행되지 않는다.
            # 'motor        = white.motor:main',
            # ── 자율주행 ──
            'gps_imu        = white.gps_imu:main',
            'mapping        = white.mapping:main',
            'driving        = white.driving:main',
            'prompt         = white.prompt:main',
            'sensor_monitor = white.sensor_monitor:main',
            # ── 기록 ──
            #   자율주행 모드 + prompt 주행 구간만 골라 토픽을 CSV 로 남긴다.
            #   one_launch.py 가 함께 띄운다(use_record:=false 로 끌 수 있다).
            #   저장 위치: <white 패키지>/ros2bag/rec_<날짜>_<시각>_<경로명>/
            'record         = white.record:main',
            # ── 카메라 융합 ──
            'perception       = white.perception:main',        # 인지(차선 polyfit + 신호등)
            'camera_judgment  = white.camera_judgment:main',   # /lane_metrics 브리지 + 신호등 게이트
        ],
    },
)
