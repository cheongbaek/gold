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
            #   GPS(nmea_serial_driver)는 외부 패키지라 여기 없고 one_launch.py 가 띄운다.
            'iahrs   = white1.iahrs:main',      # 6축 IMU → /imu (순수 드라이버)
            'speed   = white1.speed:main',      # /imu 적분 → /speed [km/h]
            # ── 주행 ──
            #   ★driving 이 GPS·IMU 융합 + 상태기계 + 추종을 전부 맡는다★
            #   구 white 의 gps_imu 노드는 없다.
            'driving = white1.driving:main',
            'mapping = white1.mapping:main',    # /fix 만 보고 경로 수집
            'prompt  = white1.prompt:main',     # CLI (경로 선택·상태 표시)
            'prompt_g = white1.prompt_g:main',  # 위와 같은 기능의 tkinter GUI
            'record  = white1.record:main',     # 주행 구간 토픽 → CSV
        ],
    },
)
