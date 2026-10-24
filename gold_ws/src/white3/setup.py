from setuptools import setup
import os
from glob import glob

package_name = 'white3'

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
        (os.path.join('share', package_name, 'sound'), glob('sound/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='yellsote9@gmail.com',
    description='kasa 자율주행 white3 — white1 10/17 주행 코드 + white1 현재 카메라 코드',
    license='TODO',
    entry_points={
        'console_scripts': [
            'driving = white3.driving:main',   # 10/17 판 (+ C 기본 0.0)
            'gps     = white3.gps:main',       # 10/17 판 /fix + /imu → /gps_fused
            'prompt  = white3.prompt:main',    # 10/17 판 CLI — traffic_timer 를 한 프로세스로 띄운다
            'record  = white3.record:main',    # 10/17 판 주행 CSV
            'hud     = white3:hud',            # white1 hud (현재 판) 를 white3 폴더로
        ],
    },
)
