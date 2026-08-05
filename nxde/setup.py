from setuptools import setup

package_name = 'nxde'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ [2026-08-05] 런치파일이 없다 ★
        #   이 패키지는 '아두이노 A/B 보드와 통신하는 노드들'만 담는다. 실행은 전부
        #   ros2 run 이며, 자율주행을 할 때는 white 의 one_launch.py 가 arduino 노드를
        #   함께 띄운다(구 g.launch.py 는 삭제). 카메라 캘리브·GPS/IMU 는 white 소관.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='test@test.com',
    description='Arduino layer for the white autonomous stack: kasa A/B two-board bridge, '
                'GUI/joystick teleop, and a pre-flight hardware check (Ubuntu 22.04 only)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ══════════════════════════════════════════════════════════════
            #  노드 4개 — 전부 ros2 run 으로 띄운다 (런치파일 없음)
            # ══════════════════════════════════════════════════════════════
            # ★차량 구동의 필수 노드★ A/B 2보드 통신 전담. 이게 없으면 아무것도 안 움직인다.
            #   자율주행 시에는 white one_launch.py 가 이 노드를 함께 띄운다.
            #   단독 조종 시에는 손으로: ros2 run nxde arduino
            'arduino  = nxde.arduino:main',
            # 마우스·키보드 GUI 조종 (하드웨어 검증용).  ros2 run nxde master
            #   ⚠️ joystick / one_launch.py(driving_node) 와 동시에 쓰지 말 것 —
            #      /cmd_vel_raw 발행자가 겹친다(창 상단에 경고가 뜬다).
            'master   = nxde.master:main',
            # 조이스틱 메가 보드("J,"/"U,") 조종.  ros2 run nxde joystick
            #   ★자율주행 모드(B보드 D5)에서만 작동하고, 영점 후 SWA 를 눌러야 시작한다★
            'joystick = nxde.joystick:main',
            # ★런치 전 하드웨어 연결 점검★ 보고하고 종료한다.  ros2 run nxde check
            #   메가 A/B · 조이스틱 · GPS(NMEA GGA 의 RTK quality) · IMU · 카메라
            'check    = nxde.check:main',
        ],
    },
)
