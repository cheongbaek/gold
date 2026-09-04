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
            #  실행 대상 8개 — 전부 ros2 run 으로 띄운다 (런치파일 없음)
            #    ※ tts·kill 은 ROS 노드가 아닌 작업용 도구다
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
            # ★영상 기록★ 인지가 보는 화면(/image_raw)을 파일로 적는다.
            #   ros2 run nxde video    → <nxde 루트>/video/cam-<날짜>_<시각>.mp4
            #   ★Ctrl-C 로 끝내면 그 시점까지 재생 가능한 상태로 닫힌다★
            #   구독만 하므로 제어에 끼어들지 않는다. 장치를 직접 열지 않아 usb_cam 과
            #   다투지 않는다(그쪽이 죽으면 신호등 인지가 죽는다 — video.py 헤더 참고).
            #   용량이 크다 : 1080p 기준 분당 20~60MB. 줄이려면 scale:=0.5
            'video    = nxde.video:main',
            # ★음성 안내★ sound/*.mp3 를 사건에 맞춰 기본 스피커로 재생한다.
            #   구독만 하므로 제어에 끼어들지 않는다.  ros2 run nxde sound
            #   (white806 one_launch.py 가 use_sound:=true 로 함께 띄운다)
            'sound    = nxde.sound:main',
            # ★돌고 있는 ROS2 를 한 번에 끝낸다★  ros2 run nxde kill
            #   launch 의 종료가 질척거릴 때(hud 가 SIGKILL 까지 15초, gps 가
            #   트레이스백, os_driver 가 errorprocessing) 쓰는 도구다. 자기 자신과
            #   자기 조상만 빼고 전부 SIGKILL 한 뒤 시리얼 큐·FastDDS 공유메모리를
            #   초기화한다. ★ROS 노드가 아니다★ (rclpy 를 import 하지 않는다)
            #   미리 볼 때: ros2 run nxde kill --dry-run
            'kill     = nxde.kill:main',
            # ★대화형 TTS★ 문장을 입력하면 그 자리에서 읽어 준다(edge-tts + tkinter).
            #   sound/*.mp3 안내 음성을 만들거나 문구를 귀로 확인하는 작업용 도구다.
            #   ROS 노드가 아니고 ★인터넷이 필요하다★.  ros2 run nxde tts
            #   필요: pip install --user edge-tts pygame
            'tts      = nxde.tts:main',
        ],
    },
)
