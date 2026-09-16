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
            # ★조이스틱 + LCD 단독 점검 도구★  ros2 run nxde joystick
            #   [2026-09-16] 조종 노드가 아니다 — ★차를 움직이는 명령을 하나도
            #   발행하지 않는다★(/cmd_vel_raw·/control_state·/brake_level 전부).
            #   스틱·버튼을 화면에 그리고, 환산값(펄스·조향·제동)을 조이스틱 LCD 로
            #   직접 보내 배선을 눈으로 확인한다. ★단독 실행 전용★ — 실제 주행에서
            #   조이스틱을 잡고 모는 것은 arduino 노드다(use_joystick, 기본 켜짐).
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
            # ★domichat 채팅방을 읽어 주는 TTS★  ros2 run nxde tts
            #   <PC명>_TTS_M(남성) / <PC명>_TTS_W(여성) 두 방을 없으면 만들어
            #   구독하고, 올라오는 대화를 그대로 읽는다(방 비밀번호 = PC 이름).
            #   ROS 노드가 아니고 ★인터넷이 필요하다★(edge-tts 온라인 합성).
            #   서버·계정은 tts.py 상단 [1] 파라미터 절에서 고친다.
            #   필요: pip install --user edge-tts  +  ffplay(ffmpeg) 나 mpg123
            #   ※ ★pygame 은 더 이상 쓰지 않는다★ — 오디오 장치를 붙들어
            #     nxde/sound 의 안내 음성과 서로 밀어냈다(tts.py 헤더 참고).
            'tts      = nxde.tts:main',
        ],
    },
)
