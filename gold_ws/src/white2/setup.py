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
        # ★[2026-10-21] 카메라 캘리브레이션은 white1 의 것을 쓴다★ (camera_model 이 white1 공용 —
        #   share/white1/calibration). white2/calibration 은 같은 yaml 이라 지웠다.
        # ★[2026-08-14] 음성 안내 음원 — nxde/sound 에서 옮겨 왔다★
        #   .gitignore 가 *.mp3 를 막으므로 새로 clone 하면 이 폴더는 비어 있다
        #   (glob 이 빈 목록이 되어 설치도 조용히 건너뛴다). sound 노드는 '음원 없음'
        #   경고만 한 번 내고 계속 돈다 — 다시 만들려면 edge-tts 를 직접 쓴다
        #   (`edge-tts --voice ko-KR-InJoonNeural --text "문장" --write-media x.mp3`).
        (os.path.join('share', package_name, 'sound'), glob('sound/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='windo',
    maintainer_email='yellsote9@gmail.com',
    description='kasa 자율주행 투트랙의 white2 — 신호 판단만 다르고 나머지는 white1 을 쓴다',
    license='TODO',
    entry_points={
        'console_scripts': [
            # ★[2026-10-21] white2 에 남은 것은 white1 과 다른 셋뿐이다★ (white2/__init__.py)
            #   gps·iahrs·speed·mapping·traffic_light 는 white1 실행 파일을 쓴다 —
            #   white2 의 런치가 package='white1' 로 띄우고, 폴더는 WHITE_STACK=white2 로 갈린다.
            #   braketest 는 white1 에만 있다(ros2 launch white1 braketest.launch.py).
            'driving = white2.driving:main',   # T·A·B 태그 · traffic_light 가 2단으로 세운다
            'prompt  = white2.prompt:main',    # CLI (경로 선택·상태 표시) — 타이머 없음
            'record  = white2.record:main',    # 주행 구간 토픽 → CSV (/traffic_timer 열 없음)
            #  ★구독 전용 계기판★ white1 의 hud 를 white2 폴더로 — ros2 run white2 hud
            'hud     = white2:hud',
        ],
    },
)
