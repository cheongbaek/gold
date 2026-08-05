# gold — kasa(금색차) 자율주행 스택 백업

`white_ws` ROS2 워크스페이스(`/home/mad2/white_ws/white_ws`) 안의 알고리즘·런치 파일만
선별해 백업하는 저장소입니다. 빌드 산출물(`build`/`install`/`log`)과 워크스페이스에 함께
있던 다른 프로젝트(`kasa_ws`, `skku_ws` 등)는 포함하지 않습니다.

## 구성

```
white/   자율주행 판단 스택 (ament_python 패키지)
  launch/one_launch.py   GPS·IMU·카메라 연결 + 자율주행 노드 통합 런치
  white/
    driving.py           경로추종 (순수추종 + PID + LFD 스케줄)
    gps_imu.py           GPS/IMU/엔코더 융합 → /ego_state
    perception.py        카메라 인지 (차선 polyfit + 신호등 검출)
    camera_judgment.py   차선 계측 브리지 + 신호등 게이트 + 리니어 브레이크
    mapping.py           경로 수집(매핑)
    prompt.py            CLI 메뉴
    sensor_monitor.py     센서 상태 대시보드
    iahrs.py             iAHRS IMU 드라이버
    ports.py             GPS·IMU·카메라 장치 경로 식별
    kasa_units.py        차량 단위 환산 단일 소유자 (펄스↔m/s, 조향 부호 규약)
    route_remodeler.py   수집 경로 후처리(직선/코너 리모델링)

nxde/    아두이노(kasa A/B 2보드) 통신 계층 (ament_python 패키지, 런치파일 없음)
  nxde/
    arduino.py    A/B 2보드 시리얼 브리지 — 차량 구동의 필수 노드
    master.py     마우스·키보드 GUI 조종 (하드웨어 검증용)
    joystick.py   조이스틱 조종 (자율주행 모드 한정 + 영점 후 SWA 필요)
    check.py      런치 전 하드웨어 연결 점검 (GPS RTK 상태 포함) 후 종료
```

## 실행

```bash
ros2 run nxde check                    # 0) 하드웨어 연결 점검
ros2 launch white one_launch.py        # 1) GPS·IMU·카메라·아두이노 + 자율주행
ros2 run white prompt                  # 2) CLI 메뉴 (별 터미널)
```

자세한 노드 구성·토픽 계약·안전장치는 `nxde/README.md` 참고.

## 이 저장소의 성격

**작업 백업**이 목적이며, 원본 개발은 `/home/mad2/white_ws`(별도 git 저장소)에서 이루어집니다.
이 저장소는 그 워크스페이스의 `src/nxde`·`src/white` 스냅샷만 독립적으로 관리합니다.
