# arduino : kasa A/B 2보드 아두이노 시리얼 브리지 (Ubuntu 22.04 / ROS2 Humble 전용)
#
# ★ 이 파일의 위치 ★
#   kasa_ws/src/nxde/nxde/arduino.py 에서 통신 로직만 가져와, 토픽 계약을 white 패키지
#   규약으로 바꾼 것이다. **kasa_ws 쪽은 수정하지 않았다** — 저쪽은 /in·/out String
#   프로토콜을 그대로 쓰고, 이쪽은 white 의 Twist/Bool/Int32 토픽을 직접 주고받는다.
#   아두이노 펌웨어(kasa_0730_A.ino / ★kasa_0821_B.ino★)도 무수정 전제다.
#
#   ★[2026-08-21] B보드 출력이 3필드가 되었다★ "P,<조향각>,<A5원본>,<모드>".
#     parse_b 는 ★이 양식 하나만 받는다★ — 구형(0813 이하, "P,<조향각>,<모드>")과의
#     호환을 일부러 두지 않았다. 두 세대를 다 받으려면 필드 개수로 갈라야 하는데
#     (3토큰의 셋째는 모드, 4토큰의 셋째는 A5 로 자리가 겹친다) 그 분기는 보드 하나를
#     쓰는 차에서 값을 못한다. ★펌웨어를 바꾸면 이 함수도 함께 바꾼다★ 가 규약이다.
#     구형을 꽂으면 이 줄이 통째로 버려져 조향각·모드가 얼어붙는다 — 그 증상이
#     보이면 제일 먼저 여기를 의심할 것.
#
# ★ 역할 ★
#   ROS → 보드 :  /cmd_vel_raw (Twist)    linear.x = 주행 목표펄스 0~15
#                                          angular.z = 조향각 -40~40 (★− 좌 / + 우★)
#                 /control_state (Bool)   True = 구동 허용 / False = 정지
#                 /brake_level (Int32)    브레이크 단계 0 / 1 / 2 (선택 — 안 오면 0)
#                 /aeb_stop (Bool)        ★[2026-08-25] 전방 장애물 확정 = 비상정지★
#                                         True 면 ★모드와 무관하게★ 구동을 끊고
#                                         리니어를 aeb_brake_level 단으로 물린다.
#                                         ★aeb_brake_level=0 (기본) 이면 이 토픽은
#                                         통째로 무시된다★ — 아래 (1-1) 참고.
#   보드 → ROS :  /encoder (Int32)              A보드 좌+우 펄스의 ★합★
#                 /steer_angle_measured (Int32) B보드 실측 조향각 (− 좌 / + 우, 그대로 중계)
#                 /vehicle_mode (Bool)          B보드 D5 : True = 자율 / False = 수동조종
#                 /throttle_pedal (Int32)       A보드 A0 쓰로틀 페달 raw 0~1023
#                 /brake_pot (Int32)            ★B보드 A5 리니어 가변저항 raw 0~1023★
#                                               [2026-08-21 / kasa_0821_B.ino] 브레이크
#                                               페달의 ★실제 위치★ 다. 단계(0/1/2)가 아니라
#                                               값 자체라서, '리니어가 시킨 대로 갔는가'와
#                                               '사람이 발로 밟았는가'를 구별할 수 있다
#                                               (수동조종에서는 후자만 움직인다).
#                                               ※ 400 이상이면 B보드가 제동등(D11)을 켠다.
#                 /drive_pulse_cmd (Int32)      ★주행 목표펄스 (0~15 스케일)★
#                                               자율=계획값(=A보드로 실제 나간 값)
#                                               수동조종=페달 환산값(★라벨 전용★ — 실제로
#                                               나가는 것은 아래 /drive_pwm_cmd 다)
#                                               → mapping 노드의 수집 라벨(①)로 쓰인다.
#                                               ★스케일을 절대 바꾸지 않는다★ 수집·로스백이
#                                               전부 0~15 로 기록되어 있다.
#                 /drive_pwm_cmd (Int32)        ★[2026-08-25] A보드로 실제 나간 직접 PWM★
#                                               수동조종에서 페달을 밟는 동안만 16~255,
#                                               그 외(자율·정지·페달 놓음)에는 0 이다
#                                               (= 직접 PWM 경로를 쓰지 않는 상태).
#                 /estop (Bool)                 A·B 중 한쪽이라도 STOP 이면 True
#                 /board_status (String)        "A:1,B:1,ESTOP:0,MODE:1" (진단·로스백용)
#
#   ※ /motor_pwm, /steer_pwm 은 발행하지 않는다 — kasa 펌웨어가 PWM 을 텔레메트리로
#     내보내지 않는다. white 쪽 구독자도 없었으므로(로스백 진단 전용이었다) 그냥 사라진다.
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 단위·부호 규약 (이식에서 제일 많이 틀리는 곳) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  1) /cmd_vel_raw 의 linear.x 는 **m/s 가 아니라 펄스**다.
#     m/s ↔ 펄스 환산은 발행하는 쪽(white/kasa_units.py)이 하고, 이 노드는 정수 클램프만
#     한다. 그래야 이 노드가 차량 제원(타이어·PPR)을 몰라도 되고, 제원이 바뀌어도
#     이 파일을 안 고친다.
#
#  2) ★조향 부호는 ROS 와 보드가 같다 (− 좌 / + 우) → 이 노드는 뒤집지 않는다★
#     [2026-08-04 개정] 예전에는 "ROS 안은 white 부호(+좌), 여기서 반전"이었다. 그런데
#     GUI 의 가로 조향 레버는 왼쪽 끝이 −40 인데 +가 좌회전이면 **레버 방향과 바퀴 방향이
#     반대**가 된다(nxde master 실차 시험에서 확인). 그래서 ROS 토픽 전체를 kasa B보드
#     부호로 통일했다 — 화면·토픽·시리얼·펌웨어가 전부 같은 부호를 쓴다.
#       kasa B보드 : − 좌 / + 우 (kasa_0804_B.ino angleToPot: −40 → RAW_LEFT_LIMIT(576))
#     → steer_invert 기본값은 **False**. 배선/펌웨어를 뒤집었을 때만 True 로 쓴다.
#     ※ driving.py 제어기 내부는 여전히 '+좌'로 튜닝되어 있고, 그 반전은
#       driving.publish_cmd 의 to_ros_steer() 한 줄에서만 일어난다(거기서 이미 끝났다).
#
#  2-1) 브레이크 단계는 /brake_level (Int32) 로 받는다.
#     Twist 에는 브레이크 필드가 없어 별 토픽을 쓴다. 값은 ★0 / 1 / 2 단계★ 이며
#     0~255 PWM 이 아니다(kasa_0804_B.ino). 안 오면 0(놓음)으로 둔다.
#     자율주행 경로에서만 반영된다 — E-stop 과 수동조종에서는 아래 상태판단이 우선한다.
#
#  3) /encoder 는 좌·우 펄스의 **합**이다 (평균이 아니다).
#     합/평균은 어차피 상수배 차이이고, 소비측(white)에서 TICKS_PER_REV 를 2배(192)로
#     잡으면 결과 m/s 는 평균과 완전히 동일하다. 그런데 /encoder 는 Int32 라서 평균을
#     쓰면 (0,1) → 0.5 → 정수화로 정보가 깨진다. 합은 그 손실이 없고 양자화 눈금도
#     절반(0.884 → 0.442 m/s)이 된다. → 이 파일은 합, white 는 192.
#
#  4) 후진은 없다. A보드가 음수를 받지 않으므로 주행값은 항상 0 이상으로 클램프한다.
#     (후진이 필요하면 사람이 수동조종 모드에서 한다는 전제)
#
#  5) 브레이크는 0~255 PWM 이 아니라 **단계 0/1/2** 다 (kasa_0804_B.ino).
#       0 = 놓음 / 1 = 약한 브레이킹(행정 1/3) / 2 = 풀브레이킹
#     범위 밖 값은 B보드가 브레이크 필드만 무시한다.
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 주행 상태 판단 (우선순위 순서 그대로) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 리니어(브레이크)는 ROS 가 보낸 브레이크 필드만큼만 움직인다 ★★
#     펌웨어는 시킨 대로만 작동한다 — 스스로 체결하거나 복귀하지 않는다.
#     따라서 "E-STOP 도 아닌데 리니어가 튀어나왔다" 면 원인은 100% 이 파일이 보낸 값이다.
#     [2026-08-04] 실제로 그랬다 — 아래 (2) 에 '자율→수동 전환 엣지에서 2단 체결' 래치가
#     있었고 스위치를 내리는 순간 리니어가 브레이크 페달을 밟았다.
#     ★그 로직을 완전히 제거했다★ (파라미터 manual_brake_level·manual_release_raw 도 삭제).
#     지금 ROS 가 리니어를 움직이는 경로는 둘뿐이고 전부 '명시적 지시'다:
#       · /brake_level (GUI 레버 · camera_judgment) — 사람/판단이 직접 요청한 값
#       · stop_brake_level  (자율 정지 시, 기본 0)
#
#  ╔══════════════════════════════════════════════════════════════════════════╗
#  ║ ★★ 불변식 : 모드 전환은 절대로 리니어를 체결하지 않는다 (2026-08-05) ★★   ║
#  ╚══════════════════════════════════════════════════════════════════════════╝
#     자율↔수동 전환은 그 자체가 제동 지시가 아니다. 사람이 차를 넘겨주거나 넘겨받는
#     순간이므로, 그때 리니어가 밟히면 가장 위험하다. 전환 엣지에서 브레이크 관련
#     상태를 전부 '풀린' 쪽으로 되돌린다 — _disarm_brakes_on_mode_edge():
#       ① /brake_level 요청 캐시를 0 으로 지운다
#          수동조종 중에도 camera_judgment 는 계속 돌아 /brake_level=2 를 요청할 수 있다
#          (신호등 확정 — 그 노드는 D5 를 보지 않는다). 수동 분기가 0 을 보내므로 그
#          순간엔 무해하지만 값이 캐시에 남아, 자율로 되돌리는 순간 (3)/(4) 가 그것을
#          집어 리니어가 튀어나왔다.
#       ② stop_brake_level 무장을 해제한다
#          그 값이 1 이상이면 전환 직후 (3) 이 그것을 건다 — 그때 /control_state 는 아직
#          False 다(자율주행을 아직 시작하지 않았다). 자율주행이 실제로 구동 허가를 받은
#          뒤(cb_control_state 의 True)에만 다시 무장한다.
#     ★둘 다 '거는' 로직이 아니라 '지우는' 로직이다★ 2026-08-04 에 삭제된 래치를
#     되살린 것이 아니다 — 방향이 정반대다.
#
#  (1) E-stop 중        : A="0", B="x,0"
#      브레이크 0 을 보낸다 — 모드 전환이나 e-stop 자체가 제동 지시는 아니다.
#      "x,0" 의 뜻은 **해제 직후에 적용될 마지막 명령을 안전한 값으로 두는 것**이다
#      (조향 힘빼기 = 사람이 핸들을 잡고 있어도 급조향이 없다).
#
#  (1-1) AEB 비상정지  : A="0", B="<x 또는 마지막 조향각>,<aeb_brake_level>"
#      ★[2026-08-25 신설] 수동조종 중에도 통하는 유일한 제동 경로다★
#
#      ╔══════════════════════════════════════════════════════════════════════╗
#      ║ 왜 /brake_level 로는 안 되는가 — 아래 (2) 는 브레이크를 ★항상 0★ 으로 ║
#      ║ 보낸다. "제동은 사람 발이 한다" 는 2026-08-04/05 의 불변식이다. 그래서 ║
#      ║ 사람이 페달로 몰고 있는 동안 라이다가 장애물을 봐도 /brake_level 을    ║
#      ║ 아무리 발행해도 ★리니어는 움직이지 않는다★ (조용히 아무 일도 안 난다).║
#      ╚══════════════════════════════════════════════════════════════════════╝
#
#      → 그 하나의 경우를 위해 ★이름을 갈라★ 열었다. /brake_level 을 수동조종에
#        통하게 열지 않은 이유는 그것이 ★발행자가 여럿인 요청 토픽★ 이기 때문이다
#        (신호등 인지·GUI 레버가 사람이 운전하는 중에 리니어를 물릴 수 있다 —
#        2026-08-05 에 실제로 문제가 됐던 경로다). 별 토픽이면 "리니어가 왜
#        나왔나" 의 답이 ★AEB 하나로 좁혀진다★.
#
#      ★기본은 꺼져 있다★ aeb_brake_level 기본값이 0 이라, 이 파라미터를 주지
#        않는 런치(white1 one_launch.py 등)에서는 구독만 하고 아무 일도 하지
#        않는다 — 종전 거동과 ★완전히 동일★ 하다. 켜는 것은 지금 lidar 패키지의
#        one_launch.py 하나다.
#
#      ┌ 이 분기가 하는 일 ─────────────────────────────────────────────────┐
#      │ · A보드 = 단일값 "0" ★콤마 2값을 보내지 않는다★ 그래야 펌웨어의       │
#      │   setPulseTarget 이 직접 PWM 모드를 해제하고 코스트로 넘긴다.        │
#      │   ("0,0" 도 같은 경로지만, 직접 PWM 을 안 쓰는 상태에서는 단일값이    │
#      │   이 파일의 기본 규칙이다 — (2) 분기 주석과 같은 이유다)             │
#      │ · 리니어 = aeb_brake_level 단 (권장 2 = 풀브레이킹)                  │
#      │ · 조향 = 수동조종이면 'x'(힘빼기) — ★사람이 핸들을 쥐고 있다★.       │
#      │   자율이면 마지막 조향각 유지(정지 순간 정면 급조향 방지, (3) 과 같다)│
#      └──────────────────────────────────────────────────────────────────────┘
#
#      ★신선도를 본다 (aeb_stale_s)★ 판단 노드는 이 토픽을 ★상태로, 끊기지 않게★
#      낸다(20Hz, true/false 둘 다). 그 시간 넘게 안 오면 해제한다 — 그래야
#        ① '판단자가 죽었다' 와 '장애물이 없다' 가 구별되고
#        ② 죽은 노드가 리니어를 영구히 물고 있는 일이 없다
#        ③ 이 분기가 ★'살아 있는 센서의 사실'★ 이라고 말할 수 있어, 모드 전환
#           불변식(_disarm_brakes_on_mode_edge)의 대상인 '남아 있던 캐시 요청'
#           과 성질이 갈린다. 그래서 모드 엣지에서 이것을 지우지 않는다 —
#           지워도 다음 20ms 에 같은 값이 다시 오고, 그때 앞에 장애물이 있는 것은
#           사실이기 때문이다.
#      ★fail-open 이다★ 끊기면 제동을 푼다. 그 상태는 'AEB 가 없는 수동조종' =
#      원래 상태이고, 사람이 예상하지 못한 정지가 뒤차·경사에서 더 위험하다.
#
#  (2) 수동조종 모드    : A=페달 구동, 안 밟으면 "0", B="x,0"
#      D5 스위치가 개방(모드 0)인 동안. 사람이 핸들과 페달을 직접 잡으므로
#        - 조향은 'x'(힘빼기) — DC모터에 힘이 들어가면 사람이 핸들을 못 돌린다
#        - 브레이크는 ★항상 0★ — 제동은 사람 발이 한다. ROS 가 개입하지 않는다.
#        - 구동은 ★페달뿐★ — manual_use_pwm 이 경로를 가른다(아래).
#
#      ★[2026-08-11] '발을 뗐을 때 /cmd_vel_raw 지정펄스 사용' 경로를 도로 뺐다★
#        [2026-08-07] white806 의 매핑 헤딩 초기화(페달 없이 곧게 굴려 초기 헤딩을
#        잡는 절차)를 위해 열어 둔 경로였는데, 그 절차를 '사람이 페달로 직접 곧게
#        굴리는' 방식으로 바꿔 더 이상 필요 없다. 수동조종 중 소프트웨어가 펄스를
#        대신 낼 길을 남겨 두면 그만큼 사람 조작과 다툴 여지가 생기므로 없앤다.
#
#      ★★ [2026-08-26] 페달 구동 경로를 파라미터로 고른다 (manual_use_pwm) ★★
#        True  = 직접 PWM ("<pwm>,<pwm>", 16~255). 개루프라 밟은 듀티가 유지되고
#                PID·폭주감지가 없다. ★풀 엑셀 = PWM 255★ (펄스 모드 상한 170 무시).
#        False = 목표펄스 (단일값 0~manual_pulse_max). 보드 PID 가 그 속도를 맞추고
#                페달을 떼면 "0" → 코스트(서서히 감속). 펌웨어 PWM_MAX=170 에 묶인다.
#        기본 True — white1·lidar 런치 모두 2026-08-27 이후 PWM 을 전제로 넘긴다.
#
#  (3) /control_state=False : A="0", B="<마지막 조향각>,<stop_brake_level>"
#      driving.py 가 정지를 지시한 상태(instant_stop / 경로 미로드 / STOP 명령).
#      조향각을 0 으로 리셋하지 않고 마지막 값을 유지한다 — 정지 순간에 바퀴가 정면으로
#      튀는 것을 막는다(white/motor.py 가 S,0 만 보내고 조향을 건드리지 않았던 것과 같은 태도).
#
#  (4) 정상 자율주행    : A="<펄스>", B="<조향각>,<stop_brake_level 아님, 0>"
#      ★[2026-08-12] 단, 브레이크가 0 이 아니면 A 는 무조건 "0" 이다★
#      구동과 제동을 동시에 걸면 서로 밀어낸다 — 제동이 걸린 순간부터는 제동이
#      이긴다. 자세한 이유는 _compose() 의 (4) 분기 주석에 적었다.
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 전송 정책 ★★
#   - TX 는 TX_PERIOD_S(0.05s) 타이머에서 돌고, **값이 바뀌었을 때 또는 KEEPALIVE_S 마다**
#     만 실제로 시리얼에 쓴다. 매 주기 무조건 쓰면 B보드 handleLine 이 매번
#     steer_state=ST_ACTIVE 로 되돌려 조향 도달판정(SETTLE_MS=500ms)이 영구히 성립하지
#     않고, PD 가 목표 근처에서 계속 힘을 준다.
#   - A보드 펌웨어에는 무입력 타임아웃이 없다(0713에서 제거). 마지막 명령을 계속 물고
#     있으므로 ★종료 시 정지값이 반드시 시리얼까지 나가야 한다★ → stop_and_close.
#   - A보드로는 ★수동조종에서 페달을 밟는 동안을 빼면★ 항상 **단일값**을 보낸다.
#     콤마 2값만이 16~255 를 '직접 PWM(무보호 경로)' 으로 해석하는데, 자율주행에서
#     그 경로를 쓸 이유가 없고 오발동만 위험하다. ★[2026-08-25] 수동조종의 페달 구동
#     하나만 예외로 열었다★ — 사람 발이 곧 스로틀이어야 하기 때문이다(위 (2) 참고).
#     그래서 콤마 2값이 나가는 곳은 compose() (2) 분기 ★단 한 줄★ 이다. 다른 분기에서
#     콤마가 보이면 그것은 버그다.
#   - ★수동조종 중에는 TX 가 매 주기(20Hz) 나간다★ 페달 raw 가 계속 흔들려 PWM 값이
#     매번 바뀌기 때문이다. A보드는 이것을 문제 삼지 않는다(B보드처럼 도달판정을
#     되돌리는 상태기계가 없고, 직접 PWM 은 값을 그대로 출력할 뿐이다).
#
#  ★★ 연결 정책 (2026-08-04 개편) ★★
#   - ★생성자는 블로킹하지 않는다★ 예전에는 두 보드를 다 찾을 때까지 __init__ 안에서
#     돌았다. 그러면 보드가 안 꽂혀 있는 동안 노드가 spin 조차 못 해서 /board_status 도,
#     구독도 살아나지 않았다. 이제 탐색·재연결은 전부 데몬 스레드가 담당하고, 노드는
#     즉시 뜬다 → 보드가 없어도 나머지 스택(GPS/IMU/카메라/판단)이 정상 기동한다.
#   - ★도중에 끊겨도 재연결한다★ read/write 에서 SerialException·OSError 가 나면 그 보드만
#     닫고 None 으로 떨어뜨린다(_drop_board). 같은 스레드가 그것을 보고 다시 스캔한다.
#     한쪽만 빠지면 나머지 한쪽은 계속 정상 동작한다.
#   - 재연결 중에는 그 보드로 나가는 write 가 조용히 버려진다(send_line 의 ser is None).
#     A보드가 복귀하면 다음 TX 주기에 최신 명령이 즉시 나간다(변경감지 캐시를 비운다).
#
#  ★★ RX 정책 ★★
#   - SERIAL_POLL_S = 0.05 (A·B 텔레메트리 주기 50ms 와 일치).
#   - ★poll 한 번에 들어온 줄을 전부 본다★ [2026-08-27]
#     예전에는 최신 줄 하나만 썼다(latest = texts[-1]). 그런데 수동조종 PWM 경로는
#     A보드로 "<pwm>,<pwm>" 을 20Hz 로 보내고, CH340 클론은 그 TX 를 RX 로 에코하는
#     경우가 있다. 같은 50ms 창에 `S,0,0,512` 와 `180,180` 이 같이 오면 최신 줄이
#     에코가 되어 parse_a 가 조용히 return → ★스로틀 raw 가 0(또는 휴지)에 언다★.
#     타이밍이 맞으면 S, 가 최신이라 되고, 아니면 안 된다 = "가끔 스로틀이 안 받아진다".
#     사람이 Ctrl+C 로 다시 띄우면 그 레이스가 리셋될 뿐 원인은 그대로다.
#   - 그래도 근본적인 측정 공백은 남는다: 펄스 필드는 '직전 20ms 창'의 카운트인데 보고는
#     50ms 마다다 → 50ms 중 30ms 는 계측되지 않는다. gps_imu 의 DR 거리적분이 그만큼
#     거칠다는 뜻이다(펌웨어에 누적 카운터를 넣지 않기로 결정했으므로 감수한다).
#
# ══════════════════════════════════════════════════════════════════════════════
#  실행 : ros2 launch white one_launch.py     ← ★이 노드를 함께 띄운다★
#         ros2 run nxde arduino --ros-args -p baud:=115200   (단독 실행도 가능)
#  정상 동작 중에는 로그를 내지 않는다 (보드감지 / estop·모드 전환 / 오류 시에만).
#
#  ★★ [2026-08-05] 이 노드는 자립형이다 — 어떤 런치도, 다른 패키지도 필요하지 않다 ★★
#    nxde 에는 런치파일이 없다. 차량을 움직이는 최소 단위가 이 노드 하나이므로
#    의존성을 늘리지 않는다:
#      · 포트 탐색표(아두이노 VID + GPS/IMU 제외목록)를 ★이 파일이 직접 소유★한다.
#        예전에는 nxde/ports.py 에 있었고 g.launch.py 가 GPS/IMU 경로를 확정해
#        exclude_ports 로 넘겨줬는데, 런치가 갈라지면서 그 전달 경로가 끊겼다.
#        → 지금은 이 노드가 GPS/IMU VID/PID 를 스스로 보고 그 포트를 제외한다.
#      · white 패키지를 import 하지 않는다(자율주행 스택이 없어도 차는 움직여야 한다).
#    실행 조합:
#      차량 구동만        : ros2 run nxde arduino  +  ros2 run nxde master
#      조이스틱 조종      : ros2 run nxde arduino  +  ros2 run nxde joystick
#      자율주행          : ros2 launch white one_launch.py (이 노드를 포함해 함께 뜬다)

import os
import signal
import sys
import threading
import time
import traceback
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist

import serial
try:
    from serial.tools import list_ports
except Exception:      # pyserial 이 없는 환경에서도 import 는 되게
    list_ports = None
try:
    import termios
except Exception:      # POSIX 가 아닌 곳에서는 HUPCL 을 못 끈다
    termios = None

from nxde.proc_guard import watch_parent


BAUD_RATE = 115200

# ── A보드 프로토콜 상한 (kasa_0730_A.ino) ──
# 단일값 입력은 0~PULSE_MAX 만 유효하고 그 외는 펌웨어가 줄 통째로 무시한다.
PULSE_MIN, PULSE_MAX = 0, 15

# ── ★A보드 직접 PWM (콤마 2값 형식 전용)★ ──
#   "<좌>,<우>" 형식에서만 16~255 가 직접 PWM 으로 해석된다(펌웨어 applySide).
#   같은 형식의 0~15 는 여전히 펄스, 256 이상은 정지다. 단일값 줄은 0~15 펄스만 받는다
#   — 직접 PWM 오발동을 막으려고 펌웨어가 형식으로 갈라 놓은 것이다.
#   ★무보호 경로다★ PID·슬루레이트·폭주감지·기동 블랭킹이 전부 적용되지 않고,
#   펄스 모드 상한(PWM_MAX=170)도 무시하고 받은 값을 그대로 출력한다.
PWM_DIRECT_MIN, PWM_DIRECT_MAX = 16, 255

# 수동조종 페달 → 직접 PWM 환산의 기본 구간 (파라미터 manual_pwm_min/max 의 기본값).
#   min = PWM_DIRECT_MIN : 페달 초반을 ★올려치지 않는다★ (밟은 만큼 순수 비례).
#     ⚠️ 펌웨어 FF 표상 바퀴가 실제로 돌기 시작하는 지점은 PWM 60 부근이다
#        (ffPwmTable 의 첫 칸 = 펄스 1.0 → PWM 60). 그래서 기본값에서는 페달 개도
#        1/3 쯤까지가 '밟아도 안 나가는' 구간이 된다. 실차에서 그 유격이 거슬리면
#        manual_pwm_min 을 60 부근까지 올려라 — 대신 페달을 살짝만 건드려도 바로
#        기동 PWM 이 걸린다(초기 시험에서는 낮은 쪽이 안전하다).
#   max = 255 : A보드 프로토콜 상한. 끝까지 밟으면 전개가 나간다.
#     ★150 으로 두지 않는다★ 그 값은 종전 목표펄스 15 의 FF 표상 PWM(≈147)이라
#     풀 엑셀이어도 듀티가 58% 에 묶였다. 속도를 묶을 때는 런치에서
#     manual_pwm_max 를 낮춘다(90 ≈ 4펄스).
MANUAL_PWM_MIN = PWM_DIRECT_MIN
MANUAL_PWM_MAX = PWM_DIRECT_MAX

# ── B보드 프로토콜 (kasa_0804_B.ino) ──
STEER_DEG_MAX = 40           # 입력 조향각 클램프 (STEER_ANGLE_MAX 와 동일해야 한다)
BRAKE_LEVEL_MAX = 2          # 0 = 놓음 / 1 = 약 / 2 = 풀
STEER_RELEASE_TOKEN = 'x'    # 조향 힘빼기 ([0730-2])

# ── 쓰로틀 페달 raw → 펄스 환산 ──
#   데드존은 여기 최솟값 하나다. 이보다 작거나 같은 raw 는 개도 0 → 지령 0.
#   [2026-07-30] 177 — 당시 휴지 166~172
#   [2026-08-26] 220 — 실차 /throttle_pedal 휴지가 200~208 로 올라 있었다.
#     lidar one_launch 휴지 raw 200~208 이 옛 177 기준을 넘으면 지령이 나갔다.
#     220 이면 208 은 0 이다. 휴지가 더 올라가면 이 값을 더 올린다.
THROTTLE_RAW_MIN = 220       # 페달을 완전히 놓았을 때 (여유 포함)
#   ★[2026-08-27] 800→950★ 실차 /throttle_pedal 풀 밟음이 946 까지 올라간다.
#     옛 800 은 행정 중반(살짝 밟음)에서 이미 개도 1.0 → /drive_pulse_cmd=15,
#     PWM 255 가 나와 엑셀로 속도를 나눌 수 없었다. 0점은 220(휴지 196~197) 유지.
THROTTLE_RAW_MAX = 950       # 끝까지 밟았을 때 (실측 풀 행정 ≈946)
# 페달 개도 곡선. 1.0 = 선형. >1 이면 초반이 완만해져 저속 구간을 발로 나눌 수 있다.
#   1.4 : 살짝 밟으면 저속, 946≈풀. 1.0 으로 두면 포텐이 초반에 급격히 오를 때
#   다시 15 로 붙기 쉽다.
THROTTLE_GAMMA = 1.4
# 지령용 중앙값 창(20Hz 샘플). /throttle_pedal 발행값은 원값 그대로다.
#   실측이 204→581→204 처럼 한 틱에 수백 카운트 튀면, 그 스파이크가 개도 1.0 이 된다.
THROTTLE_MEDIAN_N = 5
ADC_MAX = 1023

# ── 주기 ──
SERIAL_POLL_S = 0.05         # 시리얼 수신 폴링 + 텔레메트리 발행 (보드 50ms 와 일치)
TX_PERIOD_S   = 0.05         # 전송 판정 주기 (실제 write 는 변경/keepalive 시에만)
KEEPALIVE_S   = 1.0          # 값이 안 바뀌어도 이 간격으로는 한 번 재전송
# A보드가 붙어 있는데 S, 텔레메트리(쓰로틀 필드)가 이 시간 넘게 없으면 경고.
#   에코/STOP/식별 실패가 가린 채로 페달이 죽은 것처럼 보이는 상태를 로그로 드러낸다.
TELEMETRY_STALE_S = 1.0
# ★[2026-08-14] 브레이크 해제 유예 [s]★ 자율주행 분기 (4) 에서만 쓴다.
#   0 이 아닌 요청을 받은 뒤 이 시간 동안은 ★푸는 방향으로 내려가지 않는다★.
#   근거 : 로스백 manual-20260814_151206 에서 /brake_level 이 ★0.10초 주기로 2↔0★ 을
#   50회 반복했다(mode·board·estop 정상). 리니어는 물리적으로 왕복하는 장치라
#   그 왕복이 기구에 가장 나쁘다. 상류(traffic_light·driving·master)에도 유예를
#   두었지만, 발행자는 앞으로도 늘 수 있고 ★여기가 모든 요청이 합쳐지는 마지막
#   지점★ 이므로 여기서 한 번 더 막는다. 더 센 값(0→1→2)은 즉시 반영한다.
#   ★[2026-08-14 조정] 1.0 → 0.5, 그리고 유예를 ★여기 하나로 모았다★★
#   종전에는 신호등(1.0) → master·driving(1.0) → 여기(1.0) 가 ★직렬로 쌓여★ 해제가
#   3초 가까이 늦었다. 게다가 중간 소비자가 붙들면 신호등이 0 을 내는 동안 값이
#   0↔2 로 갈려 새 왕복까지 생긴다. 그래서 소비자 유예는 걷어내고,
#       ★신호등 0.5초(빨간불 미감지 확인) + 여기 0.5초(글리치 차단) = 1.0초★
#   로 나눴다. 사람이 체감하는 해제 지연이 정확히 1초다.
BRAKE_RELEASE_HOLD_S = 0.5

# ── 보드 탐색 ──
DETECT_READ_S  = 8.0         # 포트 하나를 A/B 로 식별하기 위해 읽어보는 시간
#   ★5→8 [2026-08-27]★ USB-ACM open 이 DTR 로 보드를 리셋한다. Mega 부트로더
#   (~2s) + 스케치 setup + 첫 S,/P, 가 5초 경계에 걸리면 식별 실패 → close 가
#   또 리셋 → 다음 스캔도 부팅 중. 그 루프가 "스로틀이 안 들어와 Ctrl+C" 다.
# ★E-STOP 중에는 두 보드가 모두 "STOP" 만 내보낸다★ (kasa_0730_A / kasa_0821_B 의
#   sendOutput: estop_active 면 println("STOP") 하고 return). 그래서 그 동안에는
#   'S,'/'P,' 접두어가 아예 나오지 않아 ★역할을 알 수 없다★.
#   그때 5초 만에 포트를 닫으면 두 가지가 나쁘다:
#     ① 로그가 "보드 미발견 — 케이블 확인" 이 되어 원인을 하드웨어에서 찾게 된다
#        (실제로 그렇게 헤맸다. 정답은 'E-STOP 을 풀어라' 다)
#     ② close 가 DTR 을 토글해 ★보드를 리셋한다★ — 8초마다 리셋이 반복되고,
#        E-STOP 을 풀어도 부팅 대기 때문에 붙는 데 시간이 더 걸린다
#   → "STOP" 을 한 줄이라도 보면 ★그 포트에 kasa 보드가 있다는 증거★ 로 받아들이고,
#     포트를 닫지 않고 이 시간까지 해제를 기다린다. 풀리는 즉시 식별된다.
DETECT_ESTOP_HOLD_S = 15.0
DETECT_RETRY_S = 3.0         # 두 보드를 아직 못 찾았을 때 재스캔 간격
DETECT_OPEN_RETRY   = 5      # open 간헐 실패 시 재시도 횟수
DETECT_OPEN_DELAY_S = 1.0
PORT_SETTLE_S       = 0.5    # 한 보드를 이미 연 상태에서 다음 포트를 열기 전 USB 안정화 대기
STOP_FLUSH_S        = 0.15   # 종료 직전 정지값이 실제로 시리얼로 나갈 시간
# 잔재 프로세스를 죽인 뒤, 커널이 fd 를 회수하고 배타 잠금이 풀릴 때까지 기다리는 시간.
#   프로세스가 사라졌다고 곧바로 열리지는 않는다 — USB-serial 은 close 처리가 끝나야 한다.
PORT_RECLAIM_SETTLE_S = 0.6

# 포트는 항상 배타적으로 연다. POSIX 는 기본이 비배타적이라 두 프로세스가 같은
# /dev/ttyACM* 을 동시에 열 수 있고, 그러면 잔재 노드와 새 노드가 같은 보드에 명령을
# 섞어 쓴다. flock(pyserial 3.3+)으로 막는다.
SERIAL_EXCLUSIVE = True

BUSY_HINT = ("재시도로는 풀리지 않습니다 — 이전 런치의 잔재라면 "
             "`pkill -f nxde.arduino` 또는 `fuser -k /dev/ttyACM*` 로 정리하고, "
             "시리얼 모니터가 열려 있으면 닫으세요. "
             "(권한 오류라면 dialout 그룹 확인: sudo usermod -aG dialout $USER)")


def _round_half_away(x):
    """아두이노 round() 매크로와 같은 반올림 (파이썬 내장 round 는 은행가 반올림이라 다름)."""
    return int(x + 0.5) if x >= 0 else -int(-x + 0.5)


def _as_bool(value, default=False):
    """런치 인자는 문자열 'false' 로 온다. bool('false') 는 True 라서 쓰면 안 된다."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    if s in ('1', 'true', 'yes', 'on'):
        return True
    if s in ('0', 'false', 'no', 'off', ''):
        return False
    return default


def is_busy_error(exc):
    """포트가 이미 점유되어(또는 권한이 없어) 열리지 않은 상황인지.

    POSIX: exclusive(flock) 충돌은 EAGAIN(11), 권한 부족은 EACCES(13)."""
    lowered = str(exc).lower()
    return ('permission denied' in lowered
            or 'resource temporarily unavailable' in lowered
            or 'errno 11' in lowered or 'errno 13' in lowered)


# ══════════════════════════════════════════════════════════════════════════════
#  포트 탐색 — ★이 파일이 소유한다★ (구 nxde/ports.py 에서 흡수, 2026-08-05)
# ══════════════════════════════════════════════════════════════════════════════
#  아두이노 계열 USB-serial VID: Arduino 정품(2341) / CH340 클론(1A86) / Arduino LLC(2A03)
#  ★ A/B 두 대가 같은 VID/PID 라서 VID/PID 로는 역할을 구분할 수 없다 ★ 실제 식별은
#  포트를 열어 첫 텔레메트리 접두어('S,'=A / 'P,'=B)로 한다 — identify_port().
#  ★단, E-STOP 중에는 두 보드가 모두 "STOP" 만 보낸다★ → 그 동안은 역할을 알 수 없다.
#    그 경우를 '보드 없음' 과 구별해 다루는 이유는 DETECT_ESTOP_HOLD_S 주석에 적었다.
ARDUINO_VIDS = {0x2341, 0x1A86, 0x2A03}

#  ★★ GPS·IMU 를 스스로 제외한다 ★★
#    GPS(u-blox)·IMU(CP210x)도 같은 /dev/ttyACM*·/dev/ttyUSB* 대역에 있다. 제외하지 않으면
#      (1) 탐색이 포트당 DETECT_READ_S(5초)씩 느려지고
#      (2) ★배타 open 이 충돌해 GPS/IMU 드라이버가 자기 포트를 못 잡는다★
#         — RTK 가 안 붙는 증상의 유력한 원인이었다. nmea 드라이버는 respawn 으로 계속
#           다시 뜨는데, 그 사이 이 노드가 GPS 포트를 5초씩 물면 서로 밀어낸다.
#    예전에는 g.launch.py 가 GPS/IMU 경로를 확정해 exclude_ports 로 넘겨줬다. 런치가
#    갈라진 뒤로는 그 전달이 끊기므로 ★여기서 직접 VID/PID 를 보고 건너뛴다★.
#    (one_launch.py 가 exclude_ports 로 실제 경로를 넘겨주면 그것도 함께 반영한다 —
#     둘은 배타가 아니라 합집합이다. 같은 VID 장치가 여러 개일 때 런치가 확정한 경로가
#     더 정확하므로 둘 다 받는다.)
NON_ARDUINO_VIDPID = [
    (0x1546, 0x01A9),   # u-blox 9 계열 GPS
    (0x1546, 0x01A8),   # u-blox 8 계열 GPS
    (0x10C4, 0xEA60),   # iAHRS / CP210x IMU
]
#  udev 심볼릭링크(white/ports.py·99-white.rules 와 이름 일치) — 실제 경로로 풀어 제외한다
NON_ARDUINO_SYMLINKS = ('/dev/gps', '/dev/imu')


def _comports():
    if list_ports is None:
        return []
    try:
        return sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        return []


def looks_like_arduino(port):
    """VID(정품/CH340/Arduino LLC) 또는 설명으로 아두이노 계열 여부 판정."""
    if port.vid in ARDUINO_VIDS:
        return True
    desc = (port.description or '').lower()
    return ('arduino' in desc) or ('ch340' in desc)


def is_known_non_arduino(port):
    """GPS/IMU 로 알려진 VID/PID 인지. True 면 열어보지 않는다."""
    return any(port.vid == vid and port.pid == pid for vid, pid in NON_ARDUINO_VIDPID)


def candidate_ports(exclude=None):
    """아두이노 A/B 탐색 대상 포트 목록 (/dev/ttyACM* · /dev/ttyUSB*).

    아두이노로 추정되는 포트(VID/설명 일치)를 앞에, 그 외 USB-serial 포트를 뒤에 둔다.
    GPS/IMU 로 알려진 VID/PID 와 udev 링크(/dev/gps, /dev/imu)는 항상 제외한다.

    exclude : 추가로 제외할 경로(one_launch.py 가 넘겨주는 확정 경로). 심볼릭링크로
      들어와도 realpath 까지 함께 막는다."""
    resolved_exclude = set()
    for path in list(exclude or ()) + list(NON_ARDUINO_SYMLINKS):
        if not path:
            continue
        resolved_exclude.add(path)
        try:
            resolved_exclude.add(os.path.realpath(path))
        except OSError:
            pass

    likely, others = [], []
    for p in _comports():
        dev = p.device
        if not (('ACM' in dev) or ('USB' in dev)):
            continue
        if dev in resolved_exclude:
            continue
        try:
            if os.path.realpath(dev) in resolved_exclude:
                continue
        except OSError:
            pass
        if is_known_non_arduino(p):
            continue          # GPS/IMU — 열면 그쪽 드라이버가 포트를 못 잡는다
        (likely if looks_like_arduino(p) else others).append(dev)
    return likely + others


def port_holders(port):
    """이 포트를 지금 열고 있는 프로세스들 — [(pid, comm, cmdline), ...]

    ★왜 필요한가★ 배타 open(SERIAL_EXCLUSIVE)이 막히면 예외 문구는
    "[Errno 11] Resource temporarily unavailable" 뿐이다. ★누가 잡고 있는지가
    빠져 있고, 그게 유일하게 알아야 하는 정보다★ — 이전 런치의 잔재인지,
    Arduino IDE(arduino-cli daemon 이 보드 탐색으로 포트를 훑는다)인지, 시리얼
    모니터인지에 따라 할 일이 완전히 다르다. /proc 를 훑어 이름을 붙여 준다.

    ※ 같은 사용자 소유 프로세스만 보인다(다른 사용자의 fd 는 커널이 감춘다).
      우리가 신경 쓰는 잔재·IDE 는 모두 같은 사용자이므로 실용상 충분하다.
    """
    try:
        target = os.path.realpath(port)
    except OSError:
        return []
    me = os.getpid()
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        fd_dir = f'/proc/{entry}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue          # 권한 없음 / 그 사이에 죽음
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
            except OSError:
                continue
            try:
                with open(f'/proc/{entry}/comm', encoding='utf-8') as f:
                    comm = f.read().strip()
            except OSError:
                comm = '?'
            try:
                with open(f'/proc/{entry}/cmdline', encoding='utf-8') as f:
                    cmdline = f.read().replace('\0', ' ').strip()
            except OSError:
                cmdline = ''
            found.append((pid, comm, cmdline))
            break             # 한 프로세스는 한 번만 센다
    return found


def is_own_stale(cmdline):
    """이 cmdline 이 ★우리 자신의 잔재★(nxde arduino 노드)인가.

    ★여기를 좁게 유지하는 것이 안전의 핵심이다★ 이 판정이 True 면 아래
    reclaim_port() 가 그 프로세스를 죽인다. Arduino IDE·시리얼 모니터·GPS 드라이버는
    ★절대 여기에 걸리지 않아야 한다★ — 남의 프로세스를 죽이는 것은 이 노드의 일이
    아니고, 그건 사람이 판단할 문제다. 그래서 'nxde' 와 'arduino' 가 함께 있는
    경우만 인정한다 (`ros2 run nxde arduino` / 런치가 띄운 nxde/arduino 실행파일).
    """
    low = cmdline.lower()
    if 'nxde' not in low:
        return False
    return ('arduino' in low) and ('arduino-ide' not in low) and ('arduino-cli' not in low)


def reclaim_port(port, logger):
    """포트를 물고 있는 ★우리 자신의 잔재★ 를 정리한다. 정리했으면 True.

    ★왜 자동으로 죽이는가★ 잔재가 배타 open 을 물고 있으면 재실행은 ★영원히★
    보드를 못 잡는다(재시도로 풀리지 않는다). 그때 사람이 해야 하는 일은 늘 같다:
    `pkill -f nxde.arduino`. 그 한 가지를 노드가 대신한다 — 대상이 '방금 그 포트를
    물고 있는, 우리와 같은 실행파일' 로 특정되므로 판단의 여지가 없다.
    ★남의 프로세스는 손대지 않는다★ (is_own_stale 참고) — 이름만 알려주고 끝낸다.
    """
    holders = port_holders(port)
    if not holders:
        return False

    mine = [h for h in holders if is_own_stale(h[2])]
    others = [h for h in holders if not is_own_stale(h[2])]

    for pid, comm, _ in others:
        logger.error(
            f"{port} 를 다른 프로그램이 잡고 있습니다 — ★{comm} (pid {pid})★. "
            f"이 노드는 남의 프로세스를 죽이지 않습니다. "
            f"Arduino IDE 라면 시리얼 모니터를 닫으세요 "
            f"(IDE 의 보드 탐색이 포트를 주기적으로 훑습니다)")

    if not mine:
        return False

    for pid, comm, _ in mine:
        logger.warn(f"{port} 를 이전 실행의 잔재가 물고 있습니다 — "
                    f"{comm} (pid {pid}) 를 정리합니다")
        for sig, label in ((signal.SIGTERM, 'SIGTERM'), (signal.SIGKILL, 'SIGKILL')):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                break          # 이미 죽었다
            except PermissionError:
                logger.error(f"pid {pid} 를 정리할 권한이 없습니다 (같은 사용자인지 확인)")
                break
            # 죽을 시간을 준다. SIGTERM 으로 안 죽으면 다음 바퀴에서 SIGKILL.
            for _ in range(20):
                time.sleep(0.05)
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                except PermissionError:
                    break
            else:
                if sig is signal.SIGTERM:
                    logger.warn(f"pid {pid} 가 {label} 로 안 죽습니다 — SIGKILL 합니다")
                continue
            break
    # 커널이 fd 를 회수하고 배타 잠금이 풀릴 시간
    time.sleep(PORT_RECLAIM_SETTLE_S)
    return True


def _keep_dtr_on_close(ser):
    """close() 가 DTR 을 떨어뜨려 보드를 리셋하지 않게 HUPCL 을 끈다.

    USB-ACM 은 기본적으로 HUPCL 이 켜져 있어, 식별 실패 후 ser.close() 가
    DTR 하강 → Mega 리셋 → 부트로더 침묵을 만든다. 그 다음 스캔의 open 이
    또 리셋이라, A보드가 영원히 S, 줄을 못 내고 ★스로틀 raw 가 0 에 언다★.
    식별에 성공한 포트에도 걸어 둔다 — 종료 시 리셋이 다음 런치의 '가끔 실패'를
    만든다."""
    if termios is None or ser is None:
        return
    try:
        fd = ser.fd
        attrs = termios.tcgetattr(fd)
        if attrs[2] & termios.HUPCL:
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except (AttributeError, OSError, ValueError):
        pass


def open_serial(port, baud, logger, reclaim=True):
    """포트를 재시도하며 연다. 끝내 실패하면 None (예외를 밖으로 던지지 않는다 —
    한 포트의 open 실패가 노드 전체를 죽이지 않도록).

    ★[2026-08-25] 점유 오류는 재시도만으로 풀리지 않는다★ 배타 open 이 막힌 것은
    '아직 준비가 안 됐다' 가 아니라 '누가 갖고 있다' 는 뜻이므로, 같은 동작을 5번
    반복해도 결과가 같다. 그래서 그 경우에는
      ① 누가 잡고 있는지 이름을 찍고 (port_holders)
      ② 그게 우리 자신의 잔재면 정리한 뒤 다시 시도한다 (reclaim_port)
    reclaim=False 면 ②를 하지 않는다(진단용)."""
    reclaimed = False
    for attempt in range(1, DETECT_OPEN_RETRY + 1):
        try:
            ser = serial.Serial(port, baud, timeout=0.2, exclusive=SERIAL_EXCLUSIVE)
            _keep_dtr_on_close(ser)
            return ser
        except (serial.SerialException, OSError) as e:
            busy = is_busy_error(e)
            # ★점유 오류는 한 번만 회수 시도한다★ 무한 kill 루프가 되지 않게.
            if busy and reclaim and not reclaimed:
                reclaimed = True
                if reclaim_port(port, logger):
                    logger.info(f"{port} 잔재를 정리했습니다 — 다시 엽니다")
                    continue                     # 재시도 횟수를 쓰지 않고 곧장
            if attempt < DETECT_OPEN_RETRY:
                logger.warn(f"{port} 열기 실패({attempt}/{DETECT_OPEN_RETRY}), "
                            f"{DETECT_OPEN_DELAY_S}s 후 재시도: {e}")
                time.sleep(DETECT_OPEN_DELAY_S)
            else:
                logger.warn(f"{port} 열기 최종 실패({DETECT_OPEN_RETRY}회 시도): {e}")
                if busy:
                    held = port_holders(port)
                    if held:
                        who = ', '.join(f"{c}(pid {i})" for i, c, _ in held)
                        logger.error(f"{port} 를 ★{who}★ 가 잡고 있습니다. {BUSY_HINT}")
                    else:
                        logger.error(f"{port}를 열 수 없습니다. {BUSY_HINT}")
    return None


def identify_port(port, baud, logger, reclaim=True):
    """포트를 열어 DETECT_READ_S 동안 읽으며 첫 'S,'/'P,' 줄로 보드를 식별.
       반환: ('A'|'B'|None, serial.Serial 또는 None(실패 시))"""
    ser = open_serial(port, baud, logger, reclaim=reclaim)
    if ser is None:
        return None, None

    buf = b''
    deadline = time.monotonic() + DETECT_READ_S
    saw_stop = False
    while time.monotonic() < deadline:
        try:
            data = ser.read(256)
        except (serial.SerialException, OSError):
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
            return None, None
        if data:
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore').strip()
                if text.startswith('S,'):
                    return 'A', ser
                if text.startswith('P,'):
                    return 'B', ser
                # ★E-STOP 중 — 보드는 있는데 역할을 말해주지 않는다★ (상수 주석 참고)
                #   포트를 닫지 않고 해제를 기다린다. 닫으면 보드가 리셋된다.
                if text == 'STOP' and not saw_stop:
                    saw_stop = True
                    deadline = time.monotonic() + DETECT_ESTOP_HOLD_S
                    logger.warn(
                        f"⛔ {port} : ★E-STOP 이 걸려 있습니다★ 보드는 붙어 있는데 "
                        f"펌웨어가 \"STOP\" 만 보내 A/B 를 구별할 수 없습니다 "
                        f"(케이블 문제가 아닙니다). 스위치를 해제하면 즉시 붙습니다 "
                        f"— {DETECT_ESTOP_HOLD_S:.0f}초까지 포트를 잡고 기다립니다")

    ser.close()
    # ★'보드가 없다' 와 'E-STOP 때문에 역할을 모른다' 를 구별해서 돌려준다★
    #   호출측(_link_loop)의 경고 문구가 갈라진다 — 사람이 볼 곳이 달라지기 때문이다.
    return ('ESTOP' if saw_stop else None), None


class Arduino(Node):
    def __init__(self):
        super().__init__('arduino')

        # ── 파라미터 ──
        self.baud = int(self.declare_parameter('baud', BAUD_RATE).value)
        # ★ 조향 부호 반전 — 기본 False(반전 없음) ★ ROS 토픽과 B보드가 같은 규약
        #   (− 좌 / + 우)을 쓴다. 배선이나 펌웨어를 뒤집었을 때만 True 로 둔다.
        #   자세한 경위는 파일 헤더 규약 2) 참고.
        self.steer_invert = bool(self.declare_parameter('steer_invert', False).value)
        # /control_state=False 일 때 걸 브레이크 단계.
        #   0 = 코스트(white/motor.py 의 'S,0' 과 같은 동작, 기본)
        #   1 = 약한 브레이킹으로 더 빨리 세우고 싶을 때
        self.stop_brake_level = int(
            self.declare_parameter('stop_brake_level', 0).value)
        # ★[2026-08-04] manual_brake_level / manual_release_raw 를 삭제했다★
        #   '수동조종 진입 시 브레이크 2단 체결 → 페달을 밟으면 해제' 로직 자체를 없앴다.
        #   모드 전환은 제동 지시가 아니고, 실차에서 스위치를 수동으로 내리는 순간 리니어가
        #   브레이크 페달을 밟고 튀어나왔다(E-STOP 도 아닌데). 수동에서는 브레이크를 항상
        #   0 으로 보낸다 — 제동은 사람 발이 한다. compose() (2) 분기 참고.
        # 수동조종에서 페달 최대치가 대응할 펄스.
        #   ★[2026-08-25] 이 값은 더 이상 차를 굴리지 않는다★ 수동조종의 실제 구동은
        #   manual_pwm_min/max 가 만드는 직접 PWM 이고, 이 값은 /drive_pulse_cmd 라벨
        #   (mapping 수집 라벨 ①)을 종전과 같은 0~15 스케일로 유지하기 위해서만 쓴다.
        #   ⚠️ 그러므로 ★속도를 낮추려고 이 값을 건드리지 말 것★ — 라벨만 줄어들고
        #     차는 그대로 나간다. 속도는 manual_pwm_max 로 조절한다.
        self.manual_pulse_max = int(
            self.declare_parameter('manual_pulse_max', PULSE_MAX).value)
        # ★수동조종 페달 → 직접 PWM 환산 구간★ (상수 주석에 근거를 적었다)
        #   페달 개도 0%→manual_pwm_min, 100%→manual_pwm_max 로 선형 대응한다.
        #   실차 튜닝은 여기 두 개로 한다 : 초반 유격은 min, 최고속은 max.
        self.manual_pwm_min = int(
            self.declare_parameter('manual_pwm_min', MANUAL_PWM_MIN).value)
        self.manual_pwm_max = int(
            self.declare_parameter('manual_pwm_max', MANUAL_PWM_MAX).value)
        # 프로토콜 밖의 값은 여기서 못 나가게 잘라 둔다 — 16 미만은 펌웨어가 '펄스'로
        # 읽어버리고(= 살짝 밟았는데 펄스 15 목표가 걸리는 사고), 255 초과는 '정지'다.
        self.manual_pwm_min = max(PWM_DIRECT_MIN, min(PWM_DIRECT_MAX, self.manual_pwm_min))
        self.manual_pwm_max = max(self.manual_pwm_min,
                                  min(PWM_DIRECT_MAX, self.manual_pwm_max))
        # ★페달 → A보드 경로★ True=직접 PWM / False=목표펄스(PID, 떼면 코스트).
        #   런치가 문자열 'false' 를 넘기므로 bool() 로 받지 않는다(_as_bool).
        self.manual_use_pwm = _as_bool(
            self.declare_parameter('manual_use_pwm', True).value, True)
        # ══════════════════════════════════════════════════════════════
        #  ★[2026-08-25] AEB 비상정지 (/aeb_stop)★ 헤더 (1-1) 분기가 쓴다
        # ══════════════════════════════════════════════════════════════
        #  aeb_brake_level = 0 이면 ★기능 자체가 꺼진다★ (구독만 하고 무시).
        #  기본을 0 으로 둔 것은 이 파일이 white1 자율주행 스택과 공용이기
        #  때문이다 — 그쪽 런치는 이 파라미터를 주지 않으므로 거동이 종전과
        #  완전히 같다. 켜는 곳은 lidar/launch/one_launch.py 하나다.
        #    2 = 풀브레이킹 (권장) / 1 = 약한 브레이킹 / 0 = 기능 꺼짐
        self.aeb_brake_level = int(
            self.declare_parameter('aeb_brake_level', 0).value)
        self.aeb_brake_level = max(0, min(BRAKE_LEVEL_MAX, self.aeb_brake_level))
        # 판단 노드와 ★같은 이름★ 이어야 한다 (lidar manual_aeb_node 의 aeb_stop_topic).
        self.aeb_topic = str(
            self.declare_parameter('aeb_topic', '/aeb_stop').value) or '/aeb_stop'
        # ★신선도 [s]★ 이 시간 넘게 안 오면 해제한다(fail-open — 헤더 (1-1) 참고).
        #   판단 노드가 20Hz 로 내므로 1.0 은 20틱이 빠진 것이다. 0 이하로 주면
        #   신선도를 보지 않는다(= 마지막 값을 영구히 물고 있는다) — 권하지 않는다.
        self.aeb_stale_s = float(
            self.declare_parameter('aeb_stale_s', 1.0).value)
        self.throttle_raw_min = int(
            self.declare_parameter('throttle_raw_min', THROTTLE_RAW_MIN).value)
        self.throttle_raw_max = int(
            self.declare_parameter('throttle_raw_max', THROTTLE_RAW_MAX).value)
        self.throttle_raw_min = max(0, min(ADC_MAX, self.throttle_raw_min))
        self.throttle_raw_max = max(self.throttle_raw_min + 1,
                                    min(ADC_MAX, self.throttle_raw_max))
        self.throttle_gamma = float(
            self.declare_parameter('throttle_gamma', THROTTLE_GAMMA).value)
        if self.throttle_gamma < 0.5:
            self.throttle_gamma = 0.5
        elif self.throttle_gamma > 3.0:
            self.throttle_gamma = 3.0
        median_n = int(
            self.declare_parameter('throttle_median_n', THROTTLE_MEDIAN_N).value)
        self._thr_buf = deque(maxlen=max(1, min(15, median_n)))
        # ★ 아두이노가 아닌 장치 경로 — 탐색에서 추가로 제외한다 ★
        #   ※ 안 넘겨도 된다 — candidate_ports() 가 GPS/IMU VID/PID(NON_ARDUINO_VIDPID)와
        #     udev 링크(/dev/gps · /dev/imu)를 이미 스스로 걸러낸다. 이 파라미터는 같은
        #     VID 장치가 여러 개 꽂혀 있어 런치가 확정한 경로가 더 정확할 때 쓴다
        #     (one_launch.py 가 GPS/IMU 실경로를 넘겨준다).
        self.exclude_ports = [
            str(p) for p in self.declare_parameter('exclude_ports', ['']).value if str(p)]

        if self.manual_use_pwm:
            self.get_logger().info(
                f"수동조종 구동 = 직접 PWM {self.manual_pwm_min}~{self.manual_pwm_max} "
                f"(개루프 — 밟은 듀티가 유지됩니다, "
                f"페달 raw {self.throttle_raw_min}~{self.throttle_raw_max}, "
                f"gamma={self.throttle_gamma:.2f})")
        else:
            self.get_logger().info(
                f"수동조종 구동 = 목표펄스 0~{self.manual_pulse_max} "
                f"(보드 PID · 페달을 떼면 코스트, "
                f"페달 raw {self.throttle_raw_min}~{self.throttle_raw_max}, "
                f"gamma={self.throttle_gamma:.2f})")

        # ── 시리얼 ──
        # ★ _running / _link_thread 를 여기서 먼저 만든다 ★ watch_parent 가 등록하는
        #   stop_and_close 는 언제든 불릴 수 있는데, 그 안에서 이 속성들을 읽는다.
        self._running = False
        self._link_thread = None
        self.ser_a = None
        self.ser_b = None
        self.rx_buf_a = b''
        self.rx_buf_b = b''
        self.last_line_a = None
        self.last_line_b = None
        self._last_s_t = 0.0           # 마지막 유효 S, 수신 (monotonic). 0 = 아직 없음
        self._skip_junk_n = 0          # 에코/잡음 줄을 건너뛴 횟수 (경고 throttle 용)
        # ★E-STOP 때문에 역할을 못 읽은 포트★ (_scan_once 가 매 바퀴 다시 채운다)
        self._estop_ports = []

        # ── 보드 → ROS 최신 상태 (STOP·형식오류 시 마지막 값 유지) ──
        self.pulse_l = 0
        self.pulse_r = 0
        self.angle_board = 0       # B보드 실측 조향각 (− 좌 / + 우 = ROS 규약과 동일)
        self.throttle_raw = 0
        # ★[2026-08-21] B보드 A5 리니어 가변저항 raw★ 브레이크 페달의 실제 위치.
        self.brake_pot = 0
        # B보드 D5 주행모드. ★페일세이프로 수동(False)에서 시작한다★ — 첫 텔레메트리를
        # 받기 전에 '자율'로 오인해 자동 명령이 나가는 것보다 수동으로 보는 편이 안전하다.
        self.switch_mode = False   # ← B보드 D5 원값 (물리 스위치가 말하는 것)
        self.estop_active = False

        # [2026-08-07] 소프트웨어 모드 오버라이드(mode_override / /vehicle_mode_cmd)를
        #   삭제했다. 주행모드의 소유자는 물리 스위치 하나다 — auto_mode 주석 참고.

        # 수동조종에서 '지금 페달을 밟고 있다'고 볼 최소 펄스. 로그용 상태이기도 하다.
        self._manual_src = None    # 'pedal' — 바뀔 때만 로그를 남긴다

        # ── ROS → 보드 명령 캐시 ──
        self.cmd_pulse = 0         # /cmd_vel_raw linear.x (펄스, 0~15)
        self.cmd_angle = 0         # /cmd_vel_raw angular.z (− 좌 / + 우, -40~40)
        self.control_enabled = False   # /control_state. 시작은 False(정지)
        self.cmd_brake = 0         # /brake_level (0/1/2). 안 오면 0(놓음)
        # ── ★[2026-08-14] 브레이크 해제 유예 — 리니어 왕복을 여기서 못 박는다★ ──
        #   /brake_level 은 ★마지막 발행자가 이기는★ 명령 토픽이고 발행자가 여럿이다
        #   (driving · traffic_light · master · GUI). 그중 하나라도 0 을 한 번 흘리면
        #   그 순간 리니어가 빠졌다가 다음 요청에 다시 나온다 — 실차에서 관측된
        #   '나왔다 들어갔다' 가 그것이다. 상류(각 발행자)에도 유예를 넣었지만,
        #   ★여기가 모든 요청이 합쳐지는 마지막 지점★ 이므로 여기서 한 번 더 막는다:
        #       0 이 아닌 요청을 받은 뒤 BRAKE_RELEASE_HOLD_S 동안은 그보다 낮은 값으로
        #       내려가지 않는다(같거나 더 센 값은 즉시 반영).
        #   ★자율주행 분기 (4) 에만 적용한다★ 수동조종·E-stop 분기는 종전 그대로
        #   브레이크 0 을 보낸다 — '모드 전환은 절대로 리니어를 체결하지 않는다'는
        #   불변식을 이 유예가 건드리면 안 되기 때문이다. 모드 전환 시에는
        #   _disarm_brakes_on_mode_edge() 가 유예까지 함께 지운다.
        # ── ★[2026-08-25] AEB 비상정지 상태★ 헤더 (1-1) ──
        #   ★모드 전환 엣지에서 지우지 않는다★ 이것은 '남아 있던 캐시 요청'이 아니고
        #   살아 있는 센서가 20Hz 로 계속 말하고 있는 사실이다. 그 성질을 보장하는
        #   것이 아래 신선도(aeb_stale_s)다 — 끊기면 스스로 풀린다.
        self.aeb_stop = False
        self.aeb_stop_t = 0.0         # 마지막 수신 시각 (monotonic)
        self._aeb_engaged = False     # 로그 엣지용
        self._brake_rx_t = 0.0        # 마지막 /brake_level 수신 시각(진단 로그용)
        self._brake_hold_level = 0    # 마지막으로 받은 '0 아닌' 단계
        self._brake_hold_t = 0.0

        # ★[2026-08-04] 수동조종 브레이크 래치 상태(_manual_brake / _manual_released)를
        #   삭제했다★ 수동에서는 브레이크가 항상 0 이다.
        #
        # ★★ [2026-08-05] _prev_auto_mode 는 되살렸다 — 단, 용도가 정반대다 ★★
        #   ⚠️ 이것은 삭제된 래치가 아니다. 이 엣지 감지는 브레이크를 ★거는★ 데 쓰지
        #      않고 ★지우는★ 데만 쓴다(_disarm_brakes_on_mode_edge 참고).
        #      브레이크를 거는 경로는 여전히 /brake_level·stop_brake_level·E-stop 뿐이다.
        self._prev_auto_mode = None

        # ★ stop_brake_level 무장 플래그 ★ 자율 정지 브레이크는 '자율주행이 한 번이라도
        #   구동 허가(/control_state=True)를 받은 뒤'에만 걸린다.
        #     False = 미무장 → 우선순위 (3) 에서 stop_brake_level 을 쓰지 않는다(0)
        #     True  = 무장   → 종전대로 max(stop_brake_level, /brake_level)
        #   모드 전환 엣지에서 다시 False 로 내려간다(_disarm_brakes_on_mode_edge).
        #   왜 : stop_brake_level 을 1 이상으로 두면, 수동에서 자율로 스위치를 올리는
        #   순간 (3) 분기가 그 값을 집어 ★사람이 차를 넘겨주는 그 순간 리니어가 체결★
        #   된다(control_state 는 아직 False 다). '자율 정지 시 제동'이라는 이 값의 뜻은
        #   자율주행이 실제로 몰다가 세울 때를 가리키지, 아직 한 번도 몰지 않은 상태를
        #   가리키지 않는다. 기본값 0 에서는 어느 쪽이든 체결이 없다.
        self._stop_brake_armed = False

        # ── 전송 변경 감지 ──
        self._last_a = None
        self._last_b = None
        self._last_a_t = 0.0
        self._last_b_t = 0.0

        # ── 퍼블리셔 ──
        self.pub_encoder = self.create_publisher(Int32, '/encoder', 10)
        self.pub_steer_angle = self.create_publisher(Int32, '/steer_angle_measured', 10)
        self.pub_mode = self.create_publisher(Bool, '/vehicle_mode', 10)
        self.pub_throttle = self.create_publisher(Int32, '/throttle_pedal', 10)
        self.pub_brake_pot = self.create_publisher(Int32, '/brake_pot', 10)
        # ★ 주행 목표펄스 (0~15 스케일) ★ 자율=계획값(A보드로 실제 나간 값)
        #   수동조종=페달 환산값(★[2026-08-25] 부터 라벨 전용★ — 실제 구동은 아래
        #   /drive_pwm_cmd 다).
        #   수동조종 수집(mapping)의 라벨 ①이 이 값이다 — 환산 규칙(throttle_raw_min/max,
        #   manual_pulse_max)이 이 노드에만 있으므로, 여기서 발행해야 소비측이 규칙을
        #   복제하지 않는다(복제하면 파라미터를 바꿀 때 조용히 어긋난다).
        self.pub_drive_pulse = self.create_publisher(Int32, '/drive_pulse_cmd', 10)
        # ★[2026-08-25] A보드로 실제 나간 직접 PWM★ 수동조종에서 페달을 밟는 동안만
        #   16~255, 그 외에는 0(직접 PWM 경로를 쓰지 않는 상태)이다.
        #   /drive_pulse_cmd 의 스케일(0~15)을 건드리지 않으려고 별 토픽으로 뺐다 —
        #   수집·로스백이 전부 그 스케일로 기록되어 있어서, 거기에 PWM 을 흘리면
        #   라벨이 조용히 10배로 어긋난다. 이쪽은 진단·로스백 전용이다.
        self.pub_drive_pwm = self.create_publisher(Int32, '/drive_pwm_cmd', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        self.pub_status = self.create_publisher(String, '/board_status', 10)

        # ── 서브스크라이버 ──
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd_vel, 10)
        self.create_subscription(Bool, '/control_state', self.cb_control_state, 10)
        # [2026-08-07] /vehicle_mode_cmd 구독을 삭제했다 — 주행모드는 물리 스위치만
        #   바꿀 수 있다. 수동조종에서 ROS 펄스가 필요한 경우는 compose() (2) 가
        #   직접 처리하므로 모드를 속일 이유가 없어졌다.
        # 브레이크 단계(0/1/2). Twist 에 필드가 없어 별 토픽으로 받는다. 선택 입력 —
        # 아무도 발행하지 않으면 0(놓음)으로 유지된다(white 의 driving 은 발행하지 않는다).
        self.create_subscription(Int32, '/brake_level', self.cb_brake_level, 10)
        # ★[2026-08-25] AEB 비상정지★ aeb_brake_level=0(기본)이면 받아도 무시한다.
        #   구독 자체는 항상 걸어 둔다 — 파라미터로 켠 순간부터 바로 듣게.
        self.create_subscription(Bool, self.aeb_topic, self.cb_aeb_stop, 10)
        if self.aeb_brake_level > 0:
            self.get_logger().warn(
                f"🛑 AEB 비상정지 켜짐 — {self.aeb_topic} 가 True 면 구동을 끊고 "
                f"리니어를 {self.aeb_brake_level}단으로 물립니다 "
                f"(★수동조종 중에도 물립니다★ / 신선도 {self.aeb_stale_s:.1f}s)")

        # ★ 종료 신호를 직접 받는다 ★ (포트를 열기 전에 걸어야 탐색 구간도 커버된다)
        self._install_exit_handlers()

        # ★ 포트를 열기 전에 부모 감시를 걸어둔다 ★
        #   탐색 스레드가 포트를 여는 도중에 런치가 내려가면 이 프로세스가 고아로 남아
        #   포트를 물고 있게 된다. 감시를 탐색보다 먼저 걸어야 그 구간까지 커버된다.
        watch_parent(cleanup=self.stop_and_close)

        # ★ 생성자는 블로킹하지 않는다 ★ 보드가 안 꽂혀 있어도 노드는 즉시 뜬다.
        #   탐색과 재연결을 같은 데몬 스레드가 담당한다(_link_loop).
        self._running = True
        self._link_thread = threading.Thread(
            target=self._link_loop, name='board_link', daemon=True)
        self._link_thread.start()

        self.create_timer(SERIAL_POLL_S, self.on_rx_timer)
        self.create_timer(TX_PERIOD_S, self.on_tx_timer)

        self.get_logger().info(
            "아두이노 브리지 시작 — A/B 보드 탐색은 백그라운드에서 진행합니다 "
            "(연결 전에도 노드는 정상 동작하며, 꽂는 순간 자동으로 붙습니다)")

    # ═══════════════════════════════════════════════════════════════
    #  보드 링크 관리 : 최초 탐색 + 도중 단절 재연결 (전용 스레드)
    # ═══════════════════════════════════════════════════════════════
    def _link_loop(self):
        """A/B 보드가 둘 다 붙어 있을 때까지(그리고 빠질 때마다 다시) 스캔한다.

        ★ 이 루프가 최초 연결과 재연결을 겸한다 ★ read/write 오류로 _drop_board 가
        포트를 None 으로 떨어뜨리면 다음 사이클이 그것을 보고 다시 찾는다. 한쪽만
        빠지면 나머지 한쪽은 계속 정상 동작한다(그쪽 포트는 건드리지 않는다).

        rclpy 콜백이 아니라 전용 스레드이므로 여기서 time.sleep / 블로킹 read 를 해도
        제어 타이머(on_tx_timer/on_rx_timer)를 막지 않는다."""
        first_round = True
        while self._running and rclpy.ok():
            if self.ser_a is not None and self.ser_b is not None:
                time.sleep(DETECT_RETRY_S)
                first_round = False
                continue

            missing = [n for n, s in (('A', self.ser_a), ('B', self.ser_b)) if s is None]
            if first_round:
                self.get_logger().info(f"{'/'.join(missing)}보드 탐색 시작...")
            self.publish_status()

            self._scan_once()

            if self.ser_a is None or self.ser_b is None:
                missing = [n for n, s in (('A', self.ser_a), ('B', self.ser_b)) if s is None]
                if self._estop_ports:
                    # ★원인이 확실할 때는 케이블을 의심하게 만들지 않는다★
                    self.get_logger().warn(
                        f"{'/'.join(missing)}보드가 아직 안 붙었습니다 — "
                        f"★E-STOP 이 걸려 있습니다★ ({', '.join(self._estop_ports)}). "
                        f"스위치를 해제하면 {DETECT_RETRY_S}s 안에 붙습니다 "
                        f"(케이블·전원은 정상입니다 — 보드가 응답하고 있습니다)",
                        throttle_duration_sec=10.0)
                else:
                    self.get_logger().warn(
                        f"{'/'.join(missing)}보드 미발견, {DETECT_RETRY_S}s 후 재스캔 "
                        f"(연결·전원·USB 케이블 확인)", throttle_duration_sec=15.0)
                self.publish_status()
                time.sleep(DETECT_RETRY_S)
            elif not first_round:
                self.get_logger().info("A/B 보드 모두 연결됨")
            first_round = False

    def _scan_once(self):
        """후보 포트를 한 바퀴 돌며 아직 못 찾은 보드를 채운다."""
        owned = {s.port for s in (self.ser_a, self.ser_b) if s is not None}
        exclude = list(self.exclude_ports) + list(owned)
        # ★이번 바퀴에서 'E-STOP 때문에 역할을 못 읽은' 포트★ (매 바퀴 새로 판단한다 —
        #   해제되면 다음 바퀴에서 정상 식별되어야 하므로 남겨두면 안 된다)
        self._estop_ports = []

        for port in candidate_ports(exclude=exclude):
            if not (self._running and rclpy.ok()):
                return
            if self.ser_a is not None and self.ser_b is not None:
                return

            # 이미 한 보드를 연 상태면 USB 컨트롤러가 안정되도록 잠깐 쉰다
            if self.ser_a is not None or self.ser_b is not None:
                time.sleep(PORT_SETTLE_S)

            try:
                role, ser = identify_port(port, self.baud, self.get_logger())
            except Exception as e:   # 한 포트의 예기치 못한 오류가 스레드를 죽이지 않도록
                self.get_logger().warn(f"{port} 감지 중 오류(건너뜀): {e}")
                continue

            if role == 'ESTOP':
                # 보드는 있다 — 역할만 모른다. identify_port 가 이미 경고했다.
                self._estop_ports.append(port)
                continue

            if role == 'A' and self.ser_a is None:
                ser.timeout = 0            # 이후 폴링은 논블로킹
                self.rx_buf_a = b''        # 재연결이면 이전 버퍼 잔재를 버린다
                self._last_a = None        # 변경감지 캐시 초기화 → 다음 TX 에서 즉시 재전송
                self._last_s_t = time.monotonic()  # 부팅 직후 S, 유예
                self.ser_a = ser
                self.get_logger().info(f"[A보드 연결] {port} (인휠 PID + 주행펄스)")
                self.publish_status()
            elif role == 'B' and self.ser_b is None:
                ser.timeout = 0
                self.rx_buf_b = b''
                self._last_b = None
                self.ser_b = ser
                self.get_logger().info(f"[B보드 연결] {port} (조향 + 제동 + 모드)")
                self.publish_status()
            elif ser is not None:
                ser.close()

    def _drop_board(self, which, reason):
        """단절된 보드의 포트를 닫고 None 으로 떨어뜨린다 → _link_loop 가 다시 찾는다.

        ★ 텔레메트리 캐시(펄스/각도/모드)는 지우지 않는다 ★ 마지막 정상값을 유지하는
        것이 STOP·형식오류 처리와 같은 태도이고, 재연결 직후 값이 0 으로 튀는 것보다 낫다.
        단, 최신 줄(last_line_*)은 지운다 — 'STOP' 이 남아 있으면 보드가 빠진 뒤에도
        e-stop 이 걸린 것으로 오판한다."""
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return
        self.get_logger().error(
            f"[{which.upper()}보드 단절] {getattr(ser, 'port', '?')}: {reason} → 재연결 시도")
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass
        if which == 'a':
            self.ser_a = None
            self.rx_buf_a = b''
            self.last_line_a = None
            self._last_s_t = 0.0
            self._thr_buf.clear()
        else:
            self.ser_b = None
            self.rx_buf_b = b''
            self.last_line_b = None
        self.publish_status()

    # ═══════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ═══════════════════════════════════════════════════════════════
    def cb_cmd_vel(self, msg: Twist):
        """/cmd_vel_raw — linear.x = 주행 목표펄스(0~15), angular.z = 조향각(− 좌 / + 우).

        ★ linear.x 는 m/s 가 아니다 ★ 환산은 white/kasa_units.py 가 발행 전에 끝낸다.
        ★ angular.z 부호는 이미 보드 규약과 같다 ★ 여기서 뒤집지 않는다(헤더 규약 2).
        후진은 없으므로 음수는 0 으로 클램프한다(A보드가 음수를 받지 않는다)."""
        pulse = _round_half_away(float(msg.linear.x))
        self.cmd_pulse = max(PULSE_MIN, min(PULSE_MAX, pulse))

        angle = _round_half_away(float(msg.angular.z))
        self.cmd_angle = max(-STEER_DEG_MAX, min(STEER_DEG_MAX, angle))

    def cb_control_state(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.control_enabled:
            self.get_logger().info(
                f"/control_state → {'구동 허용' if new_state else '정지'}")
        self.control_enabled = new_state
        # 자율주행이 실제로 구동 허가를 받은 순간부터 stop_brake_level 이 유효해진다.
        #   (그 전의 '정지'는 자율 정지가 아니라 그냥 대기 상태다 — 위 플래그 주석 참고)
        if new_state:
            self._stop_brake_armed = True

    def cb_brake_level(self, msg: Int32):
        """/brake_level — 브레이크 ★단계 0/1/2★ (0~255 PWM 이 아니다).

        범위 밖 값은 무시한다. B보드도 범위 밖이면 브레이크 필드만 버리지만, 여기서
        먼저 걸러 '왜 안 걸리는지' 로그로 드러나게 한다."""
        level = int(msg.data)
        if not (0 <= level <= BRAKE_LEVEL_MAX):
            self.get_logger().warn(
                f"/brake_level={level} 은 허용범위(0~{BRAKE_LEVEL_MAX}) 밖 — 무시. "
                f"★0~255 PWM 이 아니라 단계값이다★ (0 놓음 / 1 약 / 2 풀)",
                throttle_duration_sec=5.0)
            return
        if level != self.cmd_brake:
            # ★진단★ 값이 언제 바뀌었는지, 직전 값이 얼마나 유지됐는지 함께 남긴다.
            #   '리니어가 나왔다 들어갔다' 를 추적할 때 필요한 것은 이 시간차다.
            held = time.monotonic() - self._brake_rx_t
            self.get_logger().info(
                f"/brake_level → {level}단  (직전 {self.cmd_brake}단을 {held:.2f}초 유지)")
        self._brake_rx_t = time.monotonic()
        self.cmd_brake = level
        if level > 0:
            # 해제 유예의 기준 — ★마지막으로 '물어라' 를 받은 시각·단계★
            self._brake_hold_level = level
            self._brake_hold_t = self._brake_rx_t

    def cb_aeb_stop(self, msg: Bool):
        """/aeb_stop — 전방 장애물 확정. ★수동조종 중에도 통하는 제동 경로★

        엣지가 아니라 ★상태★ 를 받는다(발행자가 20Hz 로 계속 낸다). 여기서는
        값과 수신 시각만 남기고, 판단은 compose() (1-1) 이 한다 —
        ★aeb_brake_level=0 이면 아래 값은 아무 데도 쓰이지 않는다★."""
        self.aeb_stop = bool(msg.data)
        self.aeb_stop_t = time.monotonic()

    def aeb_engaged(self):
        """지금 AEB 로 차를 세워야 하는가. ★신선도까지 본다★ (헤더 (1-1))

        판단 노드가 죽으면(토픽이 끊기면) False 로 돌아온다 — fail-open 이다.
        그 상태는 'AEB 가 없는 수동조종' = 원래 상태이고, 사람이 예상하지 못한
        정지가 뒤차·경사에서 더 위험하기 때문이다. 대신 조용히 넘기지 않는다."""
        if self.aeb_brake_level <= 0:
            return False                      # 기능 꺼짐 (기본)
        if not self.aeb_stop:
            return False
        if self.aeb_stale_s > 0.0 and \
                (time.monotonic() - self.aeb_stop_t) > self.aeb_stale_s:
            self.get_logger().error(
                f"⚠️ AEB 정지신호({self.aeb_topic})가 {self.aeb_stale_s:.1f}초 넘게 "
                f"끊겼습니다 — ★제동을 풉니다★ (판단 노드가 살아 있는지 확인). "
                f"지금은 AEB 없는 수동조종 상태입니다",
                throttle_duration_sec=2.0)
            return False
        return True

    # ═══════════════════════════════════════════════════════════════
    #  ROS → 보드 : 명령 조립 + 전송
    # ═══════════════════════════════════════════════════════════════
    def throttle_cmd_raw(self):
        """지령에 쓰는 페달 raw. /throttle_pedal 은 원값, 지령은 최근 N샘플 중앙값.

        실측이 한 틱에 204→800 으로 튀면 옛 max=800 매핑에서 개도 1.0 이 되어
        /drive_pulse_cmd 가 15 로 점프했다. 중앙값은 그 스파이크만 걷고, 발을
        천천히 밟는 행정은 그대로 따라간다."""
        buf = self._thr_buf
        if not buf:
            return int(self.throttle_raw)
        s = sorted(buf)
        return int(s[len(s) // 2])

    def throttle_frac(self, raw):
        """쓰로틀 페달 raw(0~1023) → 개도량 0.0~1.0. 수동조종 모드 전용.

        throttle_raw_min~max 를 0~1 에 대응시킨 뒤 throttle_gamma 로 굽힌다.
        ★환산의 원점은 여기 하나다★ 펄스(라벨)와 PWM(실제 구동)이 같은 개도량에서
        갈라져 나가야 로스백에서 둘을 나란히 놓고 볼 수 있다."""
        raw = max(0, min(ADC_MAX, int(raw)))
        lo, hi = self.throttle_raw_min, self.throttle_raw_max
        if hi <= lo or raw <= lo:
            return 0.0
        lin = min(1.0, (raw - lo) / (hi - lo))
        g = self.throttle_gamma
        if g == 1.0:
            return lin
        return lin ** g

    def throttle_to_pulse(self, raw):
        """쓰로틀 페달 raw → 주행펄스 0~manual_pulse_max.

        ★[2026-08-25] 이제 이것은 차를 굴리지 않는다★ /drive_pulse_cmd 라벨 전용이다
        (mapping 수집 라벨 ①의 스케일을 종전 0~15 그대로 유지한다).
        실제 구동은 throttle_to_pwm 이 만든다 — compose() (2) 분기 참고."""
        frac = self.throttle_frac(raw)
        if frac <= 0.0:
            return 0
        return max(0, min(PULSE_MAX,
                          _round_half_away(frac * self.manual_pulse_max)))

    def throttle_to_pwm(self, raw):
        """쓰로틀 페달 raw → A보드 ★직접 PWM★. 수동조종 모드 전용.

        반환값 0 = 페달을 밟지 않음(→ 단일값 "0" 을 보내 펄스 모드로 되돌린다),
        16~255 = 직접 PWM(→ "<pwm>,<pwm>" 콤마 2값으로 보낸다).

        개도량 0~1 을 manual_pwm_min~manual_pwm_max 에 대응시킨다.
        슬루레이트는 넣지 않는다 — 발을 천천히 밟으면 천천히 오른다.
        창 중앙값(throttle_cmd_raw)과 gamma 만 쓴다. 전자는 한 틱 스파이크가
        전개가 되는 것을 막고, 후자는 페달 초반을 저속에 남겨 정밀 제어가 되게 한다."""
        frac = self.throttle_frac(raw)
        if frac <= 0.0:
            return 0
        pwm = _round_half_away(
            self.manual_pwm_min + frac * (self.manual_pwm_max - self.manual_pwm_min))
        return max(PWM_DIRECT_MIN, min(PWM_DIRECT_MAX, pwm))

    def to_board_angle(self, ros_deg):
        """ROS 조향각 → B보드로 보낼 값. ★기본은 그대로 통과다★ (같은 규약 − 좌 / + 우)

        steer_invert=True 일 때만 뒤집는다 — 배선이나 펌웨어를 바꿔 방향이 반대가 된
        경우의 탈출구다. 규약 2 참고."""
        deg = -ros_deg if self.steer_invert else ros_deg
        return max(-STEER_DEG_MAX, min(STEER_DEG_MAX, int(deg)))

    # ═══════════════════════════════════════════════════════════════
    #  주행모드 = ★물리 스위치(B보드 D5) 하나뿐★
    # ═══════════════════════════════════════════════════════════════
    @property
    def auto_mode(self):
        """실효 주행모드. ★물리 스위치가 유일한 소유자다★

        [2026-08-07] 소프트웨어 오버라이드(/vehicle_mode_cmd)를 삭제했다.
          예전에는 GUI 버튼으로 자율/수동을 덮어쓸 수 있었다. 그런데 모드는
          '사람이 핸들과 페달을 잡고 있는가'를 뜻하는 물리적 사실이라, 화면 클릭
          한 번으로 뒤집을 수 있으면 안 된다 — 사람이 운전대를 잡은 채 소프트웨어가
          자율로 바꾸면 조향모터에 힘이 들어간다.

          오버라이드가 필요했던 실제 이유는 '수동조종에서도 ROS 가 지정한 펄스를
          내보내고 싶다'였는데(white806 의 헤딩 초기화), 그건 compose() (2) 분기가
          직접 지원하도록 바꿔서 더 이상 모드를 속일 이유가 없다.
        """
        return self.switch_mode

    def _disarm_brakes_on_mode_edge(self):
        """★★ 불변식 : 모드 전환은 절대로 리니어를 체결하지 않는다 ★★

        자율↔수동 전환 엣지에서 브레이크 관련 상태를 전부 '풀린' 쪽으로 되돌린다:
          ① /brake_level 요청 캐시(self.cmd_brake) → 0
          ② stop_brake_level 무장 해제(self._stop_brake_armed) → False

        ⚠️ 삭제된 '수동 진입 시 2단 체결' 래치가 아니다. 방향이 정반대다 — 이 함수는
           브레이크를 걸지 않고 ★지우기만★ 한다. 걸 수 있는 경로는 여전히 셋뿐이다
           (/brake_level · stop_brake_level · E-stop = B보드 자체 동작).

        ★왜 필요한가 (2026-08-05)★
          수동조종으로 사람이 몰고 있어도 자율 스택은 계속 돌아간다 —
          one_launch.py 로 띄운 camera_judgment 는 D5 를 보지 않으므로, 사람이 운전하는
          동안 빨간불을 확정하면 /brake_level=2 를 그대로 발행한다.
          compose() 의 수동 분기는 브레이크를 항상 0 으로 보내니 그 순간엔 아무 일도
          없지만, 값은 self.cmd_brake 에 남는다. 그래서 사람이 D5 를 자율로 되돌리는
          순간 (3)/(4) 분기가 그 남은 2 를 집어 ★리니어가 튀어나온다★ —
          2026-08-04 에 제거한 증상과 겉모습이 똑같다(그때는 원인이 이 파일의 래치였다).
          → 모드가 바뀌는 순간 요청을 0 으로 지운다. 지금 정말로 필요한 제동이라면
            발행자가 다시 보낸다.

          ②도 같은 이유다. stop_brake_level 을 1 이상으로 두면 (3) 분기가 전환 직후
          그 값을 건다 — 그때 /control_state 는 아직 False 이므로(자율주행을 아직
          시작하지 않았다) ★사람이 차를 넘겨주는 바로 그 순간 리니어가 밟힌다★.
          그래서 전환 시 무장을 풀고, 자율주행이 실제로 구동 허가를 받은 뒤
          (cb_control_state 의 True) 다시 무장한다. 기본값 0 에서는 어차피 차이가 없다.

        ★감수하는 것★ camera_judgment 는 값이 '변할 때만' 발행한다. 그래서 자율로
          되돌린 뒤에도 같은 빨간불이 계속 확정 상태면 2 를 다시 보내지 않아, 그 구간에서
          리니어가 빠진 채로 남는다. 그래도 안전한 쪽이다 —
            · 그 상태의 속도명령은 이미 0 이다(게이트가 TL_STOP 으로 덮는다)
            · stop_brake_level 기본값도 0(코스트)이라 자율 정지의 기본 동작과 같다
            · 신호가 바뀌는 순간(또는 게이트 상태가 변하는 순간) 다시 동기화된다
          반대쪽(사람이 넘겨받거나 넘겨주는 순간 리니어가 튀어나오는 것)이 훨씬 위험하다.
        """
        mode = self.auto_mode
        if self._prev_auto_mode is None:      # 첫 판정 — 엣지가 아니다
            self._prev_auto_mode = mode
            return
        if mode == self._prev_auto_mode:
            return

        self._prev_auto_mode = mode
        if self.cmd_brake != 0:
            self.get_logger().warn(
                f"[모드 전환] 남아 있던 브레이크 요청(/brake_level={self.cmd_brake}단)을 "
                f"0 으로 지웁니다 — 전환 순간에 리니어가 체결되지 않게 합니다. "
                f"제동이 필요하면 발행자가 다시 요청합니다")
            self.cmd_brake = 0
        # ★해제 유예도 함께 지운다★ 모드 전환은 '사람이 차를 넘겨받는 순간' 이라
        #   유예가 남아 리니어가 1초 더 물려 있으면 안 된다 — 위 불변식이 우선한다.
        self._brake_hold_level = 0
        self._brake_hold_t = 0.0
        if self._stop_brake_armed and self.stop_brake_level > 0:
            self.get_logger().warn(
                f"[모드 전환] stop_brake_level({self.stop_brake_level}단) 무장을 해제합니다 "
                f"— 자율주행이 /control_state=True 로 구동 허가를 받은 뒤에 다시 무장됩니다 "
                f"(전환 순간에는 리니어를 체결하지 않습니다)")
        self._stop_brake_armed = False

    def compose(self):
        """현재 상태에서 A/B 보드로 보낼 페이로드 두 개 + 라벨용 목표펄스를 만든다.

        반환: (a_payload, b_payload, drive_pulse)
          a_payload   개행 없는 문자열. ★단일값 = 펄스 0~15★ / ★"<pwm>,<pwm>" = 직접 PWM★
                      (콤마 2값은 수동조종에서 페달을 밟는 동안에만 나온다 — 헤더 (2) 참고)
          b_payload   개행 없는 문자열 "<조향각|x>,<브레이크단계>"
          drive_pulse /drive_pulse_cmd 로 발행할 0~15 스케일 라벨. 자율에서는 A보드로
                      실제 나간 펄스와 같은 값이고, 수동조종에서는 같은 페달 개도량을
                      펄스로 환산한 값이다(실제로 나가는 것은 PWM 이다).
                      ★a_payload 를 파싱해서 라벨을 만들지 않는다★ — 콤마 2값이 생긴
                      뒤로는 파싱이 스케일을 뒤섞는 길이 되었다. 여기서 같이 돌려준다.

        우선순위는 파일 헤더의 '주행 상태 판단' 순서와 같다."""

        # (1) E-stop — B보드가 리니어 2단 체결과 0단 복귀를 스스로 한다([0804-3]).
        #     ★ 여기서 수동 래치를 건드리지 않는다 ★ 해제 시 B보드가 이미 HOME(0단)으로
        #     돌아가는데 우리가 2단을 다시 물리면 그 복귀와 싸운다.
        if self.estop_active:
            # ★'페달을 밟아도 안 나간다' 의 이유를 로그에 남긴다★ [2026-08-25]
            #   E-STOP 이 걸려 있으면 이 분기가 수동조종보다 먼저 이겨서 A보드로
            #   '0' 이 나간다 — 페달은 아무 일도 하지 않는다. 그 사실이 로그에
            #   없으면 "보드는 붙었는데 페달이 안 먹는다" 로만 보이고, 원인을
            #   포트·펌웨어에서 찾게 된다(실제로 그렇게 헤맸다). 보드 리셋 직후
            #   E-STOP 이 걸린 채 올라오는 경우가 있어 더욱 그렇다.
            if not self.auto_mode:
                self.get_logger().warn(
                    "⛔ E-STOP 이 걸려 있어 ★페달이 동작하지 않습니다★ "
                    "(수동조종이라도 E-STOP 이 우선합니다). 스위치를 해제하세요 "
                    "— B보드가 500ms 연속 단락을 확인하면 풀립니다",
                    throttle_duration_sec=5.0)
            return '0', f'{STEER_RELEASE_TOKEN},0', 0

        # (1-1) ★AEB 비상정지 — 수동조종 중에도 통하는 유일한 제동 경로★
        #       [2026-08-25 신설] 근거·불변식은 파일 헤더 (1-1) 에 전부 적었다.
        #       ★aeb_brake_level 기본값 0 이면 여기는 절대 참이 되지 않는다★
        #       (white1 런치는 이 값을 주지 않으므로 거동이 종전과 같다).
        #
        #       A보드는 ★단일값 "0"★ 이다 — 콤마 2값이 아니라야 펌웨어가 직접 PWM
        #       모드를 해제하고 코스트로 넘긴다(수동조종에서 페달이 직접 PWM 을
        #       내고 있을 수 있다. (2) 분기 참고). 구동을 끊고 리니어로 잡는다.
        if self.aeb_engaged():
            if not self._aeb_engaged:
                self._aeb_engaged = True
                self.get_logger().warn(
                    f"🛑 ★AEB 비상정지★ 구동 차단 + 리니어 {self.aeb_brake_level}단 "
                    f"— 조향은 {'힘빼기(사람이 핸들)' if not self.auto_mode else '마지막 각도 유지'}")
            # 조향 : 수동조종이면 힘빼기('x') 그대로 — 사람이 핸들을 쥐고 있다.
            #        자율이면 마지막 각도를 유지한다((3) 과 같은 이유).
            steer = (STEER_RELEASE_TOKEN if not self.auto_mode
                     else self.to_board_angle(self.cmd_angle))
            return '0', f'{steer},{self.aeb_brake_level}', 0
        if self._aeb_engaged:
            self._aeb_engaged = False
            self.get_logger().info("🟢 AEB 해제 — 리니어를 풉니다(0단)")

        # (2) 수동조종 (D5 개방) — /control_state 와 무관하게 항상 이 경로
        #     ★[2026-08-04] '진입 시 브레이크 체결' 로직을 완전히 제거했다★
        #     예전에는 자율→수동 전환 엣지에서 manual_brake_level(2단)을 물고,
        #     쓰로틀 raw >= manual_release_raw 가 되면 풀었다. 그런데 모드 전환은 그 자체가
        #     제동 지시가 아니고, 실차에서 스위치를 수동으로 내리는 순간 리니어가 브레이크
        #     페달을 밟고 튀어나왔다(E-STOP 아닌데도). 사람이 넘겨받는 순간 페달이 물려
        #     있으면 오히려 출발도 못 한다.
        #     → 수동에서는 브레이크를 ★항상 0★ 으로 보낸다. 제동은 사람 발이 한다.
        #
        #     ★구동은 페달뿐이다★ [2026-08-07] 에 열었던 'ROS 지정펄스 대체' 경로를
        #     [2026-08-11] 되돌렸다(위쪽 (2) 요약 참고) — 소프트웨어가 수동조종 중
        #     구동을 대신 낼 길이 없어야 사람 조작과 다툴 여지가 아예 사라진다.
        #
        #     ★구동 경로 = manual_use_pwm★
        #       True  : 페달 개도 → 직접 PWM, 콤마 2값. ★파일에서 콤마 2값이 나오는
        #               유일한 곳이다★ 개루프라 밟은 듀티가 유지된다.
        #       False : 페달 개도 → 목표펄스 단일값. 보드 PID 가 속도를 맞추고,
        #               떼면 "0" → 코스트(white1 과 같은 체감).
        if not self.auto_mode:
            cmd_raw = self.throttle_cmd_raw()
            pulse = self.throttle_to_pulse(cmd_raw)
            if self.manual_use_pwm:
                pwm = self.throttle_to_pwm(cmd_raw)
                src = 'pedal' if pwm > 0 else None
                a_payload = f'{pwm},{pwm}' if pwm > 0 else '0'
                engaged_msg = (
                    f"[수동조종] 페달 입력 감지 — 직접 PWM "
                    f"{self.manual_pwm_min}~{self.manual_pwm_max} 구간으로 나갑니다")
            else:
                src = 'pedal' if pulse > 0 else None
                a_payload = str(pulse)
                engaged_msg = (
                    f"[수동조종] 페달 입력 감지 — 목표펄스 "
                    f"0~{self.manual_pulse_max} (떼면 코스트)")
            if src != self._manual_src:
                self._manual_src = src
                if src == 'pedal':
                    self.get_logger().info(engaged_msg)
            return a_payload, f'{STEER_RELEASE_TOKEN},0', pulse

        # (3) ROS 가 정지를 지시한 상태. 조향각은 마지막 값을 유지한다(정면 급조향 방지).
        #     브레이크는 stop_brake_level 이 /brake_level 보다 우선한다 — '정지 지시'가
        #     더 강한 의도이므로, 그때 0 을 받고 있었다고 브레이크를 풀면 안 된다.
        if not self.control_enabled:
            # ★stop_brake_level 은 '무장된' 뒤에만 쓴다★ 자율주행이 한 번도 구동 허가를
            #   받지 않은 상태(= 방금 수동에서 넘어온 순간)는 '자율 정지'가 아니라 대기다.
            #   여기서 무장을 보지 않으면 D5 를 올리는 순간 리니어가 밟힌다
            #   (_disarm_brakes_on_mode_edge 참고). 기본값 0 에서는 차이가 없다.
            stop_brake = self.stop_brake_level if self._stop_brake_armed else 0
            brake = max(stop_brake, self.cmd_brake)
            brake = max(0, min(BRAKE_LEVEL_MAX, brake))
            return '0', f'{self.to_board_angle(self.cmd_angle)},{brake}', 0

        # (4) 정상 자율주행 — /brake_level 을 그대로 반영(안 오면 0)
        brake = max(0, min(BRAKE_LEVEL_MAX, self.cmd_brake))
        # ★해제 유예 [2026-08-14]★ 0 이 아닌 요청을 받은 뒤 BRAKE_RELEASE_HOLD_S 동안은
        #   그보다 낮은 값으로 내려가지 않는다. 발행자가 여럿인 토픽에서 누가 0 을 한 번
        #   흘려도 리니어가 빠졌다 나오지 않게 하는 ★마지막 방벽★ 이다(상수 주석 참고).
        #   더 센 값은 즉시 반영한다 — 막는 것은 '푸는 방향'뿐이다.
        if (time.monotonic() - self._brake_hold_t) <= BRAKE_RELEASE_HOLD_S:
            if self._brake_hold_level > brake:
                brake = self._brake_hold_level
        # ★[2026-08-12] 리니어가 물려 있으면 A보드 REF 는 무조건 0★
        #   구동과 제동을 동시에 걸면 둘이 서로 밀어낸다 — 리니어는 차를 잡으려 하고
        #   인휠은 목표펄스를 맞추려 전류를 더 밀어넣는다. 그 상태가 이어지면
        #   ① 차가 안 서고 ② 모터·리니어가 서로 부하가 되며 ③ A보드가 '지령대로 안
        #   구른다'고 보고 기동 블랭킹을 재트리거해 허수 카운트까지 쏟아진다.
        #   ★제동이 걸린 순간부터는 제동이 이긴다★ 로 못 박는다.
        #   (driving 은 DRIVE_DONE 에서 이미 펄스 0 을 보내므로 평소엔 값이 같다.
        #    이 줄은 camera_judgment 처럼 ★주행 중에 브레이크를 요청하는 다른
        #    발행자★ 가 있을 때 실제로 일을 한다.)
        pulse = 0 if brake > 0 else self.cmd_pulse
        return str(pulse), f'{self.to_board_angle(self.cmd_angle)},{brake}', pulse

    def on_tx_timer(self):
        """값이 바뀌었거나 KEEPALIVE_S 가 지났을 때만 실제로 시리얼에 쓴다.

        ★ 매 주기 무조건 쓰지 않는 이유 ★ B보드 handleLine 은 줄을 받을 때마다
        steer_state 를 ST_ACTIVE 로 되돌린다. 20Hz 로 계속 보내면 도달판정
        (SETTLE_MS=500ms)이 영구히 성립하지 않아 PD 가 목표 근처에서 계속 힘을 준다."""
        # 모드 전환 엣지 정리를 compose() 앞에 둔다 — compose() 는 '지금 상태로 페이로드를
        # 만드는' 순수 판정만 하게 유지한다(상태 변경은 이 함수에서).
        self._disarm_brakes_on_mode_edge()
        a_payload, b_payload, drive_pulse = self.compose()
        now = time.monotonic()

        if a_payload != self._last_a or (now - self._last_a_t) >= KEEPALIVE_S:
            if self.send_line('a', a_payload):
                self._last_a = a_payload
                self._last_a_t = now

        if b_payload != self._last_b or (now - self._last_b_t) >= KEEPALIVE_S:
            if self.send_line('b', b_payload):
                self._last_b = b_payload
                self._last_b_t = now

        # ★ 주행 목표펄스(0~15 라벨)를 발행한다 ★ 수동조종에서는 페달 환산값이므로
        #   mapping 의 수집 라벨 ①이 된다. 보드가 단절되어 전송이 안 된 주기에도
        #   '이번에 내려던 값'을 발행한다 — 라벨은 명령의 기록이기 때문이다.
        #   ★[2026-08-25] a_payload 를 int() 로 되파싱하던 것을 걷어냈다★ 수동조종의
        #   페이로드가 "<pwm>,<pwm>" 이 되면서 그 파싱은 ValueError 로 조용히 버려지거나
        #   (라벨이 통째로 끊긴다) 첫 토큰을 집어 PWM 을 0~15 라벨 자리에 흘릴 수 있다.
        #   이제 compose() 가 라벨을 직접 돌려준다.
        self.pub_drive_pulse.publish(Int32(data=int(drive_pulse)))

        # ★A보드로 실제 나간 직접 PWM★ 콤마 2값일 때만 값이 있고 그 외에는 0 이다.
        #   판정 근거를 a_payload(= 실제로 나간 줄) 하나로 두어, 발행값과 시리얼이
        #   어긋날 수 없게 한다.
        drive_pwm = int(a_payload.split(',')[0]) if ',' in a_payload else 0
        self.pub_drive_pwm.publish(Int32(data=drive_pwm))

    def send_line(self, which, text):
        """한 보드에 한 줄 전송. 성공하면 True.

        전송 실패는 곧 포트 단절이므로 _drop_board 로 넘겨 재연결 대상으로 만든다.
        False 를 돌려주면 호출측이 변경감지 캐시를 갱신하지 않아, 재연결 직후 같은 값이
        다시 전송된다(= 명령이 유실되지 않는다)."""
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return False
        try:
            ser.write((text + '\n').encode('ascii'))
            return True
        except (serial.SerialException, OSError) as e:
            self._drop_board(which, f"전송 실패: {e}")
            return False

    # ═══════════════════════════════════════════════════════════════
    #  보드 → ROS : 수신 + 발행
    # ═══════════════════════════════════════════════════════════════
    def on_rx_timer(self):
        self.poll_port('a')
        self.poll_port('b')
        self.update_estop()
        self._warn_stale_throttle()
        self.publish_telemetry()
        self.publish_status()

    def _warn_stale_throttle(self):
        """A보드는 붙었는데 S, 가 안 오면 페달이 죽은 것처럼 보인다. 조용히 넘기지 않는다."""
        if self.ser_a is None:
            return
        if self._last_s_t <= 0.0:
            self.get_logger().warn(
                "A보드는 붙었는데 S, 텔레메트리(쓰로틀)를 아직 못 받았습니다 "
                "— /throttle_pedal 이 0 으로 보입니다. 보드 부팅·E-STOP 을 확인하세요",
                throttle_duration_sec=5.0)
            return
        age = time.monotonic() - self._last_s_t
        if age > TELEMETRY_STALE_S:
            self.get_logger().warn(
                f"A보드 S, 텔레메트리가 {age:.1f}초째 없습니다 "
                f"(마지막 줄={self.last_line_a!r}). 스로틀 raw 가 갱신되지 않습니다 "
                f"— E-STOP 이 STOP 만 보내거나, TX 에코가 S, 를 가리고 있을 수 있습니다",
                throttle_duration_sec=5.0)

    def poll_port(self, which):
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return
        buf_attr = 'rx_buf_a' if which == 'a' else 'rx_buf_b'
        buf = getattr(self, buf_attr)

        try:
            data = ser.read(4096)
        except (serial.SerialException, OSError) as e:
            # 포트가 죽었다(USB 분리 등) → 그 보드만 떨어뜨리고 _link_loop 가 재연결한다
            self._drop_board(which, f"수신 실패: {e}")
            return
        if data:
            buf += data
        if b'\n' not in buf:
            setattr(self, buf_attr, buf)
            return

        lines = buf.split(b'\n')
        setattr(self, buf_attr, lines[-1])          # 미완성 줄은 버퍼에 보존
        texts = [t for t in
                 (line.decode('ascii', errors='ignore').strip() for line in lines[:-1])
                 if t]
        if not texts:
            return

        # ★ 진단 줄('#'로 시작, kasa_0804_B.ino DEBUG_LINEAR)은 걸러낸다 ★
        #   운용 시엔 DEBUG_LINEAR=false 라 나오지 않지만, 표 보정 중에 켜 두면
        #   그 줄이 최신 줄로 잡혀 텔레메트리를 덮어버린다.
        texts = [t for t in texts if not t.startswith('#')]
        if not texts:
            return

        # ★한 창의 줄을 전부 본다★ 최신 한 줄만 쓰면 TX 에코("<pwm>,<pwm>" / "0")가
        #   S,/P, 뒤에 붙어 스로틀·모드 파싱이 건너뛰어지고, 페달이 죽은 것처럼 보인다.
        #   STOP 과 텔레메트리가 같은 창에 있으면 ★더 나중 줄이 이긴다★
        #   (부팅 직후 STOP → S, 회복 / 체결 직후 S, → STOP).
        last_telem = None
        last_is_stop = False
        junk = 0
        prefix = 'S,' if which == 'a' else 'P,'
        for t in texts:
            if t == 'STOP':
                last_is_stop = True
                last_telem = None
            elif t.startswith(prefix):
                last_telem = t
                last_is_stop = False
            else:
                junk += 1
        if junk:
            self._skip_junk_n += junk
            self.get_logger().warn(
                f"[{which.upper()}보드] 텔레메트리가 아닌 줄 {junk}개를 건너뛰었습니다 "
                f"(예: TX 에코). 스로틀/모드는 S,/P, 만 반영합니다",
                throttle_duration_sec=10.0)

        if last_is_stop:
            if which == 'a':
                self.last_line_a = 'STOP'
            else:
                self.last_line_b = 'STOP'
            return
        if last_telem is None:
            return                          # 에코만 온 주기 — 직전 S,/P, 를 유지
        if which == 'a':
            self.last_line_a = last_telem
            self.parse_a(last_telem)
        else:
            self.last_line_b = last_telem
            self.parse_b(last_telem)

    def parse_a(self, text):
        """"S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" (kasa_0730_A.ino).
           STOP/형식오류 시 마지막 값 유지. 쓰로틀 필드가 없는 구버전(3필드)도 받아준다."""
        if not text.startswith('S,'):
            return
        fields = text.split(',')
        if len(fields) not in (3, 4):
            return
        try:
            self.pulse_l = int(fields[1])
            self.pulse_r = int(fields[2])
            if len(fields) == 4:
                self.throttle_raw = int(fields[3])
                self._thr_buf.append(self.throttle_raw)
            self._last_s_t = time.monotonic()
        except ValueError:
            pass

    def parse_b(self, text):
        """"P,<조향각>,<A5원본>,<모드>" (kasa_0821_B.ino). 조향각 부호는 ★− 좌 / + 우★
           로 ROS 규약과 같다(그대로 발행한다). STOP/형식오류 시 마지막 값 유지.

        ★지금 펌웨어 양식 하나만 받는다 [2026-08-21]★ 구버전 호환 분기를 두지
        않았다 — 보드는 한 대이고, 펌웨어를 바꾸면 이 함수도 같이 바꾸는 것이
        규약이다. 필드 수가 다른 줄은 통째로 버려지므로, 구형을 꽂으면 조향각·모드가
        마지막 값에 얼어붙는다(경고는 남지 않는다). 그 증상이면 여기부터 볼 것.
        """
        if not text.startswith('P,'):
            return
        fields = text.split(',')
        if len(fields) != 4:
            return
        try:
            self.angle_board = int(fields[1])
            self.brake_pot = int(fields[2])
            new_mode = bool(int(fields[3]))       # ★1 = 자율주행 / 0 = 수동조종★
        except ValueError:
            return
        if new_mode != self.switch_mode:
            self.switch_mode = new_mode
            self.get_logger().info(
                f"[주행모드 전환] {'자율주행' if new_mode else '수동조종'} (B보드 D5)")

    def update_estop(self):
        """A·B 중 한쪽이라도 최신 줄이 STOP 이면 e-stop (OR 판정). 전환 시점만 로그."""
        active = (self.last_line_a == 'STOP') or (self.last_line_b == 'STOP')
        if active and not self.estop_active:
            self.get_logger().warn(
                "[E-STOP 발동] A/B 보드 중 하나 이상이 STOP 신호를 보냄 "
                "(B보드가 리니어 2단 체결, 인휠 정지)")
        elif not active and self.estop_active:
            self.get_logger().info("[E-STOP 해제] 정상 텔레메트리 재개 (리니어 0단 복귀)")
        self.estop_active = active

    def publish_telemetry(self):
        """white 규약 토픽으로 보드 상태를 발행.

        ★ /encoder = 좌 + 우 (합) ★ 규약 3 참고. 소비측(white)의 TICKS_PER_REV=192 와
        짝을 이뤄야 m/s 가 맞는다 — 한쪽만 바꾸면 속도가 2배/절반으로 조용히 어긋난다.
        후진이 없으므로 음수는 나오지 않는다(펌웨어가 부호 없는 카운트를 보낸다)."""
        enc = Int32()
        enc.data = max(0, int(self.pulse_l) + int(self.pulse_r))
        self.pub_encoder.publish(enc)

        # 실측 조향각. ★기본은 보드 값 그대로★ (ROS 와 보드가 같은 규약 − 좌 / + 우)
        #   steer_invert=True 일 때만 뒤집어, 명령과 실측이 항상 같은 부호로 보이게 한다.
        ang = Int32()
        ang.data = -int(self.angle_board) if self.steer_invert else int(self.angle_board)
        self.pub_steer_angle.publish(ang)

        self.pub_mode.publish(Bool(data=bool(self.auto_mode)))
        self.pub_throttle.publish(Int32(data=int(self.throttle_raw)))
        # ★[2026-08-21] B보드 A5 원본 (kasa_0821_B.ino)★
        self.pub_brake_pot.publish(Int32(data=int(self.brake_pot)))
        self.pub_estop.publish(Bool(data=bool(self.estop_active)))

    def publish_status(self):
        """"A:0|1,B:0|1,ESTOP:0|1,MODE:0|1[,SRC:sw|ovr,SW:0|1]" — 진단·로스백용.
           보드 탐색 중에도 발행하므로 '연결 중'인지 바로 알 수 있다.

           MODE 는 ★실효 모드★ 다. 소프트웨어 오버라이드가 걸려 있으면 SRC:ovr 와 함께
           물리 스위치 원값(SW)도 붙인다 — 오버라이드가 조용히 숨어 있으면 "스위치를
           돌렸는데 왜 안 바뀌지"로 되돌아온다."""
        msg = String()
        # [2026-08-07] SRC:ovr / SW: 필드가 사라졌다 — 오버라이드가 없으니 MODE 가
        #   곧 물리 스위치 값이고, 둘이 어긋날 방법이 없다.
        msg.data = (f"A:{1 if self.ser_a else 0},B:{1 if self.ser_b else 0},"
                    f"ESTOP:{1 if self.estop_active else 0},"
                    f"MODE:{1 if self.auto_mode else 0}")
        self.pub_status.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    #  종료 정리 (정상 종료 / 부모 사망 공용)
    # ═══════════════════════════════════════════════════════════════
    def _install_exit_handlers(self):
        """★SIGTERM·SIGHUP 에서도 정지값이 나가고 포트가 풀리게 한다★ [2026-08-25]

        ══════════════════════════════════════════════════════════════════
         왜 필요한가 — 실측으로 확인한 구멍
        ══════════════════════════════════════════════════════════════════
        rclpy 는 ★SIGINT 만★ 처리한다. 그 경로는 정상이다(측정: Ctrl+C 에서
        프로세스 그룹 전체가 0.45초에 내려가고 포트도 풀린다). 문제는 SIGTERM 이다:
        핸들러가 없으면 파이썬 기본 동작으로 ★즉사★ 하므로
          · stop_and_close 가 돌지 않는다 → ★정지값이 보드에 안 나간다★
            (A보드 펌웨어에는 무입력 타임아웃이 없다 — 마지막 명령을 계속 물고 있다)
          · 포트가 커널에 의해 갑자기 닫힌다 → DTR 토글로 ★보드가 리셋된다★
            → 부팅 중 E-STOP 핀이 개방으로 읽혀 STOP 이 걸린 채 올라올 수 있고,
              그러면 다음 실행에서 ★페달을 밟아도 안 나간다★ (compose (1) 이 이긴다)
        SIGTERM 은 드물지 않다 — launch 가 SIGINT 후 sigterm_timeout 안에 안 죽으면
        보내고, `kill`·`pkill`·systemd·위 reclaim_port 도 그것을 쓴다.

        ★핸들러 안에서 하는 일을 최소로 둔다★ 정지값 쓰기 + close 뒤 os._exit.
        rclpy 를 건드리지 않는다(신호 문맥에서 executor 를 만지면 데드락 위험).
        proc_guard._die 와 같은 태도다.
        """
        def handler(signum, _frame):
            try:
                self.stop_and_close()
            except BaseException:      # noqa: BLE001 — 정리 실패가 종료를 막지 않게
                pass
            try:
                sys.stdout.flush()
                sys.stderr.flush()
            except Exception:          # noqa: BLE001
                pass
            # 128+signum : 셸 관례. 어디에 블로킹돼 있어도 확실히 끝난다.
            os._exit(128 + signum)

        for sig in (signal.SIGTERM, signal.SIGHUP):
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass                   # 메인 스레드가 아니면 등록할 수 없다 — 무해

    def stop_and_close(self):
        """차를 세우는 정지값을 두 보드에 직접 쓰고 포트를 닫는다.

        ★ A보드 펌웨어에는 무입력 타임아웃이 없다 ★ (0713에서 제거) 마지막 수신 명령을
        계속 물고 있으므로, 이 프로세스가 끝나기 전에 정지값이 반드시 시리얼까지 나가야
        한다. 종료 직전에는 타이머 콜백 경로를 믿을 수 없으므로 여기서 직접 쓴다.

        조향은 0 이 아니라 'x'(힘빼기)를 보낸다 — 수동조종 중이었다면 사람이 핸들을 잡고
        있어서 0도로 급조향하면 위험하다. 브레이크도 0 으로 둔다(리니어가 페달을 물고
        있으면 사람이 차를 움직일 수 없다).

        정상 종료(destroy_node)와 부모 사망 감지(proc_guard) 양쪽에서 호출되며, 두 번
        불려도 안전하다(닫힌 포트는 건너뜀)."""
        # 재연결 스레드를 먼저 멈춘다 — 아래에서 닫은 포트를 그 스레드가 다시 열면
        # 정지값을 보낸 의미가 없어지고 포트가 물린 채로 프로세스가 끝난다.
        self._running = False
        wrote = False
        for ser, text in ((self.ser_a, '0'), (self.ser_b, f'{STEER_RELEASE_TOKEN},0')):
            if ser is None:
                continue
            try:
                if not ser.is_open:
                    continue
                ser.write((text + '\n').encode('ascii'))
                ser.flush()
                wrote = True
            except (serial.SerialException, OSError):
                pass
        if wrote:
            time.sleep(STOP_FLUSH_S)   # 정지값이 실제로 나갈 시간을 준 뒤 닫는다

        for ser in (self.ser_a, self.ser_b):
            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except (serial.SerialException, OSError):
                pass

    def destroy_node(self):
        self.stop_and_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Arduino()
    except KeyboardInterrupt:
        # 보드 탐색 중 Ctrl+C 면 rclpy 의 SIGINT 핸들러가 이미 컨텍스트를 내린 뒤일 수
        # 있다. 그 상태에서 또 shutdown 하면 RCLError 로 exit 1 이 되므로 가드한다.
        if rclpy.ok():
            rclpy.shutdown()
        return
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # ★런치가 내린 SIGINT 의 정상 경로다★ rclpy 의 신호 핸들러가 컨텍스트를
        #   내리면 spin 이 이것을 던진다. 잡지 않으면 트레이스백 + 종료코드 1 이
        #   되어 launch 로그에 '죽었다' 로 남는다 — 정상 종료인데 사고처럼 보인다.
        pass
    except BaseException:              # noqa: BLE001
        # ★예상 못한 예외는 반드시 티가 나야 한다★ 아래 os._exit 로 종료를 강제하는데,
        #   그때 0 을 돌려주면 크래시가 '정상 종료' 로 보인다(launch 로그에도 그렇게
        #   남는다). 트레이스백을 찍고 종료코드를 1 로 남긴다.
        traceback.print_exc()
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # ★여기서 확실히 끝낸다★ [2026-08-25]
        #   위 정리로 포트는 이미 닫혔다. 그런데도 인터프리터 종료가 남은 스레드나
        #   rclpy 내부 정리에 걸려 늦어지면, 그 사이에 사람이 다시 런치를 올린다.
        #   그러면 새 프로세스가 (아직 살아 있는) 이 프로세스와 배타 open 을 다투고,
        #   ★재실행이 보드를 못 잡는다★. 종료를 운에 맡기지 않는다.
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except Exception:              # noqa: BLE001
            pass
        os._exit(code)


if __name__ == '__main__':
    main()
