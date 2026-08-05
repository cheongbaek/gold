// ============================================================
//  B보드 : 조향(PD 위치제어) + 제동(★엔코더 시간표 방식★) + 주행모드 스위치
//          (Arduino Mega 2560) - 0804 버전
//  kasa_0731_B.ino 기반. ★조향·모드·E-stop 판정 로직은 0731과 완전히 동일하다★
//  이번 0804 변경은 '브레이크를 어떻게 움직이는가' 하나뿐이다.
//
//  ★★★ [0804-1] 브레이크 : 고정시간 전이 -> 엔코더 실측 시간표 ★★★
//    [0731까지] 단계가 바뀔 때마다 정해진 방향·PWM으로 0.1초씩 밀었다(완전 열린루프).
//      위치를 알 방법이 없어 '0.1초면 대충 그만큼 간다'에 의존했고, 미끄러지면 누적됐다.
//    [0804] 리니어에 로터리 엔코더(E40S8-2048-3-T-5)를 붙여 위치를 알 수 있게 됐다.
//      그런데 ★엔코더로 폐루프를 돌리는 것은 실패했다★ (linear_0804.ino 실측):
//        목표 150 -> 217에서 멈춤(오버슛 73 = 49%). PWM 255에서 약 1카운트/ms로 움직이고
//        PWM을 끊은 뒤에도 관성으로 26~75카운트 더 간다. '닿는 순간 끊기'로는 원리적으로
//        못 맞춘다(판정 주기를 아무리 줄여도 남는다). 감속하려면 PD가 필요한데 브레이크는
//        반응성이 최우선이고 러프해도 되는 대상이다.
//      그래서 ★linear_0803_speed.ino로 "몇 ms 밀면 몇 카운트 가는가"를 실측해 표로 넣고,
//      필요한 카운트를 그 표로 시간으로 환산해 그 시간만큼만 민다★. 엔코더는 제어에서
//      빠지고 '보험'으로만 남는다 — 이 구조를 linear_0804.ino에서 검증한 뒤 그대로 옮겼다.
//
//    ▶ 단계별 위치 (기본위치 0 기준, 밟는 방향 카운트)
//        0 = 기본위치(놓음) / 1 = POS_FULL의 1/3 (83) / 2 = POS_FULL (250)
//        ★고치는 곳은 POS_FULL 하나★ 1단계는 비율로 따라온다.
//    ▶ 전이 시간 (표 보간 결과)
//        0->2 198ms / 0->1 57ms / 1->2 147ms / 2->1 147ms(역) / 1,2->0 = HOME 500ms(역)
//    ▶ 0단계(기본위치)만은 표를 쓰지 않는다 — 목표가 '카운트'가 아니라 '끝까지'라서
//      하드스톱까지 500ms 밀어 기준점을 되찾는 재영점 동작이다(증분 엔코더의 드리프트 대책).
//    ▶ 엔코더 보험 2단 : 목표선 도달(by=ENC) / 밟는 방향 절대 상한 ENC_HARD_MAX(by=HARDMAX)
//      정상 동작에서는 걸리지 않는다 — 표의 값이 '정착 후'라서 시간이 끝나는 순간의 카운트는
//      목표보다 30~70 작고, 그 차이를 관성이 채운다. 걸렸다면 '표가 실제보다 느리다'는 신호다.
//
//  ★★ [0804-2] 리미트 스위치(D2)가 없어졌다 — 그 자리를 엔코더가 쓴다 ★★
//    0731의 [0731-1] 리미트 스위치는 D2 ── NC ── GND 였는데, D2/D3가 엔코더 전용이 되었다
//    (Mega의 외부 인터럽트 핀은 D2/D3/D18~D21뿐이고, 이 코드는 D2/D3의 INT4/INT5 벡터에
//     직접 붙는다 — readAB()의 포트 읽기와 ISR 이름 둘 다 D2/D3 전용이다).
//    ▶ 리미트가 하던 '너무 나갔을 때 힘 빼기'는 이제 ★엔코더 보험 + 구동시간 상한★이 한다:
//        - ENC_HARD_MAX(330카운트) 초과 시 즉시 정지
//        - 어떤 계산 결과도 MAX_DRIVE_MS(300ms)보다 길게 밀지 않는다(그래서 타임아웃도 불필요)
//    ▶ ★정직하게 적어 둔다★ 엔코더 신호가 아예 끊기면(단선) 보험이 작동하지 않고 시간 상한만
//      남는다. 접점 하나로 물리 차단하던 리미트보다 그만큼 약하다. 되살리려면 D4나 A1 같은
//      빈 핀으로 옮기고 0731의 limitISR/updateLimit/limitBlocking을 복원하면 된다
//      (그 3개 함수는 D2 인터럽트에만 의존했으므로 핀만 바꾸면 그대로 쓸 수 있다).
//
//  ★★ [0804-5] E-stop 발동에도 500ms 확인 시간 (발동/해제 대칭) ★★
//    발동 조건이 "개방(HIGH)을 보면 즉시" → "★500ms 연속 개방 유지★" 로 바뀌었다.
//    해제는 종전과 같이 500ms 연속 단락(LOW). A보드(kasa_0804_A.ino)도 같은 규약이다.
//
//    [왜 0731의 '즉시 발동'을 되돌리는가]
//      0731은 "비상정지가 늦는 것이 위험하다"며 디바운스를 없앴다. 그 판단 자체는 맞지만,
//      대가로 ★13번 라인의 순간 노이즈 한 번에 조향이 힘빠지고 리니어가 밟힌다★.
//      이 라인은 NC 2접점 직렬에 A·B 병렬이고 차량 배선을 길게 타므로 접점 채터링·유도
//      노이즈에 노출된다. 주행 중 오발동은 그 자체가 위험이라 판단해 확인 시간을 되살렸다.
//
//    ★★ 트레이드오프 — 반드시 알고 쓸 것 ★★
//      비상정지가 최대 0.5초 늦게 걸린다. 그 사이 차가 가는 거리는:
//          2.65 m/s ( 9.5km/h) → 약 1.3 m   /   4.42 m/s (15.9km/h) → 약 2.2 m
//         13.26 m/s (47.7km/h) → 약 6.6 m
//      즉 '저속 시험 주행'을 전제로 한 설정이다. 고속에서는 ESTOP_TRIGGER_CONFIRM_MS를
//      100~200ms로 줄이거나, 노이즈를 배선에서 해결한 뒤 0으로 되돌릴 것.
//      ※ 물리적 최후 수단은 여전히 별개로 있어야 한다 — 이 핀은 '소프트웨어 비상정지'다.
//
//    [구현] 핀체인지 인터럽트(PCINT)를 제거했다.
//      ISR의 존재 이유는 '개방을 본 즉시 조향 PWM을 끊는' 것이었다. 발동에 500ms 확인을
//      요구하는 순간 그 즉시성은 요구와 모순되고, 남겨두면 500ms 미만 노이즈에서 조향
//      PWM만 순간 끊겨 오히려 핸들이 덜컥거린다. 판정은 loop 의 digitalRead 폴링만으로
//      하며, loop 는 어떤 주기에도 걸려 있지 않아(리니어 종료판정 때문) 해상도가 남는다.
//      ※ 이로써 [0804-4](ISR이 해제 구동만 끊는다)는 함께 사라졌다. 진행 중이던 리니어
//        구동 취소는 applyEstop() 의 cancelLinear() 가 그대로 담당한다(500ms 늦게).
//      ※ 되살리려면 : 0731_B/0804_B 이전 판의 ISR(PCINT0_vect) / setupEstopPcint() /
//        estop_pin_hit / estop_edge_seen 을 되살리고, updateEstop 의 발동 분기를
//        estop_active = true 로 바꾸면 된다.
//
//  ★★ [0804-3] E-stop : 발동 = 리니어 '2' / 해제 = 리니어 '0' ★★
//    ▶ 발동 시 : 브레이크 단계 2(풀브레이킹)와 정확히 같은 작용. 단 체결 시간만은 표(남은
//      거리 환산)가 아니라 ★ESTOP_PUSH_MS(최대 행정 시간)★를 그대로 준다 — 비상정지에서
//      부족한 것이 위험하기 때문이다. 이미 밟혀 있으면 엔코더 보험이 목표선에서 끊으므로
//      과하게 밀리지도 않는다.
//    ▶ 해제 시 : ★브레이크 단계 0(기본위치)과 같은 작용을 스스로 낸다★ 송신측이 0을
//      보내주기를 기다리지 않는다(0731은 단계 2를 유지한 채 명령을 기다렸다). 해제 확정
//      즉시 brake_cmd_level을 0으로 두므로 HOME(역방향 500ms) 복귀가 나가고, 하드스톱에서
//      기준점까지 함께 회복된다. 그 뒤에 송신측이 다시 1/2를 보내면 정상적으로 재체결된다.
//    ※ 발동·해제 모두 진행 중이던 리니어 구동을 취소하되 ★재영점은 하지 않는다★ —
//      영점을 옮기지 않아야 엔코더가 '지금 얼마나 밟혀 있는지'를 그대로 담고 있고, 그래야
//      뒤이은 체결이 남은 거리를 정확히 본다.
//    ※ 취소 시점의 진행 방향이 새 방향과 반대였다면 REVERSE_DEADTIME_MS(30ms)만 쉬고
//      시작한다(VNH2SP30 계열은 회전 중 방향 반전에서 전류 스파이크가 크다. 30ms면 차량이
//      0.6m/s에서 2cm 갈 시간이라 안전상 무시할 수준이다).
//
//  ~~[0804-4] E-stop ISR은 '해제 구동만' 끊는다~~ → ★[0804-5]에서 ISR 자체를 제거했으므로
//    이 항목은 폐기되었다★ (그 역할은 applyEstop 의 cancelLinear 가 이어받는다).
//    참고로 0804-4의 취지는 "ISR이 방향을 안 보고 리니어 PWM을 끊으면, 마침 체결(FWD)
//    중이었을 때 브레이크가 덜 밟힌 채 멈춘다"는 것이었다. cancelLinear 경로에서는
//    체결/해제 여부와 반전 보호(REVERSE_DEADTIME_MS)를 함께 판단하므로 그 문제가 없다.
//
//  --- 이하 구조는 0731과 동일 ---
//  ~~[0731-4] E-stop 즉시 발동~~ → [0804-5]로 대체 (발동도 500ms 확인, PCINT 제거)
//  [0731-3] 조향 하드리밋 실측값 : 좌 576 / 우 362 (2026-07-31)
//  [0730-1] 자율주행/수동조종 스위치(D5) + 텔레메트리 3필드, 50ms 디바운스
//  [0730-2] 조향 힘빼기 : 조향각 자리에 'x' -> DC모터 무동력
//  [0727-1] 가변저항 필터 : 9샘플 중앙값(안전 판정) + 지수평활(PD 입력)
//  [0727-2] 하드 리밋 탈출 허용 : 리밋을 '더 파고드는 방향'만 차단
//
//  ★★ 프로토콜은 0731과 완전히 같다 (송신측 수정 불필요) ★★
//    입력 : "<조향각도>,<브레이크단계>"   (콤마 구분, 개행 종료)
//           - 조향각도 : 정수 -40~40, 또는 'x'/'X'(힘빼기)
//           - 브레이크단계 : ★0 / 1 / 2★ (그 외 값은 브레이크 필드만 무시, 조향은 적용)
//           - 예: "-10,1" / "x,0"
//    출력 : "P,<조향각환산값>,<모드>" (50ms) / "STOP" (e-stop 중)
//    ※ ★setup에서도 아무 안내를 출력하지 않는다★ 출력 양식을 그대로 지키기 위함이다.
//      실차에서 표를 보정하려면 DEBUG_LINEAR를 true로 두면 '#'로 시작하는 진단 줄이
//      추가로 나간다(want/got/coast). 운용 시에는 반드시 false로 되돌릴 것.
//
//  ★ 배선 ★
//    - DC 조향 : DIR=D6, PWM=D7, 가변저항=A2                     (0731과 동일)
//    - 리니어(브레이크) MD20A : DIR=D8, PWM=D9   MB-빨강, MA-검정 (0731과 동일)
//    - ★엔코더 E40S8-2048-3-T-5 : A->D2(INT4), B->D3(INT5), C(Z) 미사용, 5V/GND★ (신규)
//        x4 디코딩 -> 1회전 8192카운트, 1카운트 = 0.044도
//        ※ 밟는 방향으로 움직였을 때 카운트가 음수로 가면 INVERT_DIR을 true로
//    - 주행모드 스위치 : D5 ── 스위치 ── GND (통전=자율 1 / 개방=수동 0)
//    - E-stop : D13, NC 2접점 직렬, A보드와 병렬 감지 (평상시 LOW / 개방 시 HIGH)
//               ★[0804-5] 발동: 500ms 연속 개방 / 해제: 500ms 연속 단락 — 대칭★
//
//  ★ 켤 때 주의 ★
//    부팅 순간의 리니어 위치를 '기본위치(0단계)'로 간주한다. 브레이크가 밟힌 상태로 켜면
//    단계 모델이 그만큼 어긋나므로, 확실하지 않으면 브레이크 0 명령을 한 번 보내 복귀시킨다.
//
//  동작 : delay() 미사용, millis() 기준 논블로킹 (엔코더 카운트는 인터럽트로만 갱신)
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- DC 조향모터 (MD20A + 가변저항 A2) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A2;   // ★ 조향 가변저항 (이 보드 전용)

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- [0804-2] 리니어 엔코더 (D2/D3 고정. 0731의 리미트 스위치 자리) ---
// Mega 2560의 외부 인터럽트 핀은 D2, D3, D18~D21뿐이고, 이 코드는 D2/D3의 INT4/INT5
// 벡터에 직접 붙는다(오버헤드 5~6us -> 2~3us). readAB()의 포트 읽기와 ISR 이름 둘 다
// D2/D3 전용이므로 핀을 바꾸려면 두 곳을 함께 고쳐야 한다.
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
// [0804-5] 폴링으로만 판정한다 (PCINT 제거 — 발동에 500ms 확인을 두므로 즉시 감지가
//   요구와 모순된다). 그래서 D13이 PB7/PCINT7 이라는 사실은 이제 쓰이지 않는다.
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// --- [0730-1] 주행모드 스위치 (단순 ON/OFF, D5 ── 스위치 ── GND) ---
const uint8_t MODE_PIN = 5;


// ================= 통신 =================
const unsigned long BAUD = 115200;

// ★ 진단 출력 (기본 false — 출력 양식을 지킨다) ★
// true로 두면 '#'로 시작하는 리니어 진단 줄이 추가로 나간다(표 보정용 want/got/coast).
// 송신측(ROS2)이 'P,'/'STOP'만 파싱한다면 켜도 무해하지만, 운용 시에는 false로 되돌릴 것.
const bool DEBUG_LINEAR = false;


// ================= 조향 PD 게인 (여기서 조절) =================
float KP_S = 6.0f;
float KD_S = 0.1f;

// ================= 조향 PWM 상한/하한 =================
const int STEER_MIN_PWM = 110;
const int STEER_MAX_PWM = 255;

// ================= 조향 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;   // dc_0702_pd.ino와 동일 (PD게인 호환)

// ================= 조향 입력 각도 범위 =================
const int STEER_ANGLE_MAX =  40;
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

// ================= [0731-3] 실측 좌/우 하드 리밋 (raw, 0~1023) =================
const int RAW_LEFT_LIMIT  = 576;   // 왼쪽 끝 (하드 리밋)
const int RAW_RIGHT_LIMIT = 362;   // 오른쪽 끝 (하드 리밋)

// ================= 조향 안전 여유값 =================
const int SAFETY_MARGIN = 10;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// -40도/+40도에 대응하는 목표 raw값 (하드 리밋보다 SAFETY_MARGIN만큼 안쪽)
const int POT_AT_ANGLE_MIN = RAW_LEFT_LIMIT  - SAFETY_MARGIN;   // 각도 -40 -> 이 raw값
const int POT_AT_ANGLE_MAX = RAW_RIGHT_LIMIT + SAFETY_MARGIN;   // 각도 +40 -> 이 raw값

// [0727-2] 하드 리밋을 raw의 상/하한으로 정규화 (좌/우 어느 쪽이 큰 값이든 동일하게 동작)
const int RAW_HI_LIMIT = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT) ? RAW_LEFT_LIMIT  : RAW_RIGHT_LIMIT;
const int RAW_LO_LIMIT = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT) ? RAW_RIGHT_LIMIT : RAW_LEFT_LIMIT;

// ================= [0727-1] 가변저항 필터 : 9샘플 중앙값 + 지수평활 =================
// ※ [0804] analogRead 9회는 약 0.9ms 블로킹이다. 그 동안 리니어 종료 판정이 늦어져
//   최대 1~2카운트(0.9ms x 약 1.8카운트/ms) 오차가 생길 수 있다. 20ms에 한 번뿐이고
//   전체 행정이 250카운트라 무시할 수준이다(엔코더 카운트는 인터럽트라 유실되지 않는다).
const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;   // -1 = 아직 초기화 안 됨
int   lastPotMedian = 512;     // 최근 중앙값 (하드리밋 판정 / 텔레메트리 공용)

// ================= 조향 도달 판정 (히스테리시스 분리) =================
const int STEER_TOLERANCE_ENTER = 3;   // 이 이하로 좁아지면 도달판정 타이머 시작/유지
const int STEER_TOLERANCE_EXIT  = 6;   // 이 이상으로 벌어져야 "도달 실패"로 재판정
const unsigned long SETTLE_MS = 500;   // 허용범위 유지 시간 -> 도달 판정 (조향 전용)

#define DIR_CW   HIGH   // 조향 왼쪽 (raw 증가 방향)
#define DIR_CCW  LOW    // 조향 오른쪽 (raw 감소 방향)


// ================= 브레이크 단계 정의 (0731과 동일한 의미) =================
const uint8_t BRAKE_NONE = 0;   // 기본위치 (놓음)
const uint8_t BRAKE_SOFT = 1;   // 약한 브레이킹
const uint8_t BRAKE_FULL = 2;   // 풀브레이킹
const uint8_t BRAKE_LEVEL_MAX = 2;

// ================= 리니어 방향 규약 (0731과 동일) =================
// 정방향(브레이크를 밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= [0804-1] 단계별 위치 (엔코더 카운트, 여기서 조절) =================
// ★ 고치는 곳은 POS_FULL 하나뿐이다 ★ 1단계는 비율로 자동 계산된다(현재 1/3).
//   ※ 정수 나눗셈이라 나머지는 내림된다(250 x 1/3 = 83.3 -> 83). 1카운트 = 0.044도.
// linear_0803_pot.ino 실측(2026-08-04)에서 전 행정이 약 400카운트(=17.6도)였다.
// 250은 그 약 63%로, 표의 신뢰구간 안쪽이면서 보험이 걸릴 여지도 남는 지점이다.
const long POS_FULL    = 250;   // ★ 2단계(풀브레이킹)의 변화량 — 여기만 고친다 ★
const long POS_MID_NUM = 1;     // 1단계 = POS_FULL x (NUM/DEN)  ★현재 1/3★
const long POS_MID_DEN = 3;

const long POS_LEVEL0 = 0;                                     // 0 : 기본위치
const long POS_LEVEL1 = POS_FULL * POS_MID_NUM / POS_MID_DEN;   // 1 : 약한 브레이킹 (83)
const long POS_LEVEL2 = POS_FULL;                              // 2 : 풀브레이킹 (250)
const long LEVEL_POS[3] = { POS_LEVEL0, POS_LEVEL1, POS_LEVEL2 };


// ================= 리니어 구동 PWM =================
// ★ 항상 이 값으로 구동한다 ★ 아래 실측표가 이 PWM에서 잰 값이므로, 이 값을 바꾸면 표가
//   전부 무효가 된다 — linear_0803_speed.ino를 같은 PWM으로 다시 돌려야 한다.
const int DRIVE_PWM = 255;


// ================= [0804-1] 실측표 : 구동시간(ms) -> 이동량(카운트) =================
// ★ 출처 : linear_0803_speed.ino, 2026-08-04, PWM 255, '정착 후' 값 ★
//   10~40ms 구간은 4~56카운트로 튀어(기동 마찰 구간) 신뢰할 수 없어 표에서 뺐다.
//
//   ★ 표가 선형이 아니다 ★ 50~140ms는 약 0.9카운트/ms인데 150ms부터 1.8카운트/ms로
//     빨라진다(마찰을 벗어나며 가속). 그래서 단일 기울기(카운트 = ms x 1.3 같은 식)를
//     쓰지 않고 ★표를 구간 선형보간★ 한다. 이 구부러짐이 이 방식의 핵심 근거다.
//
//   재측정하면 두 배열만 갈아끼우면 된다(길이는 SPD_N으로 함께 고칠 것).
//   ※ 80/90ms(113,111), 180/190ms(238,238)처럼 뒤집히거나 같은 구간이 있다. 역보간이
//     분모 0을 만나면 그 구간의 '짧은 쪽 시간'을 쓴다 — 짧게 미는 것이 안전 방향이다.
const uint8_t  SPD_N = 19;
const uint16_t SPD_MS[SPD_N]  = {  50,  60,  70,  80,  90, 100, 110, 120, 130, 140,
                                  150, 160, 170, 180, 190, 200, 210, 220, 230 };
const int16_t  SPD_CNT[SPD_N] = {  64,  91, 108, 113, 111, 120, 122, 135, 136, 147,
                                  175, 197, 218, 238, 238, 253, 266, 293, 317 };

// 표 첫 점(50ms=64카운트)보다 작은 이동은 시간으로 제어할 수 없다(위 10~40ms 참고).
// 이 값 미만이면 아예 구동하지 않고 단계만 확정한다.
const long MIN_MOVE_COUNTS = 30;

// 표 밖(317 초과)은 마지막 두 점 기울기로 외삽한다. 그 결과의 상한.
// ★ 안전 상한이다 ★ 어떤 계산 결과도 이 시간보다 길게 밀지 않는다(= 타임아웃이 불필요한 이유).
const unsigned long MAX_DRIVE_MS = 300;

// ★ 역방향(놓는 방향) 시간 보정 [%] ★
//   실측은 FWD(밟는 방향)만 했다. REV는 브레이크 스프링이 복귀를 도와 더 빠를 수 있다.
//   100 = 표 그대로. 실차에서 REV가 매번 목표를 넘으면(by=ENC가 자주 걸리면) 80, 70…으로
//   줄인다. 정확히 하려면 linear_0803_speed.ino의 방향을 바꿔 재측정할 것.
const long REV_TIME_PCT = 100;

// 0단계(기본위치) 복귀 시간. ★이것만은 표를 쓰지 않는다★ 목표가 '카운트'가 아니라
// '하드스톱까지'이기 때문이다(증분 엔코더의 누적 드리프트를 여기서 리셋한다).
const unsigned long HOME_DRIVE_MS = 500;

// [0804-3] E-stop 체결 전용 시간. 부족한 것이 위험하므로 남은거리 계산을 쓰지 않고
// 최대 행정 시간을 그대로 준다(과함은 엔코더 보험이 막는다).
const unsigned long ESTOP_PUSH_MS = 250;

// PWM을 끊은 뒤 관성이 멈추기를 기다리는 시간. 이 뒤에 영점을 잡는다.
// ★ 표의 값이 '정착 후'이므로 이 대기가 없으면 기준점이 매번 앞당겨진다 ★
const unsigned long LIN_SETTLE_MS = 100;

// 방향 반전 보호 (E-stop이 해제 구동을 취소하고 체결로 넘어갈 때만 쓰인다)
const unsigned long REVERSE_DEADTIME_MS = 30;


// ================= [0804-1] 엔코더 = 보험 (제어에서 빠졌다) =================
// 시간이 끝나기 전에 목표 카운트에 닿으면 끊는다. 정상 동작에서는 걸리지 않는다 —
// 표의 값이 '정착 후'라서 시간이 끝나는 순간의 카운트는 목표보다 30~70 작기 때문이다.
// 걸렸다면 '표가 실제보다 느리게 잡혀 있다'는 신호다(진단 로그 by=ENC).
const long POS_TOLERANCE = 5;    // 보험 판정 여유 (목표선을 이만큼 앞두면 도달로 본다)

// 영점 대비 이 카운트를 넘으면 즉시 정지 (0731의 리미트 스위치를 대신하는 최종 보험).
// ★ 밟는 방향 구동에만 적용한다 ★ 해제 방향에 걸면 깊이 밟힌 상태에서 브레이크가 풀리지
//   않는 사고가 난다(updateLinear의 (1) 주석 참고).
const long ENC_HARD_MAX = 330;

// ★ 대기 중 자동 재영점 ★ 엔코더가 이 시간 동안 IDLE_QUIET_BAND 이내로 머물면 그 지점을
//   '지금 단계의 새 기준점'으로 잡는다(단계는 그대로 유지). 증분 엔코더의 드리프트와,
//   사람이 페달을 밟았다 뗀 뒤 남는 기준점 이동을 함께 흡수한다.
//   ※ 발을 페달에 올린 채 3초를 넘기면 '밟힌 그 위치'가 기준점이 된다. 발을 뗀 뒤 다시
//     3초가 조용하면 그때 또 보정되므로 결국 제자리로 수렴한다.
const unsigned long IDLE_ZERO_MS    = 3000;
const long          IDLE_QUIET_BAND = 2;


// ================= 엔코더 스펙 / 옵션 =================
const long PPR            = 2048;      // A상 1회전당 펄스 수
const long COUNTS_PER_REV = PPR * 4;   // x4 -> 8192 (참고용)

const bool USE_PULLUP = false;         // 토템폴(T) 출력이라 INPUT이 맞다. 오픈컬렉터면 true
const bool INVERT_DIR = false;         // ★밟는 방향이 음수로 세어지면 true★ (배선 확인용)
const bool RECOVER_LOST_EDGES = true;  // 유실 엣지 ±2 보정 (linear_0803_pot.ino와 동일)

// x4 쿼드러처 전이 표. 상태 = (A<<1)|B, 인덱스 = (이전<<2)|현재.
// +1/-1 = 정상 전이(부호=방향), 0 = 무변화 또는 두 비트 동시 변화(엣지 유실).
const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

// ISR이 갱신하는 값. long 4바이트 읽기는 원자적이지 않아 loop에서 noInterrupts()로 감싼다.
volatile long     enc_count = 0;   // ★ 지금 영점 대비 변화량 ★
volatile uint8_t  enc_state = 0;   // 직전 (A<<1)|B
volatile uint16_t enc_err   = 0;   // 엣지 유실 횟수 (진단용)
volatile int8_t   enc_dir   = 0;   // 마지막으로 확정된 진행 방향 (유실 보정용)


// ================= 조향 PD 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int  steer_angle_cmd = 0;     // 마지막으로 수신한 명령 각도 (참고/디버그용)
int  target_pos = 512;        // PD 목표 raw값
int  prev_pos   = 0;          // 미분항 계산용 이전 raw값 (필터링된 값 기준)
unsigned long steer_win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;


// ================= [0804-1] 리니어(브레이크) 상태머신 =================
//   LIN_IDLE   : 무출력 대기. 사람이 페달을 밟아 엔코더가 변해도 그냥 둔다
//   LIN_MOVE   : ★표에서 뽑은 시간만큼★ 단방향 구동 (엔코더는 보험으로만 본다)
//   LIN_HOME   : 0단계 복귀. 하드스톱까지 시간 구동 (엔코더 무시, 열린루프)
//   LIN_SETTLE : PWM 0으로 관성 정착 대기 -> 끝나면 영점 리셋 + 단계 확정
// ※ 0731에 있던 '전이 중 새 명령' 처리가 필요 없다 — 구동이 끝나고 LIN_SETTLE(100ms)을
//   거쳐 IDLE이 되므로 방향 반전 사이에 100ms 간격이 자동으로 생긴다. 명령은 그동안
//   brake_cmd_level에 담아 두고 IDLE이 되는 순간 이어서 전이한다(0731과 같은 태도).
enum LinState { LIN_IDLE, LIN_MOVE, LIN_HOME, LIN_SETTLE };
LinState lin_state = LIN_IDLE;

uint8_t brake_level     = BRAKE_NONE;   // ★지금 물려 있다고 보는 단계 (영점의 기준)★
uint8_t brake_cmd_level = BRAKE_NONE;   // 수신된 목표 단계
uint8_t lin_tgt_level   = BRAKE_NONE;   // 진행 중인 구동이 끝나면 확정될 단계

long          lin_target      = 0;   // 영점 대비 목표 변화량 (보험 판정선)
int8_t        lin_dir_sign    = 0;   // +1 = 밟는 방향, -1 = 놓는 방향 (시작 시 고정)
unsigned long lin_planned_ms  = 0;   // ★표에서 뽑은, 이번에 밀 시간★ (주 종료조건)
unsigned long lin_drive_t     = 0;   // 구동 시작 시각
unsigned long lin_settle_t    = 0;   // 정착 시작 시각
long          lin_stop_count  = 0;   // PWM을 끊은 순간의 카운트 (관성분 진단용)
long          lin_need        = 0;   // 이번 구동에서 움직여야 하는 양 (HOME은 0)
long          lin_start_count = 0;   // 구동 시작 시점의 엔코더 값
const char*   lin_why         = "-"; // TIME / ENC / HARDMAX / HOME / SKIP
unsigned long lin_gate_t      = 0;   // 이 시각 전에는 새 구동을 시작하지 않는다(반전 보호)

int  brake_output = 0;               // 지금 리니어에 주고 있는 PWM (디버그용)

// [0804-4] E-stop ISR이 방향을 보고 판단할 수 있도록 현재 구동 방향을 남긴다
// ★ [0804-5] 현재 '기록만' 되고 읽히지 않는다 ★ 유일한 독자가 제거된 E-stop ISR
//   이었다([0804-4]). ISR을 되살릴 때 필요하므로 남겨 두었고, ISR이 없으니 volatile 도
//   떼었다. 진행 방향이 실제로 필요한 곳은 lin_dir_sign 을 쓴다(같은 정보).
uint8_t linear_dir_now = LINEAR_FWD;

// 대기 중 자동 재영점 감시 (LIN_IDLE 전용)
long          quiet_ref = 0;
unsigned long quiet_t   = 0;


// ================= [0730-1] 주행모드 상태 =================
bool auto_mode = false;               // setup()에서 실제 핀 상태로 프라이밍
uint8_t mode_pin_last = HIGH;         // 마지막으로 관측된 원시 레벨
unsigned long mode_change_t = 0;      // 레벨이 바뀐 시각 (0 = 확정됨/변화 없음)
const unsigned long MODE_CONFIRM_MS = 50;   // 접점 바운스 안정화 시간


// ================= [0804-5] E-stop 상태 =================
bool estop_active = false;
bool estop_latched = false;          // e-stop에 진입했다 (진입/해제 엣지 처리용)
bool estop_push_started = false;     // [0804-3] 이번 e-stop에서 체결을 이미 시작했다

// ★ 발동·해제 모두 이 시간 동안 레벨이 '연속으로' 유지되어야 인정한다 (대칭) ★
//   발동 : ESTOP_TRIGGER_CONFIRM_MS 동안 계속 개방(HIGH)
//   해제 : ESTOP_RELEASE_CONFIRM_MS 동안 계속 단락(LOW)
//   중간에 반대 레벨이 한 번이라도 관측되면 해당 타이머는 0으로 리셋된다.
//
// ⚠️ 발동을 늦추면 그만큼 차가 더 간다(헤더 [0804-5] 트레이드오프 표 참고).
//   고속 주행에서는 TRIGGER 를 100~200ms 로 줄이거나 0(즉시)으로 되돌릴 것.
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 500;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 500;

// 각 레벨이 '처음' 관측된 시각. 0 = 아직 그 레벨을 관측하지 않음(또는 리셋됨).
unsigned long estop_high_t = 0;   // HIGH(개방)가 처음 관측된 시각 -> 발동 타이머
unsigned long estop_low_t  = 0;   // LOW(단락)가 처음 관측된 시각 -> 해제 타이머


// ================= 출력용 =================
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearDrive(uint8_t dir, int p);
static inline uint8_t readAB();
long encRead();
void zeroHere();
void setupEncoder();
unsigned long msForCounts(long counts);
void beginBrake(uint8_t lv, unsigned long now, bool force_full_time);
void startLinMove(uint8_t lv, unsigned long now, bool force_full_time);
void startLinHome(unsigned long now);
void enterLinSettle(unsigned long now, long at);
void finishLinAt(uint8_t lv, unsigned long now);
void cancelLinear(unsigned long now);
void updateLinear(unsigned long now);
void armIdleZero(unsigned long now);
void updateIdleZero(unsigned long now);
void updateBrake(unsigned long now);
// (setupEstopPcint 는 [0804-5]에서 제거됨 — PCINT 미사용)
void updateEstop(unsigned long now);
void applyEstop(unsigned long now);
int  angleToPot(int angle);
int  potToAngle(int raw);
int  readPotMedian();
int  smoothPot(int med);
void updateSteer(unsigned long now);
void releaseSteer();
bool isValidNumber(const char* s);
bool isReleaseToken(const char* s);
void handleLine(char* line);
void pollSerial();
void updateMode(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
  brake_output = 0;
}

// 방향은 LINEAR_FWD(정, 체결) / LINEAR_REV(역, 해제) 둘 중 하나.
// ★ DIR을 먼저 쓰고 PWM을 나중에 쓴다 ★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 들어간다.
void linearDrive(uint8_t dir, int p) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  linear_dir_now = dir;                       // [0804-5] 기록만 (독자였던 ISR은 제거됨)
  brake_output = constrain(p, 0, 255);
  analogWrite(LINEAR_PWM_PIN, brake_output);
}


// ================= 엔코더 : A/B 레벨 읽기 =================
// ★ Mega 2560 : D2 = PE4, D3 = PE5 ★ (Uno와 포트가 다르다)
//   포트를 한 번에 읽으므로 빠르고(약 0.2us), A/B가 서로 어긋나 읽힐 일도 없다.
static inline uint8_t readAB() {
  uint8_t p = PINE;
  uint8_t a = (p >> 4) & 0x01;   // PE4 = D2
  uint8_t b = (p >> 5) & 0x01;   // PE5 = D3
  return (uint8_t)((a << 1) | b);
}


// ================= 엔코더 1엣지 처리 (두 ISR이 공유) =================
// always_inline이라 아래 두 벡터 안에 코드가 그대로 박힌다(함수 호출 오버헤드 0).
// 시리얼 출력은 절대 하지 않는다.
static inline void encStep() __attribute__((always_inline));
static inline void encStep() {
  uint8_t cur  = readAB();
  uint8_t prev = enc_state;
  if (cur == prev) return;                    // 같은 상태로의 인터럽트(글리치) — 무시
  enc_state = cur;

  int8_t step = QDEC_TABLE[(prev << 2) | cur];
  if (step == 0) {
    // A와 B가 동시에 바뀐 것으로 보인다 = 중간 상태를 건너뛰었다(= 정확히 2카운트).
    // 방향만 알 수 없으므로 직전 진행 방향을 이어 붙인다(버리면 빠른 쪽만 덜 세어진다).
    enc_err++;
    if (RECOVER_LOST_EDGES && enc_dir != 0) {
      enc_count += (long)(INVERT_DIR ? -enc_dir : enc_dir) * 2;
    }
    return;
  }
  enc_dir = step;                    // 정상 전이에서만 방향을 갱신한다
  enc_count += INVERT_DIR ? -step : step;
}


// ================= 엔코더 ISR 벡터 직결 (INT4=D2, INT5=D3) =================
// attachInterrupt는 공용 핸들러에서 함수 포인터를 거쳐 호출하며 1회 5~6us가 든다.
// 벡터에 직접 붙이면 2~3us로 줄어 누락 한계가 두 배가 된다(약 1,200 -> 2,500RPM).
// 이 보드의 리니어 속도(약 1.8카운트/ms = 1,800 인터럽트/초)에서는 CPU 점유 약 0.5%다.
ISR(INT4_vect) { encStep(); }
ISR(INT5_vect) { encStep(); }


// ================= 엔코더 값 읽기 (loop 전용) =================
long encRead() {
  long c;
  noInterrupts();
  c = enc_count;
  interrupts();
  return c;
}


// ================= 영점 재설정 =================
// 지금 위치를 0으로 만든다.
// ★ enc_state는 건드리지 않는다 ★ 진행 중인 쿼드러처 전이의 판정 기준이라, 다시 읽어
//   넣으면 마침 대기 중이던 엣지를 한 카운트 잃는다. 기준점만 옮기면 된다.
void zeroHere() {
  noInterrupts();
  enc_count = 0;
  enc_err   = 0;
  enc_dir   = 0;
  interrupts();
}


// ================= 엔코더 인터럽트 설정 =================
// INT4~INT7은 EICRB가 담당하고 감지 방식은 2비트다: 00=LOW, 01=양쪽엣지, 10=하강, 11=상승.
// -> ISCn1=0, ISCn0=1 이 CHANGE에 해당한다.
void setupEncoder() {
  const uint8_t mode = USE_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_A_PIN, mode);
  pinMode(ENC_B_PIN, mode);

  // 현재 레벨을 시작 상태로 채택한 뒤 인터럽트를 건다
  // (먼저 걸면 enc_state가 0이라 첫 전이가 엉뚱하게 판정된다)
  enc_state = readAB();
  enc_count = 0;                 // ★ 켜진 순간의 위치가 영점 = 기본위치(0단계) ★
  enc_err   = 0;
  enc_dir   = 0;

  EICRB = (EICRB & ~((1 << ISC41) | (1 << ISC40) | (1 << ISC51) | (1 << ISC50)))
                 | (1 << ISC40) | (1 << ISC50);
  EIFR  = (1 << INTF4) | (1 << INTF5);    // 설정 중에 쌓인 대기 플래그를 지운 뒤
  EIMSK |= (1 << INT4) | (1 << INT5);     // 두 인터럽트를 활성화
}


// ================= [0804-1] 이동량(카운트) -> 구동시간(ms) : 실측표 역보간 =================
// ★ 이 함수가 새 브레이크의 심장이다 ★ 엔코더로 '언제 멈출까'를 판단하는 대신 미리
//   '얼마나 밀까'를 여기서 결정한다. 표가 선형이 아니므로 구간마다 보간한다.
//   입력은 항상 양수(이동량의 크기). 방향 보정(REV_TIME_PCT)은 호출측에서 한다.
unsigned long msForCounts(long counts) {
  if (counts <= 0) return 0;

  // 표 첫 점(50ms=64카운트) 미달 : 원점과 첫 점을 이어 비례 (신뢰도 낮은 구간)
  if (counts <= (long)SPD_CNT[0]) {
    return (unsigned long)((long)SPD_MS[0] * counts / (long)SPD_CNT[0]);
  }

  // 표 안 : counts를 처음 넘어서는 구간에서 선형보간
  for (uint8_t i = 1; i < SPD_N; i++) {
    if (counts <= (long)SPD_CNT[i]) {
      long d = (long)SPD_CNT[i] - (long)SPD_CNT[i - 1];
      if (d <= 0) return SPD_MS[i - 1];   // 뒤집힌/평탄 구간 -> 짧은 쪽 시간(안전 방향)
      return (unsigned long)((long)SPD_MS[i - 1]
             + (counts - (long)SPD_CNT[i - 1])
               * ((long)SPD_MS[i] - (long)SPD_MS[i - 1]) / d);
    }
  }

  // 표 밖(317 초과) : 마지막 두 점 기울기로 외삽하고 MAX_DRIVE_MS로 자른다
  long d  = (long)SPD_CNT[SPD_N - 1] - (long)SPD_CNT[SPD_N - 2];
  long ms = (long)SPD_MS[SPD_N - 1];
  if (d > 0) {
    ms += (counts - (long)SPD_CNT[SPD_N - 1])
          * ((long)SPD_MS[SPD_N - 1] - (long)SPD_MS[SPD_N - 2]) / d;
  }
  if (ms > (long)MAX_DRIVE_MS) ms = (long)MAX_DRIVE_MS;
  if (ms < 0) ms = 0;
  return (unsigned long)ms;
}


// ================= 브레이크 구동 시작 =================
void beginBrake(uint8_t lv, unsigned long now, bool force_full_time) {
  if (lv == BRAKE_NONE) startLinHome(now);
  else                  startLinMove(lv, now, force_full_time);
}


// ================= 1/2단계 : 표에서 뽑은 시간만큼 구동 (엔코더는 보험) =================
// force_full_time = true 이면 표 대신 ESTOP_PUSH_MS를 쓴다([0804-3] E-stop 전용).
void startLinMove(uint8_t lv, unsigned long now, bool force_full_time) {
  long rel = LEVEL_POS[lv] - LEVEL_POS[brake_level];   // 현재 단계 기준 목표 변화량
  long cur = encRead();
  long err = rel - cur;                               // 실제로 움직여야 하는 양
  long mag = labs(err);

  // ★ 시간으로 제어할 수 없는 소량은 아예 움직이지 않는다 ★
  //   실측 10~40ms가 4~56카운트로 튀었으므로, 이보다 작은 이동은 '조금 밀기'가 아니라
  //   '얼마나 갈지 모르는 밀기'가 된다. 단계만 확정하는 쪽이 안전하다.
  if (mag < MIN_MOVE_COUNTS) {
    linearStop();
    lin_stop_count  = cur;
    lin_start_count = cur;
    lin_need        = 0;
    lin_tgt_level   = lv;
    lin_why         = "SKIP";
    finishLinAt(lv, now);
    return;
  }

  unsigned long ms;
  if (force_full_time) {
    ms = ESTOP_PUSH_MS;                 // E-stop 체결 : 부족한 것이 위험하다
  } else {
    ms = msForCounts(mag);
    if (err < 0) ms = ms * (unsigned long)REV_TIME_PCT / 100UL;   // 역방향 보정(미실측)
  }
  if (ms > MAX_DRIVE_MS) ms = MAX_DRIVE_MS;   // 최종 상한

  lin_target      = rel;                     // 보험 판정선 (영점 대비)
  lin_need        = err;
  lin_start_count = cur;
  lin_dir_sign    = (err > 0) ? +1 : -1;     // ★ 방향은 여기서 한 번만 정한다 ★
  lin_planned_ms  = ms;
  lin_tgt_level   = lv;
  lin_drive_t     = now;
  lin_state       = LIN_MOVE;

  linearDrive(lin_dir_sign > 0 ? LINEAR_FWD : LINEAR_REV, DRIVE_PWM);

  if (DEBUG_LINEAR) {
    Serial.print("# MOVE L");
    Serial.print(lv);
    Serial.print(" need=");
    Serial.print(err);
    Serial.print(" t=");
    Serial.print(ms);
    Serial.println("ms");
  }
}


// ================= 0단계 : 하드스톱까지 시간 구동 (열린루프 재영점) =================
// ★ 엔코더를 보지 않는다 ★ 목적이 '목표 카운트 맞추기'가 아니라 하드스톱까지 밀어 기준점을
//   되찾는 것이기 때문이다(증분 엔코더의 누적 드리프트 대책).
void startLinHome(unsigned long now) {
  lin_dir_sign    = -1;
  lin_target      = 0;             // 쓰지 않는다 (열린루프)
  lin_need        = 0;             // 0 = 진단 로그에서 want/got을 생략한다는 표시
  lin_start_count = encRead();
  lin_planned_ms  = HOME_DRIVE_MS;
  lin_tgt_level   = BRAKE_NONE;
  lin_drive_t     = now;
  lin_state       = LIN_HOME;

  linearDrive(LINEAR_REV, DRIVE_PWM);

  if (DEBUG_LINEAR) {
    Serial.print("# HOME ");
    Serial.print(HOME_DRIVE_MS);
    Serial.println("ms");
  }
}


// ================= 정지 -> 관성 정착 대기 =================
void enterLinSettle(unsigned long now, long at) {
  linearStop();
  lin_stop_count = at;
  lin_settle_t   = now;
  lin_state      = LIN_SETTLE;
}


// ================= 단계 확정 + 영점 리셋 =================
// ★ 이 함수를 지나야 '현재 단계 = lv, 영점 = 지금 위치'가 성립한다 ★
// 진단 로그의 want(움직여야 했던 양) / got(실제 움직인 양)이 표 보정의 근거다.
// got이 계통적으로 작으면 표의 시간이 짧은 것이고, by=ENC가 자주 나오면 표가 느린 것이다.
void finishLinAt(uint8_t lv, unsigned long now) {
  long settled = encRead();
  long coast   = settled - lin_stop_count;    // 끊은 뒤 관성으로 더 밀려간 양
  long got     = settled - lin_start_count;   // 이번 구동의 실제 이동량

  brake_level = lv;
  zeroHere();
  lin_state = LIN_IDLE;
  armIdleZero(now);

  if (DEBUG_LINEAR) {
    Serial.print("# DONE L");
    Serial.print(lv);
    Serial.print(" by=");
    Serial.print(lin_why);
    if (lin_need != 0) {
      Serial.print(" want=");
      Serial.print(lin_need);
      Serial.print(" got=");
      Serial.print(got);
    }
    Serial.print(" coast=");
    Serial.println(coast);
  }
}


// ================= [0804-3] 진행 중 리니어 구동 취소 (E-stop 진입 시) =================
// ★ 재영점하지 않는다 ★ 영점을 옮기지 않아야 엔코더가 '지금 얼마나 밟혀 있는지'를 그대로
//   담고 있고, 그래야 뒤이은 체결이 남은 거리를 정확히 본다. 단계(brake_level)도 그대로다.
void cancelLinear(unsigned long now) {
  linearStop();
  lin_state = LIN_IDLE;
  armIdleZero(now);
}


// ================= [0804-1] 리니어 상태 갱신 (★매 루프★ 호출) =================
// ★ 어떤 주기에도 걸지 않는다 ★ 주기를 두면 그만큼 정지가 늦어지고, PWM 255에서 그 지연은
//   곧 카운트 오차가 된다(약 1~1.8카운트/ms).
void updateLinear(unsigned long now) {
  switch (lin_state) {

    // 종료조건 3개 — 먼저 오는 것이 이긴다. 정상 동작에서는 항상 (3) TIME이 이긴다.
    case LIN_MOVE: {
      long cur = encRead();

      // (1) 최종 보험 : 너무 많이 밟혔다. ★밟는 방향(+) 구동에만 적용한다★
      //     놓는 방향에 걸면 이미 깊이 밟힌 상태에서 '해제'조차 막혀 브레이크가 풀리지
      //     않는다(절댓값으로 보면 그 사고가 난다). 해제 쪽 과주행은 기본위치 하드스톱이
      //     받아주고 시간 상한도 있어 위험하지 않다.
      if (lin_dir_sign > 0 && cur >= ENC_HARD_MAX) {
        lin_why = "HARDMAX";
        enterLinSettle(now, cur);
        return;
      }

      // (2) 보험 : 목표선에 먼저 닿았다. '그 방향으로 넘었는가'로 보므로 지나쳐도 잡힌다.
      bool reached = (lin_dir_sign > 0) ? (cur >= lin_target - POS_TOLERANCE)
                                       : (cur <= lin_target + POS_TOLERANCE);
      if (reached) {
        lin_why = "ENC";
        enterLinSettle(now, cur);
        return;
      }

      // (3) 주 종료조건 : 표에서 뽑은 시간이 지났다.
      //     ★타임아웃이 필요 없다★ lin_planned_ms 자체가 상한이라 반드시 끝난다.
      if (now - lin_drive_t >= lin_planned_ms) {
        lin_why = "TIME";
        enterLinSettle(now, cur);
      }
      return;
    }

    case LIN_HOME:
      if (now - lin_drive_t >= HOME_DRIVE_MS) {
        lin_why = "HOME";
        enterLinSettle(now, encRead());
      }
      return;

    case LIN_SETTLE:
      if (now - lin_settle_t >= LIN_SETTLE_MS) {
        finishLinAt(lin_tgt_level, now);
      }
      return;

    case LIN_IDLE:
    default:
      return;
  }
}


// ================= 대기 중 자동 재영점 : 감시 시작(재무장) =================
void armIdleZero(unsigned long now) {
  quiet_ref = encRead();
  quiet_t   = now;
}


// ================= 대기 중 자동 재영점 (LIN_IDLE에서만, 매 루프) =================
// 3초 동안 엔코더가 IDLE_QUIET_BAND 이내로 머물면 그 지점을 '지금 단계의 기준점'으로 잡는다.
//   ★ 단계(brake_level)는 건드리지 않는다 ★ 그래서 1단계에서 재영점된 뒤 2 명령이 오면
//     그 자리에서 +(POS_LEVEL2 - POS_LEVEL1)만큼만 간다.
//   구동/정착 중에는 감시하지 않는다 — 그 구간의 위치 변화는 일부러 만든 것이고, 영점은
//   finishLinAt이 잡는다.
void updateIdleZero(unsigned long now) {
  if (lin_state != LIN_IDLE) return;

  long cur = encRead();

  if (labs(cur - quiet_ref) > IDLE_QUIET_BAND) {   // 움직였다 -> 기준·타이머 재시작
    quiet_ref = cur;
    quiet_t   = now;
    return;
  }

  if (now - quiet_t < IDLE_ZERO_MS) return;

  quiet_t = now;   // 3초 확정. 타이머는 항상 재시작한다(다음 3초를 새로 센다)

  // 이미 기준점 위에 있으면 할 일이 없다(zeroHere를 부르면 enc_err 진단만 잃는다)
  if (labs(cur) <= IDLE_QUIET_BAND) {
    quiet_ref = cur;
    return;
  }

  zeroHere();
  quiet_ref = 0;

  if (DEBUG_LINEAR) {
    Serial.print("# REZERO ");
    Serial.print(cur);
    Serial.print(" -> L");
    Serial.println(brake_level);
  }
}


// ================= 브레이크 명령 처리 =================
// 구동/정착 중에는 아무것도 하지 않고, IDLE이 되는 순간 목표 단계로 전이한다.
// (0731과 같은 태도 — 송신측이 50ms마다 같은 값을 반복해도 구동을 끊지 않는다)
void updateBrake(unsigned long now) {
  if (lin_state != LIN_IDLE) return;
  if (lin_gate_t != 0) {                       // 반전 보호 게이트 통과 대기
    if ((long)(now - lin_gate_t) < 0) return;
    lin_gate_t = 0;
  }
  if (brake_level == brake_cmd_level) return;

  beginBrake(brake_cmd_level, now, false);
}


// ================= [0804-5] E-stop 핀체인지 인터럽트는 제거했다 =================
// 이전 판에는 ISR(PCINT0_vect) + setupEstopPcint()가 있었고, 개방(HIGH)을 본 즉시 조향
// PWM을 0으로(그리고 [0804-4]에 따라 해제 방향 리니어 구동만) 끊었다.
// 발동에 500ms 확인을 요구하는 순간 그 즉시성은 요구와 모순되고, 남겨두면 500ms 미만
// 노이즈에서 조향 PWM만 순간 끊겨 핸들이 덜컥거린다.
// 판정은 loop의 digitalRead 폴링(updateEstop)만으로 하고, 출력 차단·구동 취소는
// applyEstop() 이 담당한다.  되살리는 방법은 헤더 [0804-5] 마지막 문단 참고.


// ================= [0804-5] E-stop 판정 (발동·해제 모두 500ms 연속 유지) =================
// 현재 핀 레벨만 본다(ISR 없음). loop 는 어떤 주기에도 걸려 있지 않아(리니어 종료판정
// 때문에 매 루프 돈다) 폴링 해상도가 500ms 판정에 충분히 남는다.
//   - 발동 : ESTOP_TRIGGER_CONFIRM_MS 동안 계속 개방(HIGH)
//   - 해제 : ESTOP_RELEASE_CONFIRM_MS 동안 계속 단락(LOW)
//   - 반대 레벨이 한 번이라도 보이면 그쪽 타이머가 0으로 리셋된다 → '연속' 유지가 조건
// ★ 지금 상태에 필요한 타이머만 돌린다 ★ 반대쪽 타이머를 0으로 눕혀 두어, 상태가 바뀐
//   직후에 낡은 타이머 값이 남아 즉시 재전환되는 일이 없게 한다.
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_active = false;
    estop_high_t = 0;
    estop_low_t  = 0;
    return;
  }

  bool open_now = (digitalRead(ESTOP_PIN) == HIGH);   // NC 개방 = e-stop 요청

  if (!estop_active) {
    // ── 정상 상태 : 개방이 연속 유지되면 발동 ──
    estop_low_t = 0;
    if (!open_now) {
      estop_high_t = 0;                 // 단락을 봤다 → 발동 타이머 리셋
    } else if (estop_high_t == 0) {
      estop_high_t = now;               // 개방 관측 시작
    } else if (now - estop_high_t >= ESTOP_TRIGGER_CONFIRM_MS) {
      estop_active = true;              // ★ 500ms 연속 개방 확인 → 발동 ★
      estop_high_t = 0;
    }
  } else {
    // ── 발동 상태 : 단락이 연속 유지되면 해제 ──
    estop_high_t = 0;
    if (open_now) {
      estop_low_t = 0;                  // 개방을 다시 봤다 → 해제 타이머 리셋
    } else if (estop_low_t == 0) {
      estop_low_t = now;                // 단락 관측 시작
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_active = false;             // 500ms 연속 단락 확인 → 해제
      estop_low_t = 0;
    }
  }
}


// ================= e-stop 상태에서 매 루프 호출되는 안전 동작 =================
// ★ e-stop은 '브레이크 단계 2(풀브레이킹)'와 같은 작용이다 ★ 0731과 같은 태도지만,
//   체결 시간만은 표가 아니라 ESTOP_PUSH_MS를 쓴다([0804-3]) — 부족한 것이 위험하므로.
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;

  if (!estop_latched) {
    estop_latched = true;

    // ★ 진입 처리 ★ 진행 중이던 리니어 구동을 취소한다(재영점은 하지 않는다 —
    //   cancelLinear 주석 참고). 해제(REV) 구동 중이었다면 급반전을 피해 30ms 쉬고 체결한다.
    if (lin_state != LIN_IDLE) {
      bool was_rev = (lin_dir_sign < 0);
      cancelLinear(now);
      lin_gate_t = was_rev ? (now + REVERSE_DEADTIME_MS) : 0;
    }
    estop_push_started = false;
  }

  brake_cmd_level = BRAKE_FULL;   // 해제 후 단계 모델이 실제와 일치하도록 목표도 2로 둔다

  // 체결은 이번 e-stop에서 한 번만 시작한다(도달 후 재가압하지 않는다).
  // 이미 2단계에 있고 위치도 그 근처면 startLinMove가 SKIP으로 넘긴다.
  if (!estop_push_started && lin_state == LIN_IDLE) {
    if (lin_gate_t != 0) {
      if ((long)(now - lin_gate_t) < 0) return;   // 반전 보호 대기
      lin_gate_t = 0;
    }
    estop_push_started = true;
    beginBrake(BRAKE_FULL, now, true);            // ★force_full_time★
  }
}


// ================= 각도 <-> raw 변환 =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX);
}

int potToAngle(int raw) {
  int angle = map(raw, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}


// ================= [0727-1] 1단 필터 : 9샘플 중앙값 =================
int readPotMedian() {
  int s[POT_MEDIAN_N];
  for (uint8_t i = 0; i < POT_MEDIAN_N; i++) {
    int v = analogRead(DC_POT_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[POT_MEDIAN_N / 2];
}


// ================= [0727-1] 2단 필터 : 지수평활 (PD 입력 전용) =================
int smoothPot(int med) {
  if (steerAdcFiltered < 0) {
    steerAdcFiltered = med;
  } else {
    steerAdcFiltered += STEER_ADC_SMOOTH_ALPHA * ((float)med - steerAdcFiltered);
  }
  return (int)steerAdcFiltered;
}


// ================= [0730-2] 조향 힘빼기 (릴리즈) =================
void releaseSteer() {
  dcStop();
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
  target_pos = lastPotMedian;
  steer_angle_cmd = potToAngle(lastPotMedian);   // 디버그용: 스테일 명령각 대신 현재각
}


// ================= 조향 PD 제어 (CONTROL_WINDOW_MS 주기) =================
void updateSteer(unsigned long now) {
  if (now - steer_win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_win_t = now;

  int med = readPotMedian();
  lastPotMedian = med;              // 텔레메트리도 이 값을 재사용
  int cur = smoothPot(med);

  // ── 대기 상태: 가변저항 변화 무시, 정지 유지 ──
  if (steer_state == ST_SETTLED) {
    dcStop();
    prev_pos = cur;
    return;
  }

  // ── PD 제어 ──
  int err = target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - prev_pos) / dt);
  float output = p + d;
  prev_pos = cur;

  int absErr = abs(err);

  if (absErr <= STEER_TOLERANCE_ENTER) {
    dcStop();
    if (!settleTimerRunning) {
      settleTimerRunning = true;
      settleStart = now;
    } else if (now - settleStart >= SETTLE_MS) {
      steer_state = ST_SETTLED;   // 0.5초 이상 허용범위 유지 -> 도달 판정, 대기로 전환
    }
  } else if (absErr > STEER_TOLERANCE_EXIT) {
    settleTimerRunning = false;

    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    bool wantRawUp = (output > 0);   // 출력>0 -> dcCW -> raw 증가 방향

    // [0727-2] 하드 리밋 게이팅 : 파고드는 방향만 차단, 벗어나는 방향은 허용
    if (wantRawUp && med >= RAW_HI_LIMIT) {
      dcStop();
    } else if (!wantRawUp && med <= RAW_LO_LIMIT) {
      dcStop();
    } else if (wantRawUp) {
      dcCW(spd);
    } else {
      dcCCW(spd);
    }
  } else {
    // ENTER < absErr <= EXIT : 죽은 영역. 모터 정지, 타이머는 유지
    dcStop();
  }
}


// ================= 가변저항 환산 현재 조향각 (텔레메트리용) =================
int readSteerAngle() {
  return potToAngle(lastPotMedian);
}


// ================= [0730-1] 주행모드 스위치 판정 (디바운스) =================
void updateMode(unsigned long now) {
  uint8_t lv = digitalRead(MODE_PIN);

  if (lv != mode_pin_last) {
    mode_pin_last = lv;
    mode_change_t = now;          // 변화 관측 -> 안정화 타이머 시작(재시작)
  } else if (mode_change_t != 0 && now - mode_change_t >= MODE_CONFIRM_MS) {
    auto_mode = (lv == LOW);      // 안정화 완료 -> 확정
    mode_change_t = 0;
  }
}


// ================= 입력 형식 검사 (정수, 부호 허용) =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}


// ================= [0730-2] 조향 힘빼기 토큰 검사 =================
bool isReleaseToken(const char* s) {
  return s && (s[0] == 'x' || s[0] == 'X') && s[1] == '\0';
}


// ================= 입력 파서 (0731과 동일) =================
// "<조향각도>,<브레이크단계>" 콤마 구분 2개. 형식이 안 맞으면 무시.
//   조향각도 : 정수(-40~40) 또는 'x'/'X'(힘빼기)
//   브레이크단계 : 0(기본) / 1(약) / 2(풀). 범위 밖 값은 브레이크 필드만 무시한다 —
//     구 프로토콜(±255 PWM)로 보내는 송신측이 섞여 있어도 큰 값이 풀브레이킹으로
//     오해석되지 않게 하려는 것이다.
void handleLine(char* line) {
  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 || !isValidNumber(tok2)) return;

  bool release = isReleaseToken(tok1);
  if (!release && !isValidNumber(tok1)) return;

  int brake = atoi(tok2);

  // e-stop 중에는 구동 명령(조향/브레이크) 미적용 → 리니어 재구동 방지
  if (estop_active) return;

  if (release) {
    releaseSteer();
  } else {
    steer_angle_cmd = constrain(atoi(tok1), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    target_pos = angleToPot(steer_angle_cmd);
    settleTimerRunning = false;
    steer_state = ST_ACTIVE;                    // 새 각도 입력 -> PD 제어 재개
  }

  if (brake >= BRAKE_NONE && brake <= BRAKE_LEVEL_MAX) {
    brake_cmd_level = (uint8_t)brake;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}


// ================= 출력 (0731과 동일) =================
// "P,<조향각환산값>,<모드>" (평상시) / "STOP" (e-stop 중)
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("P,");
  Serial.print(readSteerAngle());
  Serial.print(',');
  Serial.println(auto_mode ? 1 : 0);
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linear_dir_now = LINEAR_FWD;
  linearStop();

  // [0804-2] 엔코더 : D2/D3 CHANGE, INT4/INT5 벡터 직결. 켠 순간의 위치가 영점(0단계).
  setupEncoder();

  // [0804-5] E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  //   폴링만 쓴다(PCINT 제거). 타이머는 0에서 시작하므로 이미 개방된 채로 켜져도 첫
  //   루프부터 발동 타이머가 돌아 500ms 뒤에 발동한다. 부팅 직후에는 모터가 정지
  //   상태이고 시리얼 명령도 아직 없어 그 500ms 는 위험하지 않다.
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_active = false;
  estop_high_t = 0;
  estop_low_t  = 0;

  // [0730-1] 주행모드 스위치 (단순 ON/OFF, 내부 풀업만 사용)
  pinMode(MODE_PIN, INPUT_PULLUP);
  mode_pin_last = digitalRead(MODE_PIN);
  auto_mode     = (mode_pin_last == LOW);
  mode_change_t = 0;

  unsigned long now = millis();
  tele_t = now;

  // 리니어 : 부팅 위치를 0단계 기준점으로 두고 대기 상태에서 시작
  brake_level     = BRAKE_NONE;
  brake_cmd_level = BRAKE_NONE;
  lin_tgt_level   = BRAKE_NONE;
  lin_state       = LIN_IDLE;
  lin_gate_t      = 0;
  armIdleZero(now);

  // 조향 PD: 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;

  // ★ 여기서 안내를 출력하지 않는다 ★ 출력 양식("P,..."/"STOP")을 그대로 지키기 위함.
  //   리니어 진단이 필요하면 DEBUG_LINEAR를 true로 두면 '#' 줄이 추가로 나간다.
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // ★ 최우선 : 진행 중인 리니어 구동의 종료 판정(시간·보험)과 정착 처리 ★
  //   e-stop 여부와 무관하게 매 루프 돌린다. 여기서 상태를 정리해 두어야 아래 applyEstop
  //   /updateBrake가 올바른 단계·영점 위에서 판단한다.
  updateLinear(now);

  // ★ [0804-5] E-stop 판정 : 발동·해제 모두 500ms 연속 유지 확인 (대칭) ★
  //   ※ pollSerial보다 앞에 둔다 — handleLine이 e-stop 중 명령을 걸러내므로, 발동 직후
  //     들어온 줄이 한 번 적용되는 틈을 없애기 위함이다.
  updateEstop(now);

  pollSerial();

  // [0730-1] 주행모드 스위치는 e-stop 여부와 무관하게 항상 갱신 (보고 전용이므로 안전)
  updateMode(now);

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      estop_latched = false;
      estop_push_started = false;

      // ★[0804-3] 해제는 리니어 '0'과 같은 작용이다 ★ 송신측이 0을 보내주기를 기다리지
      //   않고 스스로 기본위치로 복귀한다. brake_cmd_level만 0으로 두면 아래 updateBrake가
      //   HOME(역방향 500ms)을 시작하고, 그 과정에서 하드스톱 기준점까지 회복된다.
      //   ※ 이 줄은 pollSerial 뒤에 있으므로 같은 루프에서 명령에 덮이지 않는다. 다음 줄부터
      //     송신측이 다시 1/2를 보내도 HOME이 끝난 뒤에 반영된다(구동 중 명령은 대기).
      if (lin_state != LIN_IDLE) {
        // e-stop 체결(FWD)이 아직 진행 중이었다면 급반전을 피해 30ms 쉬고 복귀한다
        bool was_fwd = (lin_dir_sign > 0);
        cancelLinear(now);
        lin_gate_t = was_fwd ? (now + REVERSE_DEADTIME_MS) : 0;
      }
      brake_cmd_level = BRAKE_NONE;

      // 조향 PD 목표를 해제 시점의 현재 위치(필터 재초기화 포함)로 재동기화
      int cur = readPotMedian();
      lastPotMedian    = cur;
      steerAdcFiltered = cur;
      target_pos = cur;
      prev_pos = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
    }
    updateBrake(now);
    updateSteer(now);
  }

  // 대기 중 3초 정지 -> 그 지점을 지금 단계의 기준점으로 (드리프트 흡수)
  updateIdleZero(now);

  sendOutput(now);
}
