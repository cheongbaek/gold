#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tts.py ― domichat 채팅방을 읽어 주는 TTS [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde tts

domichat 서버의 채팅방 두 개를 구독하고, **거기에 올라오는 모든 대화를 그대로**
시스템 기본 스피커로 읽는다. 사람이 문장을 입력하는 창(GUI)은 없다 —
★말은 휴대폰·PC 의 domichat 에서 넣고, 이 노드는 듣고 읽기만 한다★.

  ┌ 방 이름 ─ PC 이름(hostname)에서 만들어진다 ─────────────────────────────┐
  │   <PC명>_TTS_M   → ★남성★ 목소리로 읽는다                               │
  │   <PC명>_TTS_W   → ★여성★ 목소리로 읽는다                               │
  │   예) hostname 이 mad1 이면 mad1_TTS_M / mad1_TTS_W                      │
  └──────────────────────────────────────────────────────────────────────────┘

  두 방 모두 **비밀번호 제한방**(domichat 의 `pw` 유형)이고, 비밀번호는
  ★PC 이름 그대로★ 다(mad1 이면 `mad1`). 없으면 만들고, 이미 있으면 그냥 들어간다.

════════════════════════════════════════════════════════════════════════════════
 고칠 곳은 아래 [1] 파라미터 절 하나다
════════════════════════════════════════════════════════════════════════════════
  서버 주소·계정은 파일 상단 상수가 ★단일 소유자★ 다. 소스를 고치지 않고 한 번만
  다르게 띄우고 싶으면 같은 이름의 환경변수로 덮을 수 있다(NXDE_TTS_*).

════════════════════════════════════════════════════════════════════════════════
 알아 둘 것
════════════════════════════════════════════════════════════════════════════════
  · ★인터넷이 필요하다★ 합성은 edge-tts(Microsoft Edge 온라인 음성합성)가 한다.
    망이 없으면 합성만 실패하고 노드는 계속 돈다(끊긴 채팅은 재접속으로 복구된다).
  · ROS 노드가 아니다. `ros2 run` 으로 띄우기만 할 뿐 토픽을 주고받지 않는다
    (kill.py 와 같은 부류의 도구다 — rclpy 를 import 하지 않는다).
  · ★다른 노드의 음원과 겹쳐 나온다★ 재생을 pygame.mixer 로 하지 않고
    ★sound.py 와 똑같이 외부 재생기(ffplay/mpg123/mpv/cvlc) 프로세스★ 로 띄운다.
    mixer 는 오디오 장치를 붙들고 있어 nxde/sound 가 안내 음성을 낼 때 서로
    밀어낸다 — 프로세스로 띄우면 PulseAudio/PipeWire 가 섞어 주므로 ★안내 음성과
    TTS 가 동시에 들린다★. 끊고 싶을 때 kill 한 방이면 되는 것도 이 방식의 이득이다.
  · ★읽던 것을 끊고 새 것을 읽는다★ 채팅이 빠르게 올라오면 앞의 것을 다 읽기 전에
    새 것이 온다. 그때는 읽던 것을 그 자리에서 끊고 ★가장 최근 채팅★ 을 읽는다.
    큐에 쌓아 두지 않는다 — 쌓으면 한참 전 대화를 뒤늦게 읽게 되어 쓸모가 없다.
  · 여러 PC 가 같은 계정으로 동시에 붙는 경우는 상정하지 않는다(domichat 서버가
    같은 ID 동시 접속을 막는다 — 그쪽이 `already_online` 을 돌려준다).
  · 필요한 것 : pip install --user edge-tts   +   ffplay(ffmpeg) 나 mpg123
    ※ ★pygame 은 더 이상 필요 없다★ (위 '겹쳐 나온다' 항목 참고)

════════════════════════════════════════════════════════════════════════════════
 전송 계층은 domichat.py 에서 옮겨 온 것이다 (cheongbaek/domiman)
════════════════════════════════════════════════════════════════════════════════
  프레임 규격 `[길이 4B][종류 1B][본문]`, 종류 `'T'` = UTF-8 JSON. 그 저장소의
  domichat.md 가 정본이고, 여기서는 **이 노드가 쓰는 것만** 옮겼다(이미지 전송,
  회원가입, 승인/블랙리스트, 로컬 대화 기록은 전부 뺐다).

  ★옮겨 오면서 반드시 지킨 것 둘★ — domichat.md 가 실측으로 못 박은 함정이다.
    ① 접속 직후 소켓 타임아웃을 READ_TIMEOUT 으로 ★다시★ 설정한다.
       create_connection(timeout=) 이 준 값이 연결 뒤에도 남아 있어서, 그대로 두면
       대화가 없는 동안 recv 가 예외를 던져 ★몇 초마다 끊고 재접속하는 고리★ 에
       빠진다. 서버가 15초마다 ping 을 보내므로 60초 침묵은 진짜 죽은 연결이다.
    ② TLS 는 ★1.2 로 고정★ 하고 재협상을 막는다. 이 소켓은 수신 스레드가 읽고
       종료 처리(logout)가 쓰므로 한 소켓을 두 스레드가 만진다. TLS 1.3 은
       핸드셰이크 뒤에도 세션 티켓·KeyUpdate 가 오가 읽기 경로가 쓰기 상태를
       건드려 record layer 가 깨진다(domiserver 쪽도 같은 이유로 1.2 고정).

════════════════════════════════════════════════════════════════════════════════
 ★[이전 판] 대화형 GUI 는 없어졌다★
════════════════════════════════════════════════════════════════════════════════
  종전 이 파일은 tkinter 창에 문장을 입력해 귀로 확인하는 도구였고,
  white1/sound/*.mp3 안내 음성을 만들 때 쓰였다. 그 창은 이 판에서 제거됐다 —
  안내 음성 mp3 를 새로 만들 일이 생기면 edge-tts 를 직접 쓰면 된다:

      edge-tts --voice ko-KR-InJoonNeural --text "읽을 문장" --write-media out.mp3
"""

import asyncio
import hashlib
import io
import json
import os
import queue
import shutil
import signal
import socket
import ssl
import struct
import subprocess
import sys
import tempfile
import threading
import time

try:
    import edge_tts
except ImportError as exc:
    raise SystemExit("edge-tts 가 없다 — pip install --user edge-tts") from exc


# ══════════════════════════════════════════════════════════════════════════════
# === [1. 파라미터 — ★고칠 곳은 여기뿐이다★] ===
# ══════════════════════════════════════════════════════════════════════════════
# 같은 이름의 환경변수(NXDE_TTS_…)가 있으면 그쪽이 이긴다. 소스를 건드리지 않고
# 한 번만 다르게 띄울 때 쓴다.  예)  NXDE_TTS_IP=192.168.0.5 ros2 run nxde tts

SERVER_IP   = '211.196.44.3'      # domichat 서버(domiserver)
SERVER_PORT = 47821               # domichat 기본 포트
USER_ID     = 'ros2'              # 이 노드가 쓸 계정 (서버에서 승인돼 있어야 한다)
USER_PW     = 'ros2'

# 방 이름의 앞부분. 비워 두면 ★이 PC 의 hostname★ 을 쓴다(권장).
PC_NAME     = ''
# 방 비밀번호. 비워 두면 ★PC 이름 그대로★ 를 쓴다(mad1 → 'mad1').
ROOM_PW     = ''

ROOM_SUFFIX_MALE   = '_TTS_M'     # 이 방 대화는 남성 목소리로 읽는다
ROOM_SUFFIX_FEMALE = '_TTS_W'     # 이 방 대화는 여성 목소리로 읽는다

VOICE_MALE   = 'ko-KR-InJoonNeural'
VOICE_FEMALE = 'ko-KR-SunHiNeural'

RATE   = '+0%'                    # 말 빠르기 / 크기 / 높이 (edge-tts 규격)
VOLUME = '+100%'                  # ★최대★ [2026-09-16] edge-tts 규격 상한(−100~+100%)
PITCH  = '+0Hz'

# ★[2026-09-16] 재생 단계에서 한 번 더 키운다★
#   edge-tts 의 VOLUME 은 ★합성 음성 자체의 크기★ 라 +100% 라도 한계가 있다
#   (SSML prosody volume — 원본을 넘어서 증폭하지는 않는다). 차 안에서 주행 소음
#   위로 들리게 하려면 재생기에서 실제로 증폭해야 한다.
#   ★리미터를 함께 건다★ 단순 증폭만 하면 큰 음절에서 파형이 잘려(클리핑) 찌그러진
#   소리가 난다 — 말이 커지는 게 아니라 알아듣기 어려워진다. ffplay 는 alimiter
#   (lookahead limiter)로 천장을 눌러 주므로 배수를 올려도 소리가 깨지지 않는다.
#   ⚠️ 그래도 찌그러져 들리면 이 값을 내린다(2.0 → 1.5). 1.0 이면 증폭하지 않는다.
PLAY_GAIN = 3.0
PLAY_LIMIT = 0.97                 # 리미터 천장(0~1). 1.0 에 가까울수록 크고 위험하다

# TLS. 서버가 평문이면 자동으로 평문으로 다시 붙는다(전환기 대응).
USE_TLS = True
# 서버 인증서 지문을 첫 접속에 기억해 고정한다(SSH 와 같은 방식 = TOFU).
# ★지문이 바뀌면 접속하지 않는다★ — 중간자와 서버 재설치를 구별할 수단이 없으므로
#   '모르면 낮게 본다'. 서버를 정말 다시 세웠다면 아래 파일을 지우고 다시 띄운다.
PIN_CERT = True
PIN_PATH = os.path.expanduser('~/.nxde_tts_server_fp.json')


def _env(name, default):
    v = os.environ.get('NXDE_TTS_' + name, '').strip()
    return v if v else default


SERVER_IP   = _env('IP', SERVER_IP)
SERVER_PORT = int(_env('PORT', str(SERVER_PORT)))
USER_ID     = _env('ID', USER_ID)
USER_PW     = _env('PW', USER_PW)
PC_NAME     = _env('PC', PC_NAME)
ROOM_PW     = _env('ROOM_PW', ROOM_PW)


# ── domichat 프레임 규격 (domiserver 와 같은 값. 임의로 바꾸지 말 것) ──────────
FRAME_HEAD = struct.Struct('>IB')
MAX_FRAME = 1024 * 1024
CONNECT_TIMEOUT = 6.0
# 접속 후 읽기 타임아웃. 서버 ping 이 15초 주기이므로 이만큼 조용하면 죽은 연결이다.
# ★접속 타임아웃을 그대로 두면 안 된다★ — 헤더 '옮겨 오면서 지킨 것 ①' 참고.
READ_TIMEOUT = 60.0
RECONNECT_BACKOFF = (1, 2, 5, 10, 30)     # 재연결 대기[s] — 마지막 값으로 고정 반복
ROOM_NAME_MAX = 30                        # domiserver 의 같은 이름 상수와 맞춘 값
ROOM_PW_MAX = 19

def _gain_args(name):
    """재생기별 증폭 인자. ★PLAY_GAIN 한 곳만 고치면 전부 따라온다★ [2026-09-16]

    네 재생기가 각자 다른 단위를 쓴다 — 여기서 한 번 환산해 두지 않으면 재생기가
    바뀔 때마다 음량이 조용히 달라진다(있는 것을 위에서부터 고르는 구조라, 어느
    것이 걸릴지는 기계마다 다르다).
    """
    g = max(1.0, float(PLAY_GAIN))
    if g <= 1.0:
        return []
    if name == 'ffplay':
        #  ★증폭 + 리미터★ 클리핑 없이 키우는 유일한 조합이다(상수 주석 참고).
        return ['-af', 'volume=%.2f,alimiter=limit=%.2f' % (g, PLAY_LIMIT)]
    if name == 'mpg123':
        #  -f 는 ★출력 스케일★ 이고 32768 이 1배다(그래서 배수 × 32768).
        return ['-f', str(int(32768 * g))]
    if name == 'mpv':
        #  --volume 은 % 이고 기본 상한이 130 이라 --volume-max 를 함께 올려야 한다.
        return ['--volume-max=%d' % int(g * 100), '--volume=%d' % int(g * 100)]
    if name == 'cvlc':
        return ['--gain', '%.2f' % g]
    return []


# 있는 것을 위에서부터 고른다. 전부 '창 없이 한 번 재생하고 끝' (sound.py 와 같은 표).
PLAYERS = tuple(
    (name, args + _gain_args(name)) for name, args in (
        ('ffplay', ['-nodisp', '-autoexit', '-loglevel', 'quiet']),
        ('mpg123', ['-q']),
        ('mpv',    ['--no-video', '--really-quiet']),
        ('cvlc',   ['--intf', 'dummy', '--play-and-exit']),
    )
)


def log(msg):
    """한 줄 로그. 런치 로그에 섞여도 어느 노드인지 보이게 접두를 붙인다."""
    print('[tts %s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


# ══════════════════════════════════════════════════════════════════════════════
# === [2. 프레임 · 소켓 클라이언트 (domichat.py 에서 옮겨 옴)] ===
# ══════════════════════════════════════════════════════════════════════════════

def _local_addrs():
    """이 PC 가 가진 주소들. 서버와 같은 PC 에서 '자기 공인 IP' 로 붙을 때 쓴다."""
    addrs = set(['127.0.0.1', 'localhost', '::1'])
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None):
            addrs.add(info[4][0])
    except Exception:
        pass
    return addrs


def connect_any(ip, port, timeout=CONNECT_TIMEOUT):
    """접속. 실패했는데 그 주소가 **이 PC 자신의 주소**면 127.0.0.1 로 한 번 더
    시도한다 — 공유기가 자기 공인 IP 로의 되돌림(헤어핀)을 막아도 붙는다."""
    try:
        return socket.create_connection((ip, port), timeout)
    except OSError:
        if ip in _local_addrs():
            return socket.create_connection(('127.0.0.1', port), timeout)
        raise


class CertChanged(Exception):
    """고정해 둔 서버 인증서 지문과 다르다 — 중간자이거나 서버를 재설치한 것이다."""

    def __init__(self, host, old, new):
        Exception.__init__(self, '%s: %s… → %s…' % (host, old[:16], new[:16]))
        self.host, self.old, self.new = host, old, new


def _tls_context():
    """자체 서명 인증서라 체인·호스트명 검증은 끄고 ★지문 고정★ 으로 신뢰한다.
    검증을 끈다고 암호화가 약해지지는 않으며, 중간자 방어는 지문 비교가 맡는다.
    ★1.2 로 고정 + 재협상 금지★ 이유는 헤더 '옮겨 오면서 지킨 것 ②' 참고."""
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.maximum_version = ssl.TLSVersion.TLSv1_2
    no_reneg = getattr(ssl, 'OP_NO_RENEGOTIATION', 0)
    if no_reneg:
        ctx.options |= no_reneg
    return ctx


def connect_secure(ip, port, want_tls, pinned):
    """(소켓, 지문|None). 지문이 고정값과 다르면 CertChanged.
    서버가 TLS 를 안 쓰면 평문으로 다시 붙는다."""
    raw = connect_any(ip, port)
    if not want_tls:
        return raw, None
    try:
        sock = _tls_context().wrap_socket(raw)
        fp = hashlib.sha256(sock.getpeercert(binary_form=True)).hexdigest()
    except (ssl.SSLError, OSError) as e:
        try:
            raw.close()
        except Exception:
            pass
        log('[TLS] 서버가 TLS 를 쓰지 않는 것 같다(%s) — 평문으로 접속한다' % e)
        return connect_any(ip, port), None
    if pinned and pinned != fp:
        try:
            sock.close()
        except Exception:
            pass
        raise CertChanged('%s:%s' % (ip, port), pinned, fp)
    return sock, fp


def load_pin(host):
    if not PIN_CERT:
        return None
    try:
        with open(PIN_PATH, 'r') as f:
            return (json.load(f) or {}).get(host)
    except Exception:
        return None


def save_pin(host, fp):
    if not PIN_CERT:
        return
    data = {}
    try:
        with open(PIN_PATH, 'r') as f:
            data = json.load(f) or {}
    except Exception:
        pass
    data[host] = fp
    try:
        with open(PIN_PATH, 'w') as f:
            json.dump(data, f)
    except Exception as e:
        log('지문을 저장하지 못했다(%s) — 다음 접속에 다시 기억한다' % e)


class ChatClient:
    """domichat 세션 하나. 접속을 끝까지 유지하고 끊기면 스스로 다시 붙는다.

    ★domichat.py 보다 훨씬 작다★ — 이 노드는 ★받기만★ 하고(로그인·입장·구독·pong
    말고는 보낼 것이 없다) 이미지도 다루지 않는다. 그래서 송신 큐·송신 스레드를
    두지 않고 잠금 하나로 직접 보낸다.

    받은 프레임은 전부 self.q 에 그대로 넣는다. 클라이언트 내부 사건은 '_ev' 키로
    구분한다(connected / disconnected / connect_fail / cert_changed).
    """

    def __init__(self, ip, port, uid, pw):
        self.q = queue.Queue()
        self.ip, self.port, self.uid, self.pw = ip, port, uid, pw
        self.sock = None
        self.want = False
        self.pinned = load_pin('%s:%s' % (ip, port))
        self._send_lock = threading.Lock()
        self._thread = None

    # ── 저수준 ────────────────────────────────────────────────────────────────
    def _raw_send(self, sock, obj):
        data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        with self._send_lock:
            sock.sendall(FRAME_HEAD.pack(len(data), ord('T')) + data)

    @staticmethod
    def _recv_exact(sock, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def _recv_obj(self, sock):
        head = self._recv_exact(sock, FRAME_HEAD.size)
        if head is None:
            return None
        ln, typ = FRAME_HEAD.unpack(head)
        if ln > MAX_FRAME:
            raise OSError('프레임 과대')
        body = self._recv_exact(sock, ln) if ln else b''
        if body is None:
            return None
        if chr(typ) != 'T':
            return {}                      # 'B'(이미지 청크) — 이 노드는 쓰지 않는다
        return json.loads(body.decode('utf-8'))

    # ── 바깥에서 부르는 것 ────────────────────────────────────────────────────
    def start(self):
        self.want = True
        self._thread = threading.Thread(target=self._session_loop, daemon=True)
        self._thread.start()

    def send(self, obj):
        """지금 붙어 있으면 보낸다. 끊긴 동안 보낸 것은 ★버린다★ — 어차피 재접속
        하면 로그인·입장·구독을 처음부터 다시 하므로 다시 보낼 것이 없다."""
        sock = self.sock
        if sock is None:
            return False
        try:
            self._raw_send(sock, obj)
            return True
        except Exception:
            return False

    def stop(self):
        self.want = False
        sock = self.sock
        if sock is not None:
            try:
                self._raw_send(sock, {'t': 'logout'})
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass
        self.sock = None

    # ── 세션 ──────────────────────────────────────────────────────────────────
    def _session_loop(self):
        idx = 0
        first = True
        while self.want:
            try:
                sock, fp = connect_secure(self.ip, self.port, USE_TLS, self.pinned)
                if fp and not self.pinned:
                    self.pinned = fp        # 첫 접속 — 이 지문을 기억해 고정한다
                    save_pin('%s:%s' % (self.ip, self.port), fp)
                    log('서버 인증서 지문을 기억했다: %s…' % fp[:16])
            except CertChanged as e:
                self.want = False
                self.q.put({'_ev': 'cert_changed', 'host': e.host,
                            'old': e.old, 'new': e.new})
                return
            except OSError as e:
                if first:
                    self.q.put({'_ev': 'connect_fail', 'msg': str(e)})
                else:
                    self.q.put({'_ev': 'disconnected', 'msg': str(e)})
                first = False
                time.sleep(RECONNECT_BACKOFF[min(idx, len(RECONNECT_BACKOFF) - 1)])
                idx += 1
                continue

            # ★접속 타임아웃을 읽기 타임아웃으로 바꾼다★ (헤더 '지킨 것 ①')
            sock.settimeout(READ_TIMEOUT)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except OSError:
                pass
            self.sock = sock
            first = False
            self.q.put({'_ev': 'connected'})
            logged = False
            try:
                self._raw_send(sock, {'t': 'login', 'id': self.uid, 'pw': self.pw})
                while self.want:
                    d = self._recv_obj(sock)
                    if d is None:
                        break
                    t = d.get('t')
                    if t == 'ping':
                        self._raw_send(sock, {'t': 'pong'})
                        continue
                    if t == 'welcome':
                        logged = True
                    self.q.put(d)
            except Exception:
                pass                       # 끊김 알림은 아래에서 한 번만 낸다
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                self.sock = None

            if not self.want:
                break
            # 한 번이라도 로그인에 성공했으면 백오프를 되돌린다 — 안 그러면 일시적인
            # 끊김이 몇 번 겹친 뒤부터 계속 30초씩 기다리게 된다(domichat.md 와 같다).
            if logged:
                idx = 0
            self.q.put({'_ev': 'disconnected', 'msg': '연결이 끊겼다'})
            time.sleep(RECONNECT_BACKOFF[min(idx, len(RECONNECT_BACKOFF) - 1)])
            idx += 1


# ══════════════════════════════════════════════════════════════════════════════
# === [3. 합성 · 재생] ===
# ══════════════════════════════════════════════════════════════════════════════

def _find_player():
    for name, args in PLAYERS:
        path = shutil.which(name)
        if path:
            return [path] + args
    return None


class Speaker:
    """합성·재생 전담 스레드. ★가장 최근 채팅 하나만★ 들고 있는다.

    ★큐를 쓰지 않는 것이 이 클래스의 설계 전부다★ — 채팅이 빠르게 올라오면 읽던
    것을 끊고 새 것을 읽으라는 요구이므로, 대기열이 있으면 안 된다. say() 가
    올 때마다 세대(gen)를 올리고, 워커는 ★자기 세대가 아직 최신일 때만★ 계속한다:

      · 합성 중  — 스트림을 받는 도중에 세대가 바뀌면 그 자리에서 버린다
                   (망 왕복을 끝까지 기다렸다가 버리면 다음 말이 그만큼 늦다)
      · 재생 중  — say() 가 재생기 프로세스를 곧바로 kill 한다(끊김이 즉시다)

    ★오디오 장치를 붙들지 않는다★ 재생은 외부 재생기 프로세스라, nxde/sound 의
    안내 음성과 ★겹쳐서★ 들린다(헤더 '알아 둘 것' 참고).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._gen = 0
        self._next = None                  # (문장, 목소리, 세대) — 최신 것 하나
        self._wake = threading.Event()
        self._quit = threading.Event()
        self._proc = None
        self._cmd = _find_player()
        self._dir = tempfile.mkdtemp(prefix='nxde_tts_')
        self._thread = threading.Thread(target=self._run, daemon=True)
        if self._cmd is None:
            log('재생기를 찾지 못했다(ffplay/mpg123/mpv/cvlc) — 소리 없이 돈다')
        else:
            #  ★어느 재생기로 얼마나 키워 트는지 한 줄 남긴다★ 음량이 기대와 다를 때
            #  제일 먼저 볼 곳이 여기다(재생기마다 증폭 단위가 다르다 — _gain_args).
            log('재생기 %s · 합성음량 %s · 재생증폭 x%.1f'
                % (os.path.basename(self._cmd[0]), VOLUME, PLAY_GAIN))

    def start(self):
        self._thread.start()

    def say(self, text, voice):
        with self._lock:
            self._gen += 1
            self._next = (text, voice, self._gen)
        self._kill()                       # ★읽던 것을 그 자리에서 끊는다★
        self._wake.set()

    def shutdown(self, timeout=1.5):
        self._quit.set()
        self._kill()
        self._wake.set()
        self._thread.join(timeout)
        shutil.rmtree(self._dir, ignore_errors=True)

    # ── 내부 ──────────────────────────────────────────────────────────────────
    def _is_current(self, gen):
        with self._lock:
            return gen == self._gen and not self._quit.is_set()

    def _kill(self):
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
            except Exception:
                pass

    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            while not self._quit.is_set():
                if not self._wake.wait(0.2):
                    continue
                self._wake.clear()
                while not self._quit.is_set():
                    with self._lock:
                        item = self._next
                        self._next = None
                    if item is None:
                        break
                    text, voice, gen = item
                    try:
                        data = loop.run_until_complete(
                            self._synth(text, voice, gen))
                    except Exception as e:
                        # 대부분 망이 없거나 목소리 이름이 틀린 경우다. 노드는 계속 돈다.
                        log('[합성 실패] %s' % e)
                        continue
                    if not data or not self._is_current(gen):
                        continue           # 그 사이 새 채팅이 왔다 — 이것은 버린다
                    self._play(data, gen)
        finally:
            loop.close()

    async def _synth(self, text, voice, gen):
        """문장 → mp3 바이트. 파일로 떨어뜨리지 않고 메모리에서 끝낸다.
        받는 도중에도 세대를 확인해, 밀렸으면 남은 것을 기다리지 않고 접는다."""
        buf = io.BytesIO()
        comm = edge_tts.Communicate(text=text, voice=voice,
                                    rate=RATE, volume=VOLUME, pitch=PITCH)
        async for chunk in comm.stream():
            if not self._is_current(gen):
                return None
            if chunk['type'] == 'audio':
                buf.write(chunk['data'])
        return buf.getvalue()

    def _play(self, data, gen):
        if self._cmd is None:
            return
        path = os.path.join(self._dir, 'say_%d.mp3' % gen)
        try:
            with open(path, 'wb') as f:
                f.write(data)
            proc = subprocess.Popen(self._cmd + [path],
                                    stdin=subprocess.DEVNULL,
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
        except Exception as e:             # 재생 실패로 노드를 죽이지 않는다
            log('[재생 실패] %s' % e)
            return
        with self._lock:
            self._proc = proc
        # say() 가 kill 하므로 이 기다림은 보통 그냥 끝난다. 세대 확인은 kill 과
        # 새 프로세스 등록 사이의 틈을 막는 두 번째 그물이다.
        while proc.poll() is None and not self._quit.is_set():
            if not self._is_current(gen):
                try:
                    proc.kill()
                except Exception:
                    pass
                break
            time.sleep(0.05)
        with self._lock:
            if self._proc is proc:
                self._proc = None
        try:
            os.remove(path)
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
# === [4. 본체 — 방 두 개를 붙잡고 올라오는 말을 읽는다] ===
# ══════════════════════════════════════════════════════════════════════════════

class TtsBridge:
    """welcome → (없으면 만들고) 입장 → 구독 → msg 를 읽는다.

    ★방 준비는 한 번에 하나씩★ 한다(_setup_next). domiserver 의 error 프레임은
    '어느 요청에 대한 것인지'를 담지 않으므로(room_create 의 room_name_taken 에
    ref 가 없다), 두 방을 동시에 요청하면 그 오류가 어느 방 것인지 알 수 없다.
    하나씩 보내면 ★지금 기다리는 방★ 이 늘 하나라 그 문제가 생기지 않는다.
    """

    def __init__(self, pc_name, room_pw):
        self.pc = pc_name
        self.room_pw = room_pw
        self.room_m = pc_name + ROOM_SUFFIX_MALE
        self.room_w = pc_name + ROOM_SUFFIX_FEMALE
        self.voice_of = {self.room_m: VOICE_MALE, self.room_w: VOICE_FEMALE}
        self.label_of = {self.room_m: '남성', self.room_w: '여성'}
        self.client = ChatClient(SERVER_IP, SERVER_PORT, USER_ID, USER_PW)
        self.speaker = Speaker()
        self._known = set()                # welcome 이 알려 준 '서버에 있는 방'
        self._todo = []                    # 아직 입장하지 못한 방
        self._wait = None                  # 지금 만들거나 들어가는 중인 방
        self._joined = set()
        self._stop = threading.Event()

    def request_stop(self):
        """바깥(시그널 핸들러)에서 부르는 종료 요청. 여기서는 깃발만 세우고
        실제 정리는 run() 이 빠져나온 뒤 shutdown() 이 한다 — 시그널 핸들러 안에서
        소켓·프로세스를 만지면 그 중간 상태로 굳을 수 있다."""
        self._stop.set()

    # ── 수명 ──────────────────────────────────────────────────────────────────
    def run(self):
        log('서버 %s:%d / 계정 %s' % (SERVER_IP, SERVER_PORT, USER_ID))
        log('방 %s → %s 목소리' % (self.room_m, self.label_of[self.room_m]))
        log('방 %s → %s 목소리' % (self.room_w, self.label_of[self.room_w]))
        log('방 비밀번호 = PC 이름 (%s)' % self.room_pw)
        self.speaker.start()
        self.client.start()
        while not self._stop.is_set():
            try:
                d = self.client.q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._on_frame(d)
            except Exception as e:         # 프레임 하나 때문에 노드를 죽이지 않는다
                log('[처리 실패] %s — %s' % (e, d.get('t') or d.get('_ev')))

    def shutdown(self):
        self._stop.set()
        self.client.stop()
        self.speaker.shutdown()

    # ── 프레임 ────────────────────────────────────────────────────────────────
    def _on_frame(self, d):
        ev = d.get('_ev')
        if ev:
            return self._on_event(ev, d)
        t = d.get('t')

        if t == 'welcome':
            self._known = set(r.get('name') for r in (d.get('rooms') or []))
            self._joined = set()
            self._todo = [self.room_m, self.room_w]
            self._wait = None
            log('로그인 성공 — 방 %d 개가 서버에 있다' % len(self._known))
            self._setup_next()

        elif t == 'ok':
            of = d.get('of')
            if of == 'room_create':
                # ★서버가 만든 사람을 곧바로 입장시킨다★(do_join) — 여기서 join 을
                #   또 보내지 않는다. 곧 joined 가 온다.
                log("채팅방 '%s' 을 새로 만들었다" % d.get('room'))
            elif of == 'sub':
                log("채팅방 '%s' 구독 %s" % (d.get('room'),
                                            '켬' if d.get('on') else '끔'))

        elif t == 'joined':
            room = d.get('room')
            self._joined.add(room)
            log("채팅방 '%s' 입장 (%s 목소리)" %
                (room, self.label_of.get(room, '?')))
            self.client.send({'t': 'sub', 'room': room, 'on': True})
            if room == self._wait:
                self._wait = None
                self._setup_next()

        elif t == 'denied':
            room = d.get('room')
            # 비밀번호가 틀렸다면, 같은 이름의 방을 ★다른 비밀번호로★ 누가 먼저
            # 만들어 둔 것이다. 방장이 아니면 고칠 수 없으므로 사람에게 알린다.
            log("★채팅방 '%s' 입장 거절★ (%s) %s"
                % (room, d.get('reason'), d.get('msg') or ''))
            if room == self._wait:
                self._wait = None
                self._setup_next()

        elif t == 'msg':
            self._speak(d)

        elif t == 'room_deleted':
            room = d.get('room')
            if room in self.voice_of:
                log("★채팅방 '%s' 이 삭제됐다★ — 다시 만든다" % room)
                self._joined.discard(room)
                self._known.discard(room)
                if room not in self._todo and room != self._wait:
                    self._todo.append(room)
                if self._wait is None:
                    self._setup_next()

        elif t == 'kicked':
            room = d.get('room')
            if room in self.voice_of:
                log("★채팅방 '%s' 에서 강제 퇴장됐다★ — 그 방은 더 읽지 않는다" % room)
                self._joined.discard(room)

        elif t == 'error':
            self._on_error(d)

    def _on_event(self, ev, d):
        if ev == 'connected':
            log('서버에 접속했다 — 로그인 중')
        elif ev == 'connect_fail':
            log('서버에 접속할 수 없다 (%s) — 다시 시도한다' % d.get('msg'))
        elif ev == 'disconnected':
            log('연결이 끊겼다 (%s) — 다시 붙는다' % d.get('msg'))
            self._joined = set()
            self._wait = None
        elif ev == 'cert_changed':
            log('★서버 인증서 지문이 바뀌었다★ %s: %s… → %s…'
                % (d.get('host'), (d.get('old') or '')[:16],
                   (d.get('new') or '')[:16]))
            log('  중간자이거나 서버를 다시 세운 것이다. 서버가 맞다면 %s 를 지우고'
                ' 다시 띄운다.' % PIN_PATH)
            self._stop.set()

    def _on_error(self, d):
        code = d.get('code')
        msg = d.get('msg') or ''
        if code == 'room_name_taken' and self._wait:
            # 이미 있는 방이다(welcome 이후에 누가 만들었거나, 목록이 어긋났거나).
            # 만들 필요가 없으니 그냥 들어간다.
            log("채팅방 '%s' 은 이미 있다 — 입장한다" % self._wait)
            self.client.send({'t': 'join', 'room': self._wait,
                              'pw': self.room_pw})
            return
        if code in ('bad_login', 'already_online', 'disabled'):
            # 계정 문제는 재접속으로 풀리지 않는다(already_online 은 먼저 붙은 쪽이
            # 정리되면 풀리지만, 그때까지 재접속을 반복해도 의미가 없다).
            log('★로그인 실패★ (%s) %s' % (code, msg))
            log('  상단 파라미터 USER_ID / USER_PW 와 서버 계정 승인 상태를 볼 것')
            self._stop.set()
            return
        log('[서버 오류] (%s) %s' % (code, msg))
        if self._wait:
            self._wait = None
            self._setup_next()

    # ── 방 준비 ───────────────────────────────────────────────────────────────
    def _setup_next(self):
        """남은 방 하나를 만들거나 들어간다. ★한 번에 하나씩★ (클래스 주석 참고)."""
        while self._todo:
            room = self._todo.pop(0)
            if room in self._joined:
                continue
            self._wait = room
            if room in self._known:
                self.client.send({'t': 'join', 'room': room, 'pw': self.room_pw})
            else:
                # 비밀번호 제한방으로 만든다. 만든 사람이 방장이 되고, 방장은
                # 이후 비밀번호 없이도 들어갈 수 있다(서버 handle_join).
                self.client.send({'t': 'room_create', 'name': room,
                                  'kind': 'pw', 'pw': self.room_pw})
            return
        self._wait = None
        if len(self._joined) == 2:
            log('두 방 모두 준비됐다 — 올라오는 말을 읽는다')

    # ── 읽기 ──────────────────────────────────────────────────────────────────
    def _speak(self, d):
        room = d.get('room')
        voice = self.voice_of.get(room)
        if voice is None:
            return                          # 우리 방이 아니다(들어간 적도 없다)
        body = (d.get('body') or '').strip()
        if not body:
            return
        who = d.get('from') or '?'
        # ★자르지 않고 그대로 읽는다★ 긴 글이 오면 오래 읽지만, 그 사이 새 채팅이
        #   오면 어차피 끊기고 새 것을 읽는다(Speaker 주석). 길이를 여기서 손보면
        #   '올라온 대화를 그대로 읽는다'는 이 노드의 약속이 깨진다.
        log('[%s/%s] %s: %s' % (self.label_of.get(room, '?'), room, who, body))
        self.speaker.say(body, voice)


# ══════════════════════════════════════════════════════════════════════════════
# === [5. 진입점] ===
# ══════════════════════════════════════════════════════════════════════════════

def resolve_names():
    """(PC 이름, 방 비밀번호). 둘 다 상단 파라미터가 비어 있으면 hostname 에서 만든다.

    ★방 이름은 30자, 방 비밀번호는 19자가 서버 상한★ 이다(domiserver 의
    ROOM_NAME_MAX / ROOM_PW_MAX). hostname 이 길면 그 상한에 걸려 방을 만들지
    못하므로, 넘치면 잘라 쓰고 그 사실을 로그로 남긴다.
    """
    pc = PC_NAME or socket.gethostname().split('.')[0]
    pc = ''.join(ch for ch in pc if ord(ch) >= 32).strip()
    if not pc:
        pc = 'nxde'
    room_max = ROOM_NAME_MAX - max(len(ROOM_SUFFIX_MALE), len(ROOM_SUFFIX_FEMALE))
    if len(pc) > room_max:
        log('★PC 이름이 너무 길다★ (%s) — 방 이름 상한(%d자)에 맞춰 앞 %d자만 쓴다'
            % (pc, ROOM_NAME_MAX, room_max))
        pc = pc[:room_max]
    pw = ROOM_PW or pc
    if len(pw) > ROOM_PW_MAX:
        log('★방 비밀번호가 너무 길다★ — 서버 상한(%d자)에 맞춰 자른다' % ROOM_PW_MAX)
        pw = pw[:ROOM_PW_MAX]
    return pc, pw


def main():
    pc, room_pw = resolve_names()
    bridge = TtsBridge(pc, room_pw)

    def on_signal(*_a):
        bridge.request_stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, on_signal)
        except (ValueError, AttributeError):   # 메인 스레드가 아니면 그냥 넘어간다
            pass

    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.shutdown()
        except Exception:
            pass
        log('종료')
        sys.stdout.flush()
        sys.stderr.flush()
        # ★여기서 못을 박는다★ 위 정리로 정상 종료되는 것이 정상이지만, 재생기
        #   프로세스·오디오 스택이 남긴 스레드에 붙들려 안 죽는 일이 실제로 있었다.
        #   이 도구는 저장하는 것이 없어 강제 종료로 잃을 상태가 없다.
        os._exit(0)


if __name__ == '__main__':
    main()
