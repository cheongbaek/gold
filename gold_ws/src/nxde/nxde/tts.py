#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tts.py ― 대화형 TTS (edge-tts + tkinter) [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde tts

문장을 입력하면 그 자리에서 합성해 ★시스템 기본 스피커★ 로 읽어 준다.
sound/*.mp3 안내 음성을 새로 만들거나, 문구를 귀로 확인할 때 쓴다.

  ┌ 화면 ─────────────────────────────────────────────────────────────────────┐
  │   [ 남 성 ]   [ 여 성 ]        ← 둘 중 하나만 선택된다                     │
  │   ┌───────────────────────────────────────────────┐                       │
  │   │ 읽을 문장                                     │  [ 읽 기 ] [ 정 지 ]  │
  │   └───────────────────────────────────────────────┘                       │
  │   상태 : 준비                                                             │
  └───────────────────────────────────────────────────────────────────────────┘

  남성 = ko-KR-InJoonNeural / 여성 = ko-KR-SunHiNeural

════════════════════════════════════════════════════════════════════════════════
 알아 둘 것
════════════════════════════════════════════════════════════════════════════════
  · ★인터넷이 필요하다★ edge-tts 는 Microsoft Edge 의 온라인 음성합성을 쓴다.
    차량에서 망이 없으면 합성이 실패한다 — 그래서 주행 중 안내는 이 도구로 미리
    만들어 둔 sound/*.mp3 를 재생하는 방식(nxde/sound.py)이고, 이 파일은 그
    음성을 만들거나 확인하는 ★작업용 도구★ 다.
  · ROS 노드가 아니다. `ros2 run` 으로 띄우기만 할 뿐 토픽을 주고받지 않는다.
  · ★합성·재생은 별도 스레드★ tkinter 는 그것을 기다리지 않는다. 상태 보고는
    큐에 넣고 메인 스레드의 after() 틱이 꺼내 화면에 쓴다(Tk 는 스레드 안전하지
    않다 — prompt_g.py 와 같은 규약).
  · 필요한 것 : pip install --user edge-tts pygame

════════════════════════════════════════════════════════════════════════════════
 ★종료가 안 되던 문제 [2026-08-12 수정]★
════════════════════════════════════════════════════════════════════════════════
  창을 닫아도(그리고 Ctrl+C 를 눌러도) 프로세스가 남아 있었다. 원인은 둘이다.

  ① pygame.mixer 를 ★워커 스레드에서 init/quit★ 했다. mainloop 자체는 정상적으로
     빠져나오는데, 그 뒤 인터프리터 종료 단계에서 메인 스레드가 C 코드에 갇힌다
     (faulthandler 로 잡으면 스레드 하나가 `<no Python frame>` 로 멈춰 있다).
     SDL 오디오는 초기화한 스레드와 정리하는 스레드가 어긋나면 이렇게 굳는다.
     → ★mixer 의 init·quit 을 메인 스레드로 옮겼다★ 워커는 load/play/stop 만 한다.
  ② tkinter 는 콜백에서 난 예외를 report_callback_exception 으로 ★삼킨다★.
     KeyboardInterrupt 도 예외라 Ctrl+C 가 화면에 역추적만 찍고 지나갔다.
     → SIGINT 핸들러를 달아 창닫기와 같은 종료 경로로 보낸다.

  그래도 남는 위험(SDL·ALSA 쪽 잔여 스레드)을 감안해 마지막에 os._exit 로 못을
  박는다. 이 도구는 저장하는 것이 없어 그렇게 끝내도 잃을 상태가 없다.
"""

import asyncio
import io
import os
import queue
import signal
import sys
import threading
import time

try:
    import edge_tts
except ImportError as exc:
    raise SystemExit("edge-tts 가 없다 — pip install --user edge-tts") from exc

try:
    import pygame
except ImportError as exc:
    raise SystemExit("pygame 이 없다 — pip install --user pygame") from exc

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError as exc:
    raise SystemExit("tkinter 가 없다 — sudo apt install python3-tk") from exc


VOICE_MALE   = "ko-KR-InJoonNeural"
VOICE_FEMALE = "ko-KR-SunHiNeural"

RATE   = "+0%"
VOLUME = "+0%"
PITCH  = "+0Hz"

UI_PERIOD_MS = 80

BG      = '#1c1f24'
BG_BOX  = '#111318'
FG      = '#e8eaed'
FG_DIM  = '#8b929c'
FG_OK   = '#7fd4a2'
FG_WARN = '#ffb454'


async def synthesize(text: str, voice: str) -> io.BytesIO:
    """문장 → mp3 바이트. 파일로 떨어뜨리지 않고 메모리에서 끝낸다."""
    audio_buffer = io.BytesIO()

    communicate = edge_tts.Communicate(
        text=text,
        voice=voice,
        rate=RATE,
        volume=VOLUME,
        pitch=PITCH,
    )

    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            audio_buffer.write(chunk["data"])

    audio_buffer.seek(0)
    return audio_buffer


class Speaker:
    """합성·재생 전담 스레드.

    ★한 문장씩 순서대로★ 처리한다. 재생 중에 [읽기]를 또 누르면 큐에 쌓여 앞의
    것이 끝난 뒤에 나온다 — 겹쳐 들리는 것보다 낫다. [정지]는 지금 나오는 것만
    끊는다(큐는 그대로).

    ★mixer 의 init·quit 은 여기서 하지 않는다★ 메인 스레드가 한다(헤더 '종료가
    안 되던 문제' ① 참고). 이 스레드는 load/play/stop 만 만진다.
    """

    def __init__(self, report):
        self.report = report              # (문구, 바쁨?) — 큐에 넣기만 한다
        self._q = queue.Queue()
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def say(self, text, voice):
        self._q.put((text, voice))

    def stop_playing(self):
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()

    def shutdown(self, timeout=1.5):
        """멈추라고 하고 ★실제로 멈출 때까지 기다린다★ — 재생 중에 창을 닫는
        경우가 흔한데, 그때 mixer 를 만지는 쪽이 남아 있으면 정리가 엉킨다."""
        self._quit.set()
        self.stop_playing()               # 재생 중이면 곧바로 끊어 대기를 짧게 한다
        self._q.put(None)                 # 대기 중인 get() 을 깨운다
        self._thread.join(timeout)

    # ── 스레드 본체 ────────────────────────────────────────────────────────────
    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            while not self._quit.is_set():
                item = self._q.get()
                if item is None:
                    break
                text, voice = item
                try:
                    self.report(f"합성 중… ({voice})", True)
                    buffer = loop.run_until_complete(synthesize(text, voice))
                    self.report("재생 중…", True)
                    self._play(buffer)
                    self.report("준비", False)
                except Exception as e:
                    # 대부분 망이 없거나 목소리 이름이 틀린 경우다
                    self.report(f"[오류] {e}", False)
        finally:
            loop.close()

    def _play(self, buffer):
        if not pygame.mixer.get_init():
            self.report("오디오 장치가 없어 재생하지 못했다", False)
            return
        pygame.mixer.music.load(buffer, "mp3")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy() and not self._quit.is_set():
            time.sleep(0.05)
        pygame.mixer.music.unload()


class App:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('nxde TTS')
        self.root.configure(bg=BG)
        self.root.minsize(620, 260)
        self.root.protocol('WM_DELETE_WINDOW', self.on_quit)

        base = tkfont.nametofont('TkDefaultFont')
        self.f_btn = base.copy(); self.f_btn.configure(size=15, weight='bold')
        self.f_txt = base.copy(); self.f_txt.configure(size=15)
        self.f_bar = base.copy(); self.f_bar.configure(size=11)

        # 상태 보고는 스레드에서 오므로 큐를 거친다
        self._status_q = queue.Queue()
        self.speaker = Speaker(
            lambda text, busy: self._status_q.put((text, busy)))

        self._build()

        # ★오디오는 메인 스레드에서 연다★ 워커에서 열면 종료가 굳는다(헤더 ①).
        self.audio_ok = False
        try:
            pygame.mixer.init()
            self.audio_ok = True
            self.v_status.set('준비')
        except Exception as e:            # 장치가 없어도 창은 떠 있어야 한다
            self.v_status.set(f'오디오 장치를 열지 못했다 — {e}')

        self.speaker.start()
        # Ctrl+C 를 창닫기와 같은 경로로 보낸다. tkinter 는 콜백에서 난
        # KeyboardInterrupt 를 삼키므로(헤더 ②) 핸들러가 없으면 아무 일도 안 난다.
        try:
            signal.signal(signal.SIGINT, self._on_sigint)
        except ValueError:                # 메인 스레드가 아니면 그냥 넘어간다
            pass
        self.tick()

    # ── 화면 ───────────────────────────────────────────────────────────────────
    def _build(self):
        r = self.root
        r.columnconfigure(0, weight=1)

        # ★목소리 선택★ Radiobutton(indicatoron=0) 이라 생김새는 버튼이고 배타 선택은
        #   위젯이 보장한다 — 버튼 두 개로 만들면 그 규칙을 손으로 지켜야 한다.
        pick = tk.Frame(r, bg=BG)
        pick.grid(row=0, column=0, sticky='ew', padx=16, pady=(14, 6))
        pick.columnconfigure((0, 1), weight=1)
        self.voice = tk.StringVar(value=VOICE_FEMALE)
        for i, (label, value) in enumerate((('남 성', VOICE_MALE),
                                            ('여 성', VOICE_FEMALE))):
            tk.Radiobutton(pick, text=label, value=value, variable=self.voice,
                           indicatoron=False, font=self.f_btn, width=10, pady=8,
                           selectcolor='#2f6f4f', bg='#2a2f37', fg=FG,
                           activebackground='#3a4049', activeforeground=FG,
                           command=self.on_voice).grid(row=0, column=i, sticky='ew',
                                                       padx=(0 if i == 0 else 8, 0))

        body = tk.Frame(r, bg=BG)
        body.grid(row=1, column=0, sticky='ew', padx=16, pady=6)
        body.columnconfigure(0, weight=1)

        tk.Label(body, text='읽을 문장', font=self.f_bar, bg=BG, fg=FG_DIM,
                 anchor='w').grid(row=0, column=0, columnspan=2, sticky='ew')
        self.entry = tk.Entry(body, font=self.f_txt, bg=BG_BOX, fg=FG,
                              insertbackground=FG, relief='flat')
        self.entry.grid(row=1, column=0, sticky='ew', ipady=8, padx=(0, 8))
        self.entry.bind('<Return>', lambda _e: self.on_say())
        self.entry.focus_set()

        btns = tk.Frame(body, bg=BG)
        btns.grid(row=1, column=1, sticky='e')
        self.b_say = tk.Button(btns, text='읽 기', font=self.f_btn, width=6,
                               command=self.on_say)
        self.b_say.grid(row=0, column=0, padx=(0, 6))
        tk.Button(btns, text='정 지', font=self.f_btn, width=6,
                  command=self.speaker.stop_playing).grid(row=0, column=1)

        self.v_status = tk.StringVar(value='오디오 장치 준비 중…')
        tk.Label(r, textvariable=self.v_status, font=self.f_bar, bg=BG, fg=FG_DIM,
                 anchor='w', padx=16, pady=10, wraplength=580, justify='left').grid(
            row=2, column=0, sticky='ew')
        r.rowconfigure(2, weight=1)

    # ── 동작 ───────────────────────────────────────────────────────────────────
    def on_voice(self):
        who = '남성' if self.voice.get() == VOICE_MALE else '여성'
        self.v_status.set(f'{who} 목소리 — {self.voice.get()}')

    def on_say(self):
        text = self.entry.get().strip()
        if not text:
            return
        self.speaker.say(text, self.voice.get())

    def _on_sigint(self, *_a):
        # 시그널 핸들러 안에서 위젯을 만지지 않는다 — 다음 틱에 정식 경로로 종료한다
        self.root.after(0, self.on_quit)

    def on_quit(self):
        """창닫기·Ctrl+C 공통 종료 경로. ★정리 순서가 중요하다★
        워커를 먼저 세우고(재생 정지 포함), 그 다음 mixer 를 메인 스레드에서 닫고,
        마지막에 창을 없앤다."""
        self.speaker.shutdown()
        if self.audio_ok:
            try:
                pygame.mixer.quit()
            except Exception:
                pass
            self.audio_ok = False
        self.root.destroy()

    def tick(self):
        busy = None
        while True:                        # 밀린 보고는 마지막 것만 화면에 남는다
            try:
                text, busy = self._status_q.get_nowait()
            except queue.Empty:
                break
            self.v_status.set(text)
        if busy is not None:
            self.b_say.config(state='disabled' if busy else 'normal')
        self.root.after(UI_PERIOD_MS, self.tick)

    def run(self):
        self.root.mainloop()


def main():
    try:
        app = App()
    except tk.TclError as e:
        raise SystemExit(f"창을 열 수 없다({e}) — DISPLAY 가 있는 화면에서 실행할 것")
    try:
        app.run()
    except KeyboardInterrupt:
        app.on_quit()
    finally:
        # ★여기서 못을 박는다★ 위 정리로 정상 종료되는 것이 정상이지만, SDL·ALSA 가
        #   남긴 스레드 하나에 붙들려 프로세스가 안 죽는 일이 실제로 있었다.
        #   이 도구는 저장하는 것이 없어 강제 종료로 잃을 상태가 없다.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
