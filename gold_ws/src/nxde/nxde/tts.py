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
"""

import asyncio
import io
import queue
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
    """

    def __init__(self, report):
        self.report = report              # (문구, 바쁨?) — 큐에 넣기만 한다
        self._q = queue.Queue()
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.ready = False

    def start(self):
        self._thread.start()

    def say(self, text, voice):
        self._q.put((text, voice))

    def stop_playing(self):
        if self.ready:
            pygame.mixer.music.stop()

    def shutdown(self):
        self._quit.set()
        self._q.put(None)                 # 대기 중인 get() 을 깨운다

    # ── 스레드 본체 ────────────────────────────────────────────────────────────
    def _run(self):
        try:
            pygame.mixer.init()
            self.ready = True
        except Exception as e:            # 오디오 장치가 없어도 창은 떠 있어야 한다
            self.report(f"오디오 장치를 열지 못했다 — {e}", False)
            return
        self.report("준비", False)

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
            try:
                pygame.mixer.music.stop()
                pygame.mixer.quit()
            except Exception:
                pass
            loop.close()

    def _play(self, buffer):
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
        self.speaker.start()
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

    def on_quit(self):
        self.speaker.shutdown()
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
    app = App()
    try:
        app.run()
    except KeyboardInterrupt:
        app.speaker.shutdown()
    except tk.TclError as e:
        raise SystemExit(f"창을 열 수 없다({e}) — DISPLAY 가 있는 화면에서 실행할 것")


if __name__ == '__main__':
    main()
