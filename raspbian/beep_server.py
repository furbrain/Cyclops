import signal
from queue import Queue, Empty
from threading import Thread, Lock
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
from typing import Sequence, Tuple
import time

import systemd.daemon

from beeper import Beeper

NOTES = {
        "C2": 65,
        "C#2": 69,
        "D2": 73,
        "D#2": 78,
        "E2": 82,
        "F2": 87,
        "F#2": 92,
        "G2": 98,
        "G#2": 104,
        "A2": 110,
        "A#2": 117,
        "B2": 123,
        "C3": 131,
        "C#3": 139,
        "D3": 147,
        "D#3": 156,
        "E3": 165,
        "F3": 175,
        "F#3": 185,
        "G3": 196,
        "G#3": 208,
        "A3": 220,
        "A#3": 233,
        "B3": 247,
        "C4": 262,
        "C#4": 277,
        "D4": 294,
        "D#4": 311,
        "E4": 330,
        "F4": 349,
        "F#4": 370,
        "G4": 392,
        "G#4": 415,
        "A4": 440,
        "A#4": 466,
        "B4": 494,
        "C5": 523,
        "C#5": 554,
        "D5": 587,
        "D#5": 622,
        "E5": 659,
        "F5": 698,
        "F#5": 740,
        "G5": 784,
        "G#5": 831,
        "A5": 880,
        "A#5": 932,
        "B5": 988,
        "C6": 1047,
        "C#6": 1109,
        "D6": 1175,
        "D#6": 1245,
        "E6": 1319,
        "F6": 1397,
        "F#6": 1480,
        "G6": 1568,
        "G#6": 1661,
        "A6": 1760,
        "A#6": 1865,
        "B6": 1976,
        "C7": 2093,
        "C#7": 2217,
        "D7": 2349,
        "D#7": 2489,
        "E7": 2637,
        "F7": 2794,
        "F#7": 2960,
        "G7": 3136,
        "G#7": 3322,
        "A7": 3520,
        "A#7": 3729,
        "B7": 3951,
        "C8": 4186,
}

class BeepThread(Thread):
    def __init__(self):
        super().__init__(name="BeepThread")
        self.b = Beeper(13, 19)
        self.running = True
        self.playing = False
        self.q: Queue[Tuple[str, float]] = Queue()
        self.interrupted = False
        self.lock = Lock()
        self.playing = False

    def run(self):
        self.b.setup()
        while self.running:
            try:
                got = self.q.get(timeout=0.05)
                self.playing = True
                note, duration = got
                self.interrupted = False
            except Empty:
                self.b.stop()
                self.playing = False
                continue
            if note in NOTES:
                self.b.beep(NOTES[note])
            else:
                self.b.stop()
            target_tm = time.monotonic() + duration/1000
            while self.running and time.monotonic() < target_tm:
                with self.lock:
                    if self.interrupted:
                        break
                time.sleep(0.05)

    def play(self, notes: Sequence[Tuple[str,float]]):
        with self.lock:
            while not self.q.empty(): # clear queue
                self.q.get()
            for note in notes:
                self.q.put(note)
            self.interrupted = True
            self.playing = True

    def stop(self):
        self.running = False
        self.b.stop()
        self.b.finish()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

def terminate(signal, frame):
    raise KeyboardInterrupt()


BOP =  (("C7", 50),)
FINISH =  (("C7", 50.0), ("G6", 50.0), ("E6", 50.0), ("C6", 50.0))

with SimpleXMLRPCServer(('localhost', 8123), allow_none=True) as server:
    signal.signal(signal.SIGTERM, terminate)
    server.register_introspection_functions()
    with BeepThread() as thread:
        thread.play(BOP)
        # Register a function under function.__name__.
        @server.register_function
        def beep(seq: Sequence[Tuple[str, float]]):
            thread.play(seq)
        systemd.daemon.notify('READY=1')
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        thread.play(FINISH)
        while (thread.playing):
            time.sleep(0.1)


