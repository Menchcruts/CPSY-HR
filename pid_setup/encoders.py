"""Quadrature encoder counting that keeps up with motor-shaft encoders.

gpiozero's RotaryEncoder runs a chain of Python code for every single edge.
At motor speeds that is thousands of edges per second per wheel, Python falls
behind, and pigpiod starts dropping events, so the counts come out far too low.

This module lets pigpiod (C code) record the edges into its notification pipe,
and decodes everything that arrived since the last call in one go with numpy.
Python work is per control-loop sample instead of per edge.

    reader = EncoderReader(pi, [(5, 6), (16, 26)])   # pi = pigpio.pi()
    enc_a, enc_b = reader.encoders
    while True:
        reader.update()          # call once per loop iteration
        print(enc_a.steps, enc_b.steps)

Counts all four edges of each cycle, and .steps is in full cycles, the same
unit and direction as gpiozero's RotaryEncoder.steps, so existing pin orders
and COUNTS_PER_REV calibrations still apply.
"""
import os

import numpy as np
import pigpio

# pigpiod notification report: seqno, flags, tick (us), levels of GPIO 0-31
_REPORT = np.dtype([("seq", "<u2"), ("flags", "<u2"), ("tick", "<u4"), ("level", "<u4")])

# Lookup tables indexed by (previous_state << 2) | state, with state = A << 1 | B
_DELTA = np.zeros(16, dtype=np.int64)
for _prev, _cur in [(3, 1), (1, 0), (0, 2), (2, 3)]:  # gpiozero's "+1" direction
    _DELTA[_prev << 2 | _cur] = 1
    _DELTA[_cur << 2 | _prev] = -1
_INVALID = np.zeros(16, dtype=bool)  # both phases changed at once: an edge was missed
for _prev, _cur in [(0, 3), (3, 0), (1, 2), (2, 1)]:
    _INVALID[_prev << 2 | _cur] = True


class Encoder:
    def __init__(self, a, b):
        self.a, self.b = a, b
        self.edges = 0
        self.state = 0

    @property
    def steps(self):
        """Full encoder cycles (4 edges each), like gpiozero's RotaryEncoder.steps."""
        return self.edges / 4


class EncoderReader:
    """Decodes one or more encoders from a single pigpiod notification pipe."""

    def __init__(self, pi, pin_pairs, fifo="/dev/pigpio{}"):
        self.pi = pi
        self.encoders = [Encoder(a, b) for a, b in pin_pairs]
        self.dropped = 0   # reports pigpiod had to throw away (pipe was full)
        self.invalid = 0   # impossible transitions (missed edges or noise)
        self._buf = b""
        self._last_seq = None
        self.handle = None

        bits = 0
        for enc in self.encoders:
            for pin in (enc.a, enc.b):
                pi.set_mode(pin, pigpio.INPUT)
                pi.set_pull_up_down(pin, pigpio.PUD_UP)
                bits |= 1 << pin

        handle = pi.notify_open()
        if handle < 0:
            raise RuntimeError(f"pigpio notify_open failed ({handle})")
        self.handle = handle
        self._fd = os.open(fifo.format(handle), os.O_RDONLY | os.O_NONBLOCK)
        level = pi.read_bank_1()
        for enc in self.encoders:
            enc.state = ((level >> enc.a) & 1) << 1 | ((level >> enc.b) & 1)
        pi.notify_begin(handle, bits)

    def update(self):
        """Decode every edge that arrived since the last call."""
        chunks = [self._buf]
        while True:
            try:
                data = os.read(self._fd, 65536)
            except BlockingIOError:
                break
            if not data:
                break
            chunks.append(data)
        buf = b"".join(chunks)
        n = len(buf) // _REPORT.itemsize
        self._buf = buf[n * _REPORT.itemsize:]  # keep any partial report
        if n == 0:
            return
        reports = np.frombuffer(buf, dtype=_REPORT, count=n)

        seq = reports["seq"].astype(np.int64)
        if self._last_seq is not None:
            seq = np.concatenate(([self._last_seq], seq))
        self.dropped += int(((np.diff(seq) % 65536) - 1).sum())
        self._last_seq = int(seq[-1])

        level = reports["level"][reports["flags"] == 0].astype(np.int64)
        if len(level) == 0:
            return
        for enc in self.encoders:
            states = ((level >> enc.a) & 1) << 1 | ((level >> enc.b) & 1)
            prev = np.concatenate(([enc.state], states[:-1]))
            idx = prev << 2 | states
            enc.edges += int(_DELTA[idx].sum())
            self.invalid += int(_INVALID[idx].sum())
            enc.state = int(states[-1])

    def close(self):
        if self.handle is not None:
            self.pi.notify_close(self.handle)
            os.close(self._fd)
            self.handle = None
