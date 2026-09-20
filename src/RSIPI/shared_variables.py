"""
receive_variables in shared memory.

The network process needs every value the robot receives from us on every
cycle, inside a 4 ms reply window. Held in a multiprocessing.Manager dict,
that snapshot is a pipe round trip to the manager process per key: dict(proxy)
is keys() plus one __getitem__ each, 13 round trips for the Drill context,
1.8 ms median and 4.3 ms worst case measured on the drilling laptop. While a
trajectory ran, each publish queued its own Manager calls ahead of them and
about 10 % of replies missed the window (KR C4, 2026-09).

SharedVariables keeps the same mapping interface over one block of shared
memory. The block holds a single frame, the waypoint sequence number followed
by the pickled variables, so a (seq, variables) pair is published whole:

    writer, under a lock no reader takes:
        gen -> odd, copy the frame in, store its length and CRC, gen -> even
    reader, no lock and no IPC:
        read gen, copy the frame out, read gen again

A reader accepts a frame only if gen was even and unchanged across the copy
and the CRC matches. The CRC is what makes that hold on CPUs that do not
order stores as x86 does, and after a writer interrupted mid-commit. A reader
that cannot get a clean frame keeps the last one it had: a publish still in
progress has not happened yet, and the waypoint goes out a cycle later.
"""

import ctypes
import logging
import multiprocessing
import pickle
import zlib
from collections.abc import MutableMapping
from multiprocessing.sharedctypes import RawArray
from typing import Any, Dict, Iterator, Mapping, Optional, Tuple

_GEN, _LEN, _CRC = 0, 1, 2
_SEQ_BYTES = 8
_MIN_CAPACITY = 65536

# A commit is a few microseconds. A reader that lands inside one spins this
# many times (about a microsecond each) before treating the writer as
# preempted and falling back.
_READ_SPINS = 32


class SharedVariables(MutableMapping):
    """
    Dict-like store for receive_variables, shared between processes.

    Values read out are copies, as with a Manager dict: mutate a group by
    writing it back (``rv["RKorr"] = {...}``), not in place.

    Args:
        initial: Starting contents (the config's RECEIVE variables).
        seq: Starting waypoint sequence. reconnect() carries the old store's
            value over so a sequence number is never reused.
        capacity: Bytes reserved for the frame. The default leaves room for
            long EStr strings.
    """

    def __init__(self, initial: Optional[Mapping] = None, seq: int = 0,
                 capacity: int = 0) -> None:
        frame = self._encode(seq, dict(initial or {}))
        self._capacity = max(int(capacity), _MIN_CAPACITY, 4 * len(frame))
        self._hdr = RawArray(ctypes.c_int64, 3)
        self._buf = RawArray(ctypes.c_char, self._capacity)
        self._wlock = multiprocessing.Lock()
        self._init_local()
        self._commit(frame)

    # ------------------------------------------------------------ plumbing

    def _init_local(self) -> None:
        """Per-process state: never shared, rebuilt after pickling."""
        self._addr = ctypes.addressof(self._buf)
        self._cache: Tuple[int, bytes] = (-1, b"")
        # Seconds a writer waits for the lock before giving up. The network
        # process lowers this: it may not stall a reply on a parent thread.
        self.write_timeout: float = 1.0
        # Cycles on which snapshot() kept the previous frame because a
        # writer was mid-commit.
        self.stale_snapshots: int = 0

    def __getstate__(self) -> dict:
        return {"_capacity": self._capacity, "_hdr": self._hdr,
                "_buf": self._buf, "_wlock": self._wlock}

    def __setstate__(self, state: dict) -> None:
        self.__dict__.update(state)
        self._init_local()
        self._read_frame(block=True)

    @staticmethod
    def _encode(seq: int, data: dict) -> bytes:
        return (seq.to_bytes(_SEQ_BYTES, "little", signed=True)
                + pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL))

    @staticmethod
    def _decode(frame: bytes) -> Tuple[int, dict]:
        return (int.from_bytes(frame[:_SEQ_BYTES], "little", signed=True),
                pickle.loads(frame[_SEQ_BYTES:]))

    def _commit(self, frame: bytes) -> None:
        """Publish *frame*. Caller holds the writer lock (or is __init__)."""
        n = len(frame)
        if n > self._capacity:
            raise ValueError(
                f"receive_variables need {n} bytes, the shared block holds "
                f"{self._capacity} - a value written to them is too large")
        hdr = self._hdr
        gen = hdr[_GEN] | 1
        hdr[_GEN] = gen
        try:
            ctypes.memmove(self._addr, frame, n)
            hdr[_LEN] = n
            hdr[_CRC] = zlib.crc32(frame)
        finally:
            # Even again whatever happened: an interrupted commit must not
            # leave readers waiting on it. If it left the block torn the CRC
            # says so, and readers keep their last frame until the next one.
            hdr[_GEN] = gen + 1
        self._cache = (gen + 1, frame)

    def _read_frame(self, block: bool) -> bytes:
        """
        Newest clean frame, without IPC.

        block=False never waits: if a writer is mid-commit for longer than a
        short spin, the previous frame is returned and stale_snapshots
        counts it. block=True waits the writer out instead.
        """
        gen, frame = self._cache
        hdr = self._hdr
        for _ in range(_READ_SPINS):
            g = hdr[_GEN]
            if g == gen:
                return frame
            if g & 1:
                continue
            n = hdr[_LEN]
            if not _SEQ_BYTES <= n <= self._capacity:
                continue
            data = ctypes.string_at(self._addr, n)
            if hdr[_GEN] == g and zlib.crc32(data) == hdr[_CRC]:
                self._cache = (g, data)
                return data
        if not block:
            self.stale_snapshots += 1
            return frame
        if self._wlock.acquire(timeout=self.write_timeout):
            try:
                return self._read_locked()
            finally:
                self._wlock.release()
        logging.error("receive_variables: writer lock not released within %g s - "
                      "returning the last values read", self.write_timeout)
        return frame

    def _read_locked(self) -> bytes:
        """Current frame. Caller holds the writer lock, so nothing is mid-commit."""
        hdr = self._hdr
        n = hdr[_LEN]
        if _SEQ_BYTES <= n <= self._capacity:
            data = ctypes.string_at(self._addr, n)
            if zlib.crc32(data) == hdr[_CRC]:
                self._cache = (hdr[_GEN], data)
                return data
        logging.error("receive_variables: shared block is torn (a write was "
                      "interrupted) - continuing from the last values read")
        return self._cache[1]

    def _modify(self, change, bump_seq: bool = False) -> int:
        """Read, apply *change(data)*, publish: one commit under the writer lock."""
        if not self._wlock.acquire(timeout=self.write_timeout):
            raise TimeoutError(
                f"receive_variables writer lock not acquired within "
                f"{self.write_timeout} s - another writer is stalled")
        try:
            seq, data = self._decode(self._read_locked())
            change(data)
            if bump_seq:
                seq += 1
            self._commit(self._encode(seq, data))
            return seq
        finally:
            self._wlock.release()

    # ------------------------------------------------------ network process

    def snapshot(self) -> Tuple[int, Dict[str, Any]]:
        """
        (seq, variables) as one consistent pair, for the per-cycle reply.

        No IPC and no lock: seeing seq N guarantees the variables carry
        waypoint N's corrections. The dict is a fresh copy every call.
        """
        return self._decode(self._read_frame(block=False))

    # --------------------------------------------------------------- parent

    def publish(self, corrections: Mapping[str, Mapping[str, Any]]) -> int:
        """
        Merge correction groups and bump the waypoint sequence in one commit.

        Returns the new sequence number. The network process sees either the
        previous (seq, corrections) pair or this one, never a mixture.
        """
        def change(data: dict) -> None:
            for key, axes in corrections.items():
                current = data.get(key)
                merged = dict(current) if isinstance(current, dict) else {}
                merged.update(axes)
                data[key] = merged

        return self._modify(change, bump_seq=True)

    @property
    def seq(self) -> int:
        """Current waypoint sequence number."""
        return self._decode(self._read_frame(block=True))[0]

    def copy(self) -> Dict[str, Any]:
        """All variables as a plain dict."""
        return self._decode(self._read_frame(block=True))[1]

    # -------------------------------------------------------------- mapping

    def __getitem__(self, key: str) -> Any:
        return self.copy()[key]

    def __setitem__(self, key: str, value: Any) -> None:
        self._modify(lambda data: data.__setitem__(key, value))

    def __delitem__(self, key: str) -> None:
        self._modify(lambda data: data.__delitem__(key))

    def __iter__(self) -> Iterator[str]:
        return iter(self.copy())

    def __len__(self) -> int:
        return len(self.copy())

    def __contains__(self, key: object) -> bool:
        return key in self.copy()

    def update(self, *args: Any, **kwargs: Any) -> None:
        """Several keys in one commit (MutableMapping's would be one each)."""
        changes = dict(*args, **kwargs)
        self._modify(lambda data: data.update(changes))

    def keys(self):
        return self.copy().keys()

    def values(self):
        return self.copy().values()

    def items(self):
        return self.copy().items()

    def __repr__(self) -> str:
        return f"SharedVariables({self.copy()!r})"
