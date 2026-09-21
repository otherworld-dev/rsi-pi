"""
Variables shared between the client and the network process, in shared memory.

The network process answers the robot inside a 4 ms window, so nothing it
does per cycle may wait on another process. Held in multiprocessing.Manager
dicts, its per-cycle read of receive_variables was a pipe round trip to the
manager process per key: dict(proxy) is keys() plus one __getitem__ each, 13
round trips for the Drill context, 1.8 ms median and 4.3 ms worst case
measured on the drilling laptop. While a trajectory ran, each publish queued
its own Manager calls ahead of them and about 10 % of replies missed the
window (KR C4, 2026-09). The robot's state went the other way through the
same Manager, which is why it was only passed on every tenth cycle.

SharedVariables keeps the dict interface over one block of shared memory
holding two slots. A publication is one frame: its commit number, the
waypoint sequence number and the pickled variables.

    writer, under a lock only writers take:
        copy the frame into the idle slot, store its length and CRC,
        then store the commit number. That last store is the publication:
        its low bit names the slot readers use from then on.
    reader, no lock and no IPC:
        read the commit number, copy that slot out

A reader accepts a frame only if its CRC matches and the commit number inside
it is the one the reader went looking for, so it never takes an older frame
for a newer one, on any CPU's memory ordering. The published slot is never
written to, so a reader always finds the newest complete frame, even while a
writer is preempted, frozen or dead mid-commit, and an interrupted commit
damages nothing.
"""

import ctypes
import logging
import multiprocessing
import pickle
import zlib
from collections.abc import MutableMapping
from multiprocessing.sharedctypes import RawArray
from typing import Any, Dict, Iterator, Mapping, Optional, Tuple

# Header: the commit number, then (length, CRC) of slot 0 and of slot 1.
_GEN = 0
# Frame: commit number (8 bytes), waypoint sequence (8 bytes), pickle.
_HEAD = 16
_MIN_CAPACITY = 65536

# A reader starts over only if a whole commit landed in the slot it was
# copying, which takes two commits inside a few microseconds.
_READ_RETRIES = 8


class SharedVariables(MutableMapping):
    """
    Dict-like store shared between processes, read without IPC.

    Values read out are copies, as with a Manager dict: change a group by
    writing it back (``rv["RKorr"] = {...}``), not in place.

    Args:
        initial: Starting contents (the config's SEND or RECEIVE variables).
        seq: Starting waypoint sequence. reconnect() carries the old store's
            value over so a sequence number is never reused.
        capacity: Bytes reserved per slot. The default leaves room for long
            EStr strings and the Full context's telegram.
    """

    def __init__(self, initial: Optional[Mapping] = None, seq: int = 0,
                 capacity: int = 0) -> None:
        data = dict(initial or {})
        needed = _HEAD + len(pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL))
        self._capacity = max(int(capacity), _MIN_CAPACITY, 4 * needed)
        self._hdr = RawArray(ctypes.c_int64, 5)
        self._buf = RawArray(ctypes.c_char, 2 * self._capacity)
        self._wlock = multiprocessing.Lock()
        self._init_local()
        self._commit(seq, data)

    # ------------------------------------------------------------ plumbing

    def _init_local(self) -> None:
        """Per-process state: never shared, rebuilt after pickling."""
        self._addr = ctypes.addressof(self._buf)
        self._cache: Tuple[int, bytes] = (-1, b"")
        # Seconds a blocking writer waits for the lock before giving up. The
        # network process lowers this: it may not stall a reply on a parent
        # thread.
        self.write_timeout: float = 1.0
        # Reads that fell back to the previous frame. Not expected: it takes
        # a corrupt slot, or a writer lapping the reader eight times over.
        self.stale_snapshots: int = 0

    def __getstate__(self) -> dict:
        return {"_capacity": self._capacity, "_hdr": self._hdr,
                "_buf": self._buf, "_wlock": self._wlock}

    def __setstate__(self, state: dict) -> None:
        self.__dict__.update(state)
        self._init_local()
        self._read_frame()

    @staticmethod
    def _decode(frame: bytes) -> Tuple[int, dict]:
        return (int.from_bytes(frame[8:_HEAD], "little", signed=True),
                pickle.loads(frame[_HEAD:]))

    def _commit(self, seq: int, data: dict) -> None:
        """Publish (seq, data). Caller holds the writer lock (or is __init__)."""
        hdr = self._hdr
        gen = hdr[_GEN] + 1
        frame = (gen.to_bytes(8, "little") + seq.to_bytes(8, "little", signed=True)
                 + pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL))
        n = len(frame)
        if n > self._capacity:
            raise ValueError(
                f"variables need {n} bytes, the shared block holds "
                f"{self._capacity} - a value written to them is too large")
        slot = gen & 1
        ctypes.memmove(self._addr + slot * self._capacity, frame, n)
        hdr[1 + 2 * slot] = n
        hdr[2 + 2 * slot] = zlib.crc32(frame)
        # The publication. Until this store readers use the other slot, and
        # a commit interrupted before it has changed nothing they can see.
        hdr[_GEN] = gen
        self._cache = (gen, frame)

    def _read_frame(self) -> bytes:
        """Newest published frame. Never waits: no lock, no IPC."""
        gen, frame = self._cache
        hdr = self._hdr
        for _ in range(_READ_RETRIES):
            g = hdr[_GEN]
            if g == gen:
                return frame
            slot = g & 1
            n = hdr[1 + 2 * slot]
            if not _HEAD <= n <= self._capacity:
                continue
            data = ctypes.string_at(self._addr + slot * self._capacity, n)
            if (zlib.crc32(data) == hdr[2 + 2 * slot]
                    and int.from_bytes(data[:8], "little") == g):
                self._cache = (g, data)
                return data
        self.stale_snapshots += 1
        return frame

    def _acquire(self, block: bool = True) -> bool:
        """Take the writer lock. False only when block=False and it is held."""
        if self._wlock.acquire(block, self.write_timeout):
            return True
        if not block:
            return False
        raise TimeoutError(
            f"variables writer lock not acquired within "
            f"{self.write_timeout:g} s - another writer is stalled")

    def _current(self) -> bytes:
        """The published frame. Caller holds the writer lock."""
        frame = self._read_frame()
        if int.from_bytes(frame[:8], "little") != self._hdr[_GEN]:
            logging.error("shared variables: the published slot is corrupt - "
                          "continuing from the last values read")
        return frame

    def _write(self, change, bump_seq: bool = False) -> int:
        """Read, apply *change(data)*, publish: one commit under the writer lock."""
        self._acquire()
        try:
            seq, data = self._decode(self._current())
            change(data)
            if bump_seq:
                seq += 1
            self._commit(seq, data)
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
        return self._decode(self._read_frame())

    def replace(self, data: Mapping, block: bool = True) -> bool:
        """
        Publish *data* as the whole contents, in one commit. The seq is kept.

        block=False is for the network loop, which publishes the robot's
        state every cycle and may not wait: if another writer holds the lock
        this returns False and the next cycle's state goes out instead.
        """
        contents = dict(data)
        if not self._acquire(block):
            return False
        try:
            seq = int.from_bytes(self._current()[8:_HEAD], "little", signed=True)
            self._commit(seq, contents)
            return True
        finally:
            self._wlock.release()

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

        return self._write(change, bump_seq=True)

    @property
    def seq(self) -> int:
        """Current waypoint sequence number."""
        return int.from_bytes(self._read_frame()[8:_HEAD], "little", signed=True)

    def copy(self) -> Dict[str, Any]:
        """All variables as a plain dict."""
        return self._decode(self._read_frame())[1]

    # -------------------------------------------------------------- mapping

    def __getitem__(self, key: str) -> Any:
        return self.copy()[key]

    def __setitem__(self, key: str, value: Any) -> None:
        self._write(lambda data: data.__setitem__(key, value))

    def __delitem__(self, key: str) -> None:
        self._write(lambda data: data.__delitem__(key))

    def __iter__(self) -> Iterator[str]:
        return iter(self.copy())

    def __len__(self) -> int:
        return len(self.copy())

    def __contains__(self, key: object) -> bool:
        return key in self.copy()

    def update(self, *args: Any, **kwargs: Any) -> None:
        """Several keys in one commit (MutableMapping's would be one each)."""
        changes = dict(*args, **kwargs)
        self._write(lambda data: data.update(changes))

    def keys(self):
        return self.copy().keys()

    def values(self):
        return self.copy().values()

    def items(self):
        return self.copy().items()

    def __repr__(self) -> str:
        return f"SharedVariables({self.copy()!r})"
