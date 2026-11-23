"""Each CSV log row carries the packet's arrival time, not just the write time.

The Timestamp column is taken by the logger thread when it writes the row, which
can be well after the packet arrived. RxTimeNs is perf_counter_ns() at arrival,
for lining the robot log up with other loggers on the same PC.
"""
import queue
import time
from types import SimpleNamespace

from RSIPI.network_handler import NetworkProcess


def queued_entry(rx_ns):
    fake = SimpleNamespace(log_queue=queue.Queue())
    NetworkProcess._queue_log_entry(fake, {"IPOC": 1004, "RIst": {"X": 1.5}},
                                    {"IPOC": 1004, "RKorr": {"X": 0.0}}, rx_ns)
    return fake.log_queue.get_nowait()


def test_rx_time_is_the_first_column():
    t = time.perf_counter_ns()
    entry = queued_entry(t)
    assert list(entry)[0] == "RxTimeNs"
    assert entry["RxTimeNs"] == t
    assert entry["Send.IPOC"] == 1004
    assert entry["Send.RIst.X"] == 1.5
    assert entry["Receive.RKorr.X"] == 0.0


def test_rx_time_defaults_to_empty():
    assert queued_entry(None)["RxTimeNs"] is None
