"""Deliberate faults, for testing how the RSI link copes with them.

Nothing in RSIPI calls this. It is for tests and bring-up scripts that need a
stall on demand - the kind a GC pause or a busy scheduler produces unasked.

    from RSIPI.fault_injection import suspended

    with suspended(api.client.network_process.pid):
        time.sleep(0.1)          # no replies for 100 ms
"""
import contextlib
import os
import sys

__all__ = ["suspended"]


@contextlib.contextmanager
def suspended(pid):
    """Freeze process *pid* for the duration of the block.

    A frozen RSIPI network process sends no replies at all, exactly as if the
    machine had stalled; its queued robot packets are answered once it
    resumes. The process is resumed on exit even if the block raises.

    Windows uses NtSuspendProcess, which needs ctypes (some application
    control policies block it); elsewhere SIGSTOP/SIGCONT.
    """
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes

        process_suspend_resume = 0x0800
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        ntdll = ctypes.WinDLL("ntdll")
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
        kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
        ntdll.NtSuspendProcess.argtypes = (wintypes.HANDLE,)
        ntdll.NtResumeProcess.argtypes = (wintypes.HANDLE,)

        handle = kernel32.OpenProcess(process_suspend_resume, False, pid)
        if not handle:
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            status = ntdll.NtSuspendProcess(handle)
            if status:
                raise OSError(f"NtSuspendProcess failed with NTSTATUS {status & 0xFFFFFFFF:#010x}")
            try:
                yield
            finally:
                ntdll.NtResumeProcess(handle)
        finally:
            kernel32.CloseHandle(handle)
    else:
        import signal

        os.kill(pid, signal.SIGSTOP)
        try:
            yield
        finally:
            os.kill(pid, signal.SIGCONT)
