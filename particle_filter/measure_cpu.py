# Container-aware CPU sampler that records system and per-process CPU%.
import time
import threading
import csv
import os

try:
    import psutil
except Exception:
    psutil = None

class CPUSampler(threading.Thread):
    def __init__(self, sample_interval=0.05, target_pid=None):
        super().__init__()
        self.interval = sample_interval
        self.samples = []  # (timestamp, sys_pct, proc_pct)
        self._stop = threading.Event()
        self.daemon = True
        self.target_pid = target_pid
        self._proc = None
        if psutil and target_pid is not None:
            try:
                self._proc = psutil.Process(target_pid)
            except Exception:
                self._proc = None

    def run(self):
        if psutil:
            psutil.cpu_percent(None)
            if self._proc:
                self._proc.cpu_percent(None)
        while not self._stop.is_set():
            ts = time.time()
            if psutil:
                sys_pct = psutil.cpu_percent(interval=None)
                proc_pct = self._proc.cpu_percent(interval=None) if self._proc else 0.0
            else:
                sys_pct = 0.0
                proc_pct = 0.0
            self.samples.append((ts, sys_pct, proc_pct))
            time.sleep(self.interval)

    def stop(self):
        self._stop.set()

    def to_csv(self, path):
        parent = os.path.dirname(path)
        if parent and not os.path.exists(parent):
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "cpu_system_percent", "cpu_process_percent"])
            for row in self.samples:
                w.writerow(row)