# CPU monitoring utility for containerized workloads
import time
import threading
import csv
import os

try:
    import psutil
    PSUTIL_AVAILABLE = True
except Exception:
    PSUTIL_AVAILABLE = False


class ProcessTreeMonitor:
    """Monitors CPU usage of a process and optionally its descendants."""
    
    def __init__(self, root_pid, include_descendants=True):
        self.root_pid = root_pid
        self.include_descendants = include_descendants
        self.root_process = None
        
        if PSUTIL_AVAILABLE and root_pid is not None:
            try:
                self.root_process = psutil.Process(root_pid)
            except:
                self.root_process = None
    
    def initialize_monitoring(self):
        """Prime CPU measurements by calling once before actual monitoring."""
        if not PSUTIL_AVAILABLE or not self.root_process:
            return
        
        try:
            self.root_process.cpu_percent(interval=None)
            if self.include_descendants:
                for child in self.root_process.children(recursive=True):
                    try:
                        child.cpu_percent(interval=None)
                    except:
                        pass
        except:
            pass
    
    def measure_tree_cpu(self):
        """Returns cumulative CPU percentage across process tree."""
        if not PSUTIL_AVAILABLE or not self.root_process:
            return 0.0
        
        total_cpu = 0.0
        try:
            # Measure root process
            total_cpu = self.root_process.cpu_percent(interval=None)
            
            # Add descendant processes if enabled
            if self.include_descendants:
                descendant_list = self.root_process.children(recursive=True)
                for descendant in descendant_list:
                    try:
                        total_cpu += descendant.cpu_percent(interval=None)
                    except:
                        # Process may have terminated
                        continue
        except:
            # Root process may have terminated
            return 0.0
        
        return total_cpu


class CPUSampler(threading.Thread):
    """Background thread that periodically samples system and process CPU."""
    
    def __init__(self, sample_interval=0.05, target_pid=None, aggregate_children=True):
        super().__init__()
        self.interval = sample_interval
        self.samples = []
        self._stop = threading.Event()
        self.daemon = True
        
        # Create process tree monitor
        self.process_monitor = ProcessTreeMonitor(target_pid, aggregate_children)
    
    def run(self):
        """Main sampling loop."""
        # Initialize measurements
        if PSUTIL_AVAILABLE:
            psutil.cpu_percent(interval=None)
            self.process_monitor.initialize_monitoring()
        
        # Sampling loop
        while not self._stop.is_set():
            timestamp = time.time()
            
            if PSUTIL_AVAILABLE:
                system_cpu = psutil.cpu_percent(interval=None)
                process_cpu = self.process_monitor.measure_tree_cpu()
            else:
                system_cpu = 0.0
                process_cpu = 0.0
            
            self.samples.append((timestamp, system_cpu, process_cpu))
            time.sleep(self.interval)
    
    def stop(self):
        """Request thread to stop."""
        self._stop.set()
    
    def to_csv(self, path):
        """Export samples to CSV file."""
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        
        with open(path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "cpu_system_percent", "cpu_process_percent"])
            for sample in self.samples:
                writer.writerow(sample)