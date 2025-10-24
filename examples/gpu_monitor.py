import subprocess
import threading
import time
import ipywidgets as widgets
from IPython.display import display, clear_output


class GPUMonitor:
    """Non-blocking real-time GPU usage monitor for Jupyter notebooks."""

    def __init__(self, interval=2.0, low_mem_thred=2000):
        """
        Initialize the GPU monitor.

        Args:
            interval (float): Refresh interval in seconds.
        """
        self.interval = interval
        self.output = widgets.HTML(value="")
        self.thread = None
        self.low_mem_thred = low_mem_thred
        self.low_mem_msg = f"""
        <div style='color:red'>
        ⚠️ Warning: Low available GPU memory (<{self.low_mem_thred:} MB). 
        Simulations may failed to start!
        <div>
        """
        # Display the UI in the notebook
        display(self.output)
        # Start loop
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def _run_command(self, command):
        """
        Run a shell command and return its standard output.
        """
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            return result.stdout.strip()
        except subprocess.CalledProcessError as e:
            return None

    def _get_gpu_stats(self):
        """
        Retrieve GPU usage and memory statistics using nvidia-smi.
        Returns:
            list[dict]: A list of GPU information dictionaries.
        """
        output = self._run_command([
            "nvidia-smi",
            "--query-gpu=index,name,utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits"
        ])
        if not output:
            return []

        gpu_info = []
        for line in output.split("\n"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 5:
                idx, name, util, mem_used, mem_total = parts
                gpu_info.append({
                    "index": int(idx),
                    "name": name,
                    "utilization": int(util),
                    "memory_used": int(mem_used),
                    "memory_total": int(mem_total),
                    "memory_free": int(mem_total) - int(mem_used)
                })
        return gpu_info
        
    def _format_gpu_stats(self, gpus):
        """
        Format GPU statistics into a readable text table and add warnings if memory is low.
        """
        if not gpus:
            return "No GPU detected or nvidia-smi not available."
        lines = []
        low_memory_detected = False
        for g in gpus:
            lines.append(
                f"<b>GPU-Util: {g['utilization']}% | Mem Free: {g['memory_free']} (MiB)</b>"
            )
            
            if g["memory_free"] < self.low_mem_thred:
                low_memory_detected = True

        if low_memory_detected:
            lines.append(self.low_mem_msg)

        return "\n".join(lines)

    def _monitor_loop(self):
        """
        The background thread that continuously updates GPU statistics.
        """
        while True:
            gpus = self._get_gpu_stats()
            text = self._format_gpu_stats(gpus)
            self.output.value = text
            time.sleep(self.interval)
