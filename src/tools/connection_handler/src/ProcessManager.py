import psutil, os, signal


class ProcessManager:
    def __init__(self, process):
        self.process = process

    def close_all_child(self):
        if self.process is None or self.process.poll() is not None:
            return

        # Getting child processes
        children_pids = [x.pid for x in psutil.Process(self.process.pid).children(recursive=True)]

        # Killing all the child processes
        for pid in children_pids:
            os.killpg(pid, signal.SIGINT)

    def sigint(self):
        if self.process is None or self.process.poll() is not None:
            return

        # Sending sigint signal
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
