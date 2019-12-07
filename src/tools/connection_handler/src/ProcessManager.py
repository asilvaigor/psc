import psutil, os, signal


class ProcessManager:
    def __init__(self, process):
        self.process = process

    def close_all_child(self):
        # Getting child processes
        children_pids = [x.pid for x in psutil.Process(self.process.pid).children(recursive=True)]

        # Killing all the child processes
        for pid in children_pids:
            os.killpg(pid, signal.SIGINT)

    def sigint(self):
        # Sending sigint signal
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
