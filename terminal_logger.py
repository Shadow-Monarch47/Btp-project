import sys
import os

class TerminalLogger:
    def __init__(self, logfile_path):
        self.terminal = sys.stdout
        self.log = open(logfile_path, "a", buffering=1)

    def write(self, message):
        self.terminal.write(message)

        self.log.write(message)
        self.log.flush()                  # Python buffer → OS
        os.fsync(self.log.fileno())       # OS buffer → disk

    def flush(self):
        self.terminal.flush()
        self.log.flush()
        os.fsync(self.log.fileno())
