"""Read f without Enter, preserving Ctrl+C and restoring terminal settings."""
import os
import select
import sys
import termios
import time


class TerminalFailureKey:
    def __init__(self, stream=None):
        self.fd=None
        self.previous=None
        try:
            self.fd=(sys.stdin if stream is None else stream).fileno()
            if os.isatty(self.fd):
                self.previous=termios.tcgetattr(self.fd)
                current=termios.tcgetattr(self.fd)
                current[3] &= ~(termios.ICANON | termios.ECHO)
                # Keep ISIG: Ctrl+C still reaches the application's signal handler.
                current[6][termios.VMIN]=1
                current[6][termios.VTIME]=0
                termios.tcsetattr(self.fd,termios.TCSANOW,current)
        except (AttributeError,OSError,ValueError):
            self.fd=None

    def poll(self, active):
        pressed=False
        if self.fd is None:
            return False
        while select.select([self.fd],[],[],0)[0]:
            data=os.read(self.fd,4096)
            if not data:
                break
            pressed |= b'f' in data.lower()
        # Keys typed during HOME/reset are consumed, never queued for the next trial.
        return bool(active and pressed)

    def close(self):
        if self.previous is not None:
            termios.tcsetattr(self.fd,termios.TCSANOW,self.previous)
            self.previous=None


def wait_prediction(future, keys, stop_requested, manual_failure):
    """Keep terminal failure responsive even during a long model inference.

    An abandoned response belongs to this future only; never apply it to the
    next trial. A single inference worker preserves the socket request order.
    """
    while True:
        if stop_requested():
            future.cancel()
            return None
        if keys.poll(active=True):
            manual_failure()
            future.cancel()
            return None
        if future.done():
            return future.result()
        time.sleep(.01)
