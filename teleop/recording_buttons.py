"""Controller-event edge detection, including short presses between polls."""
import multiprocessing as mp


class RecordingButtons:
    def __init__(self, capacity=64):
        self.capacity = capacity
        self.operations = mp.Array("i", capacity, lock=False)
        self.times = mp.Array("d", capacity, lock=False)
        self.serial = mp.Value("Q", 0)
        self.armed = False  # owned by the WSS child, not read by its parent
        self.previous = (False, False, False)
        self.consumed = 0  # owned by the Quest parent

    def update(self, left, right, now, valid=True):
        if not valid:
            self.armed = False
            self.previous = (False, False, False)
            return
        current = (bool(right.get("aButton", False)), bool(right.get("bButton", False)),
                   bool(left.get("aButton", False)))
        if not self.armed:
            self.armed = not any(current)
        else:
            # Simultaneous presses: discard takes priority, then stop, then A.
            rising = [k for k in (2, 1, 0) if current[k] and not self.previous[k]]
            if rising:
                i = self.serial.value % self.capacity
                self.operations[i] = rising[0]
                self.times[i] = now
                self.serial.value += 1
        self.previous = current

    def drain(self, now, timeout=.5):
        serial = self.serial.value
        events = []
        for k in range(max(self.consumed, serial - self.capacity), serial):
            i = k % self.capacity
            if 0 <= now - self.times[i] <= timeout:
                events.append(("a", "b", "x")[self.operations[i]])
        self.consumed = serial
        return events

