"""Disk writes off the ROS callback thread.

A slow write (SD card, journal flush) must not stall the executor: while it
blocks, best-effort sensor messages queue up and are dropped, which looks like
a telemetry gap. Items are written in order by one background thread. A write
failure or a full queue is reported to the caller on the next put(); nothing is
dropped silently.
"""
import queue
import threading

_STOP = object()


class AsyncWriter:
    def __init__(self, write, max_queue=50000, name='blueboat-writer'):
        self._write = write
        self._queue = queue.Queue(max_queue)
        self.error = None
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def put(self, item):
        if self.error is not None:
            raise RuntimeError(self.error)
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.error = 'log_queue_full'
            raise RuntimeError(self.error)

    def _run(self):
        while True:
            item = self._queue.get()
            if item is _STOP:
                return
            if self.error is None:
                try:
                    self._write(item)
                except Exception as error:  # reported through put()
                    self.error = 'log_write_failed: ' + str(error)

    def close(self, timeout=10.):
        """Write everything queued so far, then stop the thread."""
        self._queue.put(_STOP)
        self._thread.join(timeout)
