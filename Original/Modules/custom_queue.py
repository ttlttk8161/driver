# custom_queue.py
# 커스터마이즈 큐 모듈 (인터페이스 설계)

import queue
import threading

class CustomQueue:
    def __init__(self, maxsize=10):
        self._queue = queue.Queue(maxsize=maxsize)
        self._lock = threading.Lock()

    def put(self, item, block=True, timeout=None):
        # 예외/로깅/통계 등 커스터마이즈 가능
        return self._queue.put(item, block=block, timeout=timeout)

    def get(self, block=True, timeout=None):
        return self._queue.get(block=block, timeout=timeout)

    def qsize(self):
        return self._queue.qsize()

    def empty(self):
        return self._queue.empty()

    def full(self):
        return self._queue.full()

    def clear(self):
        with self._lock:
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break

    # 필요시 추가 인터페이스: peek, flush, stats 등
