import queue
import threading
import logging
import time

# thread_queue_manager.py
# 예외 발생 시 큐/스레드 관리 보강용 인터페이스 설계

class ThreadQueueManager:
    def __init__(self, maxsize=10, name="ThreadQueue"):
        self._queue = queue.Queue(maxsize=maxsize)
        self._lock = threading.Lock()
        self._name = name
        self._latest_timestamp = None
        self.logger = logging.getLogger(f"{self._name}")

    def put(self, item, block=True, timeout=None):
        """
        데이터의 timestamp가 self._latest_timestamp보다 과거면 무시, 최신이면 큐에 삽입.
        큐가 가득 찼을 때는 가장 오래된 데이터 자동 폐기 후 삽입.
        큐가 full이 아니어도, 현재 넣으려는 데이터보다 0.5초 이상 차이나는(오래된) 데이터는 폐기.
        """
        ts = getattr(item, 'timestamp', None)
        with self._lock:
            if ts is not None:
                if self._latest_timestamp is not None and ts <= self._latest_timestamp:
                    self.logger.debug(f"{self._name}: 과거 데이터(timestamp={ts}) 무시")
                    return False
                # 큐 내부에 0.5초 이상 오래된 데이터 폐기
                temp_items = []
                while not self._queue.empty():
                    try:
                        q_item = self._queue.get_nowait()
                        q_ts = getattr(q_item, 'timestamp', None)
                        if q_ts is not None and ts - q_ts > 0.5:
                            self.logger.debug(f"{self._name}: 0.5초 이상 오래된 데이터(timestamp={q_ts}) 폐기")
                            continue
                        temp_items.append(q_item)
                    except queue.Empty:
                        break
                # 남은 데이터만 다시 큐에 삽입
                for q_item in temp_items:
                    self._queue.put(q_item)
                self._latest_timestamp = ts
            if self._queue.full():
                try:
                    dropped = self._queue.get_nowait()
                    self.logger.warning(f"{self._name}: 큐가 가득 차서 가장 오래된 데이터(timestamp={getattr(dropped, 'timestamp', None)})를 폐기함.")
                except queue.Empty:
                    pass
            self._queue.put(item, block=block, timeout=timeout)
            return True

    def get(self, block=True, timeout=None):
        """
        큐에서 데이터 추출. (최신성 보장은 put에서 처리)
        """
        return self._queue.get(block=block, timeout=timeout)

    def task_done(self):
        """
        큐에서 get()한 작업이 완료되었음을 알림 (queue.Queue와 호환)
        """
        self._queue.task_done()

    def join(self):
        """
        큐의 모든 작업이 완료될 때까지 블록 (queue.Queue와 호환)
        """
        self._queue.join()

    def qsize(self):
        return self._queue.qsize()

    def empty(self):
        return self._queue.empty()

    def full(self):
        return self._queue.full()

    def clear_queue(self):
        """
        큐 객체를 안전하게 비움.
        """
        with self._lock:
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            self.logger.info(f"{self._name}: 큐를 비움.")

    def handle_exception(self, module_name, exception):
        self.logger.error(f"[{module_name}] 예외 발생: {exception}. 큐를 비웁니다.")
        self.clear_queue()
        # 필요시 스레드 재시작 등 추가 정책 구현 가능

    def restart_thread(self, thread_obj):
        # 스레드 재시작 로직 (필요시 구현)
        self.logger.warning(f"{self._name}: 스레드 재시작 로직은 미구현.")
        pass

    def monitor(self):
        # 주기적으로 큐/스레드 상태 모니터링 (옵션)
        self.logger.info(f"{self._name}: 큐 상태 모니터링 - 크기: {self.qsize()}")
        pass

# 실제 구현은 시스템 정책에 맞게 확장
