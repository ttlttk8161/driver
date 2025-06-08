import threading

class ThreadManager:
    def __init__(self):
        self.threads = []

    def add_thread(self, target, args=()):
        t = threading.Thread(target=target, args=args)
        t.daemon = True
        self.threads.append(t)
        return t

    def start_all(self):
        for t in self.threads:
            t.start()

    def join_all(self):
        for t in self.threads:
            t.join()
