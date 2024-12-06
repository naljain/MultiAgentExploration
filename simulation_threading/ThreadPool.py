import threading
import threading
import queue

class ThreadPoolManager:
    def __init__(self, num_threads, worker_fn=None, planner=False, planner_fnc=None):
        if planner:
            num_threads += 1
            worker_fns = [worker_fn, planner_fnc]
            self.pool = ThreadPool(num_threads=num_threads, worker_fn=worker_fns)
        else:
            self.pool = ThreadPool(num_threads=num_threads, worker_fn=[worker_fn])

    def __enter__(self):
        return self.pool
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pool.close()
        self.pool.join()

class ThreadPool:
    def __init__(self, num_threads, worker_fn=None):
        self.num_threads = num_threads
        self.tasks = queue.Queue()
        self.results = []
        self.lock = threading.Lock()
        self.workers = []
        self.worker_fn = worker_fn

    def worker(self, fcn_index):
        while True:
            task = self.tasks.get()
            if task is None:
                self.tasks.task_done()
                break
            result = self.worker_fn[fcn_index](task)
            with self.lock:
                self.results.append(result)
            self.tasks.task_done()

    def map(self, func, iterable):
        self.results = []
        self.workers = []
        if len(self.worker_fn) == 1:
            for _ in range(self.num_threads):
                t = threading.Thread(target=self.worker, args=(0,))
                t.start()
                self.workers.append(t)
        else:
            for _ in range(self.num_threads-1):
                t = threading.Thread(target=self.worker, args=(0,))
                t.start()
                self.workers.append(t)
            t = threading.Thread(target=self.worker, args=(1,))
            t.start()
            self.workers.append(t)

        for item in iterable:
            self.tasks.put(item)
        
        self.tasks.join()
        return self.results
    
    def close(self):
        for _ in range(self.num_threads):
            self.tasks.put(None)

    def join(self):
        for worker in self.workers:
            worker.join()