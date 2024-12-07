import threading
import threading
import queue

class ThreadPoolManager:
    def __init__(self, num_threads, worker_fn=None, planner=False, planner_fnc=None):
        self.num_threads = num_threads
        self.worker_fn = worker_fn
        self.planner = planner
        self.planner_fnc = planner_fnc

    def __enter__(self):
        if self.planner:
            worker_fns = [self.worker_fn]*self.num_threads + [self.planner_fnc]
            self.pool = ThreadPool(num_threads=self.num_threads + 1, worker_fns=worker_fns)
        else:
            self.pool = ThreadPool(num_threads=self.num_threads, worker_fns=[self.worker_fn])
        return self.pool
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pool.close()
        self.pool.join()

class ThreadPool:
    def __init__(self, num_threads, worker_fns=None):
        self.num_threads = num_threads
        self.tasks = queue.Queue()
        self.results = []
        self.lock = threading.Lock()
        self.workers = []
        self.worker_fns = worker_fns

    def worker(self, fcn_index):
        while True:
            task = self.tasks.get()
            if task is None:
                self.tasks.task_done()
                break
            if task == 'planner':
                result = self.worker_fns[-1]()
            else:
                result = self.worker_fns[fcn_index](task)

            with self.lock:
                self.results.append(result)
            self.tasks.task_done()

    def map(self, iterable):
        self.results = []
        self.workers = []
        for i in range(self.num_threads):
            t = threading.Thread(target=self.worker, args=(i % len(self.worker_fns),))
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