import threading
import threading
import queue
import itertools

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
        self.worker_fns = worker_fns
        self.results = []
        self.lock = threading.Lock()
        self.workers = []
        self.task_queues = [queue.Queue() for _ in worker_fns]
        self.stop_event = threading.Event()


    def worker(self, fcn_index):
        while not self.stop_event.is_set():
            try:
                task = self.task_queues[fcn_index].get(timeout=5)
                if task is None:
                    break

                func = self.worker_fns[fcn_index]
                if isinstance(task, tuple) and task[0] == 'planner':
                    result = func()
                elif isinstance(task, tuple):
                    result = func(task)
                elif func.__code__.co_argcount == 0:
                    result = func()
                else:
                    result = func(task)

                with self.lock:
                    self.results.append(result)
                self.task_queues[fcn_index].task_done()
            except queue.Empty:
                continue
    def task_dispatcher(self, iterable):
        for item, queue in zip(iterable, itertools.cycle(self.task_queues)):
            queue.put(item)

    def map(self, iterable):
        self.results = []
        self.workers = []
        self.stop_event.clear()
        
        for i in range(len(self.worker_fns)):
            for _ in range(self.num_threads // len(self.worker_fns)):
                t = threading.Thread(target=self.worker, args=(i,))
                t.start()
                self.workers.append(t)

        dispatcher = threading.Thread(target=self.task_dispatcher, args=(iterable,))
        dispatcher.start()
        for q in self.task_queues:
            q.join()

        dispatcher.join()
        return self.results


    def close(self):
        self.stop_event.set()
        for q in self.task_queues:
            q.put(None)


    def join(self):
        for worker in self.workers:
            worker.join()