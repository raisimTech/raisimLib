

# import threading
#
#
# class Test(object):
#
#     def __init__(self, n=2):
#         self.a = 0
#         self.results = [-1 for _ in range(n)]
#         self.threads = [threading.Thread(target=self.step_thread) for _ in range(n)]
#         for idx, thread in enumerate(self.threads):
#             thread.idx = idx
#         self.b = 0
#
#     def step(self):
#         for thread in self.threads:
#             thread.start()
#         for thread in self.threads:
#             thread.join()
#
#     def step_thread(self):
#         self.a += 1
#         self.b += 1
#         idx = threading.current_thread().idx
#         print("Thread {}: a={}, b={}".format(idx, self.a, self.b))
#         self.results[idx] = idx
#
#
# t = Test(n=2)
# t.step()
# t.step()
#
# print("Results: {}".format(t.results))

from multiprocessing.pool import ThreadPool
from algos import PPO2
from envs.anymal import ANYMAL_RESOURCE_DIRECTORY as RSCDIR


class Test(object):

    def __init__(self, n=2):
        self.a = 0
        self.n = n
        self.threads = ThreadPool(processes=n)
        self.b = 0

    def step(self):
        # results = self.threads.apply(self.step_thread)
        results = self.threads.map(self.step_thread, [i for i in range(self.n)])
        return results

    def step_thread(self, idx=None):
        self.a += 1
        self.b += 1
        print("Thread {}: a={}, b={}".format(idx, self.a, self.b))


t = Test(n=2)
t.step()
