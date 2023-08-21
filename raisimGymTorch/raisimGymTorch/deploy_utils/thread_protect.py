import concurrent.futures
import threading
import time
import numpy as np
from raisimGymTorch.deploy_log.csv_saver import CSV_saver
def cal():
    print('calculating model')
    time.sleep(0.3)


def save(saver:CSV_saver, data):
    print('add data')
    saver.add_list(data)


if __name__=='__main__':
    print('start')

    csv = CSV_saver('./test.csv')

    for i in range(100):
        data = np.random.rand(10)
        cal()
        print(data)
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as exec:
            exec.submit(save, data)
        print('running robot')