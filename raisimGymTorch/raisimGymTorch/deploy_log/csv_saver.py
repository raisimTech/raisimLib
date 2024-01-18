import pandas as pd
import numpy as np

class CSV_saver:
    def __init__(self, name, path=None):
        self.name = name
        self.path = name+'.csv' if path == None else path+name+'.csv'
        self.csv_list = []

    def add_list(self, l):
        self.csv_list.append(l.copy())

    def save(self):
        now = np.stack(self.csv_list)
        now = now.transpose()

        now = pd.DataFrame(now)
        now.to_csv(self.path)
        print(f"{self.name} has been saved to {self.path}")

if __name__ == "__main__":
    obs = CSV_saver('real_obs')
    for i in range(133):
        obs.add_list(np.array([x for x in range(17)]))

    obs.save()
