import re
import pandas as pd
ans_list = []
import numpy as np
with open("./train.log", 'r') as f:
    lines = f.readlines()
    for line in lines:
        # x = re.search("work_action", line)
        # print(x)
        if "average" in line:
            line = line.strip()
            # print(line[-20:])
            x = float(line[-20:])
            ans_list.append(x)


df = np.array(ans_list)
dd = pd.DataFrame(df).to_csv('./tss_trot_wheel.csv')


