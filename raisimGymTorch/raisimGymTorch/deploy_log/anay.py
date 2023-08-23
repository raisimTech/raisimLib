import re
import pandas as pd
from draw_map import Drawer
ans_list = []
import numpy as np
drawer = Drawer('tt')
with open("./tmp.log", 'r') as f:
    lines = f.readlines()
    for line in lines:
        # x = re.search("work_action", line)
        # print(x)
        if "average" in line:
            line = line.strip()
            # print(line[-20:])
            x = float(line[-21:])
            ans_list.append(x)
            drawer.add_map_list([x])

drawer.draw()
df = np.array(ans_list)
dd = pd.DataFrame(df).to_csv('./tmp.csv')


