import re
import pandas as pd
from draw_map import Drawer
ans_list = []
import numpy as np
drawer = Drawer('tt')
# with open("./best.log", 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         # x = re.search("work_action", line)
#         # print(x)
#         if "average" in line:
#             line = line.strip()
#             # print(line[-20:])
#             x = float(line[-21:])
#             ans_list.append(x)
#             drawer.add_map_list([x])

# cont = [27.41,24.57,69.16,38.64]

with open("./tmp.log", 'r') as f:
    lines = f.readlines()
    for line in lines:
        # x = re.search("work_action", line)
        # print(x)
        if "est vel [" in line:
            print("line ",line)
            # print(line)
            line = line.strip().split('l [')[1][1:-1].split(' ')
            x = []
            it =0
            for i in line:
                if i == '' :
                    continue
                i = float(i.strip())
                x.append(i)
            # print(line[-20:])
            # x = list(line[-21:])
            print(x)
            ans_list.append(x)
            drawer.add_map_list(x)

drawer.draw(['x','y','z'])
df = np.array(ans_list)
dd = pd.DataFrame(df).to_csv('./foot.csv')


