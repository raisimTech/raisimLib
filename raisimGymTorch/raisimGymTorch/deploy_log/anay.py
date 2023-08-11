import re
with open("./08-11-15-28", 'r') as f:
    lines = f.readlines()
    for line in lines:
        # x = re.search("work_action", line)
        # print(x)
        if "work_action" in line:
            print(line)