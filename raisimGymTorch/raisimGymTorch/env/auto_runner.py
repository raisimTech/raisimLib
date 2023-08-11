from ruamel.yaml import YAML, dump, RoundTripDumper
import os

# file = open('/home/lr-2002/code/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_go1/cfg.yaml')
def change_data(path, flag, data):
    y = YAML().load(open(path, 'r'))
    for a,b in zip(flag, data):
        y['environment'][a] = b
    # wr = yaml.dump(y)
    print(y)
    with open(path, 'w') as f :
        dump(y, f, Dumper=RoundTripDumper)

test_list = [x/10 for x in range(1,20,4)]
test_T = [40]
# print(test_list)
length = 500
for j in test_T:
    for i in test_list:
        change_data('/home/lr-2002/code/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg/cfg.yaml',['action_std','schedule'], [i, j])
        if i == 0:
            continue
        else:
            os.system(f'python /home/lr-2002/code/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg/runner.py -u {length} |grep biggest' )

# os.system('python /home/lr-2002/code/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_go1/runner.py -u 3 |grep biggest' )
