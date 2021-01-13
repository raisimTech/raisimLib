## raisim_env_anymal

### How to use this repo
There is nothing to compile here. This only provides a header file for an environment definition. Read the instruction of raisimGym. 

### Dependencies
- raisimgym

### Run
1. Compile raisimgym: ```python setup develop```
2. run runner.py of the task (for anymal example): ```cd raisimGymTorch/env/envs/rsg_anymal && python ./runner.py```

### Test policy
1. Compile raisimgym: ```python setup develop```
2. run tester.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Retrain policy
1. run runner.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/runner.py --mode retrain --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Debugging
1. Compile raisimgym with debug symbols: ```python setup develop --Debug```. This compiles <YOUR_APP_NAME>_debug_app
2. Run it with Valgrind. I strongly recommend using Clion for 
