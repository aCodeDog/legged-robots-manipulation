# legged-robots-manipulation
## Introduction
legged-robots-manipulation is a loco-manipulation repository for (wheel-)legged robots. The code is built on  [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master). We provide two pre-trained models for airbot and b2w.

*Project Page*:[wheel-legged-loco-manipulation](https://acodedog.github.io/wheel-legged-loco-manipulation/)

The current repository contains airbot and b2w. The current repository is a partial implementation of the paper. The whole code will be released once the paper is accepted.
### airbot
airbot is a loco-manipulation task implemented using PPO. It is the baseline for comparison in the Paper.
![loco-manipulation](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/resources/pictures/airbot_demo.gif)

### b2w
b2w is a locomotion demo for the Unitree b2w robot.
![locomotion](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/resources/pictures/b2w_demo.gif)

## Installation

### install isaac_gym,rsl_rl,legged_gym

1. You are supposed to install the isaac_gym,rsl_rl and legged_gym.Please install the above dependencies ([Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym), [rsl_rl](https://github.com/leggedrobotics/rsl_rl), [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master)) as described in [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master).




### Copy required files to [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master).

1. Copy the contents of ~/legged-robots-manipulation/legged_gym/envs/__init__.py into [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master).

```
   # Add those contents

import os
from legged_gym.envs.airbot.airbot_config import AirbotRoughCfg, AirbotRoughCfgPPO

from legged_gym.envs.b2w.b2w_config import B2wRoughCfg, B2wRoughCfgPPO


from legged_gym.utils.task_registry import task_registry

from .airbot.airbot_robot import Airbot
from .b2w.b2w_robot import B2w


from .airbot.airbot_config import AirbotRoughCfg, AirbotRoughCfgPPO
from .b2w.b2w_config import B2wRoughCfg, B2wRoughCfgPPO


```

2. Copy envs files into legged_gym/envs corresponding.

```
   # Add those files
cp path/to/legged-robots-manipulation/legged_gym/envs path/to/legged_gym/legged_gym/envs
```


## Train
```
python tran.py --task=b2w 

# or

python train.py --task=airbot  

 ```

## Citing

If you use this work, please cite [this paper](https://arxiv.org/abs/2403.16535):

```text
      @misc{wang2024armconstrained,
        title={Arm-Constrained Curriculum Learning for Loco-Manipulation of the Wheel-Legged Robot}, 
        author={Zifan Wang, Yufei Jia, Lu Shi, Haoyu Wang, Haizhou Zhao, Xueyang Li, Jinni Zhou, Jun Ma and Guyue Zhou},
        year={2024},
        eprint={2403.16535},
        archivePrefix={arXiv},
        primaryClass={cs.RO}
  }
```