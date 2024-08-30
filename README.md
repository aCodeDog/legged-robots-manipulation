# legged-robots-manipulation
## Introduction
legged-robots-manipulation is a loco-manipulation repository for (wheel-)legged robots. The code is built on  [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master). 

*Project Page*:[wheel-legged-loco-manipulation](https://acodedog.github.io/wheel-legged-loco-manipulation/)

The current repository contains airbot ,go2_arx,b2w_z1,aliengo_z1 and b2w. The current repository is a partial implementation of the paper.The repository is still under construction for various reasons, and will be releasing issac lab versions and vision-based versions in the near future.

### Solar System Exploration, 1950s – 1960s

- [x] legged-robots-manipulation for isaacgym 
- [ ] legged-robots-manipulation for isaaclab
- [ ] end2end vision-based loco-manipulation method

### airbot
airbot is a loco-manipulation task implemented using PPO. It is the baseline for comparison in the Paper.
![loco-manipulation](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/resources/pictures/airbot_demo.gif)

### go2_arx
go2_arx is a loco-manipulation task implemented using PPO. It is the baseline for comparison in the Paper.

### aliengo_Z1

### B2W_Z1

### b2w
b2w is a locomotion demo for the Unitree b2w robot.
![locomotion](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/resources/pictures/b2w_demo.gif)

## Installation

### install isaac_gym,rsl_rl,legged_gym

1. You are supposed to install the isaac_gym,rsl_rl and legged_gym.Please install the above dependencies ([Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym), [rsl_rl](https://github.com/leggedrobotics/rsl_rl), [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master)) as described in [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master).




### install legged-robots-manipulation

1. 
```
cd loco_manipulation_gym
pip install -e .

```


## Train
```
python tran.py --task=b2w --rl_device=cuda:0

# or

python train.py --task=airbot  --rl_device=cuda:0 

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
