# legged-robots-manipulation
## Introduction
legged-robots-manipulation is a loco-manipulation repository for (wheel-)legged robots. The code is built on  [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master). 

*Project Page*:[wheel-legged-loco-manipulation](https://acodedog.github.io/wheel-legged-loco-manipulation/) (IROS Oral 2024)

The current repository contains airbot ,go2_arx,b2w_z1,aliengo_z1 and b2w. The current repository is a partial implementation of the paper.The repository is still under construction for various reasons, and will be releasing issac lab versions and vision-based versions in the near future.

### To do list

- [x] legged-robots-manipulation for isaacgym 
- [ ] legged-robots-manipulation for isaaclab
- [ ] end2end vision-based loco-manipulation method

### airbot

![loco-manipulation](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/loco_manipulation_gym/resources/pictures/airbot_demo.gif)

### go2_arx

![loco-manipulation](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/loco_manipulation_gym/resources/pictures/go2_arx.gif)
### aliengo_Z1
![loco-manipulation](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/loco_manipulation_gym/resources/pictures/aliengo_z1.gif)
### B2W_Z1
### b2w
b2w is a locomotion demo for the Unitree b2w robot.
![locomotion](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/loco_manipulation_gym/resources/pictures/b2w_demo.gif)

## Installation

### install isaac_gym,rsl_rl,legged_gym

1. You are supposed to install the isaac_gym,rsl_rl and legged_gym.Please install the above dependencies ([Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym), [rsl_rl](https://github.com/leggedrobotics/rsl_rl), [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master)) as described in [legged_gym](https://github.com/leggedrobotics/legged_gym/tree/master).




### install legged-robots-manipulation


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

# Citing

If you use this work, please cite [this paper](https://arxiv.org/abs/2403.16535):

```text
@INPROCEEDINGS{10802062,
  author={Wang, Zifan and Jia, Yufei and Shi, Lu and Wang, Haoyu and Zhao, Haizhou and Li, Xueyang and Zhou, Jinni and Ma, Jun and Zhou, Guyue},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Arm-Constrained Curriculum Learning for Loco-Manipulation of a Wheel-Legged Robot}, 
  year={2024},
  volume={},
  number={},
  pages={10770-10776},
  doi={10.1109/IROS58592.2024.10802062}}

```
# IEEE concur reimbursement process reference:
  [IEEE_concur_reimbursement_process.pdf](https://github.com/aCodeDog/legged-robots-manipulation/blob/master/loco_manipulation_gym/resources/ref/IEEE_concur_reimbursement_process.pdf)
# Acknowledgements
This code builds upon following open-source code-bases. Please visit the URLs to see the respective LICENSES:

1. https://github.com/leggedrobotics/legged_gym/tree/master
2. https://github.com/aCodeDog/awesome-loco-manipulation
3. https://github.com/MarkFzp/Deep-Whole-Body-Control
