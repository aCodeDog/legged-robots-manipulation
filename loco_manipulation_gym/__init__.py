import os

LOCO_MANI_GYM_ROOT_DIR_PRE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LOCO_MANI_GYM_ROOT_DIR =  os.path.join(LOCO_MANI_GYM_ROOT_DIR_PRE, 'loco_manipulation_gym')
LOCO_MANI_GYM_ENVS_DIR = os.path.join(LOCO_MANI_GYM_ROOT_DIR, 'loco_manipulation_gym', 'envs')
