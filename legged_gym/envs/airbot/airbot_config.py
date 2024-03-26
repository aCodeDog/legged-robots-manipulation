
import sys
import os
from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import numpy as np
class AirbotRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 6000
        num_actions = 12
        num_observations = 73
    class commands( LeggedRobotCfg ):
        obj_global = False
        locomotion_only = False
        curriculum = True
        max_curriculum = 2.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [0.5, 0.5] # min max [m/s]
            lin_vel_y = [-0., 0.]   # min max [m/s]
            ang_vel_yaw = [-0.5,0.5]    # min max [rad/s]
            heading = [-3.14, 3.14]
            obj_pos_x=[1.5,3]
            obj_pos_y=[-0.1,0.1]
            obj_pos_z=[0.1,0.6]
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.001 # [m]
        border_size = 25 # [m]
        curriculum = True
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measure_heights = False
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows= 10 # number of terrain rows (levels)
        num_cols = 20 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0, 0, 1, 0.0, 0.0]
        # trimesh only:
        slope_treshold = 0.2 # slopes above this threshold will be corrected to vertical surfaces
   
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.284] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'right_ankle': 0.,   # [rad]
            'right_knee': -1.54,   # [rad]
            'right_hip': 0.88 ,  # [rad]

            'left_ankle': 0.,   # [rad]
            'left_knee': -1.54,   # [rad]
            'left_hip': 0.88 ,  # [rad]
            
            'arm_joint00':0.,           
            'arm_joint01':0.,
            'arm_joint02':0.,
            'arm_joint03':0.,
            'arm_joint04':0.,
            'arm_joint05':0.,

        }
        init_joint_angles = { # = target angles [rad] when action = 0.0
            'right_ankle': 0.0,   # [rad]
            'right_knee': -1.54,   # [rad]
            'right_hip': 0.88 ,  # [rad]

            'left_ankle': 0.0,   # [rad]
            'left_knee': -1.54,   # [rad]
            'left_hip': 0.88 ,  # [rad]

            
            'arm_joint00':0.,           
            'arm_joint01':0.,
            'arm_joint02':0.,
            'arm_joint03':0.,
            'arm_joint04':0.,
            'arm_joint05':0.,

        }
    class rewards:
        class scales:
            termination = -1
            tracking_lin_vel = 3.0
            tracking_ang_vel = 0.5
            lin_vel_rate = -0.
            lin_vel_z = -0.02
            ang_vel_xy = -0.02
            orientation = -0.1
            torques = -0.0001
            dof_vel = -2.5e-5
            dof_acc = -2.5e-7
            base_height = -0.5
            feet_air_time =  0.
            collision = -0.5
            feet_stumble = -0.01
            action_rate = -0.001
            stand_still = -0.1
            dof_pos_limits = -0.4
            arm_pos = -0.
            heading = -0.
            body_to_obj = 0.#0.05
            object_distance = 2
            joint_pos_rate = -0.03
            dof_action_pos_limits = -0.01
            hip_action = -0.01
            gripper_vel = -0.1
        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.4 # tracking reward = exp(-error^2/sigma)
        object_sigma = 0.4
        dist_err_sigma = 2
        soft_dof_pos_limit = 0.9 # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 1.
        base_height_target = 0.284
        max_contact_force = 100. # forces above this value are penalized

    class goal_ee:
        init_local_cube_object_pos = [0.5,0,0.35]
    
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        arm_control_type='position'
        stiffness = {'joint': 10.,"ankle":10,"knee":10,"hip":10}  # [N*m/rad]
        damping = {'joint': 0.1,"ankle":0.2,"knee":0.2,"hip":0.2}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        wheel_speed = 0. # [rad/s]
        task = "rough_terrain"

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/airbot/urdf/airbot.urdf'
        name = "airbot"
        foot_name = "wheel"  #link name
        arm_name = ["joint" ] #joint name
        penalize_contacts_on = ["base","knee","hip"]  #link name
        terminate_after_contacts_on = []  #link name
        wheel_name =[ "ankle"] #wheel joints name, joint name
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter "base","calf","hip","thigh"
        flip_visual_attachments = False

class AirbotRoughCfgPPO( LeggedRobotCfgPPO ):
    seed = 21
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        learning_rate = 1.e-3
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'airbot_0'
        resume = False
        num_steps_per_env = 60 # per iteration
        max_iterations = 1000
        save_interval =100
        load_run = -1 #'/path/to/pretrained_model' -1 = last run
        load_behaviour_cloning = False
        behaviour_cloning_path = ''#/home/wang/isaac_gym/legged_gym/legged_gym/envs/airbot/airbot_pretrained_parameters_legged_gym.pth
        checkpoint = 0
        train =True
    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu'   # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid