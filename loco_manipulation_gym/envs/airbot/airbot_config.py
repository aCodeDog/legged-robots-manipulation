
import sys
import os
from loco_manipulation_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import numpy as np
class AirbotRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 57
        symmetric = False  #true :  set num_privileged_obs = None;    false: num_privileged_obs = observations + 187 ,set "terrain.measure_heights" to true
        num_privileged_obs = num_observations + 187 # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds
    class commands( LeggedRobotCfg.commands ):
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-1.0, 1.0] # min max [m/s]
            lin_vel_y = [-0., 0.]   # min max [m/s]
            ang_vel_yaw = [-0., 1.]    # min max [rad/s]
            heading = [-3.14, 3.14]
    class terrain( LeggedRobotCfg.env ):
        mesh_type = 'plane' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = True
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measure_heights = True
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
        terrain_proportions = [0,1, 0, 0, 0]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces

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
    class goal_ee:
        local_axis_z_offset = 0.3
        init_local_cube_object_pos = [0.5,0,0.35]
        num_commands = 3
        traj_time = [0.6, 1.2]
        hold_time = [0.2, 0.4]
        collision_upper_limits = [0.3, 0.15, 0.05 - 0.165]
        collision_lower_limits = [-0.2, -0.15, -0.35 - 0.165]
        underground_limit = -0.57
        num_collision_check_samples = 10
        command_mode = 'cart'
        class ranges:

            init_pos_l = [0.3, 0.6]
            init_pos_p = [-1 * np.pi / 6, 1 * np.pi / 3]
            init_pos_y = [-1 * np.pi / 4, 1 * np.pi / 4]
            final_delta_orn = [[-0, 0], [-0, 0], [-0, 0]]

        class init_ranges:
            pos_l = [0.3, 0.5] # min max [m/s]
            pos_p = [np.pi / 4, 3 * np.pi / 4]   # min max [m/s]
            pos_y = [0, 0]    # min max [rad/s]

    class rewards( LeggedRobotCfg.rewards ):
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.0
            tracking_lin_vel = 2.0
            tracking_ang_vel = 0.5
            lin_vel_z = -0.0
            ang_vel_xy = -0.0
            orientation = -0.5
            torques = -0.0002
            dof_vel = -0.
            dof_acc = -2.5e-8
            base_height = -0.2
            feet_air_time =  1.0
            collision = -1.
            feet_stumble = -0.0 
            action_rate = -0.01
            stand_still = -0.
            dof_pos_limits =-10.
            object_distance = 2.
            object_distance_l2=-10

        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 0.9 # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.25
        max_contact_force = 100

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        
        stiffness = {"arm_joint":20,"ankle":10,"knee":10,"hip":10}  # [N*m/rad] inlcudes all joints
        damping = {"arm_joint":0.2,"ankle":10,"knee":0.2,"hip":0.2}     # [N*m*s/rad] inlcudes all joints
        # action scale: target angle = actionScale * action + defaultAngle
        arm_control_type='position'
        arm_stiffness = 20.  #postion control
        arm_damping = 0.5 #postion control
        
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class domain_rand( LeggedRobotCfg.domain_rand ):
        randomize_friction = False
        friction_range = [0.2, 1.5]
        randomize_base_mass = False
        added_mass_range = [-4., 4.]
        push_robots = False
        push_interval_s = 15
        max_push_vel_xy = 1.

        randomize_base_com = False
        added_com_range = [-0.15, 0.15]
        randomize_motor = False
        motor_strength_range = [0.8, 1.2]


    class asset( LeggedRobotCfg.asset ):
        file = '{LOCO_MANI_GYM_ROOT_DIR}/resources/robots/airbot/urdf/airbot.urdf'
        name = "airbot"

        foot_name = "wheel"  #link name
        arm_link_name = ["arm"]
        arm_joint_name = ["arm_joint"]
        wheel_joint_name =[ "ankle"] #wheel joints name, joint name
        leg_joint_name = ["right","left"]
        arm_gripper_name = "gripper_link"

        penalize_contacts_on = ["base","knee","hip"]  #link name
        terminate_after_contacts_on = []  #link name
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter "base","calf","hip","thigh"
        flip_visual_attachments = False

    class normalization(LeggedRobotCfg.normalization):
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.05
            height_measurements = 5.0
            gripper_track = 1.0
        clip_observations = 100.
        clip_actions = 100.
class AirbotRoughCfgPPO( LeggedRobotCfgPPO ):
    seed = 21
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        learning_rate = 1.e-3
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'airbot_0'
        resume = False
        num_steps_per_env = 24 # per iteration
        max_iterations = 1000
        save_interval =100
        load_run = -1
        checkpoint = -1
        train =True

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu'   # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid