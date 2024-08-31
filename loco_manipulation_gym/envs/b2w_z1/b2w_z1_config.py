from loco_manipulation_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import numpy as np
class B2WZ1RoughCfg( LeggedRobotCfg ):

    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_observations = 87
        symmetric = False  #true :  set num_privileged_obs = None;    false: num_privileged_obs = observations + 187 ,set "terrain.measure_heights" to true
        num_privileged_obs = num_observations + 187 # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 22
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds
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
        pos = [0.0, 0.0, 0.60] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]

            'FL_foot_joint': 0.0,   # [rad]
            'RL_foot_joint': 0.0,    # [rad]
            'FR_foot_joint': 0.0,  # [rad]
            'RR_foot_joint': 0.0,    # [rad]

            'arm_joint1':0.,
            'arm_joint2':0.,
            'arm_joint3':0.,
            'arm_joint4':0.,
            'arm_joint5':0.,
            'arm_joint6':0.,

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

            init_pos_l = [0.6, 1.1]
            init_pos_p = [-1 * np.pi / 6, 1 * np.pi / 3]
            init_pos_y = [-1 * np.pi / 4, 1 * np.pi / 4]
            final_delta_orn = [[-0, 0], [-0, 0], [-0, 0]]

        class init_ranges:
            pos_l = [0.3, 0.5] # min max [m/s]
            pos_p = [np.pi / 4, 3 * np.pi / 4]   # min max [m/s]
            pos_y = [0, 0]    # min max [rad/s]

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

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 50.}  # [N*m/rad] inlcudes all joints
        damping = {'joint': 1}     # [N*m*s/rad] inlcudes all joints

        arm_stiffness = 20.
        arm_damping = 0.5
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        arm_control_type = 'position'

    class asset( LeggedRobotCfg.asset ):
        file = '{LOCO_MANI_GYM_ROOT_DIR}/resources/robots/b2w_z1/urdf/b2w_z1.urdf'
        name = "b2w_z1"
        foot_name = "foot"   #link name of contact plane
        arm_link_name = ["arm"]
        arm_joint_name = ["arm_joint"]
        arm_gripper_name = "gripperMover"
        wheel_joint_name =["foot_joint"]  #joint name of wheel,if no wheel, set to []
        leg_joint_name = ["hip_joint","thigh_joint","calf_joint"]
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["hip"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True
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

    class rewards( LeggedRobotCfg.rewards ):
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -1
            tracking_lin_vel = 2.0
            tracking_ang_vel = 0.5
            lin_vel_z = -0.0
            ang_vel_xy = -0.1
            orientation = -0.5
            torques = -0.0002
            dof_vel = -0.
            dof_acc = -2.5e-9
            base_height = -0.2
            feet_air_time =  1.0
            collision = -1.
            feet_stumble = -0.0 
            action_rate = -0.001
            stand_still = -0.
            dof_pos_limits =-10.
            object_distance = 2.
            object_distance_l2=-1

        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 0.9 # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.5
        max_contact_force = 100. # forces above this value are penalized

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
class B2WZ1RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'b2w_z1'

  
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 24 # per iteration
        max_iterations = 3000 # number of policy updates

        # logging
        save_interval = 50 # check for potential saves every this many iterations

        resume = False
        load_run = -1 # -1 = last run
        checkpoint = 6000 # -1 = last saved model
        resume_path = None # updated from load_run and chkpt