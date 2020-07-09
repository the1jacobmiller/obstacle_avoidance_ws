#!/usr/bin/env python3

import rospy

def mph_to_ms(mph):
    return mph * 0.447

def set_path_planning_params():
    rospy.set_param('/path_planning/file_name', "/home/jacob/Desktop/csv_files/straight_path_5mph")

    print("Finished setting path planning parameters")

def set_motion_planning_params():
    # Motion planning paramters
    rospy.set_param('/motion_planning/control_period', 1.0)
    rospy.set_param('/motion_planning/max_uncertainty_multiplier', 5.0)
    rospy.set_param('/motion_planning/yaw_rate_threshold', 0.3)
    rospy.set_param('/motion_planning/num_random_samples', 100)
    rospy.set_param('/motion_planning/max_time_step', 10.0)
    rospy.set_param('/motion_planning/k_d', 1.0)
    rospy.set_param('/motion_planning/k_prob', 1500.0)
    rospy.set_param('/motion_planning/k_p', 0.1)
    rospy.set_param('/motion_planning/k_a', 0.1)
    rospy.set_param('/motion_planning/path_smoothing', 0.7)
    rospy.set_param('/motion_planning/min_speed', 2.0)
    rospy.set_param('/motion_planning/occupancy_threshold', 0.175)
    rospy.set_param('/motion_planning/max_dist_to_traj', 1.0)

    print("Finished setting motion planning parameters")

def set_control_params():
    # # control parameters - 5 mph
    rospy.set_param('/control/W_cte', 1000.0)
    rospy.set_param('/control/W_epsi', 1000.0)
    rospy.set_param('/control/W_v', 10.0)
    rospy.set_param('/control/W_delta', 500.0)
    rospy.set_param('/control/W_a', 10.0)
    rospy.set_param('/control/W_ddelta', 200.0)
    rospy.set_param('/control/W_da', 15.0)

    # control parameters - 10 mph - needs tuning
    # rospy.set_param('/control/W_cte', 1000.0)
    # rospy.set_param('/control/W_epsi', 1000.0)
    # rospy.set_param('/control/W_v', 10.0)
    # rospy.set_param('/control/W_delta', 8000.0)
    # rospy.set_param('/control/W_a', 10.0)
    # rospy.set_param('/control/W_ddelta', 4000.0)
    # rospy.set_param('/control/W_da', 15.0)

    # # control parameters - 15 mph - needs tuning
    # rospy.set_param('/control/W_cte', 1000.0)
    # rospy.set_param('/control/W_epsi', 1000.0)
    # rospy.set_param('/control/W_v', 10.0)
    # rospy.set_param('/control/W_delta', 12000.0)
    # rospy.set_param('/control/W_a', 10.0)
    # rospy.set_param('/control/W_ddelta', 8000.0)
    # rospy.set_param('/control/W_da', 15.0)

    # control Constraints
    rospy.set_param('/control/deltaHigh', 0.3)
    rospy.set_param('/control/aHigh', 1.0)
    rospy.set_param('/control/deltaLow', -0.3)
    rospy.set_param('/control/aLow', -1.0)

    rospy.set_param('/control/delay_time', 0.3)

    print("Finished setting control parameters")

def set_twist_controller_params():
    # TwistControllerNode parameters
    rospy.set_param('/twist_controller/angle_scale', 1.0)
    rospy.set_param('/twist_controller/steering_smoothing', 1.0)
    rospy.set_param('/twist_controller/accel_cmd_smoothing', 1.0)
    rospy.set_param('/twist_controller/throttle_cmd_smoothing', 1.0)
    rospy.set_param('/twist_controller/brake_cmd_smoothing', 1.0)
    rospy.set_param('/twist_controller/brake_deadband', 0.3)
    rospy.set_param("/twist_controller/accel_limit", 2.0)
    rospy.set_param("/twist_controller/decel_limit", 2.0)

    # VSI Controller parameters
    rospy.set_param('/twist_controller/enable_steering', True)
    rospy.set_param('/twist_controller/enable_throttle', True)
    rospy.set_param('/twist_controller/enable_brake', True)

    print("Finished setting TwistController parameters")

def set_global_params():
    rospy.set_param('/center_degree_offset', 0.0)
    rospy.set_param('/simulation_mode', True)

    print("Finished setting global parameters")

if __name__ == '__main__':
    rospy.init_node('ros_configs', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    set_path_planning_params()
    set_motion_planning_params()
    set_control_params()
    set_twist_controller_params()
    set_global_params()
