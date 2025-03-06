#!/usr/bin/env python3
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_mini import RobotisMini
from dynamic_reconfigure.server import Server
from robotis_mini_control.cfg import RobotisMiniConfig

# Parameters for generating wave inputs
start_time = 0.0       # seconds
end_time = 30.0        # seconds
control_period_ = 0.001  # seconds (sample time)
freq_n = 0.25          # Hz (natural frequency)
amplitude = 30         # mm (amplitude)
# max x direction : 30 mm
# min x direction : -60 mm
# max y direction : 60 mm
# min y direction : -60 mm

signal_type_ = 'triangle'  # 'sine' or 'triangle'


def callback(config, level):
    """
    Dynamic reconfigure callback used for execute_static_foot_position().
    When the user changes parameters (x_foot_pos, y_foot_pos, z_foot_pos)
    via rqt_dynamic_reconfigure, the robot will move to the new foot position.
    """
    x_foot_pos = config['x_foot_pos']
    y_foot_pos = config['y_foot_pos']
    z_foot_pos = config['z_foot_pos']

    execute_static_foot_position(robot, x_foot_pos=x_foot_pos,
                                 y_foot_pos=y_foot_pos,
                                 z_foot_pos=z_foot_pos,
                                 time=1.0)

    print("x_foot_pos: ", x_foot_pos,
          "\ny_foot_pos: ", y_foot_pos,
          "\nz_foot_pos: ", z_foot_pos,
          "\n--------------------")

    return config


def execute_static_foot_position(robot, x_foot_pos, y_foot_pos, z_foot_pos, time):
    joint_values_right_hand = [0, 0, 0]
    joint_values_left_hand = [0, 0, 0]
    joint_values_right_foot = robot.ik_right_foot(robot.x_RF0 + x_foot_pos,
                                                  robot.y_RF0 + y_foot_pos,
                                                  z_foot_pos,
                                                  robot.roll_RF0,
                                                  robot.pitch_RF0)
    joint_values_left_foot = robot.ik_left_foot(robot.x_LF0 + x_foot_pos,
                                                robot.y_LF0 + y_foot_pos,
                                                z_foot_pos,
                                                robot.roll_LF0,
                                                robot.pitch_LF0)

    joint_pos_values = (joint_values_right_hand + joint_values_left_hand +
                        joint_values_right_foot + joint_values_left_foot)

    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = [
        'r_shoulder_joint', 'r_biceps_joint', 'r_elbow_joint',
        'l_shoulder_joint', 'l_biceps_joint', 'l_elbow_joint',
        'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
        'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint'
    ]

    point = JointTrajectoryPoint()
    point.positions = joint_pos_values
    point.time_from_start = rospy.Duration(time)
    traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)
    rospy.sleep(time)


def sine_wave_input():
    """
    TODO: Implement sine wave signal with a natural frequency (freq_n) 
            of 0.25hz sampled every 0.001 sec (control_period_).
    """
    global start_time
    global end_time
    global control_period_
    global freq_n
    global amplitude
    t = start_time
    wave_signal = []
    while t <= end_time:
        value = amplitude * math.sin(2 * math.pi * freq_n * t)
        wave_signal.append(value)
        t += control_period_
    return wave_signal


def triangle_wave_input():
    """
    TODO: Implement triangle wave signal with a natural frequency (freq_n) 
            of 0.25hz sampled every 0.001 sec (control_period_).
    """
    global start_time
    global end_time
    global control_period_
    global freq_n
    global amplitude
    t = start_time
    wave_signal = []
    period = 1.0 / freq_n
    while t <= end_time:
        t_mod = t % period
        if t_mod < period / 2:
            value = (2 * amplitude / (period / 2)) * t_mod - amplitude
        else:
            value = - (2 * amplitude / (period / 2)) * \
                (t_mod - period/2) + amplitude
        wave_signal.append(value)
        t += control_period_
    return wave_signal


def execute_variable_foot_position(robot, z_foot_pos):
    """
        TODO: Implement variable foot position input using the sine 
        and triangle wave inputs. Calc joint angles using ik and append 
        all waypoints to a single JointTrajectory msg.

        Reference execute_static_foot_position() code above for how to use the 
        inverse kinematics functions, construct JointTrajectory msg, and publish 
        the final trajectory to the controller.

        Hint: You need to increment point.time_from_start by control_period_ 
        for every point you add to the trajectory.
        """
    global control_period_
    global signal_type_

    if signal_type_ == 'sine':
        wave_signal = sine_wave_input()
    else:
        wave_signal = triangle_wave_input()

    time_from_start = 0.0
    x_offset = 0
    y_offset = 0

    time_from_start = 0.0
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = [
        'r_shoulder_joint', 'r_biceps_joint', 'r_elbow_joint',
        'l_shoulder_joint', 'l_biceps_joint', 'l_elbow_joint',
        'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
        'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint'
    ]

    for offset in wave_signal:
        time_from_start += control_period_
        x_offset = 0
        y_offset = offset

        joint_values_right_hand = [0, 0, 0]
        joint_values_left_hand = [0, 0, 0]
        joint_values_right_foot = robot.ik_right_foot(robot.x_RF0 + x_offset,
                                                      robot.y_RF0 + y_offset,
                                                      z_foot_pos,
                                                      robot.roll_RF0,
                                                      robot.pitch_RF0)
        joint_values_left_foot = robot.ik_left_foot(robot.x_LF0 + x_offset,
                                                    robot.y_LF0 + y_offset,
                                                    z_foot_pos,
                                                    robot.roll_LF0,
                                                    robot.pitch_LF0)

        joint_pos_values = (joint_values_right_hand + joint_values_left_hand +
                            joint_values_right_foot + joint_values_left_foot)

        point = JointTrajectoryPoint()
        point.positions = joint_pos_values
        point.time_from_start = rospy.Duration(time_from_start)
        traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)


if __name__ == '__main__':
    rospy.init_node('foot_position')

    robot = RobotisMini()
    # init dynamic reconfigure server
    srv = Server(RobotisMiniConfig, callback)

    robot.init_pose(z_foot_pos=-166)

    # Uncomment below to test variable foot position
    execute_variable_foot_position(robot, z_foot_pos=-166)

    rospy.spin()
