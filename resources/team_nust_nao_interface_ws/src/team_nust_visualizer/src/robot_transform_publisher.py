#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from team_nust_msgs.msg import TeamNUSTState
from geometry_msgs.msg import Transform
import rotation_utils

def handle_team_nust_state(msg):
    # left or right foot
    torso_in_foot = msg.l_foot_transform if msg.foot_on_ground == 0 else msg.r_foot_transform
    foot_y = 0.05 if msg.foot_on_ground == 0 else -0.05

    # foot pose in world
    foot_pose = Transform()
    foot_pose.translation.x = msg.robot_pose_2d.x - foot_y * math.sin(msg.robot_pose_2d.theta)
    foot_pose.translation.y = msg.robot_pose_2d.y + foot_y * math.cos(msg.robot_pose_2d.theta)
    # fixed according to rviz world
    foot_pose.translation.z = 0.05
    foot_pose.rotation.x = 0
    foot_pose.rotation.y = 0
    foot_pose.rotation.z = msg.robot_pose_2d.theta
    foot_pose.rotation.w = 1

    foot_trans = np.identity(4)
    foot_trans[0, 3] = foot_pose.translation.x
    foot_trans[1, 3] = foot_pose.translation.y
    foot_trans[2, 3] = foot_pose.translation.z
    foot_rot = \
        tf.transformations.quaternion_matrix(
            [foot_pose.rotation.x, foot_pose.rotation.y, foot_pose.rotation.z, foot_pose.rotation.w])
    foot_mat = np.dot(foot_trans, foot_rot)
    torso_trans = np.identity(4)
    torso_trans[0, 3] = torso_in_foot.translation.x
    torso_trans[1, 3] = torso_in_foot.translation.y
    torso_trans[2, 3] = torso_in_foot.translation.z
    torso_rot = \
        tf.transformations.quaternion_matrix(
            [torso_in_foot.rotation.x, torso_in_foot.rotation.y, torso_in_foot.rotation.z, torso_in_foot.rotation.w])
    torso_mat = np.dot(torso_trans, torso_rot)
    torso_in_world = np.dot(foot_mat, torso_mat)
    torso_in_world_euler = rotation_utils.rot_to_euler(torso_in_world[:3, :3]);
    br = tf.TransformBroadcaster()
    br.sendTransform((torso_in_world[0, 3], torso_in_world[1, 3], torso_in_world[2, 3]),
                     tf.transformations.quaternion_from_euler(torso_in_world_euler[0], torso_in_world_euler[1], torso_in_world_euler[2]),
                     rospy.Time.now(),
                     'base_link',
                     'world_frame')


if __name__ == '__main__':
    rospy.init_node('robot_tf_publisher')
    rospy.Subscriber('team_nust_state',
                     TeamNUSTState,
                     handle_team_nust_state)
    rospy.spin()
