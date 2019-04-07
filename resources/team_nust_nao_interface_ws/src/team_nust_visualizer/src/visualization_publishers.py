import rospy
import math
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D
from humanoid_nav_msgs.msg import StepTarget
from team_nust_msgs.msg import \
    JointInfo, SensorState, BallInfo, GoalInfo, \
    ObsObstacles, Obstacle, TeamNUSTState, TeamInfo, \
    TeamRobot, LocalizationState, ObsLandmarks, \
    StepTargetArr


class ObstacleType:
  UNKNOWN = 0
  GOALPOST = 1
  OPPONENT = 2
  TEAMMATE = 3
  OPPONENT_FALLEN = 4
  TEAMMATE_FALLEN = 5
  COUNT = 6


class Visualizer(object):
    def __init__(self, name, ros_msg):
        self._subscribe_topic_name = name
        self._subscribe_msg_type = ros_msg
        self._team_nust_state_subscriber = None
        self._subscriber = None
        self._robot_pose_2d = Pose2D()
        self._player_number = None

    def init(self):
        self._team_nust_state_subscriber = rospy.Subscriber('team_nust_state', TeamNUSTState, self.update_team_nust_state_data)
        self._subscriber = rospy.Subscriber(self._subscribe_topic_name, self._subscribe_msg_type, self.visualize)

    def update_team_nust_state_data(self, msg):
        self._player_number = msg.player_number
        self.update_robot_transform(msg)

    def update_robot_transform(self, msg):
        self._robot_pose_2d = msg.robot_pose_2d

    def robot_to_world(self, pose):
        transformed = Pose2D()
        transformed.x = self._robot_pose_2d.x + math.cos(self._robot_pose_2d.theta) * pose.x - math.sin(self._robot_pose_2d.theta) * pose.y
        transformed.y = self._robot_pose_2d.y + math.sin(self._robot_pose_2d.theta) * pose.x + math.cos(self._robot_pose_2d.theta) * pose.y
        return transformed

    def visualize(self, msg):
        pass

#Tested
class BallInfoVisualizer(Visualizer):
    def __init__(self):
        super(BallInfoVisualizer, self).__init__('ball_info', BallInfo)
        self._publisher = None
        self._markers = []
        self._ns = "ball_info"

    def init(self):
        super(BallInfoVisualizer, self).init()
        self._publisher = rospy.Publisher('ball_info_markers', MarkerArray, queue_size=1)

    def visualize(self, msg):
        if not msg.found:
          return
        self._markers = []
        #removal_marker = Marker()  
        #removal_marker.action = Marker.DELETE
        #for idx, marker in enumerate(self._markers):
        #    removal_marker.header.frame_id = "world_frame"
        #    removal_marker.ns = self._ns
        #    removal_marker.id = idx
        #    removal_marker.lifetime = rospy.Duration(0.2)
        #self._publisher.publish(self._markers)
        
        pos_in_world = self.robot_to_world(msg.pos)
        ball_marker = Marker()
        ball_marker.type = Marker.SPHERE
        ball_marker.action = Marker.ADD
        ball_marker.pose.position.x = pos_in_world.x
        ball_marker.pose.position.y = pos_in_world.y
        ball_marker.pose.position.z = msg.radius + 0.05
        ball_marker.pose.orientation.x = 0
        ball_marker.pose.orientation.y = 0
        ball_marker.pose.orientation.z = 0
        ball_marker.scale.x = msg.radius * 2
        ball_marker.scale.y = msg.radius * 2
        ball_marker.scale.z = msg.radius * 2
        ball_marker.color.r = 0.3
        ball_marker.color.g = 0
        ball_marker.color.b = 0
        ball_marker.color.a = 1.0
        self._markers.append(ball_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(0.2)
        self._publisher.publish(self._markers)

#Tested
class GoalInfoVisualizer(Visualizer):
    def __init__(self):
        super(GoalInfoVisualizer, self).__init__('goal_info', GoalInfo)
        self._publisher = None
        self._markers = []
        self._ns = "goal_info"

    def init(self):
        super(GoalInfoVisualizer, self).init()
        self._publisher = rospy.Publisher('goal_info_markers', MarkerArray, queue_size=10)

    def visualize(self, msg):
        if not msg.found:
            return
        self._markers = []
        left_post_in_robot = self.robot_to_world(msg.left_post)
        right_post_in_robot = self.robot_to_world(msg.right_post)
        
        left_marker = Marker()
        left_marker.type = Marker.CYLINDER
        left_marker.action = Marker.ADD
        left_marker.pose.position.x = left_post_in_robot.x
        left_marker.pose.position.y = left_post_in_robot.y
        left_marker.pose.position.z = 0.45
        left_marker.pose.orientation.x = 0
        left_marker.pose.orientation.y = 0
        left_marker.pose.orientation.z = 0
        left_marker.scale.x = 0.1
        left_marker.scale.y = 0.1
        left_marker.scale.z = 0.9
        if msg.ours:
            left_marker.color.r = 0
            left_marker.color.g = 0
            left_marker.color.b = 0.3
            left_marker.color.a = 1.0
        else:
            left_marker.color.r = 0.3
            left_marker.color.g = 0
            left_marker.color.b = 0
            left_marker.color.a = 1.0
        self._markers.append(left_marker)
        
        right_marker = Marker()
        right_marker.type = Marker.CYLINDER
        right_marker.action = Marker.ADD
        right_marker.pose.position.x = right_post_in_robot.x
        right_marker.pose.position.y = right_post_in_robot.y
        right_marker.pose.position.z = 0.45
        right_marker.pose.orientation.x = 0
        right_marker.pose.orientation.y = 0
        right_marker.pose.orientation.z = 0
        right_marker.scale.x = 0.1
        right_marker.scale.y = 0.1
        right_marker.scale.z = 0.9
        if msg.ours:
            right_marker.color.r = 0
            right_marker.color.g = 0
            right_marker.color.b = 0.3
            right_marker.color.a = 1.0
        else:
            right_marker.color.r = 0.3
            right_marker.color.g = 0
            right_marker.color.b = 0
            right_marker.color.a = 1.0
        self._markers.append(right_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(0.2)
        self._publisher.publish(self._markers)
  
#Tested
class ObstaclesVisualizer(Visualizer):
    def __init__(self):
        super(ObstaclesVisualizer, self).__init__('obs_obstacles', ObsObstacles)
        self._publisher = None
        self._markers = []
        self._ns = "obs_obstacles"

    def init(self):
        super(ObstaclesVisualizer, self).init()
        self._publisher = rospy.Publisher('obs_obstacles_markers', MarkerArray, queue_size=10)

    def visualize(self, msg):
        self._markers = []
        for obstacle in msg.obstacles:
            if ObstacleType.OPPONENT > obstacle.type > ObstacleType.TEAMMATE_FALLEN:
                continue
            try:
              left_in_world = self.robot_to_world(obstacle.left_bound)
              right_in_world = self.robot_to_world(obstacle.right_bound)
              slope = (right_in_world.y - left_in_world.y) / (right_in_world.x - left_in_world.x)
              slope_perp = -1 / slope
              perp = np.array([left_in_world.x + 0.1, 0.1 * slope_perp + left_in_world.y])
              diff = np.array([perp[0] - left_in_world.x, perp[1] - left_in_world.y])
              unit = np.array([diff[0] / np.linalg.norm(diff), diff[1] / np.linalg.norm(diff)])
              left_depth_point = np.array(
                [
                  left_in_world.x + unit[0] * obstacle.depth,
                  left_in_world.y + unit[1] * obstacle.depth
                ]
              )
              right_depth_point = np.array(
                [
                  right_in_world.x + unit[0] * obstacle.depth,
                  right_in_world.y + unit[1] * obstacle.depth
                ]
              )
              left_center_point = np.array(
                [
                  (left_in_world.x + left_depth_point[0]) / 2,
                  (left_in_world.y + left_depth_point[1]) / 2
                ]
              )
              right_center_point = np.array(
                [
                  (right_in_world.x + right_depth_point[0]) / 2,
                  (right_in_world.y + right_depth_point[1]) / 2
                ]
              )
              obstacle_center_point = Pose2D()
              obstacle_center_point.x = (left_center_point[0] + right_center_point[0]) / 2
              obstacle_center_point.y = (left_center_point[1] + right_center_point[1]) / 2
              obstacle_center_point.theta = math.atan2(
                right_in_world.y - left_in_world.y, right_in_world.x - left_in_world.x)
              obstacle_marker = Marker()
              obstacle_marker.type = Marker.CUBE
              obstacle_marker.action = Marker.ADD
              obstacle_marker.pose.position.x = obstacle_center_point.x
              obstacle_marker.pose.position.y = obstacle_center_point.y
              obstacle_marker.pose.position.z = 0.05
              q = \
                  tf.transformations.quaternion_from_euler(
                      0,
                      0,
                      obstacle_center_point.theta)
              obstacle_marker.pose.orientation.x = q[0]
              obstacle_marker.pose.orientation.y = q[1]
              obstacle_marker.pose.orientation.z = q[2]
              obstacle_marker.pose.orientation.w = q[3]
              obstacle_marker.scale.x = \
                  math.sqrt((right_in_world.y - left_in_world.y)**2 + (right_in_world.x - left_in_world.x)**2)
              obstacle_marker.scale.y = obstacle.depth
              if obstacle.type == ObstacleType.TEAMMATE:
                  # Height of the robot is chosen as 80cm
                  obstacle_marker.scale.z = 0.8
                  obstacle_marker.color.r = 0
                  obstacle_marker.color.g = 0
                  # Teammate is blue
                  obstacle_marker.color.b = 0.3
                  obstacle_marker.color.a = 1.0
              elif obstacle.type == ObstacleType.OPPONENT:
                  obstacle_marker.scale.z = 0.8
                  # Opponent is red
                  obstacle_marker.color.r = 0.3
                  obstacle_marker.color.g = 0
                  obstacle_marker.color.b = 0
                  obstacle_marker.color.a = 1.0
              elif obstacle.type == ObstacleType.TEAMMATE_FALLEN:
                  # Height of the fallen robot is chosen as 30cm
                  obstacle_marker.scale.z = 0.3
                  obstacle_marker.color.r = 0
                  obstacle_marker.color.g = 0
                  # Teammate is blue
                  obstacle_marker.color.b = 0.3
                  obstacle_marker.color.a = 1.0
              elif obstacle.type == ObstacleType.OPPONENT_FALLEN:
                  obstacle_marker.scale.z = 0.3
                  # Opponent is red
                  obstacle_marker.color.r = 0.3
                  obstacle_marker.color.g = 0
                  obstacle_marker.color.b = 0
                  obstacle_marker.color.a = 1.0
              self._markers.append(obstacle_marker)
            except ZeroDivisionError as error:
              continue
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(2.0)
        self._publisher.publish(self._markers)


class TeamInfoVisualizer(Visualizer):
    def __init__(self):
        super(TeamInfoVisualizer, self).__init__('team_info', TeamInfo)
        self._publisher = None
        self._markers = []
        self._ns = "team_info"

    def init(self):
        super(TeamInfoVisualizer, self).init()
        self._publisher = rospy.Publisher('team_info_markers', MarkerArray, queue_size=10)

    def visualize(self, msg):
        tr = TeamRobot()
        tr.data_received = True
        tr.pose_2d.x = 1.0
        tr.pose_2d.y = 1.0
        tr.pose_2d.theta = math.pi / 2
        msg.robots.append(tr)
        self._player_number = 1
        for idx, team_robot in enumerate(msg.robots):
            if idx == self._player_number or not team_robot.data_received:
                continue
            robot_pos = self.robot_to_world(team_robot.pose_2d)
            team_robot_marker = Marker()
            team_robot_marker.type = Marker.CYLINDER
            team_robot_marker.action = Marker.ADD
            team_robot_marker.pose.position.x = robot_pos.x
            team_robot_marker.pose.position.y = robot_pos.y
            team_robot_marker.pose.position.z = 0.05
            q = \
                tf.transformations.quaternion_from_euler(
                    0,
                    0,
                    team_robot.pose_2d.theta)
            team_robot_marker.pose.orientation.x = q[0]
            team_robot_marker.pose.orientation.y = q[1]
            team_robot_marker.pose.orientation.z = q[2]
            team_robot_marker.pose.orientation.w = q[3]
            team_robot_marker.scale.x = 0.15
            team_robot_marker.scale.y = 0.3
            team_robot_marker.scale.z = 0.7
            team_robot_marker.color.r = 0
            team_robot_marker.color.g = 0
            # Teammate is blue
            team_robot_marker.color.b = 0.3
            team_robot_marker.color.a = 0.5
            self._markers.append(team_robot_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(0.2)
        self._publisher.publish(self._markers)


class ObsLandmarkVisualizer(Visualizer):
    def __init__(self):
        super(ObsLandmarkVisualizer, self).__init__('obs_known_landmarks', ObsLandmarks)
        self._publisher = None
        self._markers = []
        self._ns = 'obs_known_landmark'
        self._landmarks_map = {0: 'Goal-Post', 1: 'Line', 2: 'T', 3: 'L', 4: 'Ellipse', 5: 'Penalty'}

    def init(self):
        super(ObsLandmarkVisualizer, self).init()
        self._publisher = rospy.Publisher('obs_known_landmark_markers', MarkerArray, queue_size=10)

    def visualize(self, msg):
        for landmark in msg.landmarks:
            landmark_marker = Marker()
            landmark_marker.type = Marker.TEXT_VIEW_FACING
            landmark_marker.action = Marker.ADD
            landmark_marker.text = self._landmarks_map[landmark.type]
            landmark_marker.pose.position.x = landmark.pos.x
            landmark_marker.pose.position.y = landmark.pos.y
            landmark_marker.pose.position.z = 0.1
            landmark_marker.scale.z = 0.1
            landmark_marker.color.r = 0
            landmark_marker.color.g = 0
            landmark_marker.color.b = 0
            landmark_marker.color.a = 1.0
            self._markers.append(landmark_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(0.2)
        self._publisher.publish(self._markers)

class FootstepsVisualizer(Visualizer):
    def __init__(self):
        super(FootstepsVisualizer, self).__init__('cmd_footsteps', StepTargetArr)
        self._publisher = None
        self._markers = []
        self._ns = 'cmd_footsteps'

    def init(self):
        super(FootstepsVisualizer, self).init()
        self._publisher = rospy.Publisher('cmd_footsteps_markers', MarkerArray, queue_size=10)

    def visualize(self, msg):
        for idx, footstep in enumerate(msg.steps):
            footstep_marker = Marker()
            footstep_marker.type = Marker.CUBE
            footstep_marker.action = Marker.ADD
            footstep_marker.pose.position.x = footstep.pose.x
            footstep_marker.pose.position.y = footstep.pose.y
            footstep_marker.pose.position.z = 0.052
            q = \
                tf.transformations.quaternion_from_euler(
                    0,
                    0,
                    footstep.pose.theta)
            footstep_marker.pose.orientation.x = q[0]
            footstep_marker.pose.orientation.y = q[1]
            footstep_marker.pose.orientation.z = q[2]
            footstep_marker.pose.orientation.w = q[3]
            footstep_marker.scale.x = 0.06
            footstep_marker.scale.y = 0.06
            footstep_marker.scale.z = 0.015
            if idx < len(msg.steps) - 2:
                if footstep.leg == StepTarget.left:
                    footstep_marker.color.r = 0.5
                    footstep_marker.color.g = 0
                    footstep_marker.color.b = 0
                    footstep_marker.color.a = 0.1
                elif footstep.leg == StepTarget.right:
                    footstep_marker.color.r = 0
                    footstep_marker.color.g = 0
                    footstep_marker.color.b = 0.5
                    footstep_marker.color.a = 0.1
            else:
                footstep_marker.color.r = 0
                footstep_marker.color.g = 0.5
                footstep_marker.color.b = 0
                footstep_marker.color.a = 0.1
            self._markers.append(footstep_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx
            marker.lifetime = rospy.Duration(0.2)
        self._publisher.publish(self._markers)

