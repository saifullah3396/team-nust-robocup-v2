import rospy
import json
import re
import struct
import numpy as np
import tf
import cv2
import cv_bridge
from numpy import array_split
import rotation_utils
from std_msgs.msg import String
from geometry_msgs.msg import Point, Point32, Pose2D, Transform
from sensor_msgs.msg import JointState, Image, PointCloud
from nav_msgs.msg import OccupancyGrid
from humanoid_nav_msgs.msg import StepTarget
from team_nust_msgs.msg import \
    JointInfo, SensorState, BallInfo, GoalInfo, \
    ObsObstacles, Obstacle, TeamNUSTState, TeamInfo, \
    TeamRobot, LocalizationState, Landmark, ObsLandmarks, \
    StepTargetArr, BehaviorInfo

class SensorNames:
    JOINTS_INCOMING = \
        [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipYawPitch",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll"
        ]
    JOINTS = \
        [
          'HeadYaw',
          'HeadPitch',
          'LHipYawPitch',
          'LHipRoll',
          'LHipPitch',
          'LKneePitch',
          'LAnklePitch',
          'LAnkleRoll',
          'RHipYawPitch',
          'RHipRoll',
          'RHipPitch',
          'RKneePitch',
          'RAnklePitch',
          'RAnkleRoll',
          'LShoulderPitch',
          'LShoulderRoll',
          'LElbowYaw',
          'LElbowRoll',
          'LWristYaw',
          'LHand',
          'RShoulderPitch',
          'RShoulderRoll',
          'RElbowYaw',
          'RElbowRoll',
          'RWristYaw',
          'RHand',
          'RFinger23',
          'RFinger13',
          'RFinger12',
          'LFinger21',
          'LFinger13',
          'LFinger11',
          'RFinger22',
          'LFinger22',
          'RFinger21',
          'LFinger12',
          'RFinger11',
          'LFinger23',
          'LThumb1',
          'RThumb1',
          'RThumb2',
          'LThumb2'
        ]
    HANDS = \
        [
            'LHand',
            'RHand'
        ]
    TOUCH = \
        [
            'HeadFront',
            'HeadRear',
            'HeadMiddle',
            'LHandBack',
            'LHandLeft',
            'LHandRight',
            'RHandBack',
            'RHandLeft',
            'RHandRight'
        ]
    BATTERY = \
        [
            'CPUTemperature',
            'Temperature',
            'Current',
            'Charge',
        ]
    SWITCH = \
        [
            'ChestBoard',
            'LBumperRight',
            'LBumperLeft',
            'RBumperRight',
            'RBumperLeft'
        ]
    INERTIAL = \
        [
            'GyrX',
            'GyrY',
            'GyrZ',
            'AngleX',
            'AngleY',
            'AngleZ',
            'AccelerometerX',
            'AccelerometerY',
            'AccelerometerZ'
        ]
    SONAR = \
        [
            "USLeft",
            "USLeft1",
            "USLeft2",
            "USLeft3",
            "USLeft4",
            "USLeft5",
            "USLeft6",
            "USLeft7",
            "USLeft8",
            "USLeft9",
            "USRight",
            "USRight1",
            "USRight2",
            "USRight3",
            "USRight4",
            "USRight5",
            "USRight6",
            "USRight7",
            "USRight8",
            "USRight9",
        ]
    FORCE = \
        [
            "LFsrFL",
            "LFsrFR",
            "LFsrRL",
            "LFsrRR",
            "LFsrTotalWeight",
            "LFsrCopX",
            "LFsrCopY",
            "RFsrFL",
            "RFsrFR",
            "RFsrRL",
            "RFsrRR",
            "RFsrTotalWeight",
            "RFsrCopX",
            "RFsrCopY",
        ]
    LED = \
        [
          "Face/Led/Red/Left/0Deg",
          "Face/Led/Red/Left/45Deg",
          "Face/Led/Red/Left/90Deg",
          "Face/Led/Red/Left/135Deg",
          "Face/Led/Red/Left/180Deg",
          "Face/Led/Red/Left/225Deg",
          "Face/Led/Red/Left/270Deg",
          "Face/Led/Red/Left/315Deg",
          "Face/Led/Green/Left/0Deg",
          "Face/Led/Green/Left/45Deg",
          "Face/Led/Green/Left/90Deg",
          "Face/Led/Green/Left/135Deg",
          "Face/Led/Green/Left/180Deg",
          "Face/Led/Green/Left/225Deg",
          "Face/Led/Green/Left/270Deg",
          "Face/Led/Green/Left/315Deg",
          "Face/Led/Blue/Left/0Deg",
          "Face/Led/Blue/Left/45Deg",
          "Face/Led/Blue/Left/90Deg",
          "Face/Led/Blue/Left/135Deg",
          "Face/Led/Blue/Left/180Deg",
          "Face/Led/Blue/Left/225Deg",
          "Face/Led/Blue/Left/270Deg",
          "Face/Led/Blue/Left/315Deg",
          "Face/Led/Red/Right/0Deg",
          "Face/Led/Red/Right/45Deg",
          "Face/Led/Red/Right/90Deg",
          "Face/Led/Red/Right/135Deg",
          "Face/Led/Red/Right/180Deg",
          "Face/Led/Red/Right/225Deg",
          "Face/Led/Red/Right/270Deg",
          "Face/Led/Red/Right/315Deg",
          "Face/Led/Green/Right/0Deg",
          "Face/Led/Green/Right/45Deg",
          "Face/Led/Green/Right/90Deg",
          "Face/Led/Green/Right/135Deg",
          "Face/Led/Green/Right/180Deg",
          "Face/Led/Green/Right/225Deg",
          "Face/Led/Green/Right/270Deg",
          "Face/Led/Green/Right/315Deg",
          "Face/Led/Blue/Right/0Deg",
          "Face/Led/Blue/Right/45Deg",
          "Face/Led/Blue/Right/90Deg",
          "Face/Led/Blue/Right/135Deg",
          "Face/Led/Blue/Right/180Deg",
          "Face/Led/Blue/Right/225Deg",
          "Face/Led/Blue/Right/270Deg",
          "Face/Led/Blue/Right/315Deg",
          "Ears/Led/Left/0Deg",
          "Ears/Led/Left/36Deg",
          "Ears/Led/Left/72Deg",
          "Ears/Led/Left/108Deg",
          "Ears/Led/Left/144Deg",
          "Ears/Led/Left/180Deg",
          "Ears/Led/Left/216Deg",
          "Ears/Led/Left/252Deg",
          "Ears/Led/Left/288Deg",
          "Ears/Led/Left/324Deg",
          "Ears/Led/Right/0Deg",
          "Ears/Led/Right/36Deg",
          "Ears/Led/Right/72Deg",
          "Ears/Led/Right/108Deg",
          "Ears/Led/Right/144Deg",
          "Ears/Led/Right/180Deg",
          "Ears/Led/Right/216Deg",
          "Ears/Led/Right/252Deg",
          "Ears/Led/Right/288Deg",
          "Ears/Led/Right/324Deg",
          "ChestBoard/Led/Red",
          "ChestBoard/Led/Green",
          "ChestBoard/Led/Blue",
          "Head/Led/Rear/Left/0",
          "Head/Led/Rear/Left/1",
          "Head/Led/Rear/Left/2",
          "Head/Led/Rear/Right/0",
          "Head/Led/Rear/Right/1",
          "Head/Led/Rear/Right/2",
          "Head/Led/Middle/Right/0",
          "Head/Led/Front/Right/0",
          "Head/Led/Front/Right/1",
          "Head/Led/Front/Left/0",
          "Head/Led/Front/Left/1",
          "Head/Led/Middle/Left/0",
          "LFoot/Led/Red",
          "LFoot/Led/Green",
          "LFoot/Led/Blue",
          "RFoot/Led/Red",
          "RFoot/Led/Green",
          "RFoot/Led/Blue",
        ]


class ROSPublisher(object):
    def __init__(self, name, ros_msg, queue_size=1):
        self._publisher = rospy.Publisher(name, ros_msg, queue_size=queue_size)
        self._msg = ros_msg()

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    @staticmethod
    def string_list_to_float(lst):
        try:
          lst = re.sub('[{}]', '', lst)
          lst = lst.split(',')
          lst = [float(i) for i in lst]
        except ValueError as error:
          print(error)
          return []
        return lst


class TeamNUSTStatePublisher(ROSPublisher):
    def __init__(self):
        super(TeamNUSTStatePublisher, self).__init__('team_nust_state', TeamNUSTState)

    @property
    def motion_thread_period(self):
        return self._msg.motion_thread_period

    @motion_thread_period.setter
    def motion_thread_period(self, motion_thread_period):
        self._msg.motion_thread_period = int(motion_thread_period)

    @property
    def planning_thread_period(self):
        return self._msg.planning_thread_period

    @planning_thread_period.setter
    def planning_thread_period(self, planning_thread_period):
        self._msg.planning_thread_period = int(planning_thread_period)

    @property
    def gb_thread_period(self):
        return self._msg.gb_thread_period

    @gb_thread_period.setter
    def gb_thread_period(self, gb_thread_period):
        self._msg.gb_thread_period = int(gb_thread_period)

    @property
    def vision_thread_period(self):
        return self._msg.vision_thread_period

    @vision_thread_period.setter
    def vision_thread_period(self, vision_thread_period):
        self._msg.vision_thread_period = int(vision_thread_period)

    @property
    def localization_thread_period(self):
        return self._msg.localization_thread_period

    @localization_thread_period.setter
    def localization_thread_period(self, localization_thread_period):
        self._msg.localization_thread_period = int(localization_thread_period)

    @property
    def user_comm_thread_period(self):
        return self._msg.user_comm_thread_period

    @user_comm_thread_period.setter
    def user_comm_thread_period(self, user_comm_thread_period):
        self._msg.user_comm_thread_period = int(user_comm_thread_period)

    @property
    def game_comm_thread_period(self):
        return self._msg.game_comm_thread_period

    @game_comm_thread_period.setter
    def game_comm_thread_period(self, game_comm_thread_period):
        self._msg.game_comm_thread_period = int(game_comm_thread_period)

    @property
    def motion_time_taken(self):
        return self._msg.motion_time_taken

    @motion_time_taken.setter
    def motion_time_taken(self, motion_time_taken):
        self._msg.motion_time_taken = int(motion_time_taken)

    @property
    def planning_time_taken(self):
        return self._msg.planning_time_taken

    @planning_time_taken.setter
    def planning_time_taken(self, planning_time_taken):
        self._msg.planning_time_taken = int(planning_time_taken)

    @property
    def gb_time_taken(self):
        return self._msg.gb_time_taken

    @gb_time_taken.setter
    def gb_time_taken(self, gb_time_taken):
        self._msg.gb_time_taken = int(gb_time_taken)

    @property
    def vision_time_taken(self):
        return self._msg.vision_time_taken

    @vision_time_taken.setter
    def vision_time_taken(self, vision_time_taken):
        self._msg.vision_time_taken = int(vision_time_taken)

    @property
    def localization_time_taken(self):
        return self._msg.localization_time_taken

    @localization_time_taken.setter
    def localization_time_taken(self, localization_time_taken):
        self._msg.localization_time_taken = int(localization_time_taken)

    @property
    def user_comm_time_taken(self):
        return self._msg.user_comm_time_taken

    @user_comm_time_taken.setter
    def user_comm_time_taken(self, user_comm_time_taken):
        self._msg.user_comm_time_taken = int(user_comm_time_taken)

    @property
    def game_comm_time_taken(self):
        return self._msg.game_comm_time_taken

    @game_comm_time_taken.setter
    def game_comm_time_taken(self, game_comm_time_taken):
        self._msg.game_comm_time_taken = int(game_comm_time_taken)

    @property
    def heart_beat(self):
        return self._msg.heart_beat

    @heart_beat.setter
    def heart_beat(self, heart_beat):
        self._msg.heart_beat = int(heart_beat)

    @property
    def player_number(self):
        return self._msg.player_number

    @player_number.setter
    def player_number(self, player_number):
        self._msg.player_number = int(player_number)

    @property
    def team_number(self):
        return self._msg.team_number

    @team_number.setter
    def team_number(self, team_number):
        self._msg.team_number = int(team_number)

    @property
    def team_port(self):
        return self._msg.team_port

    @team_port.setter
    def team_port(self, team_port):
        self._msg.team_port = int(team_port)

    @property
    def team_color(self):
        return self._msg.team_color

    @team_color.setter
    def team_color(self, team_color):
        self._msg.team_color = int(team_color)

    @property
    def robocup_role(self):
        return self._msg.robocup_role

    @robocup_role.setter
    def robocup_role(self, robocup_role):
        self._msg.robocup_role = int(robocup_role)

    @property
    def robot_intention(self):
        return self._msg.robot_intention

    @robot_intention.setter
    def robot_intention(self, robot_intention):
        self._msg.robot_intention = int(robot_intention)

    @property
    def robot_pose_2d(self):
        return self._msg.robot_pose_2d

    @robot_pose_2d.setter
    def robot_pose_2d(self, robot_pose_2d):
        self._msg.robot_pose_2d = robot_pose_2d

    @robot_pose_2d.setter
    def robot_pose_2d(self, robot_pose_2d):
        if robot_pose_2d == None:
          return
        self._msg.robot_pose_2d = Pose2D(robot_pose_2d['pose2D'][0][0], robot_pose_2d['pose2D'][0][1], robot_pose_2d['pose2D'][0][2])

    @property
    def stiffness_state(self):
        return self._msg.stiffness_state

    @stiffness_state.setter
    def stiffness_state(self, stiffness_state):
        self._msg.stiffness_state = int(stiffness_state)

    @property
    def posture_state(self):
        return self._msg.posture_state

    @posture_state.setter
    def posture_state(self, posture_state):
        self._msg.posture_state = int(posture_state)

    @property
    def planning_state(self):
        return self._msg.planning_state

    @planning_state.setter
    def planning_state(self, planning_state):
        self._msg.planning_state = int(planning_state)

    @property
    def whistle_detected(self):
        return self._msg.whistle_detected

    @whistle_detected.setter
    def whistle_detected(self, whistle_detected):
        self._msg.whistle_detected = bool(whistle_detected)

    @property
    def robot_fallen(self):
        return self._msg.robot_fallen

    @robot_fallen.setter
    def robot_fallen(self, robot_fallen):
        self._msg.robot_fallen = bool(robot_fallen)

    @property
    def robot_in_motion(self):
        return self._msg.robot_in_motion

    @robot_in_motion.setter
    def robot_in_motion(self, robot_in_motion):
        self._msg.robot_in_motion = bool(robot_in_motion)

    @property
    def kick_target(self):
        return self._msg.kick_target

    @kick_target.setter
    def kick_target(self, kick_target):
        self._msg.kick_target = kick_target

    @kick_target.setter
    def kick_target(self, kick_target):
        if kick_target == None:
          return
        self._msg.kick_target = Point(kick_target[0], kick_target[1], 0.0)

    @property
    def foot_on_ground(self):
        return self._msg.foot_on_ground

    @foot_on_ground.setter
    def foot_on_ground(self, foot_on_ground):
        self._msg.foot_on_ground = foot_on_ground

    @foot_on_ground.setter
    def foot_on_ground(self, foot_on_ground):
        if foot_on_ground == None:
          return
        self._msg.foot_on_ground = foot_on_ground

    @property
    def l_foot_transform(self):
        return self._msg.l_foot_transform

    @l_foot_transform.setter
    def l_foot_transform(self, l_foot_transform):
        self._msg.l_foot_transform = l_foot_transform

    @l_foot_transform.setter
    def l_foot_transform(self, l_foot_transform):
        l_foot_transform_matrix = l_foot_transform
        if l_foot_transform_matrix == None:
          return
        l_foot_transform_matrix = np.array(l_foot_transform_matrix, ndmin=2).reshape(4, 4)
        l_foot_transform_matrix = np.linalg.inv(l_foot_transform_matrix)
        euler_angles = rotation_utils.rot_to_euler(l_foot_transform_matrix[0:3, 0:3])
        quaternion = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
        transform = Transform()
        transform.translation.x = l_foot_transform_matrix[0, 3]
        transform.translation.y = l_foot_transform_matrix[1, 3]
        transform.translation.z = l_foot_transform_matrix[2, 3]
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]
        self._msg.l_foot_transform = transform

    @property
    def r_foot_transform(self):
        return self._msg.r_foot_transform

    @r_foot_transform.setter
    def r_foot_transform(self, r_foot_transform):
        self._msg.r_foot_transform = r_foot_transform

    @r_foot_transform.setter
    def r_foot_transform(self, r_foot_transform):
        r_foot_transform_matrix = r_foot_transform
        if r_foot_transform_matrix == None:
          return
        r_foot_transform_matrix = np.array(r_foot_transform_matrix, ndmin=2).reshape(4, 4)
        r_foot_transform_matrix = np.linalg.inv(r_foot_transform_matrix)
        euler_angles = rotation_utils.rot_to_euler(r_foot_transform_matrix[0:3, 0:3])
        quaternion = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
        transform = Transform()
        transform.translation.x = r_foot_transform_matrix[0, 3]
        transform.translation.y = r_foot_transform_matrix[1, 3]
        transform.translation.z = r_foot_transform_matrix[2, 3]
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]
        self._msg.r_foot_transform = transform

    @property
    def move_target(self):
        return self._msg.move_target

    @move_target.setter
    def move_target(self, move_target):
        self._msg.move_target = move_target

    @move_target.setter
    def move_target(self, move_target):
        if move_target == None:
          return
        self._msg.move_target = Point(move_target['pose2D'][0][0], move_target['pose2D'][0][1], 0.0)


class TeamInfoPublisher(ROSPublisher):
    def __init__(self):
        super(TeamInfoPublisher, self).__init__('team_info', TeamInfo)

    @property
    def team_info(self):
        return self._msg

    @team_info.setter
    def team_info(self, team_info):
        if team_info == None:
          return
        team_robots = []
        for robot_info in team_info:
            tr = TeamRobot()
            tr.data_received = robot_info['dataReceived']
            tr.fallen = robot_info['fallen']
            tr.intention = robot_info['intention']
            tr.suggestion_to_me = robot_info['suggestionToMe']
            tr.pose_2d = Pose2D(robot_info['pose'][0][0], robot_info['pose'][0][1], robot_info['pose'][0][2])
            tr.walking_to = Point(robot_info['walkingTo'][0], robot_info['walkingTo'][1], 0.0)
            tr.shooting_to = Point(robot_info['shootingTo'][0], robot_info['shootingTo'][1], 0.0)
            team_robots.append(tr)
        self._msg.robots = team_robots

class BehaviorInfoPublisher(ROSPublisher):
    def __init__(self, name):
        super(BehaviorInfoPublisher, self).__init__(name, BehaviorInfo)

    @property
    def behavior_info(self):
        return self._msg

    @behavior_info.setter
    def behavior_info(self, info):
        if info == None:
          return
        self._msg.name = info['name']
        self._msg.fsm_state = info['fsmState']
        self._msg.initiated = info['initiated']
        self._msg.running = info['running']
        self._msg.finished = info['finished']
        self._msg.config = json.dumps(info['ConfigTree'])

class PBInfoPublisher(BehaviorInfoPublisher):
    def __init__(self):
        super(PBInfoPublisher, self).__init__('pb_info')

class MBInfoPublisher(BehaviorInfoPublisher):
    def __init__(self):
        super(MBInfoPublisher, self).__init__('mb_info')

class GBInfoPublisher(BehaviorInfoPublisher):
    def __init__(self):
        super(GBInfoPublisher, self).__init__('gb_info')

class ObsObstaclesPublisher(ROSPublisher):
    def __init__(self):
        super(ObsObstaclesPublisher, self).__init__('obs_obstacles', ObsObstacles)

    @property
    def obs_obstacles(self):
        return self._msg

    @obs_obstacles.setter
    def obs_obstacles(self, obs_obstacles):
        self._msg = obs_obstacles

    @obs_obstacles.setter
    def obs_obstacles(self, obs_obstacles):
        if obs_obstacles == None:
          return
        obstacles = []
        for ob_data in obs_obstacles:
            ob = Obstacle()
            print('ob_data', ob_data)
            ob.type = ob_data[0]
            ob.center.x = ob_data[1]
            ob.center.y = ob_data[2]
            ob.left_bound.x = ob_data[3]
            ob.left_bound.y = ob_data[4]
            ob.right_bound.x = ob_data[5]
            ob.right_bound.y = ob_data[6]
            ob.depth = ob_data[7]
            obstacles.append(ob)
        self._msg.obstacles = obstacles


class LocalizationStatePublisher(ROSPublisher):
    def __init__(self):
        super(LocalizationStatePublisher, self).__init__('localization_state', LocalizationState)

    @property
    def robot_localized(self):
        return self._msg.robot_localized

    @robot_localized.setter
    def robot_localized(self, robot_localized):
        self._msg.robot_localized = bool(robot_localized)

    @property
    def position_confidence(self):
        return self._msg.position_confidence

    @position_confidence.setter
    def position_confidence(self, position_confidence):
        self._msg.position_confidence = int(position_confidence)

    @property
    def side_confidence(self):
        return self._msg.side_confidence

    @side_confidence.setter
    def side_confidence(self, side_confidence):
        self._msg.side_confidence = int(side_confidence)

    @property
    def robot_on_side_line(self):
        return self._msg.robot_on_side_line

    @robot_on_side_line.setter
    def robot_on_side_line(self, robot_on_side_line):
        self._msg.robot_on_side_line = bool(robot_on_side_line)

    @property
    def localize_with_last_known(self):
        return self._msg.localize_with_last_known

    @localize_with_last_known.setter
    def localize_with_last_known(self, localize_with_last_known):
        self._msg.localize_with_last_known = bool(localize_with_last_known)

    @property
    def landmarks_found(self):
        return self._msg.landmarks_found

    @landmarks_found.setter
    def landmarks_found(self, landmarks_found):
        self._msg.landmarks_found = bool(landmarks_found)


class OccupancyGridPublisher(ROSPublisher):
    def __init__(self):
        super(OccupancyGridPublisher, self).__init__('occupancy_grid', OccupancyGrid)

    @property
    def occupancy_grid(self):
        return self._msg

    @occupancy_grid.setter
    def occupancy_grid(self, occupancy_grid):
        self._msg = occupancy_grid

    @occupancy_grid.setter
    def occupancy_grid(self, occupancy_grid):
        pass
        #og = OccupancyGrid()
        #og.info.resolution = occupancy_grid[0]
        #og.info.resolution = occupancy_grid[0]
        #og.info.width = occupancy_grid[1]
        #og.info.height = occupancy_grid[2]
        #og.info.origin.position.x = occupancy_grid[3]
        #og.info.origin.position.y = occupancy_grid[4]
        #og.info.origin.orientation.w = math.cos(occupancy_grid[5] / 2)
        #og.info.origin.orientation.z = math.sin(occupancy_grid[5] / 2)
        #data = occupancy_grid[6]


class GoalInfoPublisher(ROSPublisher):
    def __init__(self):
        super(GoalInfoPublisher, self).__init__('goal_info', GoalInfo)

    @property
    def goal_info(self):
        return self._msg

    @goal_info.setter
    def goal_info(self, goal_info):
        self._msg = goal_info

    @goal_info.setter
    def goal_info(self, goal_info):
        if goal_info == None:
          return
        self._msg.found = goal_info['found']
        self._msg.left_post = Point(goal_info['leftPost'][0], goal_info['leftPost'][1], 0.0)
        self._msg.right_post = Point(goal_info['rightPost'][0], goal_info['rightPost'][1], 0.0)
        self._msg.mid = Point(goal_info['mid'][0], goal_info['mid'][1], 0.0)
        self._msg.poseFromGoal = Pose2D(goal_info['poseFromGoal'][0][0], goal_info['poseFromGoal'][0][1], goal_info['poseFromGoal'][0][2])
        self._msg.goal_type = goal_info['type']


class BallInfoPublisher(ROSPublisher):
    def __init__(self):
        super(BallInfoPublisher, self).__init__('ball_info', BallInfo)

    @property
    def ball_info(self):
        return self._msg

    @ball_info.setter
    def ball_info(self, ball_info):
        self._msg = ball_info

    @ball_info.setter
    def ball_info_rel(self, ball_info_rel):
        if ball_info_rel == None:
          return
        self._msg.camera = ball_info_rel['camera']
        self._msg.found = ball_info_rel['found']
        self._msg.pos = Point(ball_info_rel['posRel'][0], ball_info_rel['posRel'][1], ball_info_rel['radius'])
        self._msg.vel = Point(ball_info_rel['velRel'][0], ball_info_rel['velRel'][1], 0.0)
        self._msg.image = Point(ball_info_rel['posImage'][0], ball_info_rel['posImage'][1], 0.0)
        self._msg.age = ball_info_rel['ballAge']
        self._msg.radius = ball_info_rel['radius']

    @ball_info.setter
    def ball_info_world(self, ball_info_world):
        if ball_info_world == None:
          return
        self._msg.world_pos = Point(ball_info_world['posWorld'][0], ball_info_world['posWorld'][1], 0.0)
        self._msg.world_vel = Point(ball_info_world['velWorld'][0], ball_info_world['velWorld'][1], 0.0)


class SensorsPublisher(ROSPublisher):
    def __init__(self, name, sensor_name):
        super(SensorsPublisher, self).__init__(name, SensorState)
        self._msg.name = sensor_name

    @property
    def sensor_name(self):
        return self._msg.name

    @sensor_name.setter
    def sensor_name(self, sensor_name):
        self._msg.name = sensor_name

    @property
    def sensor_value(self):
        return self._msg.value

    @sensor_value.setter
    def sensor_value(self, sensor_value):
        if sensor_value == None:
          return
        self._msg.value = sensor_value


class JointStatePublisher(ROSPublisher):
    def __init__(self):
        super(JointStatePublisher, self).__init__('joint_states', JointState)
        self._msg.name = SensorNames.JOINTS
        self._msg.position = [0] * len(SensorNames.JOINTS)

    @property
    def joint_positions(self):
        return self._msg.position

    @joint_positions.setter
    def joint_positions(self, joint_positions):
        if joint_positions == None:
          return
        for idx, joint in enumerate(joint_positions):
            real_idx = SensorNames.JOINTS.index(SensorNames.JOINTS_INCOMING[idx])
            self._msg.position[real_idx] = joint_positions[idx]
        #print('position updated', self._msg.position)

class JointInfoPublisher(ROSPublisher):
    def __init__(self):
        super(JointInfoPublisher, self).__init__('joint_info', JointInfo)
        self._msg.name = SensorNames.JOINTS

    @property
    def joint_stiffness(self):
        return self._msg.stiffness

    @joint_stiffness.setter
    def joint_stiffness(self, joint_stiffness):
        if joint_stiffness == None:
          return
        self._msg.stiffness = joint_stiffness

    @property
    def joint_temperature(self):
        return self._msg.temperature

    @joint_temperature.setter
    def joint_temperature(self, joint_temperature):
        if joint_temperature == None:
          return
        self._msg.temperature = joint_temperature

    @property
    def joint_current(self):
        return self._msg.current

    @joint_current.setter
    def joint_current(self, joint_current):
        if joint_current == None:
          return
        self._msg.current = joint_current


class ImagePublisher:
    def __init__(self, name):
        self._image_msg = Image
        self._bridge = cv_bridge.CvBridge()
        self._publisher = rospy.Publisher(name, Image, queue_size=1)

    def publish(self):
        self._image_msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._image_msg)

    def handle_image_msg(self, image_msg):
        np_bytes_array = np.frombuffer(bytearray.fromhex(image_msg), dtype=np.uint8)
        np_image = cv2.imdecode(np_bytes_array, 1)
        self._image_msg = self._bridge.cv2_to_imgmsg(np_image)


class UnknownLandmarkPublisher:
    def __init__(self, name):
        self._msg = PointCloud()
        self._msg.header.frame_id = 'world_frame'
        self._publisher = rospy.Publisher(name, PointCloud, queue_size=1)

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    def handle_landmark_data(self, _msg):
        self._msg = PointCloud()
        self._msg.header.frame_id = 'world_frame'
        bytes_array = bytearray.fromhex(_msg)
        float_array = struct.unpack(str(len(bytes_array) / 4) + 'f', bytes_array)
        for i in xrange(0, len(float_array), 2):
            p = Point32()
            p.x = float_array[i]
            p.y = float_array[i+1]
            # Fixed height according to rviz
            p.z = 0.052
            self._msg.points.append(p)


class KnownLandmarkPublisher:
    def __init__(self, name):
        self._msg = ObsLandmarks()
        self._msg.header.frame_id = 'world_frame'
        self._publisher = rospy.Publisher(name, ObsLandmarks, queue_size=1)

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    def handle_landmark_data(self, _msg):
        self._msg.landmarks = []
        self._msg = ObsLandmarks()
        self._msg.header.frame_id = 'world_frame'
        bytes_array = bytearray.fromhex(_msg)
        float_array = struct.unpack(str(len(bytes_array) / 4) + 'f', bytes_array)
        for i in xrange(0, len(float_array), 3):
            landmark = Landmark()
            landmark.type = int(float_array[i])
            landmark.pos.x = float_array[i+1]
            landmark.pos.y = float_array[i+2]
            # Fixed height according to rviz
            landmark.pos.z = 0.052
            self._msg.landmarks.append(landmark)


class MessagePublisher:
    def __init__(self, name):
        self._msg = String()
        self._publisher = rospy.Publisher(name, String, queue_size=1)

    def publish(self):
        self._publisher.publish(self._msg)

    def handle_log_message(self, _msg):
        self._msg = _msg


class PFStatePublisher:
    def __init__(self, name):
        self._msg = PointCloud()
        self._msg.header.frame_id = 'world_frame'
        self._publisher = rospy.Publisher(name, PointCloud, queue_size=1)

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    def handle_particle_data(self, _msg):
        self._msg = PointCloud()
        self._msg.header.frame_id = 'world_frame'
        bytes_array = bytearray.fromhex(_msg)
        float_array = struct.unpack(str(len(bytes_array) / 4) + 'f', bytes_array)
        for i in xrange(0, len(float_array), 2):
            p = Point32()
            p.x = float_array[i]
            p.y = float_array[i+1]
            # Fixed height according to rviz
            p.z = 0.1
            self._msg.points.append(p)


class FootstepsPublisher:
    def __init__(self, name):
        self._msg = StepTargetArr()
        self._msg.header.frame_id = 'world_frame'
        self._publisher = rospy.Publisher(name, StepTargetArr, queue_size=1)

    def publish(self):
        self._msg.header.stamp = rospy.Time.now()
        self._publisher.publish(self._msg)

    def handle_footsteps_data(self, _msg):
        self._msg = StepTargetArr()
        self._msg.header.frame_id = 'world_frame'
        bytes_array = bytearray.fromhex(_msg)
        float_array = struct.unpack(str(len(bytes_array) / 4) + 'f', bytes_array)
        for i in xrange(0, len(float_array), 4):
            step = StepTarget()
            step.pose.x = float_array[i]
            step.pose.y = float_array[i+1]
            step.pose.theta = float_array[i+2]
            step.leg = int(float_array[i+3])
            self._msg.steps.append(step)

