import re
import json
from publishers import \
    SensorNames, JointStatePublisher, JointInfoPublisher, \
    SensorsPublisher, BallInfoPublisher, GoalInfoPublisher, OccupancyGridPublisher, \
    LocalizationStatePublisher, ObsObstaclesPublisher, TeamInfoPublisher, \
    TeamNUSTStatePublisher, PBInfoPublisher, MBInfoPublisher, GBInfoPublisher


class MemoryDataPublisher:
    MEMORY_DATA_REGEX = ',(?![^{}]*\})'

    def __init__(self):
        self.memory_data_size = None
        self.memory_header = None
        self.joint_state_publisher = JointStatePublisher()
        self.joint_info_publisher = JointInfoPublisher()
        self.hand_sensors_publisher = SensorsPublisher('nao_hand_sensors', SensorNames.HANDS)
        self.touch_sensors_publisher = SensorsPublisher('nao_touch_sensors', SensorNames.TOUCH)
        self.switch_sensors_publisher = SensorsPublisher('nao_switch_sensors', SensorNames.SWITCH)
        self.battery_sensors_publisher = SensorsPublisher('nao_battery_sensors', SensorNames.BATTERY)
        self.inertial_sensors_publisher = SensorsPublisher('nao_inertial_sensors', SensorNames.INERTIAL)
        self.sonar_sensors_publisher = SensorsPublisher('nao_sonar_sensors', SensorNames.SONAR)
        self.fsr_sensors_publisher = SensorsPublisher('nao_fsr_sensors', SensorNames.FORCE)
        self.led_sensors_publisher = SensorsPublisher('nao_led_sensors', SensorNames.LED)
        self.ball_info_publisher = BallInfoPublisher()
        self.goal_info_publisher = GoalInfoPublisher()
        # Expensive to display get occupancy map over network
        # self.occupancy_grid_publisher = OccupancyGridPublisher()
        self.localization_state_publisher = LocalizationStatePublisher()
        self.obs_obstacles_publisher = ObsObstaclesPublisher()
        self.team_info_publisher = TeamInfoPublisher()
        self.pb_info_publisher = PBInfoPublisher()
        self.mb_info_publisher = MBInfoPublisher()
        self.gb_info_publisher = GBInfoPublisher()
        self.team_nust_state_publisher = TeamNUSTStatePublisher()

    def publish(self):
        self.team_nust_state_publisher.publish()
        self.joint_info_publisher.publish()
        self.ball_info_publisher.publish()
        self.goal_info_publisher.publish()
        #self.occupancy_grid_publisher.publish()
        self.obs_obstacles_publisher.publish()
        self.localization_state_publisher.publish()

    def handle_memory_data_msg(self, memory_data):
        for name in memory_data:
            if name == 'motionThreadPeriod':
              self.team_nust_state_publisher.motion_thread_period = memory_data[name]
            elif name == 'planningThreadPeriod':
              self.team_nust_state_publisher.planning_thread_period = memory_data[name]
            elif name == 'gbThreadPeriod':
              self.team_nust_state_publisher.gb_thread_period = memory_data[name]
            elif name == 'visionThreadPeriod':
              self.team_nust_state_publisher.vision_thread_period = memory_data[name]
            elif name == 'localizationThreadPeriod':
              self.team_nust_state_publisher.localization_thread_period = memory_data[name]
            elif name == 'gameCommThreadPeriod':
              self.team_nust_state_publisher.game_comm_thread_period = memory_data[name]
            elif name == 'userCommThreadPeriod':
              self.team_nust_state_publisher.user_comm_thread_period = memory_data[name]
            elif name == 'lolaThreadPeriod':
              self.team_nust_state_publisher.l_thread_period = memory_data[name]
            elif name == 'motionThreadTimeTaken':
              self.team_nust_state_publisher.motion_time_taken = memory_data[name]
            elif name == 'planningThreadTimeTaken':
              self.team_nust_state_publisher.planning_time_taken = memory_data[name]
            elif name == 'gbThreadTimeTaken':
              self.team_nust_state_publisher.gb_time_taken = memory_data[name]
            elif name == 'visionThreadTimeTaken':
              self.team_nust_state_publisher.vision_time_taken = memory_data[name]
            elif name == 'localizationThreadTimeTaken':
              self.team_nust_state_publisher.localization_time_taken = memory_data[name]
            elif name == 'gameCommThreadTimeTaken':
              self.team_nust_state_publisher.game_comm_time_taken = memory_data[name]
            elif name == 'userCommThreadTimeTaken':
              self.team_nust_state_publisher.user_comm_time_taken = memory_data[name]
            elif name == 'lolaThreadTimeTaken':
              self.team_nust_state_publisher.lola_time_taken = memory_data[name]
            elif name == 'heartBeat':
              self.team_nust_state_publisher.heart_beat = memory_data[name]
            elif name == 'jointPositionSensors':
              self.joint_state_publisher.joint_positions = memory_data[name]
              self.joint_state_publisher.publish()
            elif name == 'jointStiffnessSensors':
              self.joint_info_publisher.joint_stiffness = memory_data[name]
            elif name == 'jointTemperatureSensors':
              self.joint_info_publisher.joint_temperature = memory_data[name]
            elif name == 'jointCurrentSensors':
              self.joint_info_publisher.joint_current = memory_data[name]
            elif name == 'jointCurrentSensors':
              self.joint_info_publisher.sensor_value = memory_data[name]
            elif name == 'handSensors':
              self.hand_sensors_publisher.sensor_value = memory_data[name]
              self.hand_sensors_publisher.publish()
            elif name == 'touchSensors':
                self.touch_sensors_publisher.sensor_value = memory_data[name]
                self.touch_sensors_publisher.publish()
            elif name == 'switchSensors':
                self.switch_sensors_publisher.sensor_value = memory_data[name]
                self.switch_sensors_publisher.publish()
            elif name == 'batterySensors':
                self.battery_sensors_publisher.sensor_value = memory_data[name]
                self.battery_sensors_publisher.publish()
            elif name == 'inertialSensors':
                self.inertial_sensors_publisher.sensor_value = memory_data[name]
                self.inertial_sensors_publisher.publish()
            elif name == 'fsrSensors':
                self.fsr_sensors_publisher.sensor_value = memory_data[name]
                self.fsr_sensors_publisher.publish()
            elif name == 'sonarSensors':
                self.sonar_sensors_publisher.sensor_value = memory_data[name]
                self.sonar_sensors_publisher.publish()
            #elif name == 'ledSensors':
            #    self.led_sensors_publisher.sensor_value = memory_data[name]
            #    .publish()
            elif name == 'ballInfo':
                self.ball_info_publisher.ball_info_rel = memory_data[name]
                self.ball_info_publisher.publish()
            elif name == 'worldBallInfo':
                self.ball_info_publisher.ball_info_world = memory_data[name]
                self.ball_info_publisher.publish()
            elif name == 'goalInfo':
                self.goal_info_publisher.goal_info = memory_data[name]
                self.goal_info_publisher.publish()
            elif name == 'kickTarget':
                self.team_nust_state_publisher.kick_target = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'footOnGround':
                self.team_nust_state_publisher.foot_on_ground = memory_data[name]
                self.team_nust_state_publisher.publish()
            # Ignored memory variables upperCamInFeet,
            # Ignored memory variables lowerCamInFeet,
            elif name == 'lFootOnGround':
                self.team_nust_state_publisher.l_foot_transform = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'rFootOnGround':
                self.team_nust_state_publisher.r_foot_transform = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'obstaclesObs':
                self.obs_obstacles_publisher.obs_obstacles = memory_data[name]
                self.obs_obstacles_publisher.publish()
            elif name == 'obstaclesComm':
                self.obs_obstacles_publisher.comm_obstacles = memory_data[name]
                self.obs_obstacles_publisher.publish()
            elif name == 'robotPose2D':
                self.team_nust_state_publisher.robot_pose_2d = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'lastKnownPose2D':
                self.team_nust_state_publisher.last_known_pose_2d = memory_data[name]
                self.team_nust_state_publisher.publish()
            # No need to show these in gui
            # fieldWidth
            # fieldHeight
            # Expensive to display get occupancy map over network
            # elif name == 'occupancyMap':
            #     self.occupancy_grid_publisher.occupancy_grid = memory_data[name]
            #     self.occupancy_grid_publisher.publish()
            elif name == 'stiffnessState':
                self.team_nust_state_publisher.stiffness_state = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'postureState':
                self.team_nust_state_publisher.posture_state = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'planningState':
                self.team_nust_state_publisher.planning_state = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'robotFallen':
                self.team_nust_state_publisher.robot_fallen = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'robotInMotion':
                self.team_nust_state_publisher.robot_in_motion = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'currStepLeg':
                self.team_nust_state_publisher.curr_step_leg = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'robotLocalized':
                self.localization_state_publisher.robot_localized = memory_data[name]
            elif name == 'positionConfidence':
                self.localization_state_publisher.position_confidence = memory_data[name]
            elif name == 'sideConfidence':
                self.localization_state_publisher.side_confidence = memory_data[name]
            elif name == 'robotOnSideLine':
                self.localization_state_publisher.robot_on_sideLine = memory_data[name]
            elif name == 'localizeWithLastKnown':
                self.localization_state_publisher.localize_with_last_known = memory_data[name]
            elif name == 'landmarksFound':
                self.localization_state_publisher.landmarks_found = memory_data[name]
            elif name == 'robocupRole':
                self.team_nust_state_publisher.robocup_role = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'robotIntention':
                self.team_nust_state_publisher.robot_intention = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'nFootsteps':
                self.team_nust_state_publisher.n = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'moveTarget':
                self.team_nust_state_publisher.move_target = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'moveTarget':
                self.team_nust_state_publisher.move_target = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'gameData':
                self.team_nust_state_publisher.game_data = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'playerNumber':
                self.team_nust_state_publisher.player_number = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'teamNumber':
                self.team_nust_state_publisher.team_number = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'teamPort':
                self.team_nust_state_publisher.team_port = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'teamColor':
                self.team_nust_state_publisher.team_color = memory_data[name]
                self.team_nust_state_publisher.publish()
            elif name == 'teamRobots':
                self.team_info_publisher.team_info = memory_data[name]
                self.team_info_publisher.publish()
            elif name == 'whistleDetected':
                self.team_nust_state_publisher.whistle_detected = memory_data[name]
                self.team_nust_state_publisher.publish() 
            elif name == 'landmarksFound':
                self.team_nust_state_publisher.landmarks_found = memory_data[name]
                self.team_nust_state_publisher.publish() 
            elif name == "pBehaviorInfo":
                self.pb_info_publisher.behavior_info = memory_data[name]
                self.pb_info_publisher.publish()
            elif name == "mBehaviorInfo":
                self.mb_info_publisher.behavior_info = memory_data[name]
                self.mb_info_publisher.publish()
            elif name == "gBehaviorInfo":
                self.gb_info_publisher.behavior_info = memory_data[name]
                self.gb_info_publisher.publish()
