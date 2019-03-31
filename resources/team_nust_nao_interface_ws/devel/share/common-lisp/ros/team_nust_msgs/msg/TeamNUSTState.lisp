; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude TeamNUSTState.msg.html

(cl:defclass <TeamNUSTState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motion_thread_period
    :reader motion_thread_period
    :initarg :motion_thread_period
    :type cl:integer
    :initform 0)
   (planning_thread_period
    :reader planning_thread_period
    :initarg :planning_thread_period
    :type cl:integer
    :initform 0)
   (sb_thread_period
    :reader sb_thread_period
    :initarg :sb_thread_period
    :type cl:integer
    :initform 0)
   (vision_thread_period
    :reader vision_thread_period
    :initarg :vision_thread_period
    :type cl:integer
    :initform 0)
   (localization_thread_period
    :reader localization_thread_period
    :initarg :localization_thread_period
    :type cl:integer
    :initform 0)
   (game_comm_thread_period
    :reader game_comm_thread_period
    :initarg :game_comm_thread_period
    :type cl:integer
    :initform 0)
   (user_comm_thread_period
    :reader user_comm_thread_period
    :initarg :user_comm_thread_period
    :type cl:integer
    :initform 0)
   (heart_beat
    :reader heart_beat
    :initarg :heart_beat
    :type cl:integer
    :initform 0)
   (player_number
    :reader player_number
    :initarg :player_number
    :type cl:integer
    :initform 0)
   (team_number
    :reader team_number
    :initarg :team_number
    :type cl:integer
    :initform 0)
   (team_port
    :reader team_port
    :initarg :team_port
    :type cl:integer
    :initform 0)
   (team_color
    :reader team_color
    :initarg :team_color
    :type cl:integer
    :initform 0)
   (robocup_role
    :reader robocup_role
    :initarg :robocup_role
    :type cl:integer
    :initform 0)
   (robot_intention
    :reader robot_intention
    :initarg :robot_intention
    :type cl:integer
    :initform 0)
   (robot_pose_2d
    :reader robot_pose_2d
    :initarg :robot_pose_2d
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (stiffness_state
    :reader stiffness_state
    :initarg :stiffness_state
    :type cl:integer
    :initform 0)
   (posture_state
    :reader posture_state
    :initarg :posture_state
    :type cl:integer
    :initform 0)
   (planning_state
    :reader planning_state
    :initarg :planning_state
    :type cl:integer
    :initform 0)
   (whistle_detected
    :reader whistle_detected
    :initarg :whistle_detected
    :type cl:boolean
    :initform cl:nil)
   (robot_fallen
    :reader robot_fallen
    :initarg :robot_fallen
    :type cl:boolean
    :initform cl:nil)
   (robot_in_motion
    :reader robot_in_motion
    :initarg :robot_in_motion
    :type cl:boolean
    :initform cl:nil)
   (kick_target
    :reader kick_target
    :initarg :kick_target
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (move_target
    :reader move_target
    :initarg :move_target
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (foot_on_ground
    :reader foot_on_ground
    :initarg :foot_on_ground
    :type cl:integer
    :initform 0)
   (l_foot_transform
    :reader l_foot_transform
    :initarg :l_foot_transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (r_foot_transform
    :reader r_foot_transform
    :initarg :r_foot_transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform)))
)

(cl:defclass TeamNUSTState (<TeamNUSTState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TeamNUSTState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TeamNUSTState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<TeamNUSTState> is deprecated: use team_nust_msgs-msg:TeamNUSTState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'motion_thread_period-val :lambda-list '(m))
(cl:defmethod motion_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:motion_thread_period-val is deprecated.  Use team_nust_msgs-msg:motion_thread_period instead.")
  (motion_thread_period m))

(cl:ensure-generic-function 'planning_thread_period-val :lambda-list '(m))
(cl:defmethod planning_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:planning_thread_period-val is deprecated.  Use team_nust_msgs-msg:planning_thread_period instead.")
  (planning_thread_period m))

(cl:ensure-generic-function 'sb_thread_period-val :lambda-list '(m))
(cl:defmethod sb_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:sb_thread_period-val is deprecated.  Use team_nust_msgs-msg:sb_thread_period instead.")
  (sb_thread_period m))

(cl:ensure-generic-function 'vision_thread_period-val :lambda-list '(m))
(cl:defmethod vision_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:vision_thread_period-val is deprecated.  Use team_nust_msgs-msg:vision_thread_period instead.")
  (vision_thread_period m))

(cl:ensure-generic-function 'localization_thread_period-val :lambda-list '(m))
(cl:defmethod localization_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:localization_thread_period-val is deprecated.  Use team_nust_msgs-msg:localization_thread_period instead.")
  (localization_thread_period m))

(cl:ensure-generic-function 'game_comm_thread_period-val :lambda-list '(m))
(cl:defmethod game_comm_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:game_comm_thread_period-val is deprecated.  Use team_nust_msgs-msg:game_comm_thread_period instead.")
  (game_comm_thread_period m))

(cl:ensure-generic-function 'user_comm_thread_period-val :lambda-list '(m))
(cl:defmethod user_comm_thread_period-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:user_comm_thread_period-val is deprecated.  Use team_nust_msgs-msg:user_comm_thread_period instead.")
  (user_comm_thread_period m))

(cl:ensure-generic-function 'heart_beat-val :lambda-list '(m))
(cl:defmethod heart_beat-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:heart_beat-val is deprecated.  Use team_nust_msgs-msg:heart_beat instead.")
  (heart_beat m))

(cl:ensure-generic-function 'player_number-val :lambda-list '(m))
(cl:defmethod player_number-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:player_number-val is deprecated.  Use team_nust_msgs-msg:player_number instead.")
  (player_number m))

(cl:ensure-generic-function 'team_number-val :lambda-list '(m))
(cl:defmethod team_number-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:team_number-val is deprecated.  Use team_nust_msgs-msg:team_number instead.")
  (team_number m))

(cl:ensure-generic-function 'team_port-val :lambda-list '(m))
(cl:defmethod team_port-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:team_port-val is deprecated.  Use team_nust_msgs-msg:team_port instead.")
  (team_port m))

(cl:ensure-generic-function 'team_color-val :lambda-list '(m))
(cl:defmethod team_color-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:team_color-val is deprecated.  Use team_nust_msgs-msg:team_color instead.")
  (team_color m))

(cl:ensure-generic-function 'robocup_role-val :lambda-list '(m))
(cl:defmethod robocup_role-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robocup_role-val is deprecated.  Use team_nust_msgs-msg:robocup_role instead.")
  (robocup_role m))

(cl:ensure-generic-function 'robot_intention-val :lambda-list '(m))
(cl:defmethod robot_intention-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_intention-val is deprecated.  Use team_nust_msgs-msg:robot_intention instead.")
  (robot_intention m))

(cl:ensure-generic-function 'robot_pose_2d-val :lambda-list '(m))
(cl:defmethod robot_pose_2d-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_pose_2d-val is deprecated.  Use team_nust_msgs-msg:robot_pose_2d instead.")
  (robot_pose_2d m))

(cl:ensure-generic-function 'stiffness_state-val :lambda-list '(m))
(cl:defmethod stiffness_state-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:stiffness_state-val is deprecated.  Use team_nust_msgs-msg:stiffness_state instead.")
  (stiffness_state m))

(cl:ensure-generic-function 'posture_state-val :lambda-list '(m))
(cl:defmethod posture_state-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:posture_state-val is deprecated.  Use team_nust_msgs-msg:posture_state instead.")
  (posture_state m))

(cl:ensure-generic-function 'planning_state-val :lambda-list '(m))
(cl:defmethod planning_state-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:planning_state-val is deprecated.  Use team_nust_msgs-msg:planning_state instead.")
  (planning_state m))

(cl:ensure-generic-function 'whistle_detected-val :lambda-list '(m))
(cl:defmethod whistle_detected-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:whistle_detected-val is deprecated.  Use team_nust_msgs-msg:whistle_detected instead.")
  (whistle_detected m))

(cl:ensure-generic-function 'robot_fallen-val :lambda-list '(m))
(cl:defmethod robot_fallen-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_fallen-val is deprecated.  Use team_nust_msgs-msg:robot_fallen instead.")
  (robot_fallen m))

(cl:ensure-generic-function 'robot_in_motion-val :lambda-list '(m))
(cl:defmethod robot_in_motion-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_in_motion-val is deprecated.  Use team_nust_msgs-msg:robot_in_motion instead.")
  (robot_in_motion m))

(cl:ensure-generic-function 'kick_target-val :lambda-list '(m))
(cl:defmethod kick_target-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:kick_target-val is deprecated.  Use team_nust_msgs-msg:kick_target instead.")
  (kick_target m))

(cl:ensure-generic-function 'move_target-val :lambda-list '(m))
(cl:defmethod move_target-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:move_target-val is deprecated.  Use team_nust_msgs-msg:move_target instead.")
  (move_target m))

(cl:ensure-generic-function 'foot_on_ground-val :lambda-list '(m))
(cl:defmethod foot_on_ground-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:foot_on_ground-val is deprecated.  Use team_nust_msgs-msg:foot_on_ground instead.")
  (foot_on_ground m))

(cl:ensure-generic-function 'l_foot_transform-val :lambda-list '(m))
(cl:defmethod l_foot_transform-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:l_foot_transform-val is deprecated.  Use team_nust_msgs-msg:l_foot_transform instead.")
  (l_foot_transform m))

(cl:ensure-generic-function 'r_foot_transform-val :lambda-list '(m))
(cl:defmethod r_foot_transform-val ((m <TeamNUSTState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:r_foot_transform-val is deprecated.  Use team_nust_msgs-msg:r_foot_transform instead.")
  (r_foot_transform m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TeamNUSTState>) ostream)
  "Serializes a message object of type '<TeamNUSTState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'motion_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'planning_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sb_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vision_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'localization_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'game_comm_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'user_comm_thread_period)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'heart_beat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'player_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'team_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'team_port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'team_color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'robocup_role)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'robot_intention)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pose_2d) ostream)
  (cl:let* ((signed (cl:slot-value msg 'stiffness_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'posture_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'planning_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'whistle_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_fallen) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_in_motion) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'kick_target) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'move_target) ostream)
  (cl:let* ((signed (cl:slot-value msg 'foot_on_ground)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'l_foot_transform) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r_foot_transform) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TeamNUSTState>) istream)
  "Deserializes a message object of type '<TeamNUSTState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motion_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planning_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sb_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vision_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'localization_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'game_comm_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'user_comm_thread_period) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'heart_beat) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'player_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'team_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'team_port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'team_color) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robocup_role) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_intention) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pose_2d) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stiffness_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'posture_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planning_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'whistle_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'robot_fallen) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'robot_in_motion) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'kick_target) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'move_target) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'foot_on_ground) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'l_foot_transform) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r_foot_transform) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TeamNUSTState>)))
  "Returns string type for a message object of type '<TeamNUSTState>"
  "team_nust_msgs/TeamNUSTState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TeamNUSTState)))
  "Returns string type for a message object of type 'TeamNUSTState"
  "team_nust_msgs/TeamNUSTState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TeamNUSTState>)))
  "Returns md5sum for a message object of type '<TeamNUSTState>"
  "f38be50129fbe02202e870fd322ce791")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TeamNUSTState)))
  "Returns md5sum for a message object of type 'TeamNUSTState"
  "f38be50129fbe02202e870fd322ce791")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TeamNUSTState>)))
  "Returns full string definition for message of type '<TeamNUSTState>"
  (cl:format cl:nil "Header header~%int32 motion_thread_period~%int32 planning_thread_period~%int32 sb_thread_period~%int32 vision_thread_period~%int32 localization_thread_period~%int32 game_comm_thread_period~%int32 user_comm_thread_period~%int32 heart_beat~%int32 player_number~%int32 team_number~%int32 team_port~%int32 team_color~%int32 robocup_role~%int32 robot_intention~%geometry_msgs/Pose2D robot_pose_2d~%int32 stiffness_state~%int32 posture_state~%int32 planning_state~%bool whistle_detected~%bool robot_fallen~%bool robot_in_motion~%geometry_msgs/Point kick_target~%geometry_msgs/Point move_target~%int32 foot_on_ground~%geometry_msgs/Transform l_foot_transform~%geometry_msgs/Transform r_foot_transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TeamNUSTState)))
  "Returns full string definition for message of type 'TeamNUSTState"
  (cl:format cl:nil "Header header~%int32 motion_thread_period~%int32 planning_thread_period~%int32 sb_thread_period~%int32 vision_thread_period~%int32 localization_thread_period~%int32 game_comm_thread_period~%int32 user_comm_thread_period~%int32 heart_beat~%int32 player_number~%int32 team_number~%int32 team_port~%int32 team_color~%int32 robocup_role~%int32 robot_intention~%geometry_msgs/Pose2D robot_pose_2d~%int32 stiffness_state~%int32 posture_state~%int32 planning_state~%bool whistle_detected~%bool robot_fallen~%bool robot_in_motion~%geometry_msgs/Point kick_target~%geometry_msgs/Point move_target~%int32 foot_on_ground~%geometry_msgs/Transform l_foot_transform~%geometry_msgs/Transform r_foot_transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TeamNUSTState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pose_2d))
     4
     4
     4
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'kick_target))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'move_target))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'l_foot_transform))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r_foot_transform))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TeamNUSTState>))
  "Converts a ROS message object to a list"
  (cl:list 'TeamNUSTState
    (cl:cons ':header (header msg))
    (cl:cons ':motion_thread_period (motion_thread_period msg))
    (cl:cons ':planning_thread_period (planning_thread_period msg))
    (cl:cons ':sb_thread_period (sb_thread_period msg))
    (cl:cons ':vision_thread_period (vision_thread_period msg))
    (cl:cons ':localization_thread_period (localization_thread_period msg))
    (cl:cons ':game_comm_thread_period (game_comm_thread_period msg))
    (cl:cons ':user_comm_thread_period (user_comm_thread_period msg))
    (cl:cons ':heart_beat (heart_beat msg))
    (cl:cons ':player_number (player_number msg))
    (cl:cons ':team_number (team_number msg))
    (cl:cons ':team_port (team_port msg))
    (cl:cons ':team_color (team_color msg))
    (cl:cons ':robocup_role (robocup_role msg))
    (cl:cons ':robot_intention (robot_intention msg))
    (cl:cons ':robot_pose_2d (robot_pose_2d msg))
    (cl:cons ':stiffness_state (stiffness_state msg))
    (cl:cons ':posture_state (posture_state msg))
    (cl:cons ':planning_state (planning_state msg))
    (cl:cons ':whistle_detected (whistle_detected msg))
    (cl:cons ':robot_fallen (robot_fallen msg))
    (cl:cons ':robot_in_motion (robot_in_motion msg))
    (cl:cons ':kick_target (kick_target msg))
    (cl:cons ':move_target (move_target msg))
    (cl:cons ':foot_on_ground (foot_on_ground msg))
    (cl:cons ':l_foot_transform (l_foot_transform msg))
    (cl:cons ':r_foot_transform (r_foot_transform msg))
))
