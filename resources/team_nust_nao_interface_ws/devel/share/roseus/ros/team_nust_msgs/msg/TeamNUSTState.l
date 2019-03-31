;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::TeamNUSTState)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'TeamNUSTState (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::TEAMNUSTSTATE")
  (make-package "TEAM_NUST_MSGS::TEAMNUSTSTATE"))

(in-package "ROS")
;;//! \htmlinclude TeamNUSTState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::TeamNUSTState
  :super ros::object
  :slots (_header _motion_thread_period _planning_thread_period _sb_thread_period _vision_thread_period _localization_thread_period _game_comm_thread_period _user_comm_thread_period _heart_beat _player_number _team_number _team_port _team_color _robocup_role _robot_intention _robot_pose_2d _stiffness_state _posture_state _planning_state _whistle_detected _robot_fallen _robot_in_motion _kick_target _move_target _foot_on_ground _l_foot_transform _r_foot_transform ))

(defmethod team_nust_msgs::TeamNUSTState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:motion_thread_period __motion_thread_period) 0)
    ((:planning_thread_period __planning_thread_period) 0)
    ((:sb_thread_period __sb_thread_period) 0)
    ((:vision_thread_period __vision_thread_period) 0)
    ((:localization_thread_period __localization_thread_period) 0)
    ((:game_comm_thread_period __game_comm_thread_period) 0)
    ((:user_comm_thread_period __user_comm_thread_period) 0)
    ((:heart_beat __heart_beat) 0)
    ((:player_number __player_number) 0)
    ((:team_number __team_number) 0)
    ((:team_port __team_port) 0)
    ((:team_color __team_color) 0)
    ((:robocup_role __robocup_role) 0)
    ((:robot_intention __robot_intention) 0)
    ((:robot_pose_2d __robot_pose_2d) (instance geometry_msgs::Pose2D :init))
    ((:stiffness_state __stiffness_state) 0)
    ((:posture_state __posture_state) 0)
    ((:planning_state __planning_state) 0)
    ((:whistle_detected __whistle_detected) nil)
    ((:robot_fallen __robot_fallen) nil)
    ((:robot_in_motion __robot_in_motion) nil)
    ((:kick_target __kick_target) (instance geometry_msgs::Point :init))
    ((:move_target __move_target) (instance geometry_msgs::Point :init))
    ((:foot_on_ground __foot_on_ground) 0)
    ((:l_foot_transform __l_foot_transform) (instance geometry_msgs::Transform :init))
    ((:r_foot_transform __r_foot_transform) (instance geometry_msgs::Transform :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _motion_thread_period (round __motion_thread_period))
   (setq _planning_thread_period (round __planning_thread_period))
   (setq _sb_thread_period (round __sb_thread_period))
   (setq _vision_thread_period (round __vision_thread_period))
   (setq _localization_thread_period (round __localization_thread_period))
   (setq _game_comm_thread_period (round __game_comm_thread_period))
   (setq _user_comm_thread_period (round __user_comm_thread_period))
   (setq _heart_beat (round __heart_beat))
   (setq _player_number (round __player_number))
   (setq _team_number (round __team_number))
   (setq _team_port (round __team_port))
   (setq _team_color (round __team_color))
   (setq _robocup_role (round __robocup_role))
   (setq _robot_intention (round __robot_intention))
   (setq _robot_pose_2d __robot_pose_2d)
   (setq _stiffness_state (round __stiffness_state))
   (setq _posture_state (round __posture_state))
   (setq _planning_state (round __planning_state))
   (setq _whistle_detected __whistle_detected)
   (setq _robot_fallen __robot_fallen)
   (setq _robot_in_motion __robot_in_motion)
   (setq _kick_target __kick_target)
   (setq _move_target __move_target)
   (setq _foot_on_ground (round __foot_on_ground))
   (setq _l_foot_transform __l_foot_transform)
   (setq _r_foot_transform __r_foot_transform)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:motion_thread_period
   (&optional __motion_thread_period)
   (if __motion_thread_period (setq _motion_thread_period __motion_thread_period)) _motion_thread_period)
  (:planning_thread_period
   (&optional __planning_thread_period)
   (if __planning_thread_period (setq _planning_thread_period __planning_thread_period)) _planning_thread_period)
  (:sb_thread_period
   (&optional __sb_thread_period)
   (if __sb_thread_period (setq _sb_thread_period __sb_thread_period)) _sb_thread_period)
  (:vision_thread_period
   (&optional __vision_thread_period)
   (if __vision_thread_period (setq _vision_thread_period __vision_thread_period)) _vision_thread_period)
  (:localization_thread_period
   (&optional __localization_thread_period)
   (if __localization_thread_period (setq _localization_thread_period __localization_thread_period)) _localization_thread_period)
  (:game_comm_thread_period
   (&optional __game_comm_thread_period)
   (if __game_comm_thread_period (setq _game_comm_thread_period __game_comm_thread_period)) _game_comm_thread_period)
  (:user_comm_thread_period
   (&optional __user_comm_thread_period)
   (if __user_comm_thread_period (setq _user_comm_thread_period __user_comm_thread_period)) _user_comm_thread_period)
  (:heart_beat
   (&optional __heart_beat)
   (if __heart_beat (setq _heart_beat __heart_beat)) _heart_beat)
  (:player_number
   (&optional __player_number)
   (if __player_number (setq _player_number __player_number)) _player_number)
  (:team_number
   (&optional __team_number)
   (if __team_number (setq _team_number __team_number)) _team_number)
  (:team_port
   (&optional __team_port)
   (if __team_port (setq _team_port __team_port)) _team_port)
  (:team_color
   (&optional __team_color)
   (if __team_color (setq _team_color __team_color)) _team_color)
  (:robocup_role
   (&optional __robocup_role)
   (if __robocup_role (setq _robocup_role __robocup_role)) _robocup_role)
  (:robot_intention
   (&optional __robot_intention)
   (if __robot_intention (setq _robot_intention __robot_intention)) _robot_intention)
  (:robot_pose_2d
   (&rest __robot_pose_2d)
   (if (keywordp (car __robot_pose_2d))
       (send* _robot_pose_2d __robot_pose_2d)
     (progn
       (if __robot_pose_2d (setq _robot_pose_2d (car __robot_pose_2d)))
       _robot_pose_2d)))
  (:stiffness_state
   (&optional __stiffness_state)
   (if __stiffness_state (setq _stiffness_state __stiffness_state)) _stiffness_state)
  (:posture_state
   (&optional __posture_state)
   (if __posture_state (setq _posture_state __posture_state)) _posture_state)
  (:planning_state
   (&optional __planning_state)
   (if __planning_state (setq _planning_state __planning_state)) _planning_state)
  (:whistle_detected
   (&optional __whistle_detected)
   (if __whistle_detected (setq _whistle_detected __whistle_detected)) _whistle_detected)
  (:robot_fallen
   (&optional __robot_fallen)
   (if __robot_fallen (setq _robot_fallen __robot_fallen)) _robot_fallen)
  (:robot_in_motion
   (&optional __robot_in_motion)
   (if __robot_in_motion (setq _robot_in_motion __robot_in_motion)) _robot_in_motion)
  (:kick_target
   (&rest __kick_target)
   (if (keywordp (car __kick_target))
       (send* _kick_target __kick_target)
     (progn
       (if __kick_target (setq _kick_target (car __kick_target)))
       _kick_target)))
  (:move_target
   (&rest __move_target)
   (if (keywordp (car __move_target))
       (send* _move_target __move_target)
     (progn
       (if __move_target (setq _move_target (car __move_target)))
       _move_target)))
  (:foot_on_ground
   (&optional __foot_on_ground)
   (if __foot_on_ground (setq _foot_on_ground __foot_on_ground)) _foot_on_ground)
  (:l_foot_transform
   (&rest __l_foot_transform)
   (if (keywordp (car __l_foot_transform))
       (send* _l_foot_transform __l_foot_transform)
     (progn
       (if __l_foot_transform (setq _l_foot_transform (car __l_foot_transform)))
       _l_foot_transform)))
  (:r_foot_transform
   (&rest __r_foot_transform)
   (if (keywordp (car __r_foot_transform))
       (send* _r_foot_transform __r_foot_transform)
     (progn
       (if __r_foot_transform (setq _r_foot_transform (car __r_foot_transform)))
       _r_foot_transform)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _motion_thread_period
    4
    ;; int32 _planning_thread_period
    4
    ;; int32 _sb_thread_period
    4
    ;; int32 _vision_thread_period
    4
    ;; int32 _localization_thread_period
    4
    ;; int32 _game_comm_thread_period
    4
    ;; int32 _user_comm_thread_period
    4
    ;; int32 _heart_beat
    4
    ;; int32 _player_number
    4
    ;; int32 _team_number
    4
    ;; int32 _team_port
    4
    ;; int32 _team_color
    4
    ;; int32 _robocup_role
    4
    ;; int32 _robot_intention
    4
    ;; geometry_msgs/Pose2D _robot_pose_2d
    (send _robot_pose_2d :serialization-length)
    ;; int32 _stiffness_state
    4
    ;; int32 _posture_state
    4
    ;; int32 _planning_state
    4
    ;; bool _whistle_detected
    1
    ;; bool _robot_fallen
    1
    ;; bool _robot_in_motion
    1
    ;; geometry_msgs/Point _kick_target
    (send _kick_target :serialization-length)
    ;; geometry_msgs/Point _move_target
    (send _move_target :serialization-length)
    ;; int32 _foot_on_ground
    4
    ;; geometry_msgs/Transform _l_foot_transform
    (send _l_foot_transform :serialization-length)
    ;; geometry_msgs/Transform _r_foot_transform
    (send _r_foot_transform :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _motion_thread_period
       (write-long _motion_thread_period s)
     ;; int32 _planning_thread_period
       (write-long _planning_thread_period s)
     ;; int32 _sb_thread_period
       (write-long _sb_thread_period s)
     ;; int32 _vision_thread_period
       (write-long _vision_thread_period s)
     ;; int32 _localization_thread_period
       (write-long _localization_thread_period s)
     ;; int32 _game_comm_thread_period
       (write-long _game_comm_thread_period s)
     ;; int32 _user_comm_thread_period
       (write-long _user_comm_thread_period s)
     ;; int32 _heart_beat
       (write-long _heart_beat s)
     ;; int32 _player_number
       (write-long _player_number s)
     ;; int32 _team_number
       (write-long _team_number s)
     ;; int32 _team_port
       (write-long _team_port s)
     ;; int32 _team_color
       (write-long _team_color s)
     ;; int32 _robocup_role
       (write-long _robocup_role s)
     ;; int32 _robot_intention
       (write-long _robot_intention s)
     ;; geometry_msgs/Pose2D _robot_pose_2d
       (send _robot_pose_2d :serialize s)
     ;; int32 _stiffness_state
       (write-long _stiffness_state s)
     ;; int32 _posture_state
       (write-long _posture_state s)
     ;; int32 _planning_state
       (write-long _planning_state s)
     ;; bool _whistle_detected
       (if _whistle_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _robot_fallen
       (if _robot_fallen (write-byte -1 s) (write-byte 0 s))
     ;; bool _robot_in_motion
       (if _robot_in_motion (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _kick_target
       (send _kick_target :serialize s)
     ;; geometry_msgs/Point _move_target
       (send _move_target :serialize s)
     ;; int32 _foot_on_ground
       (write-long _foot_on_ground s)
     ;; geometry_msgs/Transform _l_foot_transform
       (send _l_foot_transform :serialize s)
     ;; geometry_msgs/Transform _r_foot_transform
       (send _r_foot_transform :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _motion_thread_period
     (setq _motion_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _planning_thread_period
     (setq _planning_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sb_thread_period
     (setq _sb_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _vision_thread_period
     (setq _vision_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _localization_thread_period
     (setq _localization_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _game_comm_thread_period
     (setq _game_comm_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _user_comm_thread_period
     (setq _user_comm_thread_period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _heart_beat
     (setq _heart_beat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _player_number
     (setq _player_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _team_number
     (setq _team_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _team_port
     (setq _team_port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _team_color
     (setq _team_color (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _robocup_role
     (setq _robocup_role (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _robot_intention
     (setq _robot_intention (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose2D _robot_pose_2d
     (send _robot_pose_2d :deserialize buf ptr-) (incf ptr- (send _robot_pose_2d :serialization-length))
   ;; int32 _stiffness_state
     (setq _stiffness_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _posture_state
     (setq _posture_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _planning_state
     (setq _planning_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _whistle_detected
     (setq _whistle_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _robot_fallen
     (setq _robot_fallen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _robot_in_motion
     (setq _robot_in_motion (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _kick_target
     (send _kick_target :deserialize buf ptr-) (incf ptr- (send _kick_target :serialization-length))
   ;; geometry_msgs/Point _move_target
     (send _move_target :deserialize buf ptr-) (incf ptr- (send _move_target :serialization-length))
   ;; int32 _foot_on_ground
     (setq _foot_on_ground (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Transform _l_foot_transform
     (send _l_foot_transform :deserialize buf ptr-) (incf ptr- (send _l_foot_transform :serialization-length))
   ;; geometry_msgs/Transform _r_foot_transform
     (send _r_foot_transform :deserialize buf ptr-) (incf ptr- (send _r_foot_transform :serialization-length))
   ;;
   self)
  )

(setf (get team_nust_msgs::TeamNUSTState :md5sum-) "f38be50129fbe02202e870fd322ce791")
(setf (get team_nust_msgs::TeamNUSTState :datatype-) "team_nust_msgs/TeamNUSTState")
(setf (get team_nust_msgs::TeamNUSTState :definition-)
      "Header header
int32 motion_thread_period
int32 planning_thread_period
int32 sb_thread_period
int32 vision_thread_period
int32 localization_thread_period
int32 game_comm_thread_period
int32 user_comm_thread_period
int32 heart_beat
int32 player_number
int32 team_number
int32 team_port
int32 team_color
int32 robocup_role
int32 robot_intention
geometry_msgs/Pose2D robot_pose_2d
int32 stiffness_state
int32 posture_state
int32 planning_state
bool whistle_detected
bool robot_fallen
bool robot_in_motion
geometry_msgs/Point kick_target
geometry_msgs/Point move_target
int32 foot_on_ground
geometry_msgs/Transform l_foot_transform
geometry_msgs/Transform r_foot_transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :team_nust_msgs/TeamNUSTState "f38be50129fbe02202e870fd322ce791")


