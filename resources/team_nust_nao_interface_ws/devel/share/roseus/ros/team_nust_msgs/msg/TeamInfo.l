;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::TeamInfo)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'TeamInfo (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::TEAMINFO")
  (make-package "TEAM_NUST_MSGS::TEAMINFO"))

(in-package "ROS")
;;//! \htmlinclude TeamInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::TeamInfo
  :super ros::object
  :slots (_header _robots ))

(defmethod team_nust_msgs::TeamInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robots __robots) (let (r) (dotimes (i 0) (push (instance team_nust_msgs::TeamRobot :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robots __robots)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robots
   (&rest __robots)
   (if (keywordp (car __robots))
       (send* _robots __robots)
     (progn
       (if __robots (setq _robots (car __robots)))
       _robots)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; team_nust_msgs/TeamRobot[] _robots
    (apply #'+ (send-all _robots :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; team_nust_msgs/TeamRobot[] _robots
     (write-long (length _robots) s)
     (dolist (elem _robots)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; team_nust_msgs/TeamRobot[] _robots
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robots (let (r) (dotimes (i n) (push (instance team_nust_msgs::TeamRobot :init) r)) r))
     (dolist (elem- _robots)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get team_nust_msgs::TeamInfo :md5sum-) "9993a1bc96866937a298df2560090037")
(setf (get team_nust_msgs::TeamInfo :datatype-) "team_nust_msgs/TeamInfo")
(setf (get team_nust_msgs::TeamInfo :definition-)
      "Header header
TeamRobot[] robots

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
MSG: team_nust_msgs/TeamRobot
bool data_received
bool fallen
int32 intention
int32 suggestion_to_me
geometry_msgs/Pose2D pose_2d
geometry_msgs/Point walking_to
geometry_msgs/Point shooting_to

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

")



(provide :team_nust_msgs/TeamInfo "9993a1bc96866937a298df2560090037")


