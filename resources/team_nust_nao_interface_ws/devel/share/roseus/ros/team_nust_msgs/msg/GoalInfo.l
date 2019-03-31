;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::GoalInfo)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'GoalInfo (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::GOALINFO")
  (make-package "TEAM_NUST_MSGS::GOALINFO"))

(in-package "ROS")
;;//! \htmlinclude GoalInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::GoalInfo
  :super ros::object
  :slots (_header _found _left_post _right_post _mid _poseFromGoal _goal_type ))

(defmethod team_nust_msgs::GoalInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:found __found) nil)
    ((:left_post __left_post) (instance geometry_msgs::Point :init))
    ((:right_post __right_post) (instance geometry_msgs::Point :init))
    ((:mid __mid) (instance geometry_msgs::Point :init))
    ((:poseFromGoal __poseFromGoal) (instance geometry_msgs::Pose2D :init))
    ((:goal_type __goal_type) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _found __found)
   (setq _left_post __left_post)
   (setq _right_post __right_post)
   (setq _mid __mid)
   (setq _poseFromGoal __poseFromGoal)
   (setq _goal_type (round __goal_type))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:found
   (&optional __found)
   (if __found (setq _found __found)) _found)
  (:left_post
   (&rest __left_post)
   (if (keywordp (car __left_post))
       (send* _left_post __left_post)
     (progn
       (if __left_post (setq _left_post (car __left_post)))
       _left_post)))
  (:right_post
   (&rest __right_post)
   (if (keywordp (car __right_post))
       (send* _right_post __right_post)
     (progn
       (if __right_post (setq _right_post (car __right_post)))
       _right_post)))
  (:mid
   (&rest __mid)
   (if (keywordp (car __mid))
       (send* _mid __mid)
     (progn
       (if __mid (setq _mid (car __mid)))
       _mid)))
  (:poseFromGoal
   (&rest __poseFromGoal)
   (if (keywordp (car __poseFromGoal))
       (send* _poseFromGoal __poseFromGoal)
     (progn
       (if __poseFromGoal (setq _poseFromGoal (car __poseFromGoal)))
       _poseFromGoal)))
  (:goal_type
   (&optional __goal_type)
   (if __goal_type (setq _goal_type __goal_type)) _goal_type)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _found
    1
    ;; geometry_msgs/Point _left_post
    (send _left_post :serialization-length)
    ;; geometry_msgs/Point _right_post
    (send _right_post :serialization-length)
    ;; geometry_msgs/Point _mid
    (send _mid :serialization-length)
    ;; geometry_msgs/Pose2D _poseFromGoal
    (send _poseFromGoal :serialization-length)
    ;; uint8 _goal_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _found
       (if _found (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _left_post
       (send _left_post :serialize s)
     ;; geometry_msgs/Point _right_post
       (send _right_post :serialize s)
     ;; geometry_msgs/Point _mid
       (send _mid :serialize s)
     ;; geometry_msgs/Pose2D _poseFromGoal
       (send _poseFromGoal :serialize s)
     ;; uint8 _goal_type
       (write-byte _goal_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _found
     (setq _found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _left_post
     (send _left_post :deserialize buf ptr-) (incf ptr- (send _left_post :serialization-length))
   ;; geometry_msgs/Point _right_post
     (send _right_post :deserialize buf ptr-) (incf ptr- (send _right_post :serialization-length))
   ;; geometry_msgs/Point _mid
     (send _mid :deserialize buf ptr-) (incf ptr- (send _mid :serialization-length))
   ;; geometry_msgs/Pose2D _poseFromGoal
     (send _poseFromGoal :deserialize buf ptr-) (incf ptr- (send _poseFromGoal :serialization-length))
   ;; uint8 _goal_type
     (setq _goal_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get team_nust_msgs::GoalInfo :md5sum-) "7ca7f41f373e0f0dcd0e485cfff71366")
(setf (get team_nust_msgs::GoalInfo :datatype-) "team_nust_msgs/GoalInfo")
(setf (get team_nust_msgs::GoalInfo :definition-)
      "Header header
bool found
geometry_msgs/Point left_post
geometry_msgs/Point right_post
geometry_msgs/Point mid
geometry_msgs/Pose2D poseFromGoal
uint8 goal_type

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")



(provide :team_nust_msgs/GoalInfo "7ca7f41f373e0f0dcd0e485cfff71366")


