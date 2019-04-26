;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::StepTargetArr)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'StepTargetArr (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::STEPTARGETARR")
  (make-package "TEAM_NUST_MSGS::STEPTARGETARR"))

(in-package "ROS")
;;//! \htmlinclude StepTargetArr.msg.html
(if (not (find-package "HUMANOID_NAV_MSGS"))
  (ros::roseus-add-msgs "humanoid_nav_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::StepTargetArr
  :super ros::object
  :slots (_header _steps ))

(defmethod team_nust_msgs::StepTargetArr
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:steps __steps) (let (r) (dotimes (i 0) (push (instance humanoid_nav_msgs::StepTarget :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _steps __steps)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:steps
   (&rest __steps)
   (if (keywordp (car __steps))
       (send* _steps __steps)
     (progn
       (if __steps (setq _steps (car __steps)))
       _steps)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; humanoid_nav_msgs/StepTarget[] _steps
    (apply #'+ (send-all _steps :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; humanoid_nav_msgs/StepTarget[] _steps
     (write-long (length _steps) s)
     (dolist (elem _steps)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; humanoid_nav_msgs/StepTarget[] _steps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _steps (let (r) (dotimes (i n) (push (instance humanoid_nav_msgs::StepTarget :init) r)) r))
     (dolist (elem- _steps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get team_nust_msgs::StepTargetArr :md5sum-) "5d94e0eebae888055949c996dbe5e324")
(setf (get team_nust_msgs::StepTargetArr :datatype-) "team_nust_msgs/StepTargetArr")
(setf (get team_nust_msgs::StepTargetArr :definition-)
      "Header header
humanoid_nav_msgs/StepTarget[] steps

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
MSG: humanoid_nav_msgs/StepTarget
# Target for a single stepping motion of a humanoid's leg

geometry_msgs/Pose2D pose   # step pose as relative offset to last leg
uint8 leg                   # which leg to use (left/right, see below)

uint8 right=0               # right leg constant
uint8 left=1                # left leg constant

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
")



(provide :team_nust_msgs/StepTargetArr "5d94e0eebae888055949c996dbe5e324")


