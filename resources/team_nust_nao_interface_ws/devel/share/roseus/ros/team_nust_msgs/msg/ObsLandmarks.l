;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::ObsLandmarks)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'ObsLandmarks (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::OBSLANDMARKS")
  (make-package "TEAM_NUST_MSGS::OBSLANDMARKS"))

(in-package "ROS")
;;//! \htmlinclude ObsLandmarks.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::ObsLandmarks
  :super ros::object
  :slots (_header _landmarks ))

(defmethod team_nust_msgs::ObsLandmarks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:landmarks __landmarks) (let (r) (dotimes (i 0) (push (instance team_nust_msgs::Landmark :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _landmarks __landmarks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:landmarks
   (&rest __landmarks)
   (if (keywordp (car __landmarks))
       (send* _landmarks __landmarks)
     (progn
       (if __landmarks (setq _landmarks (car __landmarks)))
       _landmarks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; team_nust_msgs/Landmark[] _landmarks
    (apply #'+ (send-all _landmarks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; team_nust_msgs/Landmark[] _landmarks
     (write-long (length _landmarks) s)
     (dolist (elem _landmarks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; team_nust_msgs/Landmark[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance team_nust_msgs::Landmark :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get team_nust_msgs::ObsLandmarks :md5sum-) "71ddf4bd513db27a5697d59d383562d2")
(setf (get team_nust_msgs::ObsLandmarks :datatype-) "team_nust_msgs/ObsLandmarks")
(setf (get team_nust_msgs::ObsLandmarks :definition-)
      "Header header
Landmark[] landmarks

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
MSG: team_nust_msgs/Landmark
uint8 type
geometry_msgs/Point pos

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :team_nust_msgs/ObsLandmarks "71ddf4bd513db27a5697d59d383562d2")


