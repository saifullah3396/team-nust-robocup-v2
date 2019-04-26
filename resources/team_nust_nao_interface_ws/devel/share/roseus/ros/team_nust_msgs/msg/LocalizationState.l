;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::LocalizationState)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'LocalizationState (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::LOCALIZATIONSTATE")
  (make-package "TEAM_NUST_MSGS::LOCALIZATIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude LocalizationState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::LocalizationState
  :super ros::object
  :slots (_header _robot_localized _position_confidence _side_confidence _robot_on_side_line _localize_with_last_known _landmarks_found ))

(defmethod team_nust_msgs::LocalizationState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_localized __robot_localized) nil)
    ((:position_confidence __position_confidence) 0)
    ((:side_confidence __side_confidence) 0)
    ((:robot_on_side_line __robot_on_side_line) nil)
    ((:localize_with_last_known __localize_with_last_known) nil)
    ((:landmarks_found __landmarks_found) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_localized __robot_localized)
   (setq _position_confidence (round __position_confidence))
   (setq _side_confidence (round __side_confidence))
   (setq _robot_on_side_line __robot_on_side_line)
   (setq _localize_with_last_known __localize_with_last_known)
   (setq _landmarks_found __landmarks_found)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_localized
   (&optional __robot_localized)
   (if __robot_localized (setq _robot_localized __robot_localized)) _robot_localized)
  (:position_confidence
   (&optional __position_confidence)
   (if __position_confidence (setq _position_confidence __position_confidence)) _position_confidence)
  (:side_confidence
   (&optional __side_confidence)
   (if __side_confidence (setq _side_confidence __side_confidence)) _side_confidence)
  (:robot_on_side_line
   (&optional __robot_on_side_line)
   (if __robot_on_side_line (setq _robot_on_side_line __robot_on_side_line)) _robot_on_side_line)
  (:localize_with_last_known
   (&optional __localize_with_last_known)
   (if __localize_with_last_known (setq _localize_with_last_known __localize_with_last_known)) _localize_with_last_known)
  (:landmarks_found
   (&optional __landmarks_found)
   (if __landmarks_found (setq _landmarks_found __landmarks_found)) _landmarks_found)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _robot_localized
    1
    ;; int32 _position_confidence
    4
    ;; int32 _side_confidence
    4
    ;; bool _robot_on_side_line
    1
    ;; bool _localize_with_last_known
    1
    ;; bool _landmarks_found
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _robot_localized
       (if _robot_localized (write-byte -1 s) (write-byte 0 s))
     ;; int32 _position_confidence
       (write-long _position_confidence s)
     ;; int32 _side_confidence
       (write-long _side_confidence s)
     ;; bool _robot_on_side_line
       (if _robot_on_side_line (write-byte -1 s) (write-byte 0 s))
     ;; bool _localize_with_last_known
       (if _localize_with_last_known (write-byte -1 s) (write-byte 0 s))
     ;; bool _landmarks_found
       (if _landmarks_found (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _robot_localized
     (setq _robot_localized (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _position_confidence
     (setq _position_confidence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _side_confidence
     (setq _side_confidence (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _robot_on_side_line
     (setq _robot_on_side_line (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _localize_with_last_known
     (setq _localize_with_last_known (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _landmarks_found
     (setq _landmarks_found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get team_nust_msgs::LocalizationState :md5sum-) "6443aeeb1985422e92a70b31dbb67942")
(setf (get team_nust_msgs::LocalizationState :datatype-) "team_nust_msgs/LocalizationState")
(setf (get team_nust_msgs::LocalizationState :definition-)
      "Header header
bool robot_localized
int32 position_confidence
int32 side_confidence
bool robot_on_side_line
bool localize_with_last_known
bool landmarks_found

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

")



(provide :team_nust_msgs/LocalizationState "6443aeeb1985422e92a70b31dbb67942")


