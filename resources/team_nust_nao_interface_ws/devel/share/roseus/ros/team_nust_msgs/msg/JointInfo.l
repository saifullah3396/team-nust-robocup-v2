;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::JointInfo)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'JointInfo (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::JOINTINFO")
  (make-package "TEAM_NUST_MSGS::JOINTINFO"))

(in-package "ROS")
;;//! \htmlinclude JointInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::JointInfo
  :super ros::object
  :slots (_header _name _stiffness _temperature _current ))

(defmethod team_nust_msgs::JointInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:stiffness __stiffness) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:temperature __temperature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:current __current) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _stiffness __stiffness)
   (setq _temperature __temperature)
   (setq _current __current)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:stiffness
   (&optional __stiffness)
   (if __stiffness (setq _stiffness __stiffness)) _stiffness)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float64[] _stiffness
    (* 8    (length _stiffness)) 4
    ;; float64[] _temperature
    (* 8    (length _temperature)) 4
    ;; float64[] _current
    (* 8    (length _current)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _stiffness
     (write-long (length _stiffness) s)
     (dotimes (i (length _stiffness))
       (sys::poke (elt _stiffness i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _temperature
     (write-long (length _temperature) s)
     (dotimes (i (length _temperature))
       (sys::poke (elt _temperature i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (sys::poke (elt _current i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _stiffness
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stiffness (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stiffness i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _temperature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temperature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temperature i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get team_nust_msgs::JointInfo :md5sum-) "1acc6d92d2e8a85b41f217ac954f0cdd")
(setf (get team_nust_msgs::JointInfo :datatype-) "team_nust_msgs/JointInfo")
(setf (get team_nust_msgs::JointInfo :definition-)
      "Header header
string[] name
float64[] stiffness
float64[] temperature
float64[] current

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



(provide :team_nust_msgs/JointInfo "1acc6d92d2e8a85b41f217ac954f0cdd")


