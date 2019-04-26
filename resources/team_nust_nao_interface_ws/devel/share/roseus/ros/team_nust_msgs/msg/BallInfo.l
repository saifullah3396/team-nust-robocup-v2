;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::BallInfo)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'BallInfo (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::BALLINFO")
  (make-package "TEAM_NUST_MSGS::BALLINFO"))

(in-package "ROS")
;;//! \htmlinclude BallInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::BallInfo
  :super ros::object
  :slots (_header _camera _found _pos _world_pos _vel _world_vel _image _age _radius ))

(defmethod team_nust_msgs::BallInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera __camera) 0)
    ((:found __found) nil)
    ((:pos __pos) (instance geometry_msgs::Point :init))
    ((:world_pos __world_pos) (instance geometry_msgs::Point :init))
    ((:vel __vel) (instance geometry_msgs::Point :init))
    ((:world_vel __world_vel) (instance geometry_msgs::Point :init))
    ((:image __image) (instance geometry_msgs::Point :init))
    ((:age __age) 0.0)
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera (round __camera))
   (setq _found __found)
   (setq _pos __pos)
   (setq _world_pos __world_pos)
   (setq _vel __vel)
   (setq _world_vel __world_vel)
   (setq _image __image)
   (setq _age (float __age))
   (setq _radius (float __radius))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera
   (&optional __camera)
   (if __camera (setq _camera __camera)) _camera)
  (:found
   (&optional __found)
   (if __found (setq _found __found)) _found)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:world_pos
   (&rest __world_pos)
   (if (keywordp (car __world_pos))
       (send* _world_pos __world_pos)
     (progn
       (if __world_pos (setq _world_pos (car __world_pos)))
       _world_pos)))
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:world_vel
   (&rest __world_vel)
   (if (keywordp (car __world_vel))
       (send* _world_vel __world_vel)
     (progn
       (if __world_vel (setq _world_vel (car __world_vel)))
       _world_vel)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _camera
    1
    ;; bool _found
    1
    ;; geometry_msgs/Point _pos
    (send _pos :serialization-length)
    ;; geometry_msgs/Point _world_pos
    (send _world_pos :serialization-length)
    ;; geometry_msgs/Point _vel
    (send _vel :serialization-length)
    ;; geometry_msgs/Point _world_vel
    (send _world_vel :serialization-length)
    ;; geometry_msgs/Point _image
    (send _image :serialization-length)
    ;; float32 _age
    4
    ;; float32 _radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _camera
       (write-byte _camera s)
     ;; bool _found
       (if _found (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _pos
       (send _pos :serialize s)
     ;; geometry_msgs/Point _world_pos
       (send _world_pos :serialize s)
     ;; geometry_msgs/Point _vel
       (send _vel :serialize s)
     ;; geometry_msgs/Point _world_vel
       (send _world_vel :serialize s)
     ;; geometry_msgs/Point _image
       (send _image :serialize s)
     ;; float32 _age
       (sys::poke _age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _camera
     (setq _camera (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _found
     (setq _found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; geometry_msgs/Point _world_pos
     (send _world_pos :deserialize buf ptr-) (incf ptr- (send _world_pos :serialization-length))
   ;; geometry_msgs/Point _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;; geometry_msgs/Point _world_vel
     (send _world_vel :deserialize buf ptr-) (incf ptr- (send _world_vel :serialization-length))
   ;; geometry_msgs/Point _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; float32 _age
     (setq _age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get team_nust_msgs::BallInfo :md5sum-) "f99f7385aa73a1572aa87b070548316e")
(setf (get team_nust_msgs::BallInfo :datatype-) "team_nust_msgs/BallInfo")
(setf (get team_nust_msgs::BallInfo :definition-)
      "Header header
uint8 camera
bool found
geometry_msgs/Point pos
geometry_msgs/Point world_pos
geometry_msgs/Point vel
geometry_msgs/Point world_vel
geometry_msgs/Point image
float32 age
float32 radius

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

")



(provide :team_nust_msgs/BallInfo "f99f7385aa73a1572aa87b070548316e")


