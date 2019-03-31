;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::Obstacle)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'Obstacle (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::OBSTACLE")
  (make-package "TEAM_NUST_MSGS::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass team_nust_msgs::Obstacle
  :super ros::object
  :slots (_type _center _left_bound _right_bound _depth ))

(defmethod team_nust_msgs::Obstacle
  (:init
   (&key
    ((:type __type) 0)
    ((:center __center) (instance geometry_msgs::Point :init))
    ((:left_bound __left_bound) (instance geometry_msgs::Point :init))
    ((:right_bound __right_bound) (instance geometry_msgs::Point :init))
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _center __center)
   (setq _left_bound __left_bound)
   (setq _right_bound __right_bound)
   (setq _depth (float __depth))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:left_bound
   (&rest __left_bound)
   (if (keywordp (car __left_bound))
       (send* _left_bound __left_bound)
     (progn
       (if __left_bound (setq _left_bound (car __left_bound)))
       _left_bound)))
  (:right_bound
   (&rest __right_bound)
   (if (keywordp (car __right_bound))
       (send* _right_bound __right_bound)
     (progn
       (if __right_bound (setq _right_bound (car __right_bound)))
       _right_bound)))
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; geometry_msgs/Point _center
    (send _center :serialization-length)
    ;; geometry_msgs/Point _left_bound
    (send _left_bound :serialization-length)
    ;; geometry_msgs/Point _right_bound
    (send _right_bound :serialization-length)
    ;; float64 _depth
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Point _center
       (send _center :serialize s)
     ;; geometry_msgs/Point _left_bound
       (send _left_bound :serialize s)
     ;; geometry_msgs/Point _right_bound
       (send _right_bound :serialize s)
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; geometry_msgs/Point _left_bound
     (send _left_bound :deserialize buf ptr-) (incf ptr- (send _left_bound :serialization-length))
   ;; geometry_msgs/Point _right_bound
     (send _right_bound :deserialize buf ptr-) (incf ptr- (send _right_bound :serialization-length))
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get team_nust_msgs::Obstacle :md5sum-) "01ef883c2fc32a991b3dced6dd949fec")
(setf (get team_nust_msgs::Obstacle :datatype-) "team_nust_msgs/Obstacle")
(setf (get team_nust_msgs::Obstacle :definition-)
      "uint8 type
geometry_msgs/Point center
geometry_msgs/Point left_bound
geometry_msgs/Point right_bound
float64 depth

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :team_nust_msgs/Obstacle "01ef883c2fc32a991b3dced6dd949fec")


