;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::Landmark)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'Landmark (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::LANDMARK")
  (make-package "TEAM_NUST_MSGS::LANDMARK"))

(in-package "ROS")
;;//! \htmlinclude Landmark.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass team_nust_msgs::Landmark
  :super ros::object
  :slots (_type _pos ))

(defmethod team_nust_msgs::Landmark
  (:init
   (&key
    ((:type __type) 0)
    ((:pos __pos) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _pos __pos)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; geometry_msgs/Point _pos
    (send _pos :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Point _pos
       (send _pos :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;;
   self)
  )

(setf (get team_nust_msgs::Landmark :md5sum-) "a44c2e55d48971b074b6d4f211c997d7")
(setf (get team_nust_msgs::Landmark :datatype-) "team_nust_msgs/Landmark")
(setf (get team_nust_msgs::Landmark :definition-)
      "uint8 type
geometry_msgs/Point pos

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :team_nust_msgs/Landmark "a44c2e55d48971b074b6d4f211c997d7")


