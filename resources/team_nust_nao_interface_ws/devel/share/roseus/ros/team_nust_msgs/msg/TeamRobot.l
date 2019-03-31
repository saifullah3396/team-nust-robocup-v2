;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::TeamRobot)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'TeamRobot (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::TEAMROBOT")
  (make-package "TEAM_NUST_MSGS::TEAMROBOT"))

(in-package "ROS")
;;//! \htmlinclude TeamRobot.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass team_nust_msgs::TeamRobot
  :super ros::object
  :slots (_data_received _fallen _intention _suggestion_to_me _pose_2d _walking_to _shooting_to ))

(defmethod team_nust_msgs::TeamRobot
  (:init
   (&key
    ((:data_received __data_received) nil)
    ((:fallen __fallen) nil)
    ((:intention __intention) 0)
    ((:suggestion_to_me __suggestion_to_me) 0)
    ((:pose_2d __pose_2d) (instance geometry_msgs::Pose2D :init))
    ((:walking_to __walking_to) (instance geometry_msgs::Point :init))
    ((:shooting_to __shooting_to) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _data_received __data_received)
   (setq _fallen __fallen)
   (setq _intention (round __intention))
   (setq _suggestion_to_me (round __suggestion_to_me))
   (setq _pose_2d __pose_2d)
   (setq _walking_to __walking_to)
   (setq _shooting_to __shooting_to)
   self)
  (:data_received
   (&optional __data_received)
   (if __data_received (setq _data_received __data_received)) _data_received)
  (:fallen
   (&optional __fallen)
   (if __fallen (setq _fallen __fallen)) _fallen)
  (:intention
   (&optional __intention)
   (if __intention (setq _intention __intention)) _intention)
  (:suggestion_to_me
   (&optional __suggestion_to_me)
   (if __suggestion_to_me (setq _suggestion_to_me __suggestion_to_me)) _suggestion_to_me)
  (:pose_2d
   (&rest __pose_2d)
   (if (keywordp (car __pose_2d))
       (send* _pose_2d __pose_2d)
     (progn
       (if __pose_2d (setq _pose_2d (car __pose_2d)))
       _pose_2d)))
  (:walking_to
   (&rest __walking_to)
   (if (keywordp (car __walking_to))
       (send* _walking_to __walking_to)
     (progn
       (if __walking_to (setq _walking_to (car __walking_to)))
       _walking_to)))
  (:shooting_to
   (&rest __shooting_to)
   (if (keywordp (car __shooting_to))
       (send* _shooting_to __shooting_to)
     (progn
       (if __shooting_to (setq _shooting_to (car __shooting_to)))
       _shooting_to)))
  (:serialization-length
   ()
   (+
    ;; bool _data_received
    1
    ;; bool _fallen
    1
    ;; int32 _intention
    4
    ;; int32 _suggestion_to_me
    4
    ;; geometry_msgs/Pose2D _pose_2d
    (send _pose_2d :serialization-length)
    ;; geometry_msgs/Point _walking_to
    (send _walking_to :serialization-length)
    ;; geometry_msgs/Point _shooting_to
    (send _shooting_to :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data_received
       (if _data_received (write-byte -1 s) (write-byte 0 s))
     ;; bool _fallen
       (if _fallen (write-byte -1 s) (write-byte 0 s))
     ;; int32 _intention
       (write-long _intention s)
     ;; int32 _suggestion_to_me
       (write-long _suggestion_to_me s)
     ;; geometry_msgs/Pose2D _pose_2d
       (send _pose_2d :serialize s)
     ;; geometry_msgs/Point _walking_to
       (send _walking_to :serialize s)
     ;; geometry_msgs/Point _shooting_to
       (send _shooting_to :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data_received
     (setq _data_received (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fallen
     (setq _fallen (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _intention
     (setq _intention (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _suggestion_to_me
     (setq _suggestion_to_me (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose2D _pose_2d
     (send _pose_2d :deserialize buf ptr-) (incf ptr- (send _pose_2d :serialization-length))
   ;; geometry_msgs/Point _walking_to
     (send _walking_to :deserialize buf ptr-) (incf ptr- (send _walking_to :serialization-length))
   ;; geometry_msgs/Point _shooting_to
     (send _shooting_to :deserialize buf ptr-) (incf ptr- (send _shooting_to :serialization-length))
   ;;
   self)
  )

(setf (get team_nust_msgs::TeamRobot :md5sum-) "8b91570e4f56d05a3dd7171dea9d4212")
(setf (get team_nust_msgs::TeamRobot :datatype-) "team_nust_msgs/TeamRobot")
(setf (get team_nust_msgs::TeamRobot :definition-)
      "bool data_received
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



(provide :team_nust_msgs/TeamRobot "8b91570e4f56d05a3dd7171dea9d4212")


