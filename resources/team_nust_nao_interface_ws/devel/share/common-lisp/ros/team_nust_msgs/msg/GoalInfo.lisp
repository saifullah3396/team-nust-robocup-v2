; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude GoalInfo.msg.html

(cl:defclass <GoalInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (left_post
    :reader left_post
    :initarg :left_post
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (right_post
    :reader right_post
    :initarg :right_post
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (mid
    :reader mid
    :initarg :mid
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (poseFromGoal
    :reader poseFromGoal
    :initarg :poseFromGoal
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (goal_type
    :reader goal_type
    :initarg :goal_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GoalInfo (<GoalInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<GoalInfo> is deprecated: use team_nust_msgs-msg:GoalInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:found-val is deprecated.  Use team_nust_msgs-msg:found instead.")
  (found m))

(cl:ensure-generic-function 'left_post-val :lambda-list '(m))
(cl:defmethod left_post-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:left_post-val is deprecated.  Use team_nust_msgs-msg:left_post instead.")
  (left_post m))

(cl:ensure-generic-function 'right_post-val :lambda-list '(m))
(cl:defmethod right_post-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:right_post-val is deprecated.  Use team_nust_msgs-msg:right_post instead.")
  (right_post m))

(cl:ensure-generic-function 'mid-val :lambda-list '(m))
(cl:defmethod mid-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:mid-val is deprecated.  Use team_nust_msgs-msg:mid instead.")
  (mid m))

(cl:ensure-generic-function 'poseFromGoal-val :lambda-list '(m))
(cl:defmethod poseFromGoal-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:poseFromGoal-val is deprecated.  Use team_nust_msgs-msg:poseFromGoal instead.")
  (poseFromGoal m))

(cl:ensure-generic-function 'goal_type-val :lambda-list '(m))
(cl:defmethod goal_type-val ((m <GoalInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:goal_type-val is deprecated.  Use team_nust_msgs-msg:goal_type instead.")
  (goal_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalInfo>) ostream)
  "Serializes a message object of type '<GoalInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_post) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right_post) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poseFromGoal) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'goal_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalInfo>) istream)
  "Deserializes a message object of type '<GoalInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_post) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right_post) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poseFromGoal) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'goal_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalInfo>)))
  "Returns string type for a message object of type '<GoalInfo>"
  "team_nust_msgs/GoalInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalInfo)))
  "Returns string type for a message object of type 'GoalInfo"
  "team_nust_msgs/GoalInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalInfo>)))
  "Returns md5sum for a message object of type '<GoalInfo>"
  "7ca7f41f373e0f0dcd0e485cfff71366")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalInfo)))
  "Returns md5sum for a message object of type 'GoalInfo"
  "7ca7f41f373e0f0dcd0e485cfff71366")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalInfo>)))
  "Returns full string definition for message of type '<GoalInfo>"
  (cl:format cl:nil "Header header~%bool found~%geometry_msgs/Point left_post~%geometry_msgs/Point right_post~%geometry_msgs/Point mid~%geometry_msgs/Pose2D poseFromGoal~%uint8 goal_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalInfo)))
  "Returns full string definition for message of type 'GoalInfo"
  (cl:format cl:nil "Header header~%bool found~%geometry_msgs/Point left_post~%geometry_msgs/Point right_post~%geometry_msgs/Point mid~%geometry_msgs/Pose2D poseFromGoal~%uint8 goal_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_post))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right_post))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poseFromGoal))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalInfo
    (cl:cons ':header (header msg))
    (cl:cons ':found (found msg))
    (cl:cons ':left_post (left_post msg))
    (cl:cons ':right_post (right_post msg))
    (cl:cons ':mid (mid msg))
    (cl:cons ':poseFromGoal (poseFromGoal msg))
    (cl:cons ':goal_type (goal_type msg))
))
