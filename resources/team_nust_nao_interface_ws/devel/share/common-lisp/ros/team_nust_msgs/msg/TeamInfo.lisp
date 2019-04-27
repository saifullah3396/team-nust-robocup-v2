; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude TeamInfo.msg.html

(cl:defclass <TeamInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robots
    :reader robots
    :initarg :robots
    :type (cl:vector team_nust_msgs-msg:TeamRobot)
   :initform (cl:make-array 0 :element-type 'team_nust_msgs-msg:TeamRobot :initial-element (cl:make-instance 'team_nust_msgs-msg:TeamRobot))))
)

(cl:defclass TeamInfo (<TeamInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TeamInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TeamInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<TeamInfo> is deprecated: use team_nust_msgs-msg:TeamInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TeamInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robots-val :lambda-list '(m))
(cl:defmethod robots-val ((m <TeamInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robots-val is deprecated.  Use team_nust_msgs-msg:robots instead.")
  (robots m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TeamInfo>) ostream)
  "Serializes a message object of type '<TeamInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robots))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TeamInfo>) istream)
  "Deserializes a message object of type '<TeamInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'team_nust_msgs-msg:TeamRobot))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TeamInfo>)))
  "Returns string type for a message object of type '<TeamInfo>"
  "team_nust_msgs/TeamInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TeamInfo)))
  "Returns string type for a message object of type 'TeamInfo"
  "team_nust_msgs/TeamInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TeamInfo>)))
  "Returns md5sum for a message object of type '<TeamInfo>"
  "9993a1bc96866937a298df2560090037")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TeamInfo)))
  "Returns md5sum for a message object of type 'TeamInfo"
  "9993a1bc96866937a298df2560090037")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TeamInfo>)))
  "Returns full string definition for message of type '<TeamInfo>"
  (cl:format cl:nil "Header header~%TeamRobot[] robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: team_nust_msgs/TeamRobot~%bool data_received~%bool fallen~%int32 intention~%int32 suggestion_to_me~%geometry_msgs/Pose2D pose_2d~%geometry_msgs/Point walking_to~%geometry_msgs/Point shooting_to~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TeamInfo)))
  "Returns full string definition for message of type 'TeamInfo"
  (cl:format cl:nil "Header header~%TeamRobot[] robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: team_nust_msgs/TeamRobot~%bool data_received~%bool fallen~%int32 intention~%int32 suggestion_to_me~%geometry_msgs/Pose2D pose_2d~%geometry_msgs/Point walking_to~%geometry_msgs/Point shooting_to~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TeamInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TeamInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'TeamInfo
    (cl:cons ':header (header msg))
    (cl:cons ':robots (robots msg))
))
