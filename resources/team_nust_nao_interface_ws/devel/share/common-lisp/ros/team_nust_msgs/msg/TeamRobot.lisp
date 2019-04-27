; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude TeamRobot.msg.html

(cl:defclass <TeamRobot> (roslisp-msg-protocol:ros-message)
  ((data_received
    :reader data_received
    :initarg :data_received
    :type cl:boolean
    :initform cl:nil)
   (fallen
    :reader fallen
    :initarg :fallen
    :type cl:boolean
    :initform cl:nil)
   (intention
    :reader intention
    :initarg :intention
    :type cl:integer
    :initform 0)
   (suggestion_to_me
    :reader suggestion_to_me
    :initarg :suggestion_to_me
    :type cl:integer
    :initform 0)
   (pose_2d
    :reader pose_2d
    :initarg :pose_2d
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (walking_to
    :reader walking_to
    :initarg :walking_to
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (shooting_to
    :reader shooting_to
    :initarg :shooting_to
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TeamRobot (<TeamRobot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TeamRobot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TeamRobot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<TeamRobot> is deprecated: use team_nust_msgs-msg:TeamRobot instead.")))

(cl:ensure-generic-function 'data_received-val :lambda-list '(m))
(cl:defmethod data_received-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:data_received-val is deprecated.  Use team_nust_msgs-msg:data_received instead.")
  (data_received m))

(cl:ensure-generic-function 'fallen-val :lambda-list '(m))
(cl:defmethod fallen-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:fallen-val is deprecated.  Use team_nust_msgs-msg:fallen instead.")
  (fallen m))

(cl:ensure-generic-function 'intention-val :lambda-list '(m))
(cl:defmethod intention-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:intention-val is deprecated.  Use team_nust_msgs-msg:intention instead.")
  (intention m))

(cl:ensure-generic-function 'suggestion_to_me-val :lambda-list '(m))
(cl:defmethod suggestion_to_me-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:suggestion_to_me-val is deprecated.  Use team_nust_msgs-msg:suggestion_to_me instead.")
  (suggestion_to_me m))

(cl:ensure-generic-function 'pose_2d-val :lambda-list '(m))
(cl:defmethod pose_2d-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:pose_2d-val is deprecated.  Use team_nust_msgs-msg:pose_2d instead.")
  (pose_2d m))

(cl:ensure-generic-function 'walking_to-val :lambda-list '(m))
(cl:defmethod walking_to-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:walking_to-val is deprecated.  Use team_nust_msgs-msg:walking_to instead.")
  (walking_to m))

(cl:ensure-generic-function 'shooting_to-val :lambda-list '(m))
(cl:defmethod shooting_to-val ((m <TeamRobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:shooting_to-val is deprecated.  Use team_nust_msgs-msg:shooting_to instead.")
  (shooting_to m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TeamRobot>) ostream)
  "Serializes a message object of type '<TeamRobot>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'data_received) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fallen) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'intention)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'suggestion_to_me)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_2d) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'walking_to) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shooting_to) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TeamRobot>) istream)
  "Deserializes a message object of type '<TeamRobot>"
    (cl:setf (cl:slot-value msg 'data_received) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fallen) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'intention) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'suggestion_to_me) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_2d) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'walking_to) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shooting_to) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TeamRobot>)))
  "Returns string type for a message object of type '<TeamRobot>"
  "team_nust_msgs/TeamRobot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TeamRobot)))
  "Returns string type for a message object of type 'TeamRobot"
  "team_nust_msgs/TeamRobot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TeamRobot>)))
  "Returns md5sum for a message object of type '<TeamRobot>"
  "8b91570e4f56d05a3dd7171dea9d4212")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TeamRobot)))
  "Returns md5sum for a message object of type 'TeamRobot"
  "8b91570e4f56d05a3dd7171dea9d4212")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TeamRobot>)))
  "Returns full string definition for message of type '<TeamRobot>"
  (cl:format cl:nil "bool data_received~%bool fallen~%int32 intention~%int32 suggestion_to_me~%geometry_msgs/Pose2D pose_2d~%geometry_msgs/Point walking_to~%geometry_msgs/Point shooting_to~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TeamRobot)))
  "Returns full string definition for message of type 'TeamRobot"
  (cl:format cl:nil "bool data_received~%bool fallen~%int32 intention~%int32 suggestion_to_me~%geometry_msgs/Pose2D pose_2d~%geometry_msgs/Point walking_to~%geometry_msgs/Point shooting_to~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TeamRobot>))
  (cl:+ 0
     1
     1
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_2d))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'walking_to))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shooting_to))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TeamRobot>))
  "Converts a ROS message object to a list"
  (cl:list 'TeamRobot
    (cl:cons ':data_received (data_received msg))
    (cl:cons ':fallen (fallen msg))
    (cl:cons ':intention (intention msg))
    (cl:cons ':suggestion_to_me (suggestion_to_me msg))
    (cl:cons ':pose_2d (pose_2d msg))
    (cl:cons ':walking_to (walking_to msg))
    (cl:cons ':shooting_to (shooting_to msg))
))
