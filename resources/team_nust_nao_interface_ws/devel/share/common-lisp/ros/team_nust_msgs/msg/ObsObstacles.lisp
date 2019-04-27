; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude ObsObstacles.msg.html

(cl:defclass <ObsObstacles> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector team_nust_msgs-msg:Obstacle)
   :initform (cl:make-array 0 :element-type 'team_nust_msgs-msg:Obstacle :initial-element (cl:make-instance 'team_nust_msgs-msg:Obstacle))))
)

(cl:defclass ObsObstacles (<ObsObstacles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObsObstacles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObsObstacles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<ObsObstacles> is deprecated: use team_nust_msgs-msg:ObsObstacles instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObsObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <ObsObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:obstacles-val is deprecated.  Use team_nust_msgs-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObsObstacles>) ostream)
  "Serializes a message object of type '<ObsObstacles>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObsObstacles>) istream)
  "Deserializes a message object of type '<ObsObstacles>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'team_nust_msgs-msg:Obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObsObstacles>)))
  "Returns string type for a message object of type '<ObsObstacles>"
  "team_nust_msgs/ObsObstacles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObsObstacles)))
  "Returns string type for a message object of type 'ObsObstacles"
  "team_nust_msgs/ObsObstacles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObsObstacles>)))
  "Returns md5sum for a message object of type '<ObsObstacles>"
  "1d072144a0a5b5af851cd9f0d3b6c877")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObsObstacles)))
  "Returns md5sum for a message object of type 'ObsObstacles"
  "1d072144a0a5b5af851cd9f0d3b6c877")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObsObstacles>)))
  "Returns full string definition for message of type '<ObsObstacles>"
  (cl:format cl:nil "Header header~%Obstacle[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: team_nust_msgs/Obstacle~%uint8 type~%geometry_msgs/Point center~%geometry_msgs/Point left_bound~%geometry_msgs/Point right_bound~%float64 depth~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObsObstacles)))
  "Returns full string definition for message of type 'ObsObstacles"
  (cl:format cl:nil "Header header~%Obstacle[] obstacles~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: team_nust_msgs/Obstacle~%uint8 type~%geometry_msgs/Point center~%geometry_msgs/Point left_bound~%geometry_msgs/Point right_bound~%float64 depth~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObsObstacles>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObsObstacles>))
  "Converts a ROS message object to a list"
  (cl:list 'ObsObstacles
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
