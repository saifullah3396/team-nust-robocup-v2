; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude BallInfo.msg.html

(cl:defclass <BallInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (camera
    :reader camera
    :initarg :camera
    :type cl:fixnum
    :initform 0)
   (found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (world_pos
    :reader world_pos
    :initarg :world_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (world_vel
    :reader world_vel
    :initarg :world_vel
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (image
    :reader image
    :initarg :image
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (age
    :reader age
    :initarg :age
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass BallInfo (<BallInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BallInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BallInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<BallInfo> is deprecated: use team_nust_msgs-msg:BallInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'camera-val :lambda-list '(m))
(cl:defmethod camera-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:camera-val is deprecated.  Use team_nust_msgs-msg:camera instead.")
  (camera m))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:found-val is deprecated.  Use team_nust_msgs-msg:found instead.")
  (found m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:pos-val is deprecated.  Use team_nust_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'world_pos-val :lambda-list '(m))
(cl:defmethod world_pos-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:world_pos-val is deprecated.  Use team_nust_msgs-msg:world_pos instead.")
  (world_pos m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:vel-val is deprecated.  Use team_nust_msgs-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'world_vel-val :lambda-list '(m))
(cl:defmethod world_vel-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:world_vel-val is deprecated.  Use team_nust_msgs-msg:world_vel instead.")
  (world_vel m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:image-val is deprecated.  Use team_nust_msgs-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:age-val is deprecated.  Use team_nust_msgs-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <BallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:radius-val is deprecated.  Use team_nust_msgs-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BallInfo>) ostream)
  "Serializes a message object of type '<BallInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world_vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BallInfo>) istream)
  "Deserializes a message object of type '<BallInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world_vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'age) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BallInfo>)))
  "Returns string type for a message object of type '<BallInfo>"
  "team_nust_msgs/BallInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BallInfo)))
  "Returns string type for a message object of type 'BallInfo"
  "team_nust_msgs/BallInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BallInfo>)))
  "Returns md5sum for a message object of type '<BallInfo>"
  "f99f7385aa73a1572aa87b070548316e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BallInfo)))
  "Returns md5sum for a message object of type 'BallInfo"
  "f99f7385aa73a1572aa87b070548316e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BallInfo>)))
  "Returns full string definition for message of type '<BallInfo>"
  (cl:format cl:nil "Header header~%uint8 camera~%bool found~%geometry_msgs/Point pos~%geometry_msgs/Point world_pos~%geometry_msgs/Point vel~%geometry_msgs/Point world_vel~%geometry_msgs/Point image~%float32 age~%float32 radius~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BallInfo)))
  "Returns full string definition for message of type 'BallInfo"
  (cl:format cl:nil "Header header~%uint8 camera~%bool found~%geometry_msgs/Point pos~%geometry_msgs/Point world_pos~%geometry_msgs/Point vel~%geometry_msgs/Point world_vel~%geometry_msgs/Point image~%float32 age~%float32 radius~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BallInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world_vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BallInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BallInfo
    (cl:cons ':header (header msg))
    (cl:cons ':camera (camera msg))
    (cl:cons ':found (found msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':world_pos (world_pos msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':world_vel (world_vel msg))
    (cl:cons ':image (image msg))
    (cl:cons ':age (age msg))
    (cl:cons ':radius (radius msg))
))
