; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude LocalizationState.msg.html

(cl:defclass <LocalizationState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_localized
    :reader robot_localized
    :initarg :robot_localized
    :type cl:boolean
    :initform cl:nil)
   (position_confidence
    :reader position_confidence
    :initarg :position_confidence
    :type cl:integer
    :initform 0)
   (side_confidence
    :reader side_confidence
    :initarg :side_confidence
    :type cl:integer
    :initform 0)
   (robot_on_side_line
    :reader robot_on_side_line
    :initarg :robot_on_side_line
    :type cl:boolean
    :initform cl:nil)
   (localize_with_last_known
    :reader localize_with_last_known
    :initarg :localize_with_last_known
    :type cl:boolean
    :initform cl:nil)
   (landmarks_found
    :reader landmarks_found
    :initarg :landmarks_found
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LocalizationState (<LocalizationState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalizationState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalizationState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<LocalizationState> is deprecated: use team_nust_msgs-msg:LocalizationState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:header-val is deprecated.  Use team_nust_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_localized-val :lambda-list '(m))
(cl:defmethod robot_localized-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_localized-val is deprecated.  Use team_nust_msgs-msg:robot_localized instead.")
  (robot_localized m))

(cl:ensure-generic-function 'position_confidence-val :lambda-list '(m))
(cl:defmethod position_confidence-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:position_confidence-val is deprecated.  Use team_nust_msgs-msg:position_confidence instead.")
  (position_confidence m))

(cl:ensure-generic-function 'side_confidence-val :lambda-list '(m))
(cl:defmethod side_confidence-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:side_confidence-val is deprecated.  Use team_nust_msgs-msg:side_confidence instead.")
  (side_confidence m))

(cl:ensure-generic-function 'robot_on_side_line-val :lambda-list '(m))
(cl:defmethod robot_on_side_line-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:robot_on_side_line-val is deprecated.  Use team_nust_msgs-msg:robot_on_side_line instead.")
  (robot_on_side_line m))

(cl:ensure-generic-function 'localize_with_last_known-val :lambda-list '(m))
(cl:defmethod localize_with_last_known-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:localize_with_last_known-val is deprecated.  Use team_nust_msgs-msg:localize_with_last_known instead.")
  (localize_with_last_known m))

(cl:ensure-generic-function 'landmarks_found-val :lambda-list '(m))
(cl:defmethod landmarks_found-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:landmarks_found-val is deprecated.  Use team_nust_msgs-msg:landmarks_found instead.")
  (landmarks_found m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalizationState>) ostream)
  "Serializes a message object of type '<LocalizationState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_localized) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'position_confidence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'side_confidence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_on_side_line) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'localize_with_last_known) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'landmarks_found) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalizationState>) istream)
  "Deserializes a message object of type '<LocalizationState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'robot_localized) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position_confidence) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'side_confidence) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'robot_on_side_line) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'localize_with_last_known) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'landmarks_found) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalizationState>)))
  "Returns string type for a message object of type '<LocalizationState>"
  "team_nust_msgs/LocalizationState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalizationState)))
  "Returns string type for a message object of type 'LocalizationState"
  "team_nust_msgs/LocalizationState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalizationState>)))
  "Returns md5sum for a message object of type '<LocalizationState>"
  "6443aeeb1985422e92a70b31dbb67942")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalizationState)))
  "Returns md5sum for a message object of type 'LocalizationState"
  "6443aeeb1985422e92a70b31dbb67942")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalizationState>)))
  "Returns full string definition for message of type '<LocalizationState>"
  (cl:format cl:nil "Header header~%bool robot_localized~%int32 position_confidence~%int32 side_confidence~%bool robot_on_side_line~%bool localize_with_last_known~%bool landmarks_found~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalizationState)))
  "Returns full string definition for message of type 'LocalizationState"
  (cl:format cl:nil "Header header~%bool robot_localized~%int32 position_confidence~%int32 side_confidence~%bool robot_on_side_line~%bool localize_with_last_known~%bool landmarks_found~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalizationState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalizationState>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalizationState
    (cl:cons ':header (header msg))
    (cl:cons ':robot_localized (robot_localized msg))
    (cl:cons ':position_confidence (position_confidence msg))
    (cl:cons ':side_confidence (side_confidence msg))
    (cl:cons ':robot_on_side_line (robot_on_side_line msg))
    (cl:cons ':localize_with_last_known (localize_with_last_known msg))
    (cl:cons ':landmarks_found (landmarks_found msg))
))
