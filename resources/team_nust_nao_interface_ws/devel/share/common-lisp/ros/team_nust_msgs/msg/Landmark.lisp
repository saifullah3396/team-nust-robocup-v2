; Auto-generated. Do not edit!


(cl:in-package team_nust_msgs-msg)


;//! \htmlinclude Landmark.msg.html

(cl:defclass <Landmark> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass Landmark (<Landmark>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmark>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmark)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name team_nust_msgs-msg:<Landmark> is deprecated: use team_nust_msgs-msg:Landmark instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:type-val is deprecated.  Use team_nust_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader team_nust_msgs-msg:pos-val is deprecated.  Use team_nust_msgs-msg:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmark>) ostream)
  "Serializes a message object of type '<Landmark>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmark>) istream)
  "Deserializes a message object of type '<Landmark>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmark>)))
  "Returns string type for a message object of type '<Landmark>"
  "team_nust_msgs/Landmark")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark)))
  "Returns string type for a message object of type 'Landmark"
  "team_nust_msgs/Landmark")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmark>)))
  "Returns md5sum for a message object of type '<Landmark>"
  "a44c2e55d48971b074b6d4f211c997d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmark)))
  "Returns md5sum for a message object of type 'Landmark"
  "a44c2e55d48971b074b6d4f211c997d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmark>)))
  "Returns full string definition for message of type '<Landmark>"
  (cl:format cl:nil "uint8 type~%geometry_msgs/Point pos~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmark)))
  "Returns full string definition for message of type 'Landmark"
  (cl:format cl:nil "uint8 type~%geometry_msgs/Point pos~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmark>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmark>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmark
    (cl:cons ':type (type msg))
    (cl:cons ':pos (pos msg))
))
