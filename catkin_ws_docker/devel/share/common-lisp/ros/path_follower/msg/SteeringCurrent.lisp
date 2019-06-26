; Auto-generated. Do not edit!


(cl:in-package path_follower-msg)


;//! \htmlinclude SteeringCurrent.msg.html

(cl:defclass <SteeringCurrent> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SteeringCurrent (<SteeringCurrent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SteeringCurrent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SteeringCurrent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-msg:<SteeringCurrent> is deprecated: use path_follower-msg:SteeringCurrent instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <SteeringCurrent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:steering_angle-val is deprecated.  Use path_follower-msg:steering_angle instead.")
  (steering_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SteeringCurrent>) ostream)
  "Serializes a message object of type '<SteeringCurrent>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SteeringCurrent>) istream)
  "Deserializes a message object of type '<SteeringCurrent>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SteeringCurrent>)))
  "Returns string type for a message object of type '<SteeringCurrent>"
  "path_follower/SteeringCurrent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SteeringCurrent)))
  "Returns string type for a message object of type 'SteeringCurrent"
  "path_follower/SteeringCurrent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SteeringCurrent>)))
  "Returns md5sum for a message object of type '<SteeringCurrent>"
  "0a70504669a8dacda06aa18fb270817f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SteeringCurrent)))
  "Returns md5sum for a message object of type 'SteeringCurrent"
  "0a70504669a8dacda06aa18fb270817f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SteeringCurrent>)))
  "Returns full string definition for message of type '<SteeringCurrent>"
  (cl:format cl:nil "float64 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SteeringCurrent)))
  "Returns full string definition for message of type 'SteeringCurrent"
  (cl:format cl:nil "float64 steering_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SteeringCurrent>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SteeringCurrent>))
  "Converts a ROS message object to a list"
  (cl:list 'SteeringCurrent
    (cl:cons ':steering_angle (steering_angle msg))
))
