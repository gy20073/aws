; Auto-generated. Do not edit!


(cl:in-package path_follower-msg)


;//! \htmlinclude Uout.msg.html

(cl:defclass <Uout> (roslisp-msg-protocol:ros-message)
  ((acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (del
    :reader del
    :initarg :del
    :type cl:float
    :initform 0.0))
)

(cl:defclass Uout (<Uout>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Uout>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Uout)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-msg:<Uout> is deprecated: use path_follower-msg:Uout instead.")))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <Uout>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:acceleration-val is deprecated.  Use path_follower-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'del-val :lambda-list '(m))
(cl:defmethod del-val ((m <Uout>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:del-val is deprecated.  Use path_follower-msg:del instead.")
  (del m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Uout>) ostream)
  "Serializes a message object of type '<Uout>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'del))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Uout>) istream)
  "Deserializes a message object of type '<Uout>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'del) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Uout>)))
  "Returns string type for a message object of type '<Uout>"
  "path_follower/Uout")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Uout)))
  "Returns string type for a message object of type 'Uout"
  "path_follower/Uout")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Uout>)))
  "Returns md5sum for a message object of type '<Uout>"
  "73950a8bce4b9e8951518131a3cbe7f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Uout)))
  "Returns md5sum for a message object of type 'Uout"
  "73950a8bce4b9e8951518131a3cbe7f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Uout>)))
  "Returns full string definition for message of type '<Uout>"
  (cl:format cl:nil "float64 acceleration~%float64 del~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Uout)))
  "Returns full string definition for message of type 'Uout"
  (cl:format cl:nil "float64 acceleration~%float64 del~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Uout>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Uout>))
  "Converts a ROS message object to a list"
  (cl:list 'Uout
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':del (del msg))
))
