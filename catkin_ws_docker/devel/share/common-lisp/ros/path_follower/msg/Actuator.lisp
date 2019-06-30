; Auto-generated. Do not edit!


(cl:in-package path_follower-msg)


;//! \htmlinclude Actuator.msg.html

(cl:defclass <Actuator> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (direction
    :reader direction
    :initarg :direction
    :type cl:fixnum
    :initform 0)
   (steering_mode
    :reader steering_mode
    :initarg :steering_mode
    :type cl:fixnum
    :initform 0)
   (steering_value
    :reader steering_value
    :initarg :steering_value
    :type cl:float
    :initform 0.0)
   (brake_pressure
    :reader brake_pressure
    :initarg :brake_pressure
    :type cl:float
    :initform 0.0)
   (throttle_fraction
    :reader throttle_fraction
    :initarg :throttle_fraction
    :type cl:float
    :initform 0.0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0))
)

(cl:defclass Actuator (<Actuator>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Actuator>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Actuator)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-msg:<Actuator> is deprecated: use path_follower-msg:Actuator instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:header-val is deprecated.  Use path_follower-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:direction-val is deprecated.  Use path_follower-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'steering_mode-val :lambda-list '(m))
(cl:defmethod steering_mode-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:steering_mode-val is deprecated.  Use path_follower-msg:steering_mode instead.")
  (steering_mode m))

(cl:ensure-generic-function 'steering_value-val :lambda-list '(m))
(cl:defmethod steering_value-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:steering_value-val is deprecated.  Use path_follower-msg:steering_value instead.")
  (steering_value m))

(cl:ensure-generic-function 'brake_pressure-val :lambda-list '(m))
(cl:defmethod brake_pressure-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:brake_pressure-val is deprecated.  Use path_follower-msg:brake_pressure instead.")
  (brake_pressure m))

(cl:ensure-generic-function 'throttle_fraction-val :lambda-list '(m))
(cl:defmethod throttle_fraction-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:throttle_fraction-val is deprecated.  Use path_follower-msg:throttle_fraction instead.")
  (throttle_fraction m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:timestamp-val is deprecated.  Use path_follower-msg:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Actuator>)))
    "Constants for message type '<Actuator>"
  '((:DIRECTION_FORWARD . 0)
    (:DIRECTION_REVERSE . 1)
    (:ANGLE_CONTROL . 0)
    (:TORQUE_CONTROL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Actuator)))
    "Constants for message type 'Actuator"
  '((:DIRECTION_FORWARD . 0)
    (:DIRECTION_REVERSE . 1)
    (:ANGLE_CONTROL . 0)
    (:TORQUE_CONTROL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Actuator>) ostream)
  "Serializes a message object of type '<Actuator>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steering_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'brake_pressure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle_fraction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Actuator>) istream)
  "Deserializes a message object of type '<Actuator>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'direction)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steering_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brake_pressure) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle_fraction) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestamp) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Actuator>)))
  "Returns string type for a message object of type '<Actuator>"
  "path_follower/Actuator")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Actuator)))
  "Returns string type for a message object of type 'Actuator"
  "path_follower/Actuator")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Actuator>)))
  "Returns md5sum for a message object of type '<Actuator>"
  "8164b580b4ff14b976d32e45c6c9f9d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Actuator)))
  "Returns md5sum for a message object of type 'Actuator"
  "8164b580b4ff14b976d32e45c6c9f9d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Actuator>)))
  "Returns full string definition for message of type '<Actuator>"
  (cl:format cl:nil "Header header~%uint8 direction~%uint8 steering_mode~%float64 steering_value     # depending on steering_mode this is either angle or torque~%float64 brake_pressure~%float64 throttle_fraction~%float64 timestamp~%~%  # for direction~%uint8 DIRECTION_FORWARD = 0~%uint8 DIRECTION_REVERSE = 1~%~%  # for steering mode~%uint8 ANGLE_CONTROL = 0~%uint8 TORQUE_CONTROL = 1~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Actuator)))
  "Returns full string definition for message of type 'Actuator"
  (cl:format cl:nil "Header header~%uint8 direction~%uint8 steering_mode~%float64 steering_value     # depending on steering_mode this is either angle or torque~%float64 brake_pressure~%float64 throttle_fraction~%float64 timestamp~%~%  # for direction~%uint8 DIRECTION_FORWARD = 0~%uint8 DIRECTION_REVERSE = 1~%~%  # for steering mode~%uint8 ANGLE_CONTROL = 0~%uint8 TORQUE_CONTROL = 1~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Actuator>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Actuator>))
  "Converts a ROS message object to a list"
  (cl:list 'Actuator
    (cl:cons ':header (header msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':steering_mode (steering_mode msg))
    (cl:cons ':steering_value (steering_value msg))
    (cl:cons ':brake_pressure (brake_pressure msg))
    (cl:cons ':throttle_fraction (throttle_fraction msg))
    (cl:cons ':timestamp (timestamp msg))
))
