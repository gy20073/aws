; Auto-generated. Do not edit!


(cl:in-package mkz_intel-msg)


;//! \htmlinclude Waypoints.msg.html

(cl:defclass <Waypoints> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector mkz_intel-msg:Point2D)
   :initform (cl:make-array 0 :element-type 'mkz_intel-msg:Point2D :initial-element (cl:make-instance 'mkz_intel-msg:Point2D)))
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0))
)

(cl:defclass Waypoints (<Waypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mkz_intel-msg:<Waypoints> is deprecated: use mkz_intel-msg:Waypoints instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mkz_intel-msg:points-val is deprecated.  Use mkz_intel-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mkz_intel-msg:dt-val is deprecated.  Use mkz_intel-msg:dt instead.")
  (dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoints>) ostream)
  "Serializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoints>) istream)
  "Deserializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mkz_intel-msg:Point2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoints>)))
  "Returns string type for a message object of type '<Waypoints>"
  "mkz_intel/Waypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoints)))
  "Returns string type for a message object of type 'Waypoints"
  "mkz_intel/Waypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoints>)))
  "Returns md5sum for a message object of type '<Waypoints>"
  "dfb75f45f26568fb46a75f56fa6b81e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoints)))
  "Returns md5sum for a message object of type 'Waypoints"
  "dfb75f45f26568fb46a75f56fa6b81e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoints>)))
  "Returns full string definition for message of type '<Waypoints>"
  (cl:format cl:nil "Point2D[] points~%float32 dt~%~%================================================================================~%MSG: mkz_intel/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoints)))
  "Returns full string definition for message of type 'Waypoints"
  (cl:format cl:nil "Point2D[] points~%float32 dt~%~%================================================================================~%MSG: mkz_intel/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoints
    (cl:cons ':points (points msg))
    (cl:cons ':dt (dt msg))
))
