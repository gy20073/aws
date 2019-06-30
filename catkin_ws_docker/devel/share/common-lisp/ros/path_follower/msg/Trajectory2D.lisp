; Auto-generated. Do not edit!


(cl:in-package path_follower-msg)


;//! \htmlinclude Trajectory2D.msg.html

(cl:defclass <Trajectory2D> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type (cl:vector path_follower-msg:TrajectoryPoint2D)
   :initform (cl:make-array 0 :element-type 'path_follower-msg:TrajectoryPoint2D :initial-element (cl:make-instance 'path_follower-msg:TrajectoryPoint2D))))
)

(cl:defclass Trajectory2D (<Trajectory2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trajectory2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trajectory2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-msg:<Trajectory2D> is deprecated: use path_follower-msg:Trajectory2D instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Trajectory2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:header-val is deprecated.  Use path_follower-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Trajectory2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-msg:point-val is deprecated.  Use path_follower-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trajectory2D>) ostream)
  "Serializes a message object of type '<Trajectory2D>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'point))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trajectory2D>) istream)
  "Deserializes a message object of type '<Trajectory2D>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'path_follower-msg:TrajectoryPoint2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trajectory2D>)))
  "Returns string type for a message object of type '<Trajectory2D>"
  "path_follower/Trajectory2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trajectory2D)))
  "Returns string type for a message object of type 'Trajectory2D"
  "path_follower/Trajectory2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trajectory2D>)))
  "Returns md5sum for a message object of type '<Trajectory2D>"
  "6a195a064b3031199d911bad5e3e3d69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trajectory2D)))
  "Returns md5sum for a message object of type 'Trajectory2D"
  "6a195a064b3031199d911bad5e3e3d69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trajectory2D>)))
  "Returns full string definition for message of type '<Trajectory2D>"
  (cl:format cl:nil "Header header~%TrajectoryPoint2D[] point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: path_follower/TrajectoryPoint2D~%#Header header  not sent directly~%float64 t~%float64 x~%float64 y~%float64 theta~%float64 kappa~%float64 kappa_dot~%float64 v~%float64 a~%float64 jerk~%float64 delta_theta     # heading misalignment with center line~%float64 d               # offset to center line~%float64 a_lat           # lateral (to traj not to center line!) acceleration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trajectory2D)))
  "Returns full string definition for message of type 'Trajectory2D"
  (cl:format cl:nil "Header header~%TrajectoryPoint2D[] point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: path_follower/TrajectoryPoint2D~%#Header header  not sent directly~%float64 t~%float64 x~%float64 y~%float64 theta~%float64 kappa~%float64 kappa_dot~%float64 v~%float64 a~%float64 jerk~%float64 delta_theta     # heading misalignment with center line~%float64 d               # offset to center line~%float64 a_lat           # lateral (to traj not to center line!) acceleration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trajectory2D>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trajectory2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Trajectory2D
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
))
