;; Auto-generated. Do not edit!


(when (boundp 'path_follower::ControllerTarget)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'ControllerTarget (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::CONTROLLERTARGET")
  (make-package "PATH_FOLLOWER::CONTROLLERTARGET"))

(in-package "ROS")
;;//! \htmlinclude ControllerTarget.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass path_follower::ControllerTarget
  :super ros::object
  :slots (_header _target_velocity _target_steering_angle _cross_track_error _heading_error _timestamp ))

(defmethod path_follower::ControllerTarget
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:target_velocity __target_velocity) 0.0)
    ((:target_steering_angle __target_steering_angle) 0.0)
    ((:cross_track_error __cross_track_error) 0.0)
    ((:heading_error __heading_error) 0.0)
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _target_velocity (float __target_velocity))
   (setq _target_steering_angle (float __target_steering_angle))
   (setq _cross_track_error (float __cross_track_error))
   (setq _heading_error (float __heading_error))
   (setq _timestamp (float __timestamp))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:target_velocity
   (&optional __target_velocity)
   (if __target_velocity (setq _target_velocity __target_velocity)) _target_velocity)
  (:target_steering_angle
   (&optional __target_steering_angle)
   (if __target_steering_angle (setq _target_steering_angle __target_steering_angle)) _target_steering_angle)
  (:cross_track_error
   (&optional __cross_track_error)
   (if __cross_track_error (setq _cross_track_error __cross_track_error)) _cross_track_error)
  (:heading_error
   (&optional __heading_error)
   (if __heading_error (setq _heading_error __heading_error)) _heading_error)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _target_velocity
    8
    ;; float64 _target_steering_angle
    8
    ;; float64 _cross_track_error
    8
    ;; float64 _heading_error
    8
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _target_velocity
       (sys::poke _target_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_steering_angle
       (sys::poke _target_steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cross_track_error
       (sys::poke _cross_track_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_error
       (sys::poke _heading_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _target_velocity
     (setq _target_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_steering_angle
     (setq _target_steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cross_track_error
     (setq _cross_track_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_error
     (setq _heading_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get path_follower::ControllerTarget :md5sum-) "775bd3cf3d1e910e04e7d60f982bcda1")
(setf (get path_follower::ControllerTarget :datatype-) "path_follower/ControllerTarget")
(setf (get path_follower::ControllerTarget :definition-)
      "Header header
float64 target_velocity           # target velocity, in m/s
float64 target_steering_angle     # target steering angle, in degrees
float64 cross_track_error         # perpendicular distance to intended trajectory, in meters
float64 heading_error             # heading error, in degrees
float64 timestamp 


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :path_follower/ControllerTarget "775bd3cf3d1e910e04e7d60f982bcda1")


