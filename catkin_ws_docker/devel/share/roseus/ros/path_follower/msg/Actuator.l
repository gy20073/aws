;; Auto-generated. Do not edit!


(when (boundp 'path_follower::Actuator)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'Actuator (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::ACTUATOR")
  (make-package "PATH_FOLLOWER::ACTUATOR"))

(in-package "ROS")
;;//! \htmlinclude Actuator.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*DIRECTION_FORWARD*" (find-package "PATH_FOLLOWER::ACTUATOR"))
(shadow '*DIRECTION_FORWARD* (find-package "PATH_FOLLOWER::ACTUATOR"))
(defconstant path_follower::Actuator::*DIRECTION_FORWARD* 0)
(intern "*DIRECTION_REVERSE*" (find-package "PATH_FOLLOWER::ACTUATOR"))
(shadow '*DIRECTION_REVERSE* (find-package "PATH_FOLLOWER::ACTUATOR"))
(defconstant path_follower::Actuator::*DIRECTION_REVERSE* 1)
(intern "*ANGLE_CONTROL*" (find-package "PATH_FOLLOWER::ACTUATOR"))
(shadow '*ANGLE_CONTROL* (find-package "PATH_FOLLOWER::ACTUATOR"))
(defconstant path_follower::Actuator::*ANGLE_CONTROL* 0)
(intern "*TORQUE_CONTROL*" (find-package "PATH_FOLLOWER::ACTUATOR"))
(shadow '*TORQUE_CONTROL* (find-package "PATH_FOLLOWER::ACTUATOR"))
(defconstant path_follower::Actuator::*TORQUE_CONTROL* 1)
(defclass path_follower::Actuator
  :super ros::object
  :slots (_header _direction _steering_mode _steering_value _brake_pressure _throttle_fraction _timestamp ))

(defmethod path_follower::Actuator
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:direction __direction) 0)
    ((:steering_mode __steering_mode) 0)
    ((:steering_value __steering_value) 0.0)
    ((:brake_pressure __brake_pressure) 0.0)
    ((:throttle_fraction __throttle_fraction) 0.0)
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _direction (round __direction))
   (setq _steering_mode (round __steering_mode))
   (setq _steering_value (float __steering_value))
   (setq _brake_pressure (float __brake_pressure))
   (setq _throttle_fraction (float __throttle_fraction))
   (setq _timestamp (float __timestamp))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:steering_mode
   (&optional __steering_mode)
   (if __steering_mode (setq _steering_mode __steering_mode)) _steering_mode)
  (:steering_value
   (&optional __steering_value)
   (if __steering_value (setq _steering_value __steering_value)) _steering_value)
  (:brake_pressure
   (&optional __brake_pressure)
   (if __brake_pressure (setq _brake_pressure __brake_pressure)) _brake_pressure)
  (:throttle_fraction
   (&optional __throttle_fraction)
   (if __throttle_fraction (setq _throttle_fraction __throttle_fraction)) _throttle_fraction)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _direction
    1
    ;; uint8 _steering_mode
    1
    ;; float64 _steering_value
    8
    ;; float64 _brake_pressure
    8
    ;; float64 _throttle_fraction
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
     ;; uint8 _direction
       (write-byte _direction s)
     ;; uint8 _steering_mode
       (write-byte _steering_mode s)
     ;; float64 _steering_value
       (sys::poke _steering_value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _brake_pressure
       (sys::poke _brake_pressure (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _throttle_fraction
       (sys::poke _throttle_fraction (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _direction
     (setq _direction (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _steering_mode
     (setq _steering_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _steering_value
     (setq _steering_value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _brake_pressure
     (setq _brake_pressure (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _throttle_fraction
     (setq _throttle_fraction (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get path_follower::Actuator :md5sum-) "8164b580b4ff14b976d32e45c6c9f9d9")
(setf (get path_follower::Actuator :datatype-) "path_follower/Actuator")
(setf (get path_follower::Actuator :definition-)
      "Header header
uint8 direction
uint8 steering_mode
float64 steering_value     # depending on steering_mode this is either angle or torque
float64 brake_pressure
float64 throttle_fraction
float64 timestamp

  # for direction
uint8 DIRECTION_FORWARD = 0
uint8 DIRECTION_REVERSE = 1

  # for steering mode
uint8 ANGLE_CONTROL = 0
uint8 TORQUE_CONTROL = 1



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



(provide :path_follower/Actuator "8164b580b4ff14b976d32e45c6c9f9d9")


