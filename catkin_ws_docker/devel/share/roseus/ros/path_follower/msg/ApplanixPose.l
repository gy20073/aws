;; Auto-generated. Do not edit!


(when (boundp 'path_follower::ApplanixPose)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'ApplanixPose (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::APPLANIXPOSE")
  (make-package "PATH_FOLLOWER::APPLANIXPOSE"))

(in-package "ROS")
;;//! \htmlinclude ApplanixPose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass path_follower::ApplanixPose
  :super ros::object
  :slots (_header _timestamp _smooth_x _smooth_y _smooth_z _latitude _longitude _altitude _vel_north _vel_east _vel_up _speed _track _roll _pitch _yaw _rate_roll _rate_pitch _rate_yaw _accel_x _accel_y _accel_z _wander _id _postprocess_code _hardware_timestamp _hardware_time_mode ))

(defmethod path_follower::ApplanixPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:timestamp __timestamp) 0.0)
    ((:smooth_x __smooth_x) 0.0)
    ((:smooth_y __smooth_y) 0.0)
    ((:smooth_z __smooth_z) 0.0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:vel_north __vel_north) 0.0)
    ((:vel_east __vel_east) 0.0)
    ((:vel_up __vel_up) 0.0)
    ((:speed __speed) 0.0)
    ((:track __track) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:rate_roll __rate_roll) 0.0)
    ((:rate_pitch __rate_pitch) 0.0)
    ((:rate_yaw __rate_yaw) 0.0)
    ((:accel_x __accel_x) 0.0)
    ((:accel_y __accel_y) 0.0)
    ((:accel_z __accel_z) 0.0)
    ((:wander __wander) 0.0)
    ((:id __id) 0)
    ((:postprocess_code __postprocess_code) 0)
    ((:hardware_timestamp __hardware_timestamp) 0.0)
    ((:hardware_time_mode __hardware_time_mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _timestamp (float __timestamp))
   (setq _smooth_x (float __smooth_x))
   (setq _smooth_y (float __smooth_y))
   (setq _smooth_z (float __smooth_z))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _vel_north (float __vel_north))
   (setq _vel_east (float __vel_east))
   (setq _vel_up (float __vel_up))
   (setq _speed (float __speed))
   (setq _track (float __track))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _rate_roll (float __rate_roll))
   (setq _rate_pitch (float __rate_pitch))
   (setq _rate_yaw (float __rate_yaw))
   (setq _accel_x (float __accel_x))
   (setq _accel_y (float __accel_y))
   (setq _accel_z (float __accel_z))
   (setq _wander (float __wander))
   (setq _id (round __id))
   (setq _postprocess_code (round __postprocess_code))
   (setq _hardware_timestamp (float __hardware_timestamp))
   (setq _hardware_time_mode (round __hardware_time_mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:smooth_x
   (&optional __smooth_x)
   (if __smooth_x (setq _smooth_x __smooth_x)) _smooth_x)
  (:smooth_y
   (&optional __smooth_y)
   (if __smooth_y (setq _smooth_y __smooth_y)) _smooth_y)
  (:smooth_z
   (&optional __smooth_z)
   (if __smooth_z (setq _smooth_z __smooth_z)) _smooth_z)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:vel_north
   (&optional __vel_north)
   (if __vel_north (setq _vel_north __vel_north)) _vel_north)
  (:vel_east
   (&optional __vel_east)
   (if __vel_east (setq _vel_east __vel_east)) _vel_east)
  (:vel_up
   (&optional __vel_up)
   (if __vel_up (setq _vel_up __vel_up)) _vel_up)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:track
   (&optional __track)
   (if __track (setq _track __track)) _track)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:rate_roll
   (&optional __rate_roll)
   (if __rate_roll (setq _rate_roll __rate_roll)) _rate_roll)
  (:rate_pitch
   (&optional __rate_pitch)
   (if __rate_pitch (setq _rate_pitch __rate_pitch)) _rate_pitch)
  (:rate_yaw
   (&optional __rate_yaw)
   (if __rate_yaw (setq _rate_yaw __rate_yaw)) _rate_yaw)
  (:accel_x
   (&optional __accel_x)
   (if __accel_x (setq _accel_x __accel_x)) _accel_x)
  (:accel_y
   (&optional __accel_y)
   (if __accel_y (setq _accel_y __accel_y)) _accel_y)
  (:accel_z
   (&optional __accel_z)
   (if __accel_z (setq _accel_z __accel_z)) _accel_z)
  (:wander
   (&optional __wander)
   (if __wander (setq _wander __wander)) _wander)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:postprocess_code
   (&optional __postprocess_code)
   (if __postprocess_code (setq _postprocess_code __postprocess_code)) _postprocess_code)
  (:hardware_timestamp
   (&optional __hardware_timestamp)
   (if __hardware_timestamp (setq _hardware_timestamp __hardware_timestamp)) _hardware_timestamp)
  (:hardware_time_mode
   (&optional __hardware_time_mode)
   (if __hardware_time_mode (setq _hardware_time_mode __hardware_time_mode)) _hardware_time_mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _timestamp
    8
    ;; float64 _smooth_x
    8
    ;; float64 _smooth_y
    8
    ;; float64 _smooth_z
    8
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float32 _vel_north
    4
    ;; float32 _vel_east
    4
    ;; float32 _vel_up
    4
    ;; float32 _speed
    4
    ;; float32 _track
    4
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; float64 _rate_roll
    8
    ;; float64 _rate_pitch
    8
    ;; float64 _rate_yaw
    8
    ;; float64 _accel_x
    8
    ;; float64 _accel_y
    8
    ;; float64 _accel_z
    8
    ;; float64 _wander
    8
    ;; uint32 _id
    4
    ;; int32 _postprocess_code
    4
    ;; float64 _hardware_timestamp
    8
    ;; int32 _hardware_time_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _smooth_x
       (sys::poke _smooth_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _smooth_y
       (sys::poke _smooth_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _smooth_z
       (sys::poke _smooth_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _vel_north
       (sys::poke _vel_north (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_east
       (sys::poke _vel_east (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_up
       (sys::poke _vel_up (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track
       (sys::poke _track (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rate_roll
       (sys::poke _rate_roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rate_pitch
       (sys::poke _rate_pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rate_yaw
       (sys::poke _rate_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_x
       (sys::poke _accel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_y
       (sys::poke _accel_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_z
       (sys::poke _accel_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wander
       (sys::poke _wander (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _id
       (write-long _id s)
     ;; int32 _postprocess_code
       (write-long _postprocess_code s)
     ;; float64 _hardware_timestamp
       (sys::poke _hardware_timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _hardware_time_mode
       (write-long _hardware_time_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _smooth_x
     (setq _smooth_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _smooth_y
     (setq _smooth_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _smooth_z
     (setq _smooth_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _vel_north
     (setq _vel_north (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_east
     (setq _vel_east (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_up
     (setq _vel_up (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track
     (setq _track (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rate_roll
     (setq _rate_roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rate_pitch
     (setq _rate_pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rate_yaw
     (setq _rate_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_x
     (setq _accel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_y
     (setq _accel_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_z
     (setq _accel_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wander
     (setq _wander (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _postprocess_code
     (setq _postprocess_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _hardware_timestamp
     (setq _hardware_timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _hardware_time_mode
     (setq _hardware_time_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get path_follower::ApplanixPose :md5sum-) "cc005b38b12f428cb31db2f4286c261b")
(setf (get path_follower::ApplanixPose :datatype-) "path_follower/ApplanixPose")
(setf (get path_follower::ApplanixPose :definition-)
      "Header header
float64 timestamp
float64 smooth_x
float64 smooth_y
float64 smooth_z
float64 latitude
float64 longitude
float64 altitude
float32 vel_north
float32 vel_east
float32 vel_up
float32 speed
float32 track
float64 roll
float64 pitch
float64 yaw
float64 rate_roll
float64 rate_pitch
float64 rate_yaw
float64 accel_x
float64 accel_y
float64 accel_z
float64 wander
uint32  id
int32   postprocess_code
float64 hardware_timestamp
int32   hardware_time_mode

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



(provide :path_follower/ApplanixPose "cc005b38b12f428cb31db2f4286c261b")


