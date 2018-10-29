;; Auto-generated. Do not edit!


(when (boundp 'path_follower::Trajectory2D)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'Trajectory2D (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::TRAJECTORY2D")
  (make-package "PATH_FOLLOWER::TRAJECTORY2D"))

(in-package "ROS")
;;//! \htmlinclude Trajectory2D.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass path_follower::Trajectory2D
  :super ros::object
  :slots (_header _point ))

(defmethod path_follower::Trajectory2D
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (let (r) (dotimes (i 0) (push (instance path_follower::TrajectoryPoint2D :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; path_follower/TrajectoryPoint2D[] _point
    (apply #'+ (send-all _point :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; path_follower/TrajectoryPoint2D[] _point
     (write-long (length _point) s)
     (dolist (elem _point)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; path_follower/TrajectoryPoint2D[] _point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point (let (r) (dotimes (i n) (push (instance path_follower::TrajectoryPoint2D :init) r)) r))
     (dolist (elem- _point)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get path_follower::Trajectory2D :md5sum-) "6a195a064b3031199d911bad5e3e3d69")
(setf (get path_follower::Trajectory2D :datatype-) "path_follower/Trajectory2D")
(setf (get path_follower::Trajectory2D :definition-)
      "Header header
TrajectoryPoint2D[] point

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

================================================================================
MSG: path_follower/TrajectoryPoint2D
#Header header  not sent directly
float64 t
float64 x
float64 y
float64 theta
float64 kappa
float64 kappa_dot
float64 v
float64 a
float64 jerk
float64 delta_theta     # heading misalignment with center line
float64 d               # offset to center line
float64 a_lat           # lateral (to traj not to center line!) acceleration

")



(provide :path_follower/Trajectory2D "6a195a064b3031199d911bad5e3e3d69")


