;; Auto-generated. Do not edit!


(when (boundp 'path_follower::TrajectoryPoint2D)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'TrajectoryPoint2D (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::TRAJECTORYPOINT2D")
  (make-package "PATH_FOLLOWER::TRAJECTORYPOINT2D"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryPoint2D.msg.html


(defclass path_follower::TrajectoryPoint2D
  :super ros::object
  :slots (_t _x _y _theta _kappa _kappa_dot _v _a _jerk _delta_theta _d _a_lat ))

(defmethod path_follower::TrajectoryPoint2D
  (:init
   (&key
    ((:t __t) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:kappa __kappa) 0.0)
    ((:kappa_dot __kappa_dot) 0.0)
    ((:v __v) 0.0)
    ((:a __a) 0.0)
    ((:jerk __jerk) 0.0)
    ((:delta_theta __delta_theta) 0.0)
    ((:d __d) 0.0)
    ((:a_lat __a_lat) 0.0)
    )
   (send-super :init)
   (setq _t (float __t))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _kappa (float __kappa))
   (setq _kappa_dot (float __kappa_dot))
   (setq _v (float __v))
   (setq _a (float __a))
   (setq _jerk (float __jerk))
   (setq _delta_theta (float __delta_theta))
   (setq _d (float __d))
   (setq _a_lat (float __a_lat))
   self)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:kappa
   (&optional __kappa)
   (if __kappa (setq _kappa __kappa)) _kappa)
  (:kappa_dot
   (&optional __kappa_dot)
   (if __kappa_dot (setq _kappa_dot __kappa_dot)) _kappa_dot)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:jerk
   (&optional __jerk)
   (if __jerk (setq _jerk __jerk)) _jerk)
  (:delta_theta
   (&optional __delta_theta)
   (if __delta_theta (setq _delta_theta __delta_theta)) _delta_theta)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:a_lat
   (&optional __a_lat)
   (if __a_lat (setq _a_lat __a_lat)) _a_lat)
  (:serialization-length
   ()
   (+
    ;; float64 _t
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ;; float64 _kappa
    8
    ;; float64 _kappa_dot
    8
    ;; float64 _v
    8
    ;; float64 _a
    8
    ;; float64 _jerk
    8
    ;; float64 _delta_theta
    8
    ;; float64 _d
    8
    ;; float64 _a_lat
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _t
       (sys::poke _t (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kappa
       (sys::poke _kappa (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kappa_dot
       (sys::poke _kappa_dot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _jerk
       (sys::poke _jerk (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_theta
       (sys::poke _delta_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d
       (sys::poke _d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a_lat
       (sys::poke _a_lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _t
     (setq _t (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kappa
     (setq _kappa (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kappa_dot
     (setq _kappa_dot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _jerk
     (setq _jerk (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_theta
     (setq _delta_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d
     (setq _d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a_lat
     (setq _a_lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get path_follower::TrajectoryPoint2D :md5sum-) "63ab900fed4c2c35d54c1d98c787e72b")
(setf (get path_follower::TrajectoryPoint2D :datatype-) "path_follower/TrajectoryPoint2D")
(setf (get path_follower::TrajectoryPoint2D :definition-)
      "#Header header  not sent directly
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



(provide :path_follower/TrajectoryPoint2D "63ab900fed4c2c35d54c1d98c787e72b")


