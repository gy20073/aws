;; Auto-generated. Do not edit!


(when (boundp 'path_follower::Waypoints)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'Waypoints (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::WAYPOINTS")
  (make-package "PATH_FOLLOWER::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html


(defclass path_follower::Waypoints
  :super ros::object
  :slots (_points _dt ))

(defmethod path_follower::Waypoints
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance path_follower::Point2D :init) r)) r))
    ((:dt __dt) 0.0)
    )
   (send-super :init)
   (setq _points __points)
   (setq _dt (float __dt))
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:serialization-length
   ()
   (+
    ;; path_follower/Point2D[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; float32 _dt
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; path_follower/Point2D[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; float32 _dt
       (sys::poke _dt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; path_follower/Point2D[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance path_follower::Point2D :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _dt
     (setq _dt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get path_follower::Waypoints :md5sum-) "dfb75f45f26568fb46a75f56fa6b81e2")
(setf (get path_follower::Waypoints :datatype-) "path_follower/Waypoints")
(setf (get path_follower::Waypoints :definition-)
      "Point2D[] points
float32 dt

================================================================================
MSG: path_follower/Point2D
float64 x
float64 y

")



(provide :path_follower/Waypoints "dfb75f45f26568fb46a75f56fa6b81e2")


