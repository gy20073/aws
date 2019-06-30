;; Auto-generated. Do not edit!


(when (boundp 'mkz_intel::Waypoints)
  (if (not (find-package "MKZ_INTEL"))
    (make-package "MKZ_INTEL"))
  (shadow 'Waypoints (find-package "MKZ_INTEL")))
(unless (find-package "MKZ_INTEL::WAYPOINTS")
  (make-package "MKZ_INTEL::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html


(defclass mkz_intel::Waypoints
  :super ros::object
  :slots (_points _dt ))

(defmethod mkz_intel::Waypoints
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance mkz_intel::Point2D :init) r)) r))
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
    ;; mkz_intel/Point2D[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; float32 _dt
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; mkz_intel/Point2D[] _points
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
   ;; mkz_intel/Point2D[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance mkz_intel::Point2D :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _dt
     (setq _dt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mkz_intel::Waypoints :md5sum-) "dfb75f45f26568fb46a75f56fa6b81e2")
(setf (get mkz_intel::Waypoints :datatype-) "mkz_intel/Waypoints")
(setf (get mkz_intel::Waypoints :definition-)
      "Point2D[] points
float32 dt

================================================================================
MSG: mkz_intel/Point2D
float64 x
float64 y

")



(provide :mkz_intel/Waypoints "dfb75f45f26568fb46a75f56fa6b81e2")


