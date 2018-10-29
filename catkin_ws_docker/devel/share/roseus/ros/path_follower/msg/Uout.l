;; Auto-generated. Do not edit!


(when (boundp 'path_follower::Uout)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'Uout (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::UOUT")
  (make-package "PATH_FOLLOWER::UOUT"))

(in-package "ROS")
;;//! \htmlinclude Uout.msg.html


(defclass path_follower::Uout
  :super ros::object
  :slots (_acceleration _del ))

(defmethod path_follower::Uout
  (:init
   (&key
    ((:acceleration __acceleration) 0.0)
    ((:del __del) 0.0)
    )
   (send-super :init)
   (setq _acceleration (float __acceleration))
   (setq _del (float __del))
   self)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:del
   (&optional __del)
   (if __del (setq _del __del)) _del)
  (:serialization-length
   ()
   (+
    ;; float64 _acceleration
    8
    ;; float64 _del
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _del
       (sys::poke _del (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _acceleration
     (setq _acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _del
     (setq _del (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get path_follower::Uout :md5sum-) "73950a8bce4b9e8951518131a3cbe7f8")
(setf (get path_follower::Uout :datatype-) "path_follower/Uout")
(setf (get path_follower::Uout :definition-)
      "float64 acceleration
float64 del

")



(provide :path_follower/Uout "73950a8bce4b9e8951518131a3cbe7f8")


