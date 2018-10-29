;; Auto-generated. Do not edit!


(when (boundp 'path_follower::state_Dynamic)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'state_Dynamic (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::STATE_DYNAMIC")
  (make-package "PATH_FOLLOWER::STATE_DYNAMIC"))

(in-package "ROS")
;;//! \htmlinclude state_Dynamic.msg.html


(defclass path_follower::state_Dynamic
  :super ros::object
  :slots (_vx _vy _X _Y _psi _wz ))

(defmethod path_follower::state_Dynamic
  (:init
   (&key
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:X __X) 0.0)
    ((:Y __Y) 0.0)
    ((:psi __psi) 0.0)
    ((:wz __wz) 0.0)
    )
   (send-super :init)
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _X (float __X))
   (setq _Y (float __Y))
   (setq _psi (float __psi))
   (setq _wz (float __wz))
   self)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:Y
   (&optional __Y)
   (if __Y (setq _Y __Y)) _Y)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:wz
   (&optional __wz)
   (if __wz (setq _wz __wz)) _wz)
  (:serialization-length
   ()
   (+
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float64 _X
    8
    ;; float64 _Y
    8
    ;; float32 _psi
    4
    ;; float32 _wz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _X
       (sys::poke _X (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y
       (sys::poke _Y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wz
       (sys::poke _wz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _X
     (setq _X (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y
     (setq _Y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _psi
     (setq _psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wz
     (setq _wz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get path_follower::state_Dynamic :md5sum-) "02fc3873b6977b7d73d21690b9600a53")
(setf (get path_follower::state_Dynamic :datatype-) "path_follower/state_Dynamic")
(setf (get path_follower::state_Dynamic :definition-)
      "float32 vx      # m/s
float32 vy      # m/s
float64 X       # m
float64 Y       # m
float32 psi     # rad
float32 wz      # rad/s


")



(provide :path_follower/state_Dynamic "02fc3873b6977b7d73d21690b9600a53")


