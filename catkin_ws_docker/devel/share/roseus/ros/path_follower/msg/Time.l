;; Auto-generated. Do not edit!


(when (boundp 'path_follower::Time)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'Time (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::TIME")
  (make-package "PATH_FOLLOWER::TIME"))

(in-package "ROS")
;;//! \htmlinclude Time.msg.html


(defclass path_follower::Time
  :super ros::object
  :slots (_t_start _t_end _mpc_start _t1 _t2 _t3 _t4 _t5 _t6 _t7 _t8 _t9 _solver_start _solver_end _mpc_end ))

(defmethod path_follower::Time
  (:init
   (&key
    ((:t_start __t_start) 0.0)
    ((:t_end __t_end) 0.0)
    ((:mpc_start __mpc_start) 0.0)
    ((:t1 __t1) 0.0)
    ((:t2 __t2) 0.0)
    ((:t3 __t3) 0.0)
    ((:t4 __t4) 0.0)
    ((:t5 __t5) 0.0)
    ((:t6 __t6) 0.0)
    ((:t7 __t7) 0.0)
    ((:t8 __t8) 0.0)
    ((:t9 __t9) 0.0)
    ((:solver_start __solver_start) 0.0)
    ((:solver_end __solver_end) 0.0)
    ((:mpc_end __mpc_end) 0.0)
    )
   (send-super :init)
   (setq _t_start (float __t_start))
   (setq _t_end (float __t_end))
   (setq _mpc_start (float __mpc_start))
   (setq _t1 (float __t1))
   (setq _t2 (float __t2))
   (setq _t3 (float __t3))
   (setq _t4 (float __t4))
   (setq _t5 (float __t5))
   (setq _t6 (float __t6))
   (setq _t7 (float __t7))
   (setq _t8 (float __t8))
   (setq _t9 (float __t9))
   (setq _solver_start (float __solver_start))
   (setq _solver_end (float __solver_end))
   (setq _mpc_end (float __mpc_end))
   self)
  (:t_start
   (&optional __t_start)
   (if __t_start (setq _t_start __t_start)) _t_start)
  (:t_end
   (&optional __t_end)
   (if __t_end (setq _t_end __t_end)) _t_end)
  (:mpc_start
   (&optional __mpc_start)
   (if __mpc_start (setq _mpc_start __mpc_start)) _mpc_start)
  (:t1
   (&optional __t1)
   (if __t1 (setq _t1 __t1)) _t1)
  (:t2
   (&optional __t2)
   (if __t2 (setq _t2 __t2)) _t2)
  (:t3
   (&optional __t3)
   (if __t3 (setq _t3 __t3)) _t3)
  (:t4
   (&optional __t4)
   (if __t4 (setq _t4 __t4)) _t4)
  (:t5
   (&optional __t5)
   (if __t5 (setq _t5 __t5)) _t5)
  (:t6
   (&optional __t6)
   (if __t6 (setq _t6 __t6)) _t6)
  (:t7
   (&optional __t7)
   (if __t7 (setq _t7 __t7)) _t7)
  (:t8
   (&optional __t8)
   (if __t8 (setq _t8 __t8)) _t8)
  (:t9
   (&optional __t9)
   (if __t9 (setq _t9 __t9)) _t9)
  (:solver_start
   (&optional __solver_start)
   (if __solver_start (setq _solver_start __solver_start)) _solver_start)
  (:solver_end
   (&optional __solver_end)
   (if __solver_end (setq _solver_end __solver_end)) _solver_end)
  (:mpc_end
   (&optional __mpc_end)
   (if __mpc_end (setq _mpc_end __mpc_end)) _mpc_end)
  (:serialization-length
   ()
   (+
    ;; float64 _t_start
    8
    ;; float64 _t_end
    8
    ;; float64 _mpc_start
    8
    ;; float64 _t1
    8
    ;; float64 _t2
    8
    ;; float64 _t3
    8
    ;; float64 _t4
    8
    ;; float64 _t5
    8
    ;; float64 _t6
    8
    ;; float64 _t7
    8
    ;; float64 _t8
    8
    ;; float64 _t9
    8
    ;; float64 _solver_start
    8
    ;; float64 _solver_end
    8
    ;; float64 _mpc_end
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _t_start
       (sys::poke _t_start (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t_end
       (sys::poke _t_end (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mpc_start
       (sys::poke _mpc_start (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t1
       (sys::poke _t1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t2
       (sys::poke _t2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t3
       (sys::poke _t3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t4
       (sys::poke _t4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t5
       (sys::poke _t5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t6
       (sys::poke _t6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t7
       (sys::poke _t7 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t8
       (sys::poke _t8 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t9
       (sys::poke _t9 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _solver_start
       (sys::poke _solver_start (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _solver_end
       (sys::poke _solver_end (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mpc_end
       (sys::poke _mpc_end (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _t_start
     (setq _t_start (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t_end
     (setq _t_end (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mpc_start
     (setq _mpc_start (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t1
     (setq _t1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t2
     (setq _t2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t3
     (setq _t3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t4
     (setq _t4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t5
     (setq _t5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t6
     (setq _t6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t7
     (setq _t7 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t8
     (setq _t8 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t9
     (setq _t9 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _solver_start
     (setq _solver_start (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _solver_end
     (setq _solver_end (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mpc_end
     (setq _mpc_end (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get path_follower::Time :md5sum-) "0f100a1b959faa915aef817b8b8f13a8")
(setf (get path_follower::Time :datatype-) "path_follower/Time")
(setf (get path_follower::Time :definition-)
      "float64 t_start
float64 t_end
float64 mpc_start
float64 t1
float64 t2
float64 t3
float64 t4
float64 t5
float64 t6
float64 t7
float64 t8
float64 t9
float64 solver_start
float64 solver_end
float64 mpc_end



")



(provide :path_follower/Time "0f100a1b959faa915aef817b8b8f13a8")


