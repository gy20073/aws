;; Auto-generated. Do not edit!


(when (boundp 'dbw_mkz_msgs::GearCmd)
  (if (not (find-package "DBW_MKZ_MSGS"))
    (make-package "DBW_MKZ_MSGS"))
  (shadow 'GearCmd (find-package "DBW_MKZ_MSGS")))
(unless (find-package "DBW_MKZ_MSGS::GEARCMD")
  (make-package "DBW_MKZ_MSGS::GEARCMD"))

(in-package "ROS")
;;//! \htmlinclude GearCmd.msg.html


(defclass dbw_mkz_msgs::GearCmd
  :super ros::object
  :slots (_cmd ))

(defmethod dbw_mkz_msgs::GearCmd
  (:init
   (&key
    ((:cmd __cmd) (instance dbw_mkz_msgs::Gear :init))
    )
   (send-super :init)
   (setq _cmd __cmd)
   self)
  (:cmd
   (&rest __cmd)
   (if (keywordp (car __cmd))
       (send* _cmd __cmd)
     (progn
       (if __cmd (setq _cmd (car __cmd)))
       _cmd)))
  (:serialization-length
   ()
   (+
    ;; dbw_mkz_msgs/Gear _cmd
    (send _cmd :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dbw_mkz_msgs/Gear _cmd
       (send _cmd :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dbw_mkz_msgs/Gear _cmd
     (send _cmd :deserialize buf ptr-) (incf ptr- (send _cmd :serialization-length))
   ;;
   self)
  )

(setf (get dbw_mkz_msgs::GearCmd :md5sum-) "e4d2b0484c21e028e30e3c34f7f76ad3")
(setf (get dbw_mkz_msgs::GearCmd :datatype-) "dbw_mkz_msgs/GearCmd")
(setf (get dbw_mkz_msgs::GearCmd :definition-)
      "# Gear command enumeration
Gear cmd

================================================================================
MSG: dbw_mkz_msgs/Gear
uint8 gear

uint8 NONE=0
uint8 PARK=1
uint8 REVERSE=2
uint8 NEUTRAL=3
uint8 DRIVE=4
uint8 LOW=5

")



(provide :dbw_mkz_msgs/GearCmd "e4d2b0484c21e028e30e3c34f7f76ad3")


