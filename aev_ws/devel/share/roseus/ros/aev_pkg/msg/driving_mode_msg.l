;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::driving_mode_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'driving_mode_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::DRIVING_MODE_MSG")
  (make-package "AEV_PKG::DRIVING_MODE_MSG"))

(in-package "ROS")
;;//! \htmlinclude driving_mode_msg.msg.html


(defclass aev_pkg::driving_mode_msg
  :super ros::object
  :slots (_msg_counter _drivingMode ))

(defmethod aev_pkg::driving_mode_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:drivingMode __drivingMode) 0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _drivingMode (round __drivingMode))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:drivingMode
   (&optional __drivingMode)
   (if __drivingMode (setq _drivingMode __drivingMode)) _drivingMode)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; uint8 _drivingMode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; uint8 _drivingMode
       (write-byte _drivingMode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _drivingMode
     (setq _drivingMode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get aev_pkg::driving_mode_msg :md5sum-) "a48a69dcaa1f71cf7c0fafc132da8148")
(setf (get aev_pkg::driving_mode_msg :datatype-) "aev_pkg/driving_mode_msg")
(setf (get aev_pkg::driving_mode_msg :definition-)
      "uint32 	msg_counter
uint8 	drivingMode
")



(provide :aev_pkg/driving_mode_msg "a48a69dcaa1f71cf7c0fafc132da8148")


