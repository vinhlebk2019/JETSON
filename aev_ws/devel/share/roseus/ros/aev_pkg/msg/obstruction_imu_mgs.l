;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::obstruction_imu_mgs)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'obstruction_imu_mgs (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::OBSTRUCTION_IMU_MGS")
  (make-package "AEV_PKG::OBSTRUCTION_IMU_MGS"))

(in-package "ROS")
;;//! \htmlinclude obstruction_imu_mgs.msg.html


(defclass aev_pkg::obstruction_imu_mgs
  :super ros::object
  :slots (_msg_counter _obstruction _yaw_rate ))

(defmethod aev_pkg::obstruction_imu_mgs
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:obstruction __obstruction) nil)
    ((:yaw_rate __yaw_rate) 0.0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _obstruction __obstruction)
   (setq _yaw_rate (float __yaw_rate))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:obstruction
   (&optional __obstruction)
   (if __obstruction (setq _obstruction __obstruction)) _obstruction)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; bool _obstruction
    1
    ;; float64 _yaw_rate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; bool _obstruction
       (if _obstruction (write-byte -1 s) (write-byte 0 s))
     ;; float64 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _obstruction
     (setq _obstruction (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get aev_pkg::obstruction_imu_mgs :md5sum-) "0d4c64278182e7c8fb2d6201af193e53")
(setf (get aev_pkg::obstruction_imu_mgs :datatype-) "aev_pkg/obstruction_imu_mgs")
(setf (get aev_pkg::obstruction_imu_mgs :definition-)
      "uint32 	msg_counter
bool 	obstruction
float64 yaw_rate
")



(provide :aev_pkg/obstruction_imu_mgs "0d4c64278182e7c8fb2d6201af193e53")


