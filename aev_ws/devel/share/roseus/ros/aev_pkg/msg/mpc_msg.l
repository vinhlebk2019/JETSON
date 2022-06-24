;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::mpc_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'mpc_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::MPC_MSG")
  (make-package "AEV_PKG::MPC_MSG"))

(in-package "ROS")
;;//! \htmlinclude mpc_msg.msg.html


(defclass aev_pkg::mpc_msg
  :super ros::object
  :slots (_msg_counter _SteeringAngle ))

(defmethod aev_pkg::mpc_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:SteeringAngle __SteeringAngle) 0.0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _SteeringAngle (float __SteeringAngle))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:SteeringAngle
   (&optional __SteeringAngle)
   (if __SteeringAngle (setq _SteeringAngle __SteeringAngle)) _SteeringAngle)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; float32 _SteeringAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; float32 _SteeringAngle
       (sys::poke _SteeringAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _SteeringAngle
     (setq _SteeringAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aev_pkg::mpc_msg :md5sum-) "ab2da2dc4dc2caa0d3e85d831457bbdf")
(setf (get aev_pkg::mpc_msg :datatype-) "aev_pkg/mpc_msg")
(setf (get aev_pkg::mpc_msg :definition-)
      "uint32 	msg_counter
float32 SteeringAngle 

")



(provide :aev_pkg/mpc_msg "ab2da2dc4dc2caa0d3e85d831457bbdf")


