;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::object_detection_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'object_detection_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::OBJECT_DETECTION_MSG")
  (make-package "AEV_PKG::OBJECT_DETECTION_MSG"))

(in-package "ROS")
;;//! \htmlinclude object_detection_msg.msg.html


(defclass aev_pkg::object_detection_msg
  :super ros::object
  :slots (_msg_counter _isObject _yaw_rate ))

(defmethod aev_pkg::object_detection_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:isObject __isObject) nil)
    ((:yaw_rate __yaw_rate) 0.0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _isObject __isObject)
   (setq _yaw_rate (float __yaw_rate))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:isObject
   (&optional __isObject)
   (if __isObject (setq _isObject __isObject)) _isObject)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; bool _isObject
    1
    ;; float32 _yaw_rate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; bool _isObject
       (if _isObject (write-byte -1 s) (write-byte 0 s))
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _isObject
     (setq _isObject (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aev_pkg::object_detection_msg :md5sum-) "ec64f2e8b1cf92ea48d7fae640f274fb")
(setf (get aev_pkg::object_detection_msg :datatype-) "aev_pkg/object_detection_msg")
(setf (get aev_pkg::object_detection_msg :definition-)
      "uint32 	msg_counter
bool 	isObject
float32 yaw_rate

")



(provide :aev_pkg/object_detection_msg "ec64f2e8b1cf92ea48d7fae640f274fb")


