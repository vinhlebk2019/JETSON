;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::lane_detection_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'lane_detection_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::LANE_DETECTION_MSG")
  (make-package "AEV_PKG::LANE_DETECTION_MSG"))

(in-package "ROS")
;;//! \htmlinclude lane_detection_msg.msg.html


(defclass aev_pkg::lane_detection_msg
  :super ros::object
  :slots (_msg_counter _centerOffset _curvature ))

(defmethod aev_pkg::lane_detection_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:centerOffset __centerOffset) 0.0)
    ((:curvature __curvature) 0.0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _centerOffset (float __centerOffset))
   (setq _curvature (float __curvature))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:centerOffset
   (&optional __centerOffset)
   (if __centerOffset (setq _centerOffset __centerOffset)) _centerOffset)
  (:curvature
   (&optional __curvature)
   (if __curvature (setq _curvature __curvature)) _curvature)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; float32 _centerOffset
    4
    ;; float32 _curvature
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; float32 _centerOffset
       (sys::poke _centerOffset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _curvature
       (sys::poke _curvature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _centerOffset
     (setq _centerOffset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _curvature
     (setq _curvature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aev_pkg::lane_detection_msg :md5sum-) "4d45354acbb17a17b0a237f7d810945c")
(setf (get aev_pkg::lane_detection_msg :datatype-) "aev_pkg/lane_detection_msg")
(setf (get aev_pkg::lane_detection_msg :definition-)
      "uint32 	msg_counter
float32 centerOffset
float32 curvature

")



(provide :aev_pkg/lane_detection_msg "4d45354acbb17a17b0a237f7d810945c")


