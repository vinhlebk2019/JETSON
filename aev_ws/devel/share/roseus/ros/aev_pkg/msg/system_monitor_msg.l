;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::system_monitor_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'system_monitor_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::SYSTEM_MONITOR_MSG")
  (make-package "AEV_PKG::SYSTEM_MONITOR_MSG"))

(in-package "ROS")
;;//! \htmlinclude system_monitor_msg.msg.html


(defclass aev_pkg::system_monitor_msg
  :super ros::object
  :slots (_errorFlag _stopRequestFlag _errorInfo ))

(defmethod aev_pkg::system_monitor_msg
  (:init
   (&key
    ((:errorFlag __errorFlag) nil)
    ((:stopRequestFlag __stopRequestFlag) nil)
    ((:errorInfo __errorInfo) 0)
    )
   (send-super :init)
   (setq _errorFlag __errorFlag)
   (setq _stopRequestFlag __stopRequestFlag)
   (setq _errorInfo (round __errorInfo))
   self)
  (:errorFlag
   (&optional __errorFlag)
   (if __errorFlag (setq _errorFlag __errorFlag)) _errorFlag)
  (:stopRequestFlag
   (&optional __stopRequestFlag)
   (if __stopRequestFlag (setq _stopRequestFlag __stopRequestFlag)) _stopRequestFlag)
  (:errorInfo
   (&optional __errorInfo)
   (if __errorInfo (setq _errorInfo __errorInfo)) _errorInfo)
  (:serialization-length
   ()
   (+
    ;; bool _errorFlag
    1
    ;; bool _stopRequestFlag
    1
    ;; uint32 _errorInfo
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _errorFlag
       (if _errorFlag (write-byte -1 s) (write-byte 0 s))
     ;; bool _stopRequestFlag
       (if _stopRequestFlag (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _errorInfo
       (write-long _errorInfo s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _errorFlag
     (setq _errorFlag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stopRequestFlag
     (setq _stopRequestFlag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _errorInfo
     (setq _errorInfo (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aev_pkg::system_monitor_msg :md5sum-) "7fae4553a12c1b5cf670af37bb199a65")
(setf (get aev_pkg::system_monitor_msg :datatype-) "aev_pkg/system_monitor_msg")
(setf (get aev_pkg::system_monitor_msg :definition-)
      "bool 	errorFlag
bool 	stopRequestFlag
uint32 	errorInfo

")



(provide :aev_pkg/system_monitor_msg "7fae4553a12c1b5cf670af37bb199a65")


