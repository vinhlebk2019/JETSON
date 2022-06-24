;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::gui_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'gui_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::GUI_MSG")
  (make-package "AEV_PKG::GUI_MSG"))

(in-package "ROS")
;;//! \htmlinclude gui_msg.msg.html


(defclass aev_pkg::gui_msg
  :super ros::object
  :slots (_msg_counter _userReqStart _userReqAutoRun _userReqStop _clearError _speedSetpoint _turnSignal _horn _frontLight _steeringLeftRight ))

(defmethod aev_pkg::gui_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:userReqStart __userReqStart) nil)
    ((:userReqAutoRun __userReqAutoRun) nil)
    ((:userReqStop __userReqStop) nil)
    ((:clearError __clearError) nil)
    ((:speedSetpoint __speedSetpoint) 0)
    ((:turnSignal __turnSignal) 0)
    ((:horn __horn) nil)
    ((:frontLight __frontLight) nil)
    ((:steeringLeftRight __steeringLeftRight) 0)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _userReqStart __userReqStart)
   (setq _userReqAutoRun __userReqAutoRun)
   (setq _userReqStop __userReqStop)
   (setq _clearError __clearError)
   (setq _speedSetpoint (round __speedSetpoint))
   (setq _turnSignal (round __turnSignal))
   (setq _horn __horn)
   (setq _frontLight __frontLight)
   (setq _steeringLeftRight (round __steeringLeftRight))
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:userReqStart
   (&optional __userReqStart)
   (if __userReqStart (setq _userReqStart __userReqStart)) _userReqStart)
  (:userReqAutoRun
   (&optional __userReqAutoRun)
   (if __userReqAutoRun (setq _userReqAutoRun __userReqAutoRun)) _userReqAutoRun)
  (:userReqStop
   (&optional __userReqStop)
   (if __userReqStop (setq _userReqStop __userReqStop)) _userReqStop)
  (:clearError
   (&optional __clearError)
   (if __clearError (setq _clearError __clearError)) _clearError)
  (:speedSetpoint
   (&optional __speedSetpoint)
   (if __speedSetpoint (setq _speedSetpoint __speedSetpoint)) _speedSetpoint)
  (:turnSignal
   (&optional __turnSignal)
   (if __turnSignal (setq _turnSignal __turnSignal)) _turnSignal)
  (:horn
   (&optional __horn)
   (if __horn (setq _horn __horn)) _horn)
  (:frontLight
   (&optional __frontLight)
   (if __frontLight (setq _frontLight __frontLight)) _frontLight)
  (:steeringLeftRight
   (&optional __steeringLeftRight)
   (if __steeringLeftRight (setq _steeringLeftRight __steeringLeftRight)) _steeringLeftRight)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; bool _userReqStart
    1
    ;; bool _userReqAutoRun
    1
    ;; bool _userReqStop
    1
    ;; bool _clearError
    1
    ;; int16 _speedSetpoint
    2
    ;; uint8 _turnSignal
    1
    ;; bool _horn
    1
    ;; bool _frontLight
    1
    ;; uint8 _steeringLeftRight
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; bool _userReqStart
       (if _userReqStart (write-byte -1 s) (write-byte 0 s))
     ;; bool _userReqAutoRun
       (if _userReqAutoRun (write-byte -1 s) (write-byte 0 s))
     ;; bool _userReqStop
       (if _userReqStop (write-byte -1 s) (write-byte 0 s))
     ;; bool _clearError
       (if _clearError (write-byte -1 s) (write-byte 0 s))
     ;; int16 _speedSetpoint
       (write-word _speedSetpoint s)
     ;; uint8 _turnSignal
       (write-byte _turnSignal s)
     ;; bool _horn
       (if _horn (write-byte -1 s) (write-byte 0 s))
     ;; bool _frontLight
       (if _frontLight (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _steeringLeftRight
       (write-byte _steeringLeftRight s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _userReqStart
     (setq _userReqStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _userReqAutoRun
     (setq _userReqAutoRun (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _userReqStop
     (setq _userReqStop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clearError
     (setq _clearError (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _speedSetpoint
     (setq _speedSetpoint (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _turnSignal
     (setq _turnSignal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _horn
     (setq _horn (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _frontLight
     (setq _frontLight (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _steeringLeftRight
     (setq _steeringLeftRight (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get aev_pkg::gui_msg :md5sum-) "0288aa76c680cc5123702f434a849fe7")
(setf (get aev_pkg::gui_msg :datatype-) "aev_pkg/gui_msg")
(setf (get aev_pkg::gui_msg :definition-)
      "uint32 	msg_counter
bool 	userReqStart
bool 	userReqAutoRun
bool 	userReqStop
bool 	clearError
int16 	speedSetpoint
uint8 	turnSignal
bool 	horn
bool 	frontLight
uint8 steeringLeftRight
")



(provide :aev_pkg/gui_msg "0288aa76c680cc5123702f434a849fe7")


