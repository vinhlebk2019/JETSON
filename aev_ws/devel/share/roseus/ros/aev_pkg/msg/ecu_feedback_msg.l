;; Auto-generated. Do not edit!


(when (boundp 'aev_pkg::ecu_feedback_msg)
  (if (not (find-package "AEV_PKG"))
    (make-package "AEV_PKG"))
  (shadow 'ecu_feedback_msg (find-package "AEV_PKG")))
(unless (find-package "AEV_PKG::ECU_FEEDBACK_MSG")
  (make-package "AEV_PKG::ECU_FEEDBACK_MSG"))

(in-package "ROS")
;;//! \htmlinclude ecu_feedback_msg.msg.html


(defclass aev_pkg::ecu_feedback_msg
  :super ros::object
  :slots (_msg_counter _feedbackSpeed_b1 _feedbackSpeed_b2 _feedbackSpeed_b3 _feedbackSpeed_b4 _acceleratorLevel _acceleratorSwitch _brakeSwitch _movingDirection _turnSignal _horn _frontLight ))

(defmethod aev_pkg::ecu_feedback_msg
  (:init
   (&key
    ((:msg_counter __msg_counter) 0)
    ((:feedbackSpeed_b1 __feedbackSpeed_b1) 0)
    ((:feedbackSpeed_b2 __feedbackSpeed_b2) 0)
    ((:feedbackSpeed_b3 __feedbackSpeed_b3) 0)
    ((:feedbackSpeed_b4 __feedbackSpeed_b4) 0)
    ((:acceleratorLevel __acceleratorLevel) 0)
    ((:acceleratorSwitch __acceleratorSwitch) nil)
    ((:brakeSwitch __brakeSwitch) nil)
    ((:movingDirection __movingDirection) nil)
    ((:turnSignal __turnSignal) 0)
    ((:horn __horn) nil)
    ((:frontLight __frontLight) nil)
    )
   (send-super :init)
   (setq _msg_counter (round __msg_counter))
   (setq _feedbackSpeed_b1 (round __feedbackSpeed_b1))
   (setq _feedbackSpeed_b2 (round __feedbackSpeed_b2))
   (setq _feedbackSpeed_b3 (round __feedbackSpeed_b3))
   (setq _feedbackSpeed_b4 (round __feedbackSpeed_b4))
   (setq _acceleratorLevel (round __acceleratorLevel))
   (setq _acceleratorSwitch __acceleratorSwitch)
   (setq _brakeSwitch __brakeSwitch)
   (setq _movingDirection __movingDirection)
   (setq _turnSignal (round __turnSignal))
   (setq _horn __horn)
   (setq _frontLight __frontLight)
   self)
  (:msg_counter
   (&optional __msg_counter)
   (if __msg_counter (setq _msg_counter __msg_counter)) _msg_counter)
  (:feedbackSpeed_b1
   (&optional __feedbackSpeed_b1)
   (if __feedbackSpeed_b1 (setq _feedbackSpeed_b1 __feedbackSpeed_b1)) _feedbackSpeed_b1)
  (:feedbackSpeed_b2
   (&optional __feedbackSpeed_b2)
   (if __feedbackSpeed_b2 (setq _feedbackSpeed_b2 __feedbackSpeed_b2)) _feedbackSpeed_b2)
  (:feedbackSpeed_b3
   (&optional __feedbackSpeed_b3)
   (if __feedbackSpeed_b3 (setq _feedbackSpeed_b3 __feedbackSpeed_b3)) _feedbackSpeed_b3)
  (:feedbackSpeed_b4
   (&optional __feedbackSpeed_b4)
   (if __feedbackSpeed_b4 (setq _feedbackSpeed_b4 __feedbackSpeed_b4)) _feedbackSpeed_b4)
  (:acceleratorLevel
   (&optional __acceleratorLevel)
   (if __acceleratorLevel (setq _acceleratorLevel __acceleratorLevel)) _acceleratorLevel)
  (:acceleratorSwitch
   (&optional __acceleratorSwitch)
   (if __acceleratorSwitch (setq _acceleratorSwitch __acceleratorSwitch)) _acceleratorSwitch)
  (:brakeSwitch
   (&optional __brakeSwitch)
   (if __brakeSwitch (setq _brakeSwitch __brakeSwitch)) _brakeSwitch)
  (:movingDirection
   (&optional __movingDirection)
   (if __movingDirection (setq _movingDirection __movingDirection)) _movingDirection)
  (:turnSignal
   (&optional __turnSignal)
   (if __turnSignal (setq _turnSignal __turnSignal)) _turnSignal)
  (:horn
   (&optional __horn)
   (if __horn (setq _horn __horn)) _horn)
  (:frontLight
   (&optional __frontLight)
   (if __frontLight (setq _frontLight __frontLight)) _frontLight)
  (:serialization-length
   ()
   (+
    ;; uint32 _msg_counter
    4
    ;; uint8 _feedbackSpeed_b1
    1
    ;; uint8 _feedbackSpeed_b2
    1
    ;; uint8 _feedbackSpeed_b3
    1
    ;; uint8 _feedbackSpeed_b4
    1
    ;; uint8 _acceleratorLevel
    1
    ;; bool _acceleratorSwitch
    1
    ;; bool _brakeSwitch
    1
    ;; bool _movingDirection
    1
    ;; uint8 _turnSignal
    1
    ;; bool _horn
    1
    ;; bool _frontLight
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _msg_counter
       (write-long _msg_counter s)
     ;; uint8 _feedbackSpeed_b1
       (write-byte _feedbackSpeed_b1 s)
     ;; uint8 _feedbackSpeed_b2
       (write-byte _feedbackSpeed_b2 s)
     ;; uint8 _feedbackSpeed_b3
       (write-byte _feedbackSpeed_b3 s)
     ;; uint8 _feedbackSpeed_b4
       (write-byte _feedbackSpeed_b4 s)
     ;; uint8 _acceleratorLevel
       (write-byte _acceleratorLevel s)
     ;; bool _acceleratorSwitch
       (if _acceleratorSwitch (write-byte -1 s) (write-byte 0 s))
     ;; bool _brakeSwitch
       (if _brakeSwitch (write-byte -1 s) (write-byte 0 s))
     ;; bool _movingDirection
       (if _movingDirection (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _turnSignal
       (write-byte _turnSignal s)
     ;; bool _horn
       (if _horn (write-byte -1 s) (write-byte 0 s))
     ;; bool _frontLight
       (if _frontLight (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _msg_counter
     (setq _msg_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _feedbackSpeed_b1
     (setq _feedbackSpeed_b1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _feedbackSpeed_b2
     (setq _feedbackSpeed_b2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _feedbackSpeed_b3
     (setq _feedbackSpeed_b3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _feedbackSpeed_b4
     (setq _feedbackSpeed_b4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _acceleratorLevel
     (setq _acceleratorLevel (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _acceleratorSwitch
     (setq _acceleratorSwitch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _brakeSwitch
     (setq _brakeSwitch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _movingDirection
     (setq _movingDirection (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _turnSignal
     (setq _turnSignal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _horn
     (setq _horn (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _frontLight
     (setq _frontLight (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get aev_pkg::ecu_feedback_msg :md5sum-) "4501f6c5918ccf5a041a7524a20b1561")
(setf (get aev_pkg::ecu_feedback_msg :datatype-) "aev_pkg/ecu_feedback_msg")
(setf (get aev_pkg::ecu_feedback_msg :definition-)
      "uint32 	msg_counter
uint8 feedbackSpeed_b1
uint8 feedbackSpeed_b2
uint8 feedbackSpeed_b3 
uint8 feedbackSpeed_b4 
uint8 	acceleratorLevel
bool acceleratorSwitch
bool brakeSwitch
bool movingDirection
uint8 	turnSignal
bool 	horn
bool 	frontLight

")



(provide :aev_pkg/ecu_feedback_msg "4501f6c5918ccf5a041a7524a20b1561")


