; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude ecu_feedback_msg.msg.html

(cl:defclass <ecu_feedback_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (feedbackSpeed_b1
    :reader feedbackSpeed_b1
    :initarg :feedbackSpeed_b1
    :type cl:fixnum
    :initform 0)
   (feedbackSpeed_b2
    :reader feedbackSpeed_b2
    :initarg :feedbackSpeed_b2
    :type cl:fixnum
    :initform 0)
   (feedbackSpeed_b3
    :reader feedbackSpeed_b3
    :initarg :feedbackSpeed_b3
    :type cl:fixnum
    :initform 0)
   (feedbackSpeed_b4
    :reader feedbackSpeed_b4
    :initarg :feedbackSpeed_b4
    :type cl:fixnum
    :initform 0)
   (acceleratorLevel
    :reader acceleratorLevel
    :initarg :acceleratorLevel
    :type cl:fixnum
    :initform 0)
   (acceleratorSwitch
    :reader acceleratorSwitch
    :initarg :acceleratorSwitch
    :type cl:boolean
    :initform cl:nil)
   (brakeSwitch
    :reader brakeSwitch
    :initarg :brakeSwitch
    :type cl:boolean
    :initform cl:nil)
   (movingDirection
    :reader movingDirection
    :initarg :movingDirection
    :type cl:boolean
    :initform cl:nil)
   (turnSignal
    :reader turnSignal
    :initarg :turnSignal
    :type cl:fixnum
    :initform 0)
   (horn
    :reader horn
    :initarg :horn
    :type cl:boolean
    :initform cl:nil)
   (frontLight
    :reader frontLight
    :initarg :frontLight
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ecu_feedback_msg (<ecu_feedback_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ecu_feedback_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ecu_feedback_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<ecu_feedback_msg> is deprecated: use aev_pkg-msg:ecu_feedback_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'feedbackSpeed_b1-val :lambda-list '(m))
(cl:defmethod feedbackSpeed_b1-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:feedbackSpeed_b1-val is deprecated.  Use aev_pkg-msg:feedbackSpeed_b1 instead.")
  (feedbackSpeed_b1 m))

(cl:ensure-generic-function 'feedbackSpeed_b2-val :lambda-list '(m))
(cl:defmethod feedbackSpeed_b2-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:feedbackSpeed_b2-val is deprecated.  Use aev_pkg-msg:feedbackSpeed_b2 instead.")
  (feedbackSpeed_b2 m))

(cl:ensure-generic-function 'feedbackSpeed_b3-val :lambda-list '(m))
(cl:defmethod feedbackSpeed_b3-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:feedbackSpeed_b3-val is deprecated.  Use aev_pkg-msg:feedbackSpeed_b3 instead.")
  (feedbackSpeed_b3 m))

(cl:ensure-generic-function 'feedbackSpeed_b4-val :lambda-list '(m))
(cl:defmethod feedbackSpeed_b4-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:feedbackSpeed_b4-val is deprecated.  Use aev_pkg-msg:feedbackSpeed_b4 instead.")
  (feedbackSpeed_b4 m))

(cl:ensure-generic-function 'acceleratorLevel-val :lambda-list '(m))
(cl:defmethod acceleratorLevel-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:acceleratorLevel-val is deprecated.  Use aev_pkg-msg:acceleratorLevel instead.")
  (acceleratorLevel m))

(cl:ensure-generic-function 'acceleratorSwitch-val :lambda-list '(m))
(cl:defmethod acceleratorSwitch-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:acceleratorSwitch-val is deprecated.  Use aev_pkg-msg:acceleratorSwitch instead.")
  (acceleratorSwitch m))

(cl:ensure-generic-function 'brakeSwitch-val :lambda-list '(m))
(cl:defmethod brakeSwitch-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:brakeSwitch-val is deprecated.  Use aev_pkg-msg:brakeSwitch instead.")
  (brakeSwitch m))

(cl:ensure-generic-function 'movingDirection-val :lambda-list '(m))
(cl:defmethod movingDirection-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:movingDirection-val is deprecated.  Use aev_pkg-msg:movingDirection instead.")
  (movingDirection m))

(cl:ensure-generic-function 'turnSignal-val :lambda-list '(m))
(cl:defmethod turnSignal-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:turnSignal-val is deprecated.  Use aev_pkg-msg:turnSignal instead.")
  (turnSignal m))

(cl:ensure-generic-function 'horn-val :lambda-list '(m))
(cl:defmethod horn-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:horn-val is deprecated.  Use aev_pkg-msg:horn instead.")
  (horn m))

(cl:ensure-generic-function 'frontLight-val :lambda-list '(m))
(cl:defmethod frontLight-val ((m <ecu_feedback_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:frontLight-val is deprecated.  Use aev_pkg-msg:frontLight instead.")
  (frontLight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ecu_feedback_msg>) ostream)
  "Serializes a message object of type '<ecu_feedback_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acceleratorLevel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acceleratorSwitch) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brakeSwitch) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'movingDirection) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'turnSignal)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'horn) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'frontLight) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ecu_feedback_msg>) istream)
  "Deserializes a message object of type '<ecu_feedback_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedbackSpeed_b4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'acceleratorLevel)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleratorSwitch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'brakeSwitch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'movingDirection) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'turnSignal)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horn) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'frontLight) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ecu_feedback_msg>)))
  "Returns string type for a message object of type '<ecu_feedback_msg>"
  "aev_pkg/ecu_feedback_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ecu_feedback_msg)))
  "Returns string type for a message object of type 'ecu_feedback_msg"
  "aev_pkg/ecu_feedback_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ecu_feedback_msg>)))
  "Returns md5sum for a message object of type '<ecu_feedback_msg>"
  "4501f6c5918ccf5a041a7524a20b1561")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ecu_feedback_msg)))
  "Returns md5sum for a message object of type 'ecu_feedback_msg"
  "4501f6c5918ccf5a041a7524a20b1561")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ecu_feedback_msg>)))
  "Returns full string definition for message of type '<ecu_feedback_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%uint8 feedbackSpeed_b1~%uint8 feedbackSpeed_b2~%uint8 feedbackSpeed_b3 ~%uint8 feedbackSpeed_b4 ~%uint8 	acceleratorLevel~%bool acceleratorSwitch~%bool brakeSwitch~%bool movingDirection~%uint8 	turnSignal~%bool 	horn~%bool 	frontLight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ecu_feedback_msg)))
  "Returns full string definition for message of type 'ecu_feedback_msg"
  (cl:format cl:nil "uint32 	msg_counter~%uint8 feedbackSpeed_b1~%uint8 feedbackSpeed_b2~%uint8 feedbackSpeed_b3 ~%uint8 feedbackSpeed_b4 ~%uint8 	acceleratorLevel~%bool acceleratorSwitch~%bool brakeSwitch~%bool movingDirection~%uint8 	turnSignal~%bool 	horn~%bool 	frontLight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ecu_feedback_msg>))
  (cl:+ 0
     4
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ecu_feedback_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'ecu_feedback_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':feedbackSpeed_b1 (feedbackSpeed_b1 msg))
    (cl:cons ':feedbackSpeed_b2 (feedbackSpeed_b2 msg))
    (cl:cons ':feedbackSpeed_b3 (feedbackSpeed_b3 msg))
    (cl:cons ':feedbackSpeed_b4 (feedbackSpeed_b4 msg))
    (cl:cons ':acceleratorLevel (acceleratorLevel msg))
    (cl:cons ':acceleratorSwitch (acceleratorSwitch msg))
    (cl:cons ':brakeSwitch (brakeSwitch msg))
    (cl:cons ':movingDirection (movingDirection msg))
    (cl:cons ':turnSignal (turnSignal msg))
    (cl:cons ':horn (horn msg))
    (cl:cons ':frontLight (frontLight msg))
))
