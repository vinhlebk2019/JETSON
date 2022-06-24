; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude gui_msg.msg.html

(cl:defclass <gui_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (userReqStart
    :reader userReqStart
    :initarg :userReqStart
    :type cl:boolean
    :initform cl:nil)
   (userReqAutoRun
    :reader userReqAutoRun
    :initarg :userReqAutoRun
    :type cl:boolean
    :initform cl:nil)
   (userReqStop
    :reader userReqStop
    :initarg :userReqStop
    :type cl:boolean
    :initform cl:nil)
   (clearError
    :reader clearError
    :initarg :clearError
    :type cl:boolean
    :initform cl:nil)
   (speedSetpoint
    :reader speedSetpoint
    :initarg :speedSetpoint
    :type cl:fixnum
    :initform 0)
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
    :initform cl:nil)
   (steeringLeftRight
    :reader steeringLeftRight
    :initarg :steeringLeftRight
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gui_msg (<gui_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gui_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gui_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<gui_msg> is deprecated: use aev_pkg-msg:gui_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'userReqStart-val :lambda-list '(m))
(cl:defmethod userReqStart-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:userReqStart-val is deprecated.  Use aev_pkg-msg:userReqStart instead.")
  (userReqStart m))

(cl:ensure-generic-function 'userReqAutoRun-val :lambda-list '(m))
(cl:defmethod userReqAutoRun-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:userReqAutoRun-val is deprecated.  Use aev_pkg-msg:userReqAutoRun instead.")
  (userReqAutoRun m))

(cl:ensure-generic-function 'userReqStop-val :lambda-list '(m))
(cl:defmethod userReqStop-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:userReqStop-val is deprecated.  Use aev_pkg-msg:userReqStop instead.")
  (userReqStop m))

(cl:ensure-generic-function 'clearError-val :lambda-list '(m))
(cl:defmethod clearError-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:clearError-val is deprecated.  Use aev_pkg-msg:clearError instead.")
  (clearError m))

(cl:ensure-generic-function 'speedSetpoint-val :lambda-list '(m))
(cl:defmethod speedSetpoint-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:speedSetpoint-val is deprecated.  Use aev_pkg-msg:speedSetpoint instead.")
  (speedSetpoint m))

(cl:ensure-generic-function 'turnSignal-val :lambda-list '(m))
(cl:defmethod turnSignal-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:turnSignal-val is deprecated.  Use aev_pkg-msg:turnSignal instead.")
  (turnSignal m))

(cl:ensure-generic-function 'horn-val :lambda-list '(m))
(cl:defmethod horn-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:horn-val is deprecated.  Use aev_pkg-msg:horn instead.")
  (horn m))

(cl:ensure-generic-function 'frontLight-val :lambda-list '(m))
(cl:defmethod frontLight-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:frontLight-val is deprecated.  Use aev_pkg-msg:frontLight instead.")
  (frontLight m))

(cl:ensure-generic-function 'steeringLeftRight-val :lambda-list '(m))
(cl:defmethod steeringLeftRight-val ((m <gui_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:steeringLeftRight-val is deprecated.  Use aev_pkg-msg:steeringLeftRight instead.")
  (steeringLeftRight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gui_msg>) ostream)
  "Serializes a message object of type '<gui_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'userReqStart) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'userReqAutoRun) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'userReqStop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clearError) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'speedSetpoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'turnSignal)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'horn) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'frontLight) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steeringLeftRight)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gui_msg>) istream)
  "Deserializes a message object of type '<gui_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'userReqStart) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'userReqAutoRun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'userReqStop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clearError) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speedSetpoint) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'turnSignal)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horn) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'frontLight) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steeringLeftRight)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gui_msg>)))
  "Returns string type for a message object of type '<gui_msg>"
  "aev_pkg/gui_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gui_msg)))
  "Returns string type for a message object of type 'gui_msg"
  "aev_pkg/gui_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gui_msg>)))
  "Returns md5sum for a message object of type '<gui_msg>"
  "0288aa76c680cc5123702f434a849fe7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gui_msg)))
  "Returns md5sum for a message object of type 'gui_msg"
  "0288aa76c680cc5123702f434a849fe7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gui_msg>)))
  "Returns full string definition for message of type '<gui_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	userReqStart~%bool 	userReqAutoRun~%bool 	userReqStop~%bool 	clearError~%int16 	speedSetpoint~%uint8 	turnSignal~%bool 	horn~%bool 	frontLight~%uint8 steeringLeftRight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gui_msg)))
  "Returns full string definition for message of type 'gui_msg"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	userReqStart~%bool 	userReqAutoRun~%bool 	userReqStop~%bool 	clearError~%int16 	speedSetpoint~%uint8 	turnSignal~%bool 	horn~%bool 	frontLight~%uint8 steeringLeftRight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gui_msg>))
  (cl:+ 0
     4
     1
     1
     1
     1
     2
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gui_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'gui_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':userReqStart (userReqStart msg))
    (cl:cons ':userReqAutoRun (userReqAutoRun msg))
    (cl:cons ':userReqStop (userReqStop msg))
    (cl:cons ':clearError (clearError msg))
    (cl:cons ':speedSetpoint (speedSetpoint msg))
    (cl:cons ':turnSignal (turnSignal msg))
    (cl:cons ':horn (horn msg))
    (cl:cons ':frontLight (frontLight msg))
    (cl:cons ':steeringLeftRight (steeringLeftRight msg))
))
