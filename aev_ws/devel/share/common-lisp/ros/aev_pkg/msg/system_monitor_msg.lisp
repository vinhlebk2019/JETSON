; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude system_monitor_msg.msg.html

(cl:defclass <system_monitor_msg> (roslisp-msg-protocol:ros-message)
  ((errorFlag
    :reader errorFlag
    :initarg :errorFlag
    :type cl:boolean
    :initform cl:nil)
   (stopRequestFlag
    :reader stopRequestFlag
    :initarg :stopRequestFlag
    :type cl:boolean
    :initform cl:nil)
   (errorInfo
    :reader errorInfo
    :initarg :errorInfo
    :type cl:integer
    :initform 0))
)

(cl:defclass system_monitor_msg (<system_monitor_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <system_monitor_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'system_monitor_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<system_monitor_msg> is deprecated: use aev_pkg-msg:system_monitor_msg instead.")))

(cl:ensure-generic-function 'errorFlag-val :lambda-list '(m))
(cl:defmethod errorFlag-val ((m <system_monitor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:errorFlag-val is deprecated.  Use aev_pkg-msg:errorFlag instead.")
  (errorFlag m))

(cl:ensure-generic-function 'stopRequestFlag-val :lambda-list '(m))
(cl:defmethod stopRequestFlag-val ((m <system_monitor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:stopRequestFlag-val is deprecated.  Use aev_pkg-msg:stopRequestFlag instead.")
  (stopRequestFlag m))

(cl:ensure-generic-function 'errorInfo-val :lambda-list '(m))
(cl:defmethod errorInfo-val ((m <system_monitor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:errorInfo-val is deprecated.  Use aev_pkg-msg:errorInfo instead.")
  (errorInfo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <system_monitor_msg>) ostream)
  "Serializes a message object of type '<system_monitor_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'errorFlag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stopRequestFlag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'errorInfo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'errorInfo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'errorInfo)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'errorInfo)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <system_monitor_msg>) istream)
  "Deserializes a message object of type '<system_monitor_msg>"
    (cl:setf (cl:slot-value msg 'errorFlag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stopRequestFlag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'errorInfo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'errorInfo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'errorInfo)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'errorInfo)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<system_monitor_msg>)))
  "Returns string type for a message object of type '<system_monitor_msg>"
  "aev_pkg/system_monitor_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'system_monitor_msg)))
  "Returns string type for a message object of type 'system_monitor_msg"
  "aev_pkg/system_monitor_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<system_monitor_msg>)))
  "Returns md5sum for a message object of type '<system_monitor_msg>"
  "7fae4553a12c1b5cf670af37bb199a65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'system_monitor_msg)))
  "Returns md5sum for a message object of type 'system_monitor_msg"
  "7fae4553a12c1b5cf670af37bb199a65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<system_monitor_msg>)))
  "Returns full string definition for message of type '<system_monitor_msg>"
  (cl:format cl:nil "bool 	errorFlag~%bool 	stopRequestFlag~%uint32 	errorInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'system_monitor_msg)))
  "Returns full string definition for message of type 'system_monitor_msg"
  (cl:format cl:nil "bool 	errorFlag~%bool 	stopRequestFlag~%uint32 	errorInfo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <system_monitor_msg>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <system_monitor_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'system_monitor_msg
    (cl:cons ':errorFlag (errorFlag msg))
    (cl:cons ':stopRequestFlag (stopRequestFlag msg))
    (cl:cons ':errorInfo (errorInfo msg))
))
