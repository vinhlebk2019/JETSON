; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude object_detection_msg.msg.html

(cl:defclass <object_detection_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (isObject
    :reader isObject
    :initarg :isObject
    :type cl:boolean
    :initform cl:nil)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass object_detection_msg (<object_detection_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_detection_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_detection_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<object_detection_msg> is deprecated: use aev_pkg-msg:object_detection_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <object_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'isObject-val :lambda-list '(m))
(cl:defmethod isObject-val ((m <object_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:isObject-val is deprecated.  Use aev_pkg-msg:isObject instead.")
  (isObject m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <object_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:yaw_rate-val is deprecated.  Use aev_pkg-msg:yaw_rate instead.")
  (yaw_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_detection_msg>) ostream)
  "Serializes a message object of type '<object_detection_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isObject) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_detection_msg>) istream)
  "Deserializes a message object of type '<object_detection_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'isObject) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_detection_msg>)))
  "Returns string type for a message object of type '<object_detection_msg>"
  "aev_pkg/object_detection_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_detection_msg)))
  "Returns string type for a message object of type 'object_detection_msg"
  "aev_pkg/object_detection_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_detection_msg>)))
  "Returns md5sum for a message object of type '<object_detection_msg>"
  "ec64f2e8b1cf92ea48d7fae640f274fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_detection_msg)))
  "Returns md5sum for a message object of type 'object_detection_msg"
  "ec64f2e8b1cf92ea48d7fae640f274fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_detection_msg>)))
  "Returns full string definition for message of type '<object_detection_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	isObject~%float32 yaw_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_detection_msg)))
  "Returns full string definition for message of type 'object_detection_msg"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	isObject~%float32 yaw_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_detection_msg>))
  (cl:+ 0
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_detection_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'object_detection_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':isObject (isObject msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
))
