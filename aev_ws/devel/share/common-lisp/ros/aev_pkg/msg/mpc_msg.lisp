; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude mpc_msg.msg.html

(cl:defclass <mpc_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (SteeringAngle
    :reader SteeringAngle
    :initarg :SteeringAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass mpc_msg (<mpc_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<mpc_msg> is deprecated: use aev_pkg-msg:mpc_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <mpc_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'SteeringAngle-val :lambda-list '(m))
(cl:defmethod SteeringAngle-val ((m <mpc_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:SteeringAngle-val is deprecated.  Use aev_pkg-msg:SteeringAngle instead.")
  (SteeringAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_msg>) ostream)
  "Serializes a message object of type '<mpc_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'SteeringAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_msg>) istream)
  "Deserializes a message object of type '<mpc_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'SteeringAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_msg>)))
  "Returns string type for a message object of type '<mpc_msg>"
  "aev_pkg/mpc_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_msg)))
  "Returns string type for a message object of type 'mpc_msg"
  "aev_pkg/mpc_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_msg>)))
  "Returns md5sum for a message object of type '<mpc_msg>"
  "ab2da2dc4dc2caa0d3e85d831457bbdf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_msg)))
  "Returns md5sum for a message object of type 'mpc_msg"
  "ab2da2dc4dc2caa0d3e85d831457bbdf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_msg>)))
  "Returns full string definition for message of type '<mpc_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%float32 SteeringAngle ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_msg)))
  "Returns full string definition for message of type 'mpc_msg"
  (cl:format cl:nil "uint32 	msg_counter~%float32 SteeringAngle ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_msg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':SteeringAngle (SteeringAngle msg))
))
