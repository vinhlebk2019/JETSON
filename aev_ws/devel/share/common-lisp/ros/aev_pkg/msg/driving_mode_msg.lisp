; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude driving_mode_msg.msg.html

(cl:defclass <driving_mode_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (drivingMode
    :reader drivingMode
    :initarg :drivingMode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass driving_mode_msg (<driving_mode_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <driving_mode_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'driving_mode_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<driving_mode_msg> is deprecated: use aev_pkg-msg:driving_mode_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <driving_mode_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'drivingMode-val :lambda-list '(m))
(cl:defmethod drivingMode-val ((m <driving_mode_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:drivingMode-val is deprecated.  Use aev_pkg-msg:drivingMode instead.")
  (drivingMode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <driving_mode_msg>) ostream)
  "Serializes a message object of type '<driving_mode_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drivingMode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <driving_mode_msg>) istream)
  "Deserializes a message object of type '<driving_mode_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drivingMode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<driving_mode_msg>)))
  "Returns string type for a message object of type '<driving_mode_msg>"
  "aev_pkg/driving_mode_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'driving_mode_msg)))
  "Returns string type for a message object of type 'driving_mode_msg"
  "aev_pkg/driving_mode_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<driving_mode_msg>)))
  "Returns md5sum for a message object of type '<driving_mode_msg>"
  "a48a69dcaa1f71cf7c0fafc132da8148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'driving_mode_msg)))
  "Returns md5sum for a message object of type 'driving_mode_msg"
  "a48a69dcaa1f71cf7c0fafc132da8148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<driving_mode_msg>)))
  "Returns full string definition for message of type '<driving_mode_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%uint8 	drivingMode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'driving_mode_msg)))
  "Returns full string definition for message of type 'driving_mode_msg"
  (cl:format cl:nil "uint32 	msg_counter~%uint8 	drivingMode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <driving_mode_msg>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <driving_mode_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'driving_mode_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':drivingMode (drivingMode msg))
))
