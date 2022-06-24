; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude obstruction_imu_mgs.msg.html

(cl:defclass <obstruction_imu_mgs> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (obstruction
    :reader obstruction
    :initarg :obstruction
    :type cl:boolean
    :initform cl:nil)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass obstruction_imu_mgs (<obstruction_imu_mgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstruction_imu_mgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstruction_imu_mgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<obstruction_imu_mgs> is deprecated: use aev_pkg-msg:obstruction_imu_mgs instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <obstruction_imu_mgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'obstruction-val :lambda-list '(m))
(cl:defmethod obstruction-val ((m <obstruction_imu_mgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:obstruction-val is deprecated.  Use aev_pkg-msg:obstruction instead.")
  (obstruction m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <obstruction_imu_mgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:yaw_rate-val is deprecated.  Use aev_pkg-msg:yaw_rate instead.")
  (yaw_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstruction_imu_mgs>) ostream)
  "Serializes a message object of type '<obstruction_imu_mgs>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'obstruction) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstruction_imu_mgs>) istream)
  "Deserializes a message object of type '<obstruction_imu_mgs>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstruction) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstruction_imu_mgs>)))
  "Returns string type for a message object of type '<obstruction_imu_mgs>"
  "aev_pkg/obstruction_imu_mgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstruction_imu_mgs)))
  "Returns string type for a message object of type 'obstruction_imu_mgs"
  "aev_pkg/obstruction_imu_mgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstruction_imu_mgs>)))
  "Returns md5sum for a message object of type '<obstruction_imu_mgs>"
  "0d4c64278182e7c8fb2d6201af193e53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstruction_imu_mgs)))
  "Returns md5sum for a message object of type 'obstruction_imu_mgs"
  "0d4c64278182e7c8fb2d6201af193e53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstruction_imu_mgs>)))
  "Returns full string definition for message of type '<obstruction_imu_mgs>"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	obstruction~%float64 yaw_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstruction_imu_mgs)))
  "Returns full string definition for message of type 'obstruction_imu_mgs"
  (cl:format cl:nil "uint32 	msg_counter~%bool 	obstruction~%float64 yaw_rate~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstruction_imu_mgs>))
  (cl:+ 0
     4
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstruction_imu_mgs>))
  "Converts a ROS message object to a list"
  (cl:list 'obstruction_imu_mgs
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':obstruction (obstruction msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
))
