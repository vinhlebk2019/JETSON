; Auto-generated. Do not edit!


(cl:in-package aev_pkg-msg)


;//! \htmlinclude lane_detection_msg.msg.html

(cl:defclass <lane_detection_msg> (roslisp-msg-protocol:ros-message)
  ((msg_counter
    :reader msg_counter
    :initarg :msg_counter
    :type cl:integer
    :initform 0)
   (centerOffset
    :reader centerOffset
    :initarg :centerOffset
    :type cl:float
    :initform 0.0)
   (curvature
    :reader curvature
    :initarg :curvature
    :type cl:float
    :initform 0.0))
)

(cl:defclass lane_detection_msg (<lane_detection_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lane_detection_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lane_detection_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aev_pkg-msg:<lane_detection_msg> is deprecated: use aev_pkg-msg:lane_detection_msg instead.")))

(cl:ensure-generic-function 'msg_counter-val :lambda-list '(m))
(cl:defmethod msg_counter-val ((m <lane_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:msg_counter-val is deprecated.  Use aev_pkg-msg:msg_counter instead.")
  (msg_counter m))

(cl:ensure-generic-function 'centerOffset-val :lambda-list '(m))
(cl:defmethod centerOffset-val ((m <lane_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:centerOffset-val is deprecated.  Use aev_pkg-msg:centerOffset instead.")
  (centerOffset m))

(cl:ensure-generic-function 'curvature-val :lambda-list '(m))
(cl:defmethod curvature-val ((m <lane_detection_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aev_pkg-msg:curvature-val is deprecated.  Use aev_pkg-msg:curvature instead.")
  (curvature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lane_detection_msg>) ostream)
  "Serializes a message object of type '<lane_detection_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centerOffset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'curvature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lane_detection_msg>) istream)
  "Deserializes a message object of type '<lane_detection_msg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg_counter)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centerOffset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curvature) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lane_detection_msg>)))
  "Returns string type for a message object of type '<lane_detection_msg>"
  "aev_pkg/lane_detection_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lane_detection_msg)))
  "Returns string type for a message object of type 'lane_detection_msg"
  "aev_pkg/lane_detection_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lane_detection_msg>)))
  "Returns md5sum for a message object of type '<lane_detection_msg>"
  "4d45354acbb17a17b0a237f7d810945c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lane_detection_msg)))
  "Returns md5sum for a message object of type 'lane_detection_msg"
  "4d45354acbb17a17b0a237f7d810945c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lane_detection_msg>)))
  "Returns full string definition for message of type '<lane_detection_msg>"
  (cl:format cl:nil "uint32 	msg_counter~%float32 centerOffset~%float32 curvature~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lane_detection_msg)))
  "Returns full string definition for message of type 'lane_detection_msg"
  (cl:format cl:nil "uint32 	msg_counter~%float32 centerOffset~%float32 curvature~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lane_detection_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lane_detection_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'lane_detection_msg
    (cl:cons ':msg_counter (msg_counter msg))
    (cl:cons ':centerOffset (centerOffset msg))
    (cl:cons ':curvature (curvature msg))
))
