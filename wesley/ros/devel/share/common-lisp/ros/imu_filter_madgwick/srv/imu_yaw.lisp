; Auto-generated. Do not edit!


(cl:in-package imu_filter_madgwick-srv)


;//! \htmlinclude imu_yaw-request.msg.html

(cl:defclass <imu_yaw-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass imu_yaw-request (<imu_yaw-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imu_yaw-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imu_yaw-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu_filter_madgwick-srv:<imu_yaw-request> is deprecated: use imu_filter_madgwick-srv:imu_yaw-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu_yaw-request>) ostream)
  "Serializes a message object of type '<imu_yaw-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu_yaw-request>) istream)
  "Deserializes a message object of type '<imu_yaw-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imu_yaw-request>)))
  "Returns string type for a service object of type '<imu_yaw-request>"
  "imu_filter_madgwick/imu_yawRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu_yaw-request)))
  "Returns string type for a service object of type 'imu_yaw-request"
  "imu_filter_madgwick/imu_yawRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imu_yaw-request>)))
  "Returns md5sum for a message object of type '<imu_yaw-request>"
  "08cb8274b6ddb17af5a842bca0b17db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu_yaw-request)))
  "Returns md5sum for a message object of type 'imu_yaw-request"
  "08cb8274b6ddb17af5a842bca0b17db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu_yaw-request>)))
  "Returns full string definition for message of type '<imu_yaw-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu_yaw-request)))
  "Returns full string definition for message of type 'imu_yaw-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu_yaw-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu_yaw-request>))
  "Converts a ROS message object to a list"
  (cl:list 'imu_yaw-request
))
;//! \htmlinclude imu_yaw-response.msg.html

(cl:defclass <imu_yaw-response> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass imu_yaw-response (<imu_yaw-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imu_yaw-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imu_yaw-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu_filter_madgwick-srv:<imu_yaw-response> is deprecated: use imu_filter_madgwick-srv:imu_yaw-response instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <imu_yaw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_filter_madgwick-srv:yaw-val is deprecated.  Use imu_filter_madgwick-srv:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu_yaw-response>) ostream)
  "Serializes a message object of type '<imu_yaw-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu_yaw-response>) istream)
  "Deserializes a message object of type '<imu_yaw-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imu_yaw-response>)))
  "Returns string type for a service object of type '<imu_yaw-response>"
  "imu_filter_madgwick/imu_yawResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu_yaw-response)))
  "Returns string type for a service object of type 'imu_yaw-response"
  "imu_filter_madgwick/imu_yawResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imu_yaw-response>)))
  "Returns md5sum for a message object of type '<imu_yaw-response>"
  "08cb8274b6ddb17af5a842bca0b17db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu_yaw-response)))
  "Returns md5sum for a message object of type 'imu_yaw-response"
  "08cb8274b6ddb17af5a842bca0b17db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu_yaw-response>)))
  "Returns full string definition for message of type '<imu_yaw-response>"
  (cl:format cl:nil "float64 yaw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu_yaw-response)))
  "Returns full string definition for message of type 'imu_yaw-response"
  (cl:format cl:nil "float64 yaw~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu_yaw-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu_yaw-response>))
  "Converts a ROS message object to a list"
  (cl:list 'imu_yaw-response
    (cl:cons ':yaw (yaw msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'imu_yaw)))
  'imu_yaw-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'imu_yaw)))
  'imu_yaw-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu_yaw)))
  "Returns string type for a service object of type '<imu_yaw>"
  "imu_filter_madgwick/imu_yaw")