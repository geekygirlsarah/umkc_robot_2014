; Auto-generated. Do not edit!


(cl:in-package wesley-msg)


;//! \htmlinclude arm_point.msg.html

(cl:defclass <arm_point> (roslisp-msg-protocol:ros-message)
  ((direct_mode
    :reader direct_mode
    :initarg :direct_mode
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (p
    :reader p
    :initarg :p
    :type cl:float
    :initform 0.0)
   (r
    :reader r
    :initarg :r
    :type cl:float
    :initform 0.0)
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform ""))
)

(cl:defclass arm_point (<arm_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arm_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arm_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wesley-msg:<arm_point> is deprecated: use wesley-msg:arm_point instead.")))

(cl:ensure-generic-function 'direct_mode-val :lambda-list '(m))
(cl:defmethod direct_mode-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:direct_mode-val is deprecated.  Use wesley-msg:direct_mode instead.")
  (direct_mode m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:x-val is deprecated.  Use wesley-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:y-val is deprecated.  Use wesley-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:z-val is deprecated.  Use wesley-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:p-val is deprecated.  Use wesley-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:r-val is deprecated.  Use wesley-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <arm_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:cmd-val is deprecated.  Use wesley-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arm_point>) ostream)
  "Serializes a message object of type '<arm_point>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'direct_mode) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arm_point>) istream)
  "Deserializes a message object of type '<arm_point>"
    (cl:setf (cl:slot-value msg 'direct_mode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arm_point>)))
  "Returns string type for a message object of type '<arm_point>"
  "wesley/arm_point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arm_point)))
  "Returns string type for a message object of type 'arm_point"
  "wesley/arm_point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arm_point>)))
  "Returns md5sum for a message object of type '<arm_point>"
  "a7c5188638382dfddf2aa151cdefcc50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arm_point)))
  "Returns md5sum for a message object of type 'arm_point"
  "a7c5188638382dfddf2aa151cdefcc50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arm_point>)))
  "Returns full string definition for message of type '<arm_point>"
  (cl:format cl:nil "# wesley::arm_point~%~%bool direct_mode~%float32 x~%float32 y~%float32 z~%float32 p~%float32 r~%string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arm_point)))
  "Returns full string definition for message of type 'arm_point"
  (cl:format cl:nil "# wesley::arm_point~%~%bool direct_mode~%float32 x~%float32 y~%float32 z~%float32 p~%float32 r~%string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arm_point>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arm_point>))
  "Converts a ROS message object to a list"
  (cl:list 'arm_point
    (cl:cons ':direct_mode (direct_mode msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':p (p msg))
    (cl:cons ':r (r msg))
    (cl:cons ':cmd (cmd msg))
))
