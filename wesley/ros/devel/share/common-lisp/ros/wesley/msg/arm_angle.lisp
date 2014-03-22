; Auto-generated. Do not edit!


(cl:in-package wesley-msg)


;//! \htmlinclude arm_angle.msg.html

(cl:defclass <arm_angle> (roslisp-msg-protocol:ros-message)
  ((base
    :reader base
    :initarg :base
    :type cl:fixnum
    :initform 0)
   (shoulder
    :reader shoulder
    :initarg :shoulder
    :type cl:fixnum
    :initform 0)
   (elbow
    :reader elbow
    :initarg :elbow
    :type cl:fixnum
    :initform 0)
   (wrist_pitch
    :reader wrist_pitch
    :initarg :wrist_pitch
    :type cl:fixnum
    :initform 0)
   (wrist_roll
    :reader wrist_roll
    :initarg :wrist_roll
    :type cl:fixnum
    :initform 0)
   (hand
    :reader hand
    :initarg :hand
    :type cl:fixnum
    :initform 0))
)

(cl:defclass arm_angle (<arm_angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arm_angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arm_angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wesley-msg:<arm_angle> is deprecated: use wesley-msg:arm_angle instead.")))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:base-val is deprecated.  Use wesley-msg:base instead.")
  (base m))

(cl:ensure-generic-function 'shoulder-val :lambda-list '(m))
(cl:defmethod shoulder-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:shoulder-val is deprecated.  Use wesley-msg:shoulder instead.")
  (shoulder m))

(cl:ensure-generic-function 'elbow-val :lambda-list '(m))
(cl:defmethod elbow-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:elbow-val is deprecated.  Use wesley-msg:elbow instead.")
  (elbow m))

(cl:ensure-generic-function 'wrist_pitch-val :lambda-list '(m))
(cl:defmethod wrist_pitch-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:wrist_pitch-val is deprecated.  Use wesley-msg:wrist_pitch instead.")
  (wrist_pitch m))

(cl:ensure-generic-function 'wrist_roll-val :lambda-list '(m))
(cl:defmethod wrist_roll-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:wrist_roll-val is deprecated.  Use wesley-msg:wrist_roll instead.")
  (wrist_roll m))

(cl:ensure-generic-function 'hand-val :lambda-list '(m))
(cl:defmethod hand-val ((m <arm_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wesley-msg:hand-val is deprecated.  Use wesley-msg:hand instead.")
  (hand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arm_angle>) ostream)
  "Serializes a message object of type '<arm_angle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'base)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shoulder)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shoulder)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'elbow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'elbow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_pitch)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_pitch)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_roll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_roll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hand)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arm_angle>) istream)
  "Deserializes a message object of type '<arm_angle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'base)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shoulder)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shoulder)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'elbow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'elbow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_pitch)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_pitch)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_roll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_roll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hand)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arm_angle>)))
  "Returns string type for a message object of type '<arm_angle>"
  "wesley/arm_angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arm_angle)))
  "Returns string type for a message object of type 'arm_angle"
  "wesley/arm_angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arm_angle>)))
  "Returns md5sum for a message object of type '<arm_angle>"
  "ef33d4984a305fd75dffb44db14b17ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arm_angle)))
  "Returns md5sum for a message object of type 'arm_angle"
  "ef33d4984a305fd75dffb44db14b17ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arm_angle>)))
  "Returns full string definition for message of type '<arm_angle>"
  (cl:format cl:nil "# wesley::arm_angle~%~%uint16 base~%uint16 shoulder~%uint16 elbow~%uint16 wrist_pitch~%uint16 wrist_roll~%uint16 hand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arm_angle)))
  "Returns full string definition for message of type 'arm_angle"
  (cl:format cl:nil "# wesley::arm_angle~%~%uint16 base~%uint16 shoulder~%uint16 elbow~%uint16 wrist_pitch~%uint16 wrist_roll~%uint16 hand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arm_angle>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arm_angle>))
  "Converts a ROS message object to a list"
  (cl:list 'arm_angle
    (cl:cons ':base (base msg))
    (cl:cons ':shoulder (shoulder msg))
    (cl:cons ':elbow (elbow msg))
    (cl:cons ':wrist_pitch (wrist_pitch msg))
    (cl:cons ':wrist_roll (wrist_roll msg))
    (cl:cons ':hand (hand msg))
))
