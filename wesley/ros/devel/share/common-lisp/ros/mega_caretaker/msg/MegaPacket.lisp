; Auto-generated. Do not edit!


(cl:in-package mega_caretaker-msg)


;//! \htmlinclude MegaPacket.msg.html

(cl:defclass <MegaPacket> (roslisp-msg-protocol:ros-message)
  ((msgType
    :reader msgType
    :initarg :msgType
    :type cl:fixnum
    :initform 0)
   (mCom
    :reader mCom
    :initarg :mCom
    :type mega_caretaker-msg:MotorCommand
    :initform (cl:make-instance 'mega_caretaker-msg:MotorCommand))
   (payload
    :reader payload
    :initarg :payload
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MegaPacket (<MegaPacket>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MegaPacket>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MegaPacket)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mega_caretaker-msg:<MegaPacket> is deprecated: use mega_caretaker-msg:MegaPacket instead.")))

(cl:ensure-generic-function 'msgType-val :lambda-list '(m))
(cl:defmethod msgType-val ((m <MegaPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mega_caretaker-msg:msgType-val is deprecated.  Use mega_caretaker-msg:msgType instead.")
  (msgType m))

(cl:ensure-generic-function 'mCom-val :lambda-list '(m))
(cl:defmethod mCom-val ((m <MegaPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mega_caretaker-msg:mCom-val is deprecated.  Use mega_caretaker-msg:mCom instead.")
  (mCom m))

(cl:ensure-generic-function 'payload-val :lambda-list '(m))
(cl:defmethod payload-val ((m <MegaPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mega_caretaker-msg:payload-val is deprecated.  Use mega_caretaker-msg:payload instead.")
  (payload m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MegaPacket>) ostream)
  "Serializes a message object of type '<MegaPacket>"
  (cl:let* ((signed (cl:slot-value msg 'msgType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mCom) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'payload)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'payload)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MegaPacket>) istream)
  "Deserializes a message object of type '<MegaPacket>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msgType) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mCom) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'payload)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'payload)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MegaPacket>)))
  "Returns string type for a message object of type '<MegaPacket>"
  "mega_caretaker/MegaPacket")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MegaPacket)))
  "Returns string type for a message object of type 'MegaPacket"
  "mega_caretaker/MegaPacket")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MegaPacket>)))
  "Returns md5sum for a message object of type '<MegaPacket>"
  "70e1bad56237befa0ecfa4de7f08c859")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MegaPacket)))
  "Returns md5sum for a message object of type 'MegaPacket"
  "70e1bad56237befa0ecfa4de7f08c859")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MegaPacket>)))
  "Returns full string definition for message of type '<MegaPacket>"
  (cl:format cl:nil "int8 msgType~%MotorCommand mCom~%uint16 payload~%~%================================================================================~%MSG: mega_caretaker/MotorCommand~%int8 left~%int8 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MegaPacket)))
  "Returns full string definition for message of type 'MegaPacket"
  (cl:format cl:nil "int8 msgType~%MotorCommand mCom~%uint16 payload~%~%================================================================================~%MSG: mega_caretaker/MotorCommand~%int8 left~%int8 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MegaPacket>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mCom))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MegaPacket>))
  "Converts a ROS message object to a list"
  (cl:list 'MegaPacket
    (cl:cons ':msgType (msgType msg))
    (cl:cons ':mCom (mCom msg))
    (cl:cons ':payload (payload msg))
))
