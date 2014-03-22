; Auto-generated. Do not edit!


(cl:in-package mega_caretaker-msg)


;//! \htmlinclude MotorCommand.msg.html

(cl:defclass <MotorCommand> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:fixnum
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorCommand (<MotorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mega_caretaker-msg:<MotorCommand> is deprecated: use mega_caretaker-msg:MotorCommand instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mega_caretaker-msg:left-val is deprecated.  Use mega_caretaker-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mega_caretaker-msg:right-val is deprecated.  Use mega_caretaker-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommand>) ostream)
  "Serializes a message object of type '<MotorCommand>"
  (cl:let* ((signed (cl:slot-value msg 'left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommand>) istream)
  "Deserializes a message object of type '<MotorCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommand>)))
  "Returns string type for a message object of type '<MotorCommand>"
  "mega_caretaker/MotorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommand)))
  "Returns string type for a message object of type 'MotorCommand"
  "mega_caretaker/MotorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommand>)))
  "Returns md5sum for a message object of type '<MotorCommand>"
  "24825b8956c21f4c3dd28a5a4d09322c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommand)))
  "Returns md5sum for a message object of type 'MotorCommand"
  "24825b8956c21f4c3dd28a5a4d09322c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommand>)))
  "Returns full string definition for message of type '<MotorCommand>"
  (cl:format cl:nil "int8 left~%int8 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommand)))
  "Returns full string definition for message of type 'MotorCommand"
  (cl:format cl:nil "int8 left~%int8 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommand>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommand
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
