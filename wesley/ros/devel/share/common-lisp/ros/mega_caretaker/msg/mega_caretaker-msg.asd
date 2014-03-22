
(cl:in-package :asdf)

(defsystem "mega_caretaker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MegaPacket" :depends-on ("_package_MegaPacket"))
    (:file "_package_MegaPacket" :depends-on ("_package"))
    (:file "MotorCommand" :depends-on ("_package_MotorCommand"))
    (:file "_package_MotorCommand" :depends-on ("_package"))
  ))