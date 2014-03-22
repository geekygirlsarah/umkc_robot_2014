
(cl:in-package :asdf)

(defsystem "wesley-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "arm_point" :depends-on ("_package_arm_point"))
    (:file "_package_arm_point" :depends-on ("_package"))
    (:file "arm_angle" :depends-on ("_package_arm_angle"))
    (:file "_package_arm_angle" :depends-on ("_package"))
  ))