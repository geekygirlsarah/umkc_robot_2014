
(cl:in-package :asdf)

(defsystem "imu_filter_madgwick-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "imu_yaw" :depends-on ("_package_imu_yaw"))
    (:file "_package_imu_yaw" :depends-on ("_package"))
  ))