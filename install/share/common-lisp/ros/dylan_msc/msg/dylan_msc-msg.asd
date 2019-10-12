
(cl:in-package :asdf)

(defsystem "dylan_msc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "obj" :depends-on ("_package_obj"))
    (:file "_package_obj" :depends-on ("_package"))
  ))