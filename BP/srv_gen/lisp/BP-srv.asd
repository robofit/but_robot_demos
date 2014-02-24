
(cl:in-package :asdf)

(defsystem "BP-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :BP-msg
)
  :components ((:file "_package")
    (:file "DetectBalls" :depends-on ("_package_DetectBalls"))
    (:file "_package_DetectBalls" :depends-on ("_package"))
  ))