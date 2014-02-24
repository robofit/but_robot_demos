
(cl:in-package :asdf)

(defsystem "BP-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PointOfInterest" :depends-on ("_package_PointOfInterest"))
    (:file "_package_PointOfInterest" :depends-on ("_package"))
    (:file "GoalCoords" :depends-on ("_package_GoalCoords"))
    (:file "_package_GoalCoords" :depends-on ("_package"))
    (:file "Detections" :depends-on ("_package_Detections"))
    (:file "_package_Detections" :depends-on ("_package"))
  ))