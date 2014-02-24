; Auto-generated. Do not edit!


(cl:in-package BP-msg)


;//! \htmlinclude Detections.msg.html

(cl:defclass <Detections> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (ballcenters
    :reader ballcenters
    :initarg :ballcenters
    :type (cl:vector BP-msg:PointOfInterest)
   :initform (cl:make-array 0 :element-type 'BP-msg:PointOfInterest :initial-element (cl:make-instance 'BP-msg:PointOfInterest))))
)

(cl:defclass Detections (<Detections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Detections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Detections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name BP-msg:<Detections> is deprecated: use BP-msg:Detections instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Detections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BP-msg:type-val is deprecated.  Use BP-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'ballcenters-val :lambda-list '(m))
(cl:defmethod ballcenters-val ((m <Detections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BP-msg:ballcenters-val is deprecated.  Use BP-msg:ballcenters instead.")
  (ballcenters m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Detections>)))
    "Constants for message type '<Detections>"
  '((:BALL . 0)
    (:OTHERS . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Detections)))
    "Constants for message type 'Detections"
  '((:BALL . 0)
    (:OTHERS . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Detections>) ostream)
  "Serializes a message object of type '<Detections>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ballcenters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ballcenters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Detections>) istream)
  "Deserializes a message object of type '<Detections>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ballcenters) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ballcenters)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'BP-msg:PointOfInterest))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Detections>)))
  "Returns string type for a message object of type '<Detections>"
  "BP/Detections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Detections)))
  "Returns string type for a message object of type 'Detections"
  "BP/Detections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Detections>)))
  "Returns md5sum for a message object of type '<Detections>"
  "ca05d6b60fdc98a12a77e103e579b06f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Detections)))
  "Returns md5sum for a message object of type 'Detections"
  "ca05d6b60fdc98a12a77e103e579b06f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Detections>)))
  "Returns full string definition for message of type '<Detections>"
  (cl:format cl:nil "int32 BALL=0      # Ball detection~%int32 OTHERS=1    # Other objects detection~%int32 type~%BP/PointOfInterest[] ballcenters~%~%================================================================================~%MSG: BP/PointOfInterest~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Detections)))
  "Returns full string definition for message of type 'Detections"
  (cl:format cl:nil "int32 BALL=0      # Ball detection~%int32 OTHERS=1    # Other objects detection~%int32 type~%BP/PointOfInterest[] ballcenters~%~%================================================================================~%MSG: BP/PointOfInterest~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Detections>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ballcenters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Detections>))
  "Converts a ROS message object to a list"
  (cl:list 'Detections
    (cl:cons ':type (type msg))
    (cl:cons ':ballcenters (ballcenters msg))
))
