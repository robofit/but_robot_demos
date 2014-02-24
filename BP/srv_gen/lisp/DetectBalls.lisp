; Auto-generated. Do not edit!


(cl:in-package BP-srv)


;//! \htmlinclude DetectBalls-request.msg.html

(cl:defclass <DetectBalls-request> (roslisp-msg-protocol:ros-message)
  ((balls
    :reader balls
    :initarg :balls
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DetectBalls-request (<DetectBalls-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectBalls-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectBalls-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name BP-srv:<DetectBalls-request> is deprecated: use BP-srv:DetectBalls-request instead.")))

(cl:ensure-generic-function 'balls-val :lambda-list '(m))
(cl:defmethod balls-val ((m <DetectBalls-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BP-srv:balls-val is deprecated.  Use BP-srv:balls instead.")
  (balls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectBalls-request>) ostream)
  "Serializes a message object of type '<DetectBalls-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'balls) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectBalls-request>) istream)
  "Deserializes a message object of type '<DetectBalls-request>"
    (cl:setf (cl:slot-value msg 'balls) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectBalls-request>)))
  "Returns string type for a service object of type '<DetectBalls-request>"
  "BP/DetectBallsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectBalls-request)))
  "Returns string type for a service object of type 'DetectBalls-request"
  "BP/DetectBallsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectBalls-request>)))
  "Returns md5sum for a message object of type '<DetectBalls-request>"
  "1392111cd8a1b7423a5cb85c5feb5ea2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectBalls-request)))
  "Returns md5sum for a message object of type 'DetectBalls-request"
  "1392111cd8a1b7423a5cb85c5feb5ea2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectBalls-request>)))
  "Returns full string definition for message of type '<DetectBalls-request>"
  (cl:format cl:nil "bool balls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectBalls-request)))
  "Returns full string definition for message of type 'DetectBalls-request"
  (cl:format cl:nil "bool balls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectBalls-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectBalls-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectBalls-request
    (cl:cons ':balls (balls msg))
))
;//! \htmlinclude DetectBalls-response.msg.html

(cl:defclass <DetectBalls-response> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type BP-msg:Detections
    :initform (cl:make-instance 'BP-msg:Detections)))
)

(cl:defclass DetectBalls-response (<DetectBalls-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectBalls-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectBalls-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name BP-srv:<DetectBalls-response> is deprecated: use BP-srv:DetectBalls-response instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <DetectBalls-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BP-srv:detections-val is deprecated.  Use BP-srv:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectBalls-response>) ostream)
  "Serializes a message object of type '<DetectBalls-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'detections) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectBalls-response>) istream)
  "Deserializes a message object of type '<DetectBalls-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'detections) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectBalls-response>)))
  "Returns string type for a service object of type '<DetectBalls-response>"
  "BP/DetectBallsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectBalls-response)))
  "Returns string type for a service object of type 'DetectBalls-response"
  "BP/DetectBallsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectBalls-response>)))
  "Returns md5sum for a message object of type '<DetectBalls-response>"
  "1392111cd8a1b7423a5cb85c5feb5ea2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectBalls-response)))
  "Returns md5sum for a message object of type 'DetectBalls-response"
  "1392111cd8a1b7423a5cb85c5feb5ea2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectBalls-response>)))
  "Returns full string definition for message of type '<DetectBalls-response>"
  (cl:format cl:nil "BP/Detections detections~%~%~%================================================================================~%MSG: BP/Detections~%int32 BALL=0      # Ball detection~%int32 OTHERS=1    # Other objects detection~%int32 type~%BP/PointOfInterest[] ballcenters~%~%================================================================================~%MSG: BP/PointOfInterest~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectBalls-response)))
  "Returns full string definition for message of type 'DetectBalls-response"
  (cl:format cl:nil "BP/Detections detections~%~%~%================================================================================~%MSG: BP/Detections~%int32 BALL=0      # Ball detection~%int32 OTHERS=1    # Other objects detection~%int32 type~%BP/PointOfInterest[] ballcenters~%~%================================================================================~%MSG: BP/PointOfInterest~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectBalls-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'detections))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectBalls-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectBalls-response
    (cl:cons ':detections (detections msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectBalls)))
  'DetectBalls-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectBalls)))
  'DetectBalls-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectBalls)))
  "Returns string type for a service object of type '<DetectBalls>"
  "BP/DetectBalls")