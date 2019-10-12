; Auto-generated. Do not edit!


(cl:in-package dylan_msc-msg)


;//! \htmlinclude obj.msg.html

(cl:defclass <obj> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (centroid
    :reader centroid
    :initarg :centroid
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass obj (<obj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dylan_msc-msg:<obj> is deprecated: use dylan_msc-msg:obj instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dylan_msc-msg:index-val is deprecated.  Use dylan_msc-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'centroid-val :lambda-list '(m))
(cl:defmethod centroid-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dylan_msc-msg:centroid-val is deprecated.  Use dylan_msc-msg:centroid instead.")
  (centroid m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dylan_msc-msg:min-val is deprecated.  Use dylan_msc-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dylan_msc-msg:max-val is deprecated.  Use dylan_msc-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obj>) ostream)
  "Serializes a message object of type '<obj>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'centroid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obj>) istream)
  "Deserializes a message object of type '<obj>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'index)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'centroid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obj>)))
  "Returns string type for a message object of type '<obj>"
  "dylan_msc/obj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obj)))
  "Returns string type for a message object of type 'obj"
  "dylan_msc/obj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obj>)))
  "Returns md5sum for a message object of type '<obj>"
  "39f905263ec0534b03a70af1d99b67f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obj)))
  "Returns md5sum for a message object of type 'obj"
  "39f905263ec0534b03a70af1d99b67f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obj>)))
  "Returns full string definition for message of type '<obj>"
  (cl:format cl:nil "uint32 index~%geometry_msgs/Point centroid~%geometry_msgs/Point min~%geometry_msgs/Point max~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obj)))
  "Returns full string definition for message of type 'obj"
  (cl:format cl:nil "uint32 index~%geometry_msgs/Point centroid~%geometry_msgs/Point min~%geometry_msgs/Point max~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obj>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'centroid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obj>))
  "Converts a ROS message object to a list"
  (cl:list 'obj
    (cl:cons ':index (index msg))
    (cl:cons ':centroid (centroid msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
