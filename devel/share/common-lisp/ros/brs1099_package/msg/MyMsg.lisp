; Auto-generated. Do not edit!


(cl:in-package brs1099_package-msg)


;//! \htmlinclude MyMsg.msg.html

(cl:defclass <MyMsg> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass MyMsg (<MyMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brs1099_package-msg:<MyMsg> is deprecated: use brs1099_package-msg:MyMsg instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <MyMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brs1099_package-msg:id-val is deprecated.  Use brs1099_package-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <MyMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brs1099_package-msg:content-val is deprecated.  Use brs1099_package-msg:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyMsg>) ostream)
  "Serializes a message object of type '<MyMsg>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyMsg>) istream)
  "Deserializes a message object of type '<MyMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'content) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'content) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyMsg>)))
  "Returns string type for a message object of type '<MyMsg>"
  "brs1099_package/MyMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyMsg)))
  "Returns string type for a message object of type 'MyMsg"
  "brs1099_package/MyMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyMsg>)))
  "Returns md5sum for a message object of type '<MyMsg>"
  "5b3791c6f6999d894a380dc2e50b01e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyMsg)))
  "Returns md5sum for a message object of type 'MyMsg"
  "5b3791c6f6999d894a380dc2e50b01e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyMsg>)))
  "Returns full string definition for message of type '<MyMsg>"
  (cl:format cl:nil "int32 id~%string content~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyMsg)))
  "Returns full string definition for message of type 'MyMsg"
  (cl:format cl:nil "int32 id~%string content~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyMsg>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'MyMsg
    (cl:cons ':id (id msg))
    (cl:cons ':content (content msg))
))
