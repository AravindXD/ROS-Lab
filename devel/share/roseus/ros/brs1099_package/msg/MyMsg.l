;; Auto-generated. Do not edit!


(when (boundp 'brs1099_package::MyMsg)
  (if (not (find-package "BRS1099_PACKAGE"))
    (make-package "BRS1099_PACKAGE"))
  (shadow 'MyMsg (find-package "BRS1099_PACKAGE")))
(unless (find-package "BRS1099_PACKAGE::MYMSG")
  (make-package "BRS1099_PACKAGE::MYMSG"))

(in-package "ROS")
;;//! \htmlinclude MyMsg.msg.html


(defclass brs1099_package::MyMsg
  :super ros::object
  :slots (_id _content ))

(defmethod brs1099_package::MyMsg
  (:init
   (&key
    ((:id __id) 0)
    ((:content __content) "")
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _content (string __content))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:content
   (&optional __content)
   (if __content (setq _content __content)) _content)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; string _content
    4 (length _content)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; string _content
       (write-long (length _content) s) (princ _content s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _content
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _content (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get brs1099_package::MyMsg :md5sum-) "5b3791c6f6999d894a380dc2e50b01e2")
(setf (get brs1099_package::MyMsg :datatype-) "brs1099_package/MyMsg")
(setf (get brs1099_package::MyMsg :definition-)
      "int32 id
string content
")



(provide :brs1099_package/MyMsg "5b3791c6f6999d894a380dc2e50b01e2")


