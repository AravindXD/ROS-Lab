
(cl:in-package :asdf)

(defsystem "brs1099_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MyMsg" :depends-on ("_package_MyMsg"))
    (:file "_package_MyMsg" :depends-on ("_package"))
  ))