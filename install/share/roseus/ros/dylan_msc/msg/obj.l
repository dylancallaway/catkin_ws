;; Auto-generated. Do not edit!


(when (boundp 'dylan_msc::obj)
  (if (not (find-package "DYLAN_MSC"))
    (make-package "DYLAN_MSC"))
  (shadow 'obj (find-package "DYLAN_MSC")))
(unless (find-package "DYLAN_MSC::OBJ")
  (make-package "DYLAN_MSC::OBJ"))

(in-package "ROS")
;;//! \htmlinclude obj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass dylan_msc::obj
  :super ros::object
  :slots (_index _centroid _min _max ))

(defmethod dylan_msc::obj
  (:init
   (&key
    ((:index __index) 0)
    ((:centroid __centroid) (instance geometry_msgs::Point :init))
    ((:min __min) (instance geometry_msgs::Point :init))
    ((:max __max) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _centroid __centroid)
   (setq _min __min)
   (setq _max __max)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:centroid
   (&rest __centroid)
   (if (keywordp (car __centroid))
       (send* _centroid __centroid)
     (progn
       (if __centroid (setq _centroid (car __centroid)))
       _centroid)))
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; uint32 _index
    4
    ;; geometry_msgs/Point _centroid
    (send _centroid :serialization-length)
    ;; geometry_msgs/Point _min
    (send _min :serialization-length)
    ;; geometry_msgs/Point _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _index
       (write-long _index s)
     ;; geometry_msgs/Point _centroid
       (send _centroid :serialize s)
     ;; geometry_msgs/Point _min
       (send _min :serialize s)
     ;; geometry_msgs/Point _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _centroid
     (send _centroid :deserialize buf ptr-) (incf ptr- (send _centroid :serialization-length))
   ;; geometry_msgs/Point _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Point _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get dylan_msc::obj :md5sum-) "39f905263ec0534b03a70af1d99b67f0")
(setf (get dylan_msc::obj :datatype-) "dylan_msc/obj")
(setf (get dylan_msc::obj :definition-)
      "uint32 index
geometry_msgs/Point centroid
geometry_msgs/Point min
geometry_msgs/Point max
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :dylan_msc/obj "39f905263ec0534b03a70af1d99b67f0")


