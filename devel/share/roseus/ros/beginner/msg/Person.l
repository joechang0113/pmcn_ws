;; Auto-generated. Do not edit!


(when (boundp 'beginner::Person)
  (if (not (find-package "BEGINNER"))
    (make-package "BEGINNER"))
  (shadow 'Person (find-package "BEGINNER")))
(unless (find-package "BEGINNER::PERSON")
  (make-package "BEGINNER::PERSON"))

(in-package "ROS")
;;//! \htmlinclude Person.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass beginner::Person
  :super ros::object
  :slots (_header _user_id _direction _velocity _acc _threshold_upper _threshold_low _avg _signal _state _LCSS_state ))

(defmethod beginner::Person
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:user_id __user_id) "")
    ((:direction __direction) 0.0)
    ((:velocity __velocity) 0.0)
    ((:acc __acc) 0.0)
    ((:threshold_upper __threshold_upper) 0.0)
    ((:threshold_low __threshold_low) 0.0)
    ((:avg __avg) 0.0)
    ((:signal __signal) 0.0)
    ((:state __state) 0)
    ((:LCSS_state __LCSS_state) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _user_id (string __user_id))
   (setq _direction (float __direction))
   (setq _velocity (float __velocity))
   (setq _acc (float __acc))
   (setq _threshold_upper (float __threshold_upper))
   (setq _threshold_low (float __threshold_low))
   (setq _avg (float __avg))
   (setq _signal (float __signal))
   (setq _state (round __state))
   (setq _LCSS_state (round __LCSS_state))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:user_id
   (&optional __user_id)
   (if __user_id (setq _user_id __user_id)) _user_id)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:threshold_upper
   (&optional __threshold_upper)
   (if __threshold_upper (setq _threshold_upper __threshold_upper)) _threshold_upper)
  (:threshold_low
   (&optional __threshold_low)
   (if __threshold_low (setq _threshold_low __threshold_low)) _threshold_low)
  (:avg
   (&optional __avg)
   (if __avg (setq _avg __avg)) _avg)
  (:signal
   (&optional __signal)
   (if __signal (setq _signal __signal)) _signal)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:LCSS_state
   (&optional __LCSS_state)
   (if __LCSS_state (setq _LCSS_state __LCSS_state)) _LCSS_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _user_id
    4 (length _user_id)
    ;; float64 _direction
    8
    ;; float64 _velocity
    8
    ;; float64 _acc
    8
    ;; float64 _threshold_upper
    8
    ;; float64 _threshold_low
    8
    ;; float64 _avg
    8
    ;; float64 _signal
    8
    ;; int64 _state
    8
    ;; int64 _LCSS_state
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _user_id
       (write-long (length _user_id) s) (princ _user_id s)
     ;; float64 _direction
       (sys::poke _direction (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _threshold_upper
       (sys::poke _threshold_upper (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _threshold_low
       (sys::poke _threshold_low (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _avg
       (sys::poke _avg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _signal
       (sys::poke _signal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _state
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _state (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _state) (= (length (_state . bv)) 2)) ;; bignum
              (write-long (ash (elt (_state . bv) 0) 0) s)
              (write-long (ash (elt (_state . bv) 1) -1) s))
             ((and (class _state) (= (length (_state . bv)) 1)) ;; big1
              (write-long (elt (_state . bv) 0) s)
              (write-long (if (>= _state 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _state s)(write-long (if (>= _state 0) 0 #xffffffff) s)))
     ;; int64 _LCSS_state
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _LCSS_state (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _LCSS_state) (= (length (_LCSS_state . bv)) 2)) ;; bignum
              (write-long (ash (elt (_LCSS_state . bv) 0) 0) s)
              (write-long (ash (elt (_LCSS_state . bv) 1) -1) s))
             ((and (class _LCSS_state) (= (length (_LCSS_state . bv)) 1)) ;; big1
              (write-long (elt (_LCSS_state . bv) 0) s)
              (write-long (if (>= _LCSS_state 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _LCSS_state s)(write-long (if (>= _LCSS_state 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _user_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _user_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _direction
     (setq _direction (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acc
     (setq _acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _threshold_upper
     (setq _threshold_upper (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _threshold_low
     (setq _threshold_low (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _avg
     (setq _avg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _signal
     (setq _signal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _state
#+(or :alpha :irix6 :x86_64)
      (setf _state (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _state (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _LCSS_state
#+(or :alpha :irix6 :x86_64)
      (setf _LCSS_state (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _LCSS_state (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get beginner::Person :md5sum-) "9d6531b7d7f85370791373555eb7b8ed")
(setf (get beginner::Person :datatype-) "beginner/Person")
(setf (get beginner::Person :definition-)
      "std_msgs/Header header

##string user_id
##float64 direction
##float64 velocity

string user_id
float64 direction
float64 velocity
float64 acc
float64 threshold_upper
float64 threshold_low
float64 avg
float64 signal
int64 state
int64 LCSS_state

##int64 Imu_state
##int64 Lidar_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :beginner/Person "9d6531b7d7f85370791373555eb7b8ed")


