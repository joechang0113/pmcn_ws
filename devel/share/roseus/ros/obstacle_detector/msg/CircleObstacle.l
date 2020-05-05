;; Auto-generated. Do not edit!


(when (boundp 'obstacle_detector::CircleObstacle)
  (if (not (find-package "OBSTACLE_DETECTOR"))
    (make-package "OBSTACLE_DETECTOR"))
  (shadow 'CircleObstacle (find-package "OBSTACLE_DETECTOR")))
(unless (find-package "OBSTACLE_DETECTOR::CIRCLEOBSTACLE")
  (make-package "OBSTACLE_DETECTOR::CIRCLEOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude CircleObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass obstacle_detector::CircleObstacle
  :super ros::object
  :slots (_center _velocity _radius _true_radius _id _direction _state _LCSS_state _distance _action _state_list _direction_list _prev_direction ))

(defmethod obstacle_detector::CircleObstacle
  (:init
   (&key
    ((:center __center) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:radius __radius) 0.0)
    ((:true_radius __true_radius) 0.0)
    ((:id __id) "")
    ((:direction __direction) 0.0)
    ((:state __state) 0)
    ((:LCSS_state __LCSS_state) 0)
    ((:distance __distance) 0)
    ((:action __action) "")
    ((:state_list __state_list) (make-array 0 :initial-element 0 :element-type :integer))
    ((:direction_list __direction_list) (make-array 0 :initial-element 0 :element-type :integer))
    ((:prev_direction __prev_direction) 0)
    )
   (send-super :init)
   (setq _center __center)
   (setq _velocity __velocity)
   (setq _radius (float __radius))
   (setq _true_radius (float __true_radius))
   (setq _id (string __id))
   (setq _direction (float __direction))
   (setq _state (round __state))
   (setq _LCSS_state (round __LCSS_state))
   (setq _distance (round __distance))
   (setq _action (string __action))
   (setq _state_list __state_list)
   (setq _direction_list __direction_list)
   (setq _prev_direction (round __prev_direction))
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:true_radius
   (&optional __true_radius)
   (if __true_radius (setq _true_radius __true_radius)) _true_radius)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:LCSS_state
   (&optional __LCSS_state)
   (if __LCSS_state (setq _LCSS_state __LCSS_state)) _LCSS_state)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:state_list
   (&optional __state_list)
   (if __state_list (setq _state_list __state_list)) _state_list)
  (:direction_list
   (&optional __direction_list)
   (if __direction_list (setq _direction_list __direction_list)) _direction_list)
  (:prev_direction
   (&optional __prev_direction)
   (if __prev_direction (setq _prev_direction __prev_direction)) _prev_direction)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _center
    (send _center :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; float64 _radius
    8
    ;; float64 _true_radius
    8
    ;; string _id
    4 (length _id)
    ;; float64 _direction
    8
    ;; int64 _state
    8
    ;; int64 _LCSS_state
    8
    ;; int64 _distance
    8
    ;; string _action
    4 (length _action)
    ;; int64[] _state_list
    (* 8    (length _state_list)) 4
    ;; int64[] _direction_list
    (* 8    (length _direction_list)) 4
    ;; int64 _prev_direction
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _center
       (send _center :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _true_radius
       (sys::poke _true_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; float64 _direction
       (sys::poke _direction (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
     ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _distance (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _distance) (= (length (_distance . bv)) 2)) ;; bignum
              (write-long (ash (elt (_distance . bv) 0) 0) s)
              (write-long (ash (elt (_distance . bv) 1) -1) s))
             ((and (class _distance) (= (length (_distance . bv)) 1)) ;; big1
              (write-long (elt (_distance . bv) 0) s)
              (write-long (if (>= _distance 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _distance s)(write-long (if (>= _distance 0) 0 #xffffffff) s)))
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;; int64[] _state_list
     (write-long (length _state_list) s)
     (dotimes (i (length _state_list))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _state_list i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _state_list i)) (= (length ((elt _state_list i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _state_list i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _state_list i) . bv) 1) -1) s))
             ((and (class (elt _state_list i)) (= (length ((elt _state_list i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _state_list i) . bv) 0) s)
              (write-long (if (>= (elt _state_list i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _state_list i) s)(write-long (if (>= (elt _state_list i) 0) 0 #xffffffff) s)))
       )
     ;; int64[] _direction_list
     (write-long (length _direction_list) s)
     (dotimes (i (length _direction_list))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _direction_list i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _direction_list i)) (= (length ((elt _direction_list i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _direction_list i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _direction_list i) . bv) 1) -1) s))
             ((and (class (elt _direction_list i)) (= (length ((elt _direction_list i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _direction_list i) . bv) 0) s)
              (write-long (if (>= (elt _direction_list i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _direction_list i) s)(write-long (if (>= (elt _direction_list i) 0) 0 #xffffffff) s)))
       )
     ;; int64 _prev_direction
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _prev_direction (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _prev_direction) (= (length (_prev_direction . bv)) 2)) ;; bignum
              (write-long (ash (elt (_prev_direction . bv) 0) 0) s)
              (write-long (ash (elt (_prev_direction . bv) 1) -1) s))
             ((and (class _prev_direction) (= (length (_prev_direction . bv)) 1)) ;; big1
              (write-long (elt (_prev_direction . bv) 0) s)
              (write-long (if (>= _prev_direction 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _prev_direction s)(write-long (if (>= _prev_direction 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _true_radius
     (setq _true_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _direction
     (setq _direction (sys::peek buf ptr- :double)) (incf ptr- 8)
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
   ;; int64 _distance
#+(or :alpha :irix6 :x86_64)
      (setf _distance (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _distance (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64[] _state_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state_list (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _state_list i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _state_list i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; int64[] _direction_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _direction_list (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _direction_list i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _direction_list i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; int64 _prev_direction
#+(or :alpha :irix6 :x86_64)
      (setf _prev_direction (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _prev_direction (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get obstacle_detector::CircleObstacle :md5sum-) "ee86047c4d5777f2884d730ab57547d9")
(setf (get obstacle_detector::CircleObstacle :datatype-) "obstacle_detector/CircleObstacle")
(setf (get obstacle_detector::CircleObstacle :definition-)
      "geometry_msgs/Point center      # Central point [m]
geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
float64 radius                  # Radius with added margin [m]
float64 true_radius             # True measured radius [m]
string id
float64 direction

int64 state
int64 LCSS_state
int64 distance
string action
int64[] state_list
int64[] direction_list
int64 prev_direction
##int32 possibility
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :obstacle_detector/CircleObstacle "ee86047c4d5777f2884d730ab57547d9")


