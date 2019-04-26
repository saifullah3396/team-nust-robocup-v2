;; Auto-generated. Do not edit!


(when (boundp 'team_nust_msgs::BehaviorInfo)
  (if (not (find-package "TEAM_NUST_MSGS"))
    (make-package "TEAM_NUST_MSGS"))
  (shadow 'BehaviorInfo (find-package "TEAM_NUST_MSGS")))
(unless (find-package "TEAM_NUST_MSGS::BEHAVIORINFO")
  (make-package "TEAM_NUST_MSGS::BEHAVIORINFO"))

(in-package "ROS")
;;//! \htmlinclude BehaviorInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass team_nust_msgs::BehaviorInfo
  :super ros::object
  :slots (_header _name _fsm_state _initiated _running _paused _finished _config ))

(defmethod team_nust_msgs::BehaviorInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:fsm_state __fsm_state) "")
    ((:initiated __initiated) nil)
    ((:running __running) nil)
    ((:paused __paused) nil)
    ((:finished __finished) nil)
    ((:config __config) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _fsm_state (string __fsm_state))
   (setq _initiated __initiated)
   (setq _running __running)
   (setq _paused __paused)
   (setq _finished __finished)
   (setq _config (string __config))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:fsm_state
   (&optional __fsm_state)
   (if __fsm_state (setq _fsm_state __fsm_state)) _fsm_state)
  (:initiated
   (&optional __initiated)
   (if __initiated (setq _initiated __initiated)) _initiated)
  (:running
   (&optional __running)
   (if __running (setq _running __running)) _running)
  (:paused
   (&optional __paused)
   (if __paused (setq _paused __paused)) _paused)
  (:finished
   (&optional __finished)
   (if __finished (setq _finished __finished)) _finished)
  (:config
   (&optional __config)
   (if __config (setq _config __config)) _config)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; string _fsm_state
    4 (length _fsm_state)
    ;; bool _initiated
    1
    ;; bool _running
    1
    ;; bool _paused
    1
    ;; bool _finished
    1
    ;; string _config
    4 (length _config)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _fsm_state
       (write-long (length _fsm_state) s) (princ _fsm_state s)
     ;; bool _initiated
       (if _initiated (write-byte -1 s) (write-byte 0 s))
     ;; bool _running
       (if _running (write-byte -1 s) (write-byte 0 s))
     ;; bool _paused
       (if _paused (write-byte -1 s) (write-byte 0 s))
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;; string _config
       (write-long (length _config) s) (princ _config s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fsm_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _initiated
     (setq _initiated (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _running
     (setq _running (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _paused
     (setq _paused (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _config
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _config (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get team_nust_msgs::BehaviorInfo :md5sum-) "57de5cc9489d08ac59be4fbd5d66a023")
(setf (get team_nust_msgs::BehaviorInfo :datatype-) "team_nust_msgs/BehaviorInfo")
(setf (get team_nust_msgs::BehaviorInfo :definition-)
      "Header header
string name
string fsm_state
bool initiated
bool running
bool paused
bool finished
string config

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



(provide :team_nust_msgs/BehaviorInfo "57de5cc9489d08ac59be4fbd5d66a023")


