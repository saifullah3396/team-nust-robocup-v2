# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from team_nust_msgs/TeamNUSTState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class TeamNUSTState(genpy.Message):
  _md5sum = "f38be50129fbe02202e870fd322ce791"
  _type = "team_nust_msgs/TeamNUSTState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
int32 motion_thread_period
int32 planning_thread_period
int32 sb_thread_period
int32 vision_thread_period
int32 localization_thread_period
int32 game_comm_thread_period
int32 user_comm_thread_period
int32 heart_beat
int32 player_number
int32 team_number
int32 team_port
int32 team_color
int32 robocup_role
int32 robot_intention
geometry_msgs/Pose2D robot_pose_2d
int32 stiffness_state
int32 posture_state
int32 planning_state
bool whistle_detected
bool robot_fallen
bool robot_in_motion
geometry_msgs/Point kick_target
geometry_msgs/Point move_target
int32 foot_on_ground
geometry_msgs/Transform l_foot_transform
geometry_msgs/Transform r_foot_transform

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

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['header','motion_thread_period','planning_thread_period','sb_thread_period','vision_thread_period','localization_thread_period','game_comm_thread_period','user_comm_thread_period','heart_beat','player_number','team_number','team_port','team_color','robocup_role','robot_intention','robot_pose_2d','stiffness_state','posture_state','planning_state','whistle_detected','robot_fallen','robot_in_motion','kick_target','move_target','foot_on_ground','l_foot_transform','r_foot_transform']
  _slot_types = ['std_msgs/Header','int32','int32','int32','int32','int32','int32','int32','int32','int32','int32','int32','int32','int32','int32','geometry_msgs/Pose2D','int32','int32','int32','bool','bool','bool','geometry_msgs/Point','geometry_msgs/Point','int32','geometry_msgs/Transform','geometry_msgs/Transform']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,motion_thread_period,planning_thread_period,sb_thread_period,vision_thread_period,localization_thread_period,game_comm_thread_period,user_comm_thread_period,heart_beat,player_number,team_number,team_port,team_color,robocup_role,robot_intention,robot_pose_2d,stiffness_state,posture_state,planning_state,whistle_detected,robot_fallen,robot_in_motion,kick_target,move_target,foot_on_ground,l_foot_transform,r_foot_transform

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TeamNUSTState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.motion_thread_period is None:
        self.motion_thread_period = 0
      if self.planning_thread_period is None:
        self.planning_thread_period = 0
      if self.sb_thread_period is None:
        self.sb_thread_period = 0
      if self.vision_thread_period is None:
        self.vision_thread_period = 0
      if self.localization_thread_period is None:
        self.localization_thread_period = 0
      if self.game_comm_thread_period is None:
        self.game_comm_thread_period = 0
      if self.user_comm_thread_period is None:
        self.user_comm_thread_period = 0
      if self.heart_beat is None:
        self.heart_beat = 0
      if self.player_number is None:
        self.player_number = 0
      if self.team_number is None:
        self.team_number = 0
      if self.team_port is None:
        self.team_port = 0
      if self.team_color is None:
        self.team_color = 0
      if self.robocup_role is None:
        self.robocup_role = 0
      if self.robot_intention is None:
        self.robot_intention = 0
      if self.robot_pose_2d is None:
        self.robot_pose_2d = geometry_msgs.msg.Pose2D()
      if self.stiffness_state is None:
        self.stiffness_state = 0
      if self.posture_state is None:
        self.posture_state = 0
      if self.planning_state is None:
        self.planning_state = 0
      if self.whistle_detected is None:
        self.whistle_detected = False
      if self.robot_fallen is None:
        self.robot_fallen = False
      if self.robot_in_motion is None:
        self.robot_in_motion = False
      if self.kick_target is None:
        self.kick_target = geometry_msgs.msg.Point()
      if self.move_target is None:
        self.move_target = geometry_msgs.msg.Point()
      if self.foot_on_ground is None:
        self.foot_on_ground = 0
      if self.l_foot_transform is None:
        self.l_foot_transform = geometry_msgs.msg.Transform()
      if self.r_foot_transform is None:
        self.r_foot_transform = geometry_msgs.msg.Transform()
    else:
      self.header = std_msgs.msg.Header()
      self.motion_thread_period = 0
      self.planning_thread_period = 0
      self.sb_thread_period = 0
      self.vision_thread_period = 0
      self.localization_thread_period = 0
      self.game_comm_thread_period = 0
      self.user_comm_thread_period = 0
      self.heart_beat = 0
      self.player_number = 0
      self.team_number = 0
      self.team_port = 0
      self.team_color = 0
      self.robocup_role = 0
      self.robot_intention = 0
      self.robot_pose_2d = geometry_msgs.msg.Pose2D()
      self.stiffness_state = 0
      self.posture_state = 0
      self.planning_state = 0
      self.whistle_detected = False
      self.robot_fallen = False
      self.robot_in_motion = False
      self.kick_target = geometry_msgs.msg.Point()
      self.move_target = geometry_msgs.msg.Point()
      self.foot_on_ground = 0
      self.l_foot_transform = geometry_msgs.msg.Transform()
      self.r_foot_transform = geometry_msgs.msg.Transform()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_14i3d3i3B6di14d.pack(_x.motion_thread_period, _x.planning_thread_period, _x.sb_thread_period, _x.vision_thread_period, _x.localization_thread_period, _x.game_comm_thread_period, _x.user_comm_thread_period, _x.heart_beat, _x.player_number, _x.team_number, _x.team_port, _x.team_color, _x.robocup_role, _x.robot_intention, _x.robot_pose_2d.x, _x.robot_pose_2d.y, _x.robot_pose_2d.theta, _x.stiffness_state, _x.posture_state, _x.planning_state, _x.whistle_detected, _x.robot_fallen, _x.robot_in_motion, _x.kick_target.x, _x.kick_target.y, _x.kick_target.z, _x.move_target.x, _x.move_target.y, _x.move_target.z, _x.foot_on_ground, _x.l_foot_transform.translation.x, _x.l_foot_transform.translation.y, _x.l_foot_transform.translation.z, _x.l_foot_transform.rotation.x, _x.l_foot_transform.rotation.y, _x.l_foot_transform.rotation.z, _x.l_foot_transform.rotation.w, _x.r_foot_transform.translation.x, _x.r_foot_transform.translation.y, _x.r_foot_transform.translation.z, _x.r_foot_transform.rotation.x, _x.r_foot_transform.rotation.y, _x.r_foot_transform.rotation.z, _x.r_foot_transform.rotation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robot_pose_2d is None:
        self.robot_pose_2d = geometry_msgs.msg.Pose2D()
      if self.kick_target is None:
        self.kick_target = geometry_msgs.msg.Point()
      if self.move_target is None:
        self.move_target = geometry_msgs.msg.Point()
      if self.l_foot_transform is None:
        self.l_foot_transform = geometry_msgs.msg.Transform()
      if self.r_foot_transform is None:
        self.r_foot_transform = geometry_msgs.msg.Transform()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 259
      (_x.motion_thread_period, _x.planning_thread_period, _x.sb_thread_period, _x.vision_thread_period, _x.localization_thread_period, _x.game_comm_thread_period, _x.user_comm_thread_period, _x.heart_beat, _x.player_number, _x.team_number, _x.team_port, _x.team_color, _x.robocup_role, _x.robot_intention, _x.robot_pose_2d.x, _x.robot_pose_2d.y, _x.robot_pose_2d.theta, _x.stiffness_state, _x.posture_state, _x.planning_state, _x.whistle_detected, _x.robot_fallen, _x.robot_in_motion, _x.kick_target.x, _x.kick_target.y, _x.kick_target.z, _x.move_target.x, _x.move_target.y, _x.move_target.z, _x.foot_on_ground, _x.l_foot_transform.translation.x, _x.l_foot_transform.translation.y, _x.l_foot_transform.translation.z, _x.l_foot_transform.rotation.x, _x.l_foot_transform.rotation.y, _x.l_foot_transform.rotation.z, _x.l_foot_transform.rotation.w, _x.r_foot_transform.translation.x, _x.r_foot_transform.translation.y, _x.r_foot_transform.translation.z, _x.r_foot_transform.rotation.x, _x.r_foot_transform.rotation.y, _x.r_foot_transform.rotation.z, _x.r_foot_transform.rotation.w,) = _struct_14i3d3i3B6di14d.unpack(str[start:end])
      self.whistle_detected = bool(self.whistle_detected)
      self.robot_fallen = bool(self.robot_fallen)
      self.robot_in_motion = bool(self.robot_in_motion)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_14i3d3i3B6di14d.pack(_x.motion_thread_period, _x.planning_thread_period, _x.sb_thread_period, _x.vision_thread_period, _x.localization_thread_period, _x.game_comm_thread_period, _x.user_comm_thread_period, _x.heart_beat, _x.player_number, _x.team_number, _x.team_port, _x.team_color, _x.robocup_role, _x.robot_intention, _x.robot_pose_2d.x, _x.robot_pose_2d.y, _x.robot_pose_2d.theta, _x.stiffness_state, _x.posture_state, _x.planning_state, _x.whistle_detected, _x.robot_fallen, _x.robot_in_motion, _x.kick_target.x, _x.kick_target.y, _x.kick_target.z, _x.move_target.x, _x.move_target.y, _x.move_target.z, _x.foot_on_ground, _x.l_foot_transform.translation.x, _x.l_foot_transform.translation.y, _x.l_foot_transform.translation.z, _x.l_foot_transform.rotation.x, _x.l_foot_transform.rotation.y, _x.l_foot_transform.rotation.z, _x.l_foot_transform.rotation.w, _x.r_foot_transform.translation.x, _x.r_foot_transform.translation.y, _x.r_foot_transform.translation.z, _x.r_foot_transform.rotation.x, _x.r_foot_transform.rotation.y, _x.r_foot_transform.rotation.z, _x.r_foot_transform.rotation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robot_pose_2d is None:
        self.robot_pose_2d = geometry_msgs.msg.Pose2D()
      if self.kick_target is None:
        self.kick_target = geometry_msgs.msg.Point()
      if self.move_target is None:
        self.move_target = geometry_msgs.msg.Point()
      if self.l_foot_transform is None:
        self.l_foot_transform = geometry_msgs.msg.Transform()
      if self.r_foot_transform is None:
        self.r_foot_transform = geometry_msgs.msg.Transform()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 259
      (_x.motion_thread_period, _x.planning_thread_period, _x.sb_thread_period, _x.vision_thread_period, _x.localization_thread_period, _x.game_comm_thread_period, _x.user_comm_thread_period, _x.heart_beat, _x.player_number, _x.team_number, _x.team_port, _x.team_color, _x.robocup_role, _x.robot_intention, _x.robot_pose_2d.x, _x.robot_pose_2d.y, _x.robot_pose_2d.theta, _x.stiffness_state, _x.posture_state, _x.planning_state, _x.whistle_detected, _x.robot_fallen, _x.robot_in_motion, _x.kick_target.x, _x.kick_target.y, _x.kick_target.z, _x.move_target.x, _x.move_target.y, _x.move_target.z, _x.foot_on_ground, _x.l_foot_transform.translation.x, _x.l_foot_transform.translation.y, _x.l_foot_transform.translation.z, _x.l_foot_transform.rotation.x, _x.l_foot_transform.rotation.y, _x.l_foot_transform.rotation.z, _x.l_foot_transform.rotation.w, _x.r_foot_transform.translation.x, _x.r_foot_transform.translation.y, _x.r_foot_transform.translation.z, _x.r_foot_transform.rotation.x, _x.r_foot_transform.rotation.y, _x.r_foot_transform.rotation.z, _x.r_foot_transform.rotation.w,) = _struct_14i3d3i3B6di14d.unpack(str[start:end])
      self.whistle_detected = bool(self.whistle_detected)
      self.robot_fallen = bool(self.robot_fallen)
      self.robot_in_motion = bool(self.robot_in_motion)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_14i3d3i3B6di14d = struct.Struct("<14i3d3i3B6di14d")
_struct_3I = struct.Struct("<3I")
