# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from team_nust_msgs/ObsObstacles.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import team_nust_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class ObsObstacles(genpy.Message):
  _md5sum = "1d072144a0a5b5af851cd9f0d3b6c877"
  _type = "team_nust_msgs/ObsObstacles"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
Obstacle[] obstacles

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
MSG: team_nust_msgs/Obstacle
uint8 type
geometry_msgs/Point center
geometry_msgs/Point left_bound
geometry_msgs/Point right_bound
float64 depth

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['header','obstacles']
  _slot_types = ['std_msgs/Header','team_nust_msgs/Obstacle[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,obstacles

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObsObstacles, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.obstacles is None:
        self.obstacles = []
    else:
      self.header = std_msgs.msg.Header()
      self.obstacles = []

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
      length = len(self.obstacles)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacles:
        buff.write(_struct_B.pack(val1.type))
        _v1 = val1.center
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v2 = val1.left_bound
        _x = _v2
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v3 = val1.right_bound
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_d.pack(val1.depth))
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
      if self.obstacles is None:
        self.obstacles = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacles = []
      for i in range(0, length):
        val1 = team_nust_msgs.msg.Obstacle()
        start = end
        end += 1
        (val1.type,) = _struct_B.unpack(str[start:end])
        _v4 = val1.center
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v5 = val1.left_bound
        _x = _v5
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v6 = val1.right_bound
        _x = _v6
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.depth,) = _struct_d.unpack(str[start:end])
        self.obstacles.append(val1)
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
      length = len(self.obstacles)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacles:
        buff.write(_struct_B.pack(val1.type))
        _v7 = val1.center
        _x = _v7
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v8 = val1.left_bound
        _x = _v8
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v9 = val1.right_bound
        _x = _v9
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_d.pack(val1.depth))
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
      if self.obstacles is None:
        self.obstacles = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacles = []
      for i in range(0, length):
        val1 = team_nust_msgs.msg.Obstacle()
        start = end
        end += 1
        (val1.type,) = _struct_B.unpack(str[start:end])
        _v10 = val1.center
        _x = _v10
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v11 = val1.left_bound
        _x = _v11
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v12 = val1.right_bound
        _x = _v12
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.depth,) = _struct_d.unpack(str[start:end])
        self.obstacles.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_d = struct.Struct("<d")
_struct_3d = struct.Struct("<3d")