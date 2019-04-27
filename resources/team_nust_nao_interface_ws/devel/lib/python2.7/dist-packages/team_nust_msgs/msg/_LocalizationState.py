# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from team_nust_msgs/LocalizationState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class LocalizationState(genpy.Message):
  _md5sum = "6443aeeb1985422e92a70b31dbb67942"
  _type = "team_nust_msgs/LocalizationState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
bool robot_localized
int32 position_confidence
int32 side_confidence
bool robot_on_side_line
bool localize_with_last_known
bool landmarks_found

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
"""
  __slots__ = ['header','robot_localized','position_confidence','side_confidence','robot_on_side_line','localize_with_last_known','landmarks_found']
  _slot_types = ['std_msgs/Header','bool','int32','int32','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,robot_localized,position_confidence,side_confidence,robot_on_side_line,localize_with_last_known,landmarks_found

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LocalizationState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.robot_localized is None:
        self.robot_localized = False
      if self.position_confidence is None:
        self.position_confidence = 0
      if self.side_confidence is None:
        self.side_confidence = 0
      if self.robot_on_side_line is None:
        self.robot_on_side_line = False
      if self.localize_with_last_known is None:
        self.localize_with_last_known = False
      if self.landmarks_found is None:
        self.landmarks_found = False
    else:
      self.header = std_msgs.msg.Header()
      self.robot_localized = False
      self.position_confidence = 0
      self.side_confidence = 0
      self.robot_on_side_line = False
      self.localize_with_last_known = False
      self.landmarks_found = False

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
      buff.write(_struct_B2i3B.pack(_x.robot_localized, _x.position_confidence, _x.side_confidence, _x.robot_on_side_line, _x.localize_with_last_known, _x.landmarks_found))
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
      end += 12
      (_x.robot_localized, _x.position_confidence, _x.side_confidence, _x.robot_on_side_line, _x.localize_with_last_known, _x.landmarks_found,) = _struct_B2i3B.unpack(str[start:end])
      self.robot_localized = bool(self.robot_localized)
      self.robot_on_side_line = bool(self.robot_on_side_line)
      self.localize_with_last_known = bool(self.localize_with_last_known)
      self.landmarks_found = bool(self.landmarks_found)
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
      buff.write(_struct_B2i3B.pack(_x.robot_localized, _x.position_confidence, _x.side_confidence, _x.robot_on_side_line, _x.localize_with_last_known, _x.landmarks_found))
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
      end += 12
      (_x.robot_localized, _x.position_confidence, _x.side_confidence, _x.robot_on_side_line, _x.localize_with_last_known, _x.landmarks_found,) = _struct_B2i3B.unpack(str[start:end])
      self.robot_localized = bool(self.robot_localized)
      self.robot_on_side_line = bool(self.robot_on_side_line)
      self.localize_with_last_known = bool(self.localize_with_last_known)
      self.landmarks_found = bool(self.landmarks_found)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_B2i3B = struct.Struct("<B2i3B")