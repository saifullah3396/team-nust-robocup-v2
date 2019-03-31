# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from team_nust_msgs/Obstacle.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Obstacle(genpy.Message):
  _md5sum = "01ef883c2fc32a991b3dced6dd949fec"
  _type = "team_nust_msgs/Obstacle"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 type
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
  __slots__ = ['type','center','left_bound','right_bound','depth']
  _slot_types = ['uint8','geometry_msgs/Point','geometry_msgs/Point','geometry_msgs/Point','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       type,center,left_bound,right_bound,depth

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Obstacle, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.type is None:
        self.type = 0
      if self.center is None:
        self.center = geometry_msgs.msg.Point()
      if self.left_bound is None:
        self.left_bound = geometry_msgs.msg.Point()
      if self.right_bound is None:
        self.right_bound = geometry_msgs.msg.Point()
      if self.depth is None:
        self.depth = 0.
    else:
      self.type = 0
      self.center = geometry_msgs.msg.Point()
      self.left_bound = geometry_msgs.msg.Point()
      self.right_bound = geometry_msgs.msg.Point()
      self.depth = 0.

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
      buff.write(_struct_B10d.pack(_x.type, _x.center.x, _x.center.y, _x.center.z, _x.left_bound.x, _x.left_bound.y, _x.left_bound.z, _x.right_bound.x, _x.right_bound.y, _x.right_bound.z, _x.depth))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.center is None:
        self.center = geometry_msgs.msg.Point()
      if self.left_bound is None:
        self.left_bound = geometry_msgs.msg.Point()
      if self.right_bound is None:
        self.right_bound = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 81
      (_x.type, _x.center.x, _x.center.y, _x.center.z, _x.left_bound.x, _x.left_bound.y, _x.left_bound.z, _x.right_bound.x, _x.right_bound.y, _x.right_bound.z, _x.depth,) = _struct_B10d.unpack(str[start:end])
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
      buff.write(_struct_B10d.pack(_x.type, _x.center.x, _x.center.y, _x.center.z, _x.left_bound.x, _x.left_bound.y, _x.left_bound.z, _x.right_bound.x, _x.right_bound.y, _x.right_bound.z, _x.depth))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.center is None:
        self.center = geometry_msgs.msg.Point()
      if self.left_bound is None:
        self.left_bound = geometry_msgs.msg.Point()
      if self.right_bound is None:
        self.right_bound = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 81
      (_x.type, _x.center.x, _x.center.y, _x.center.z, _x.left_bound.x, _x.left_bound.y, _x.left_bound.z, _x.right_bound.x, _x.right_bound.y, _x.right_bound.z, _x.depth,) = _struct_B10d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B10d = struct.Struct("<B10d")
