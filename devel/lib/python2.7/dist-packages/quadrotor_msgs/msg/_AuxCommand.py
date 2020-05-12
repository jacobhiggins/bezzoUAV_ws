# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from quadrotor_msgs/AuxCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AuxCommand(genpy.Message):
  _md5sum = "94f75840e4b1e03675da764692f2c839"
  _type = "quadrotor_msgs/AuxCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 current_yaw
float64 kf_correction
float64[2] angle_corrections
bool enable_motors
bool use_external_yaw
"""
  __slots__ = ['current_yaw','kf_correction','angle_corrections','enable_motors','use_external_yaw']
  _slot_types = ['float64','float64','float64[2]','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       current_yaw,kf_correction,angle_corrections,enable_motors,use_external_yaw

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AuxCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.current_yaw is None:
        self.current_yaw = 0.
      if self.kf_correction is None:
        self.kf_correction = 0.
      if self.angle_corrections is None:
        self.angle_corrections = [0.] * 2
      if self.enable_motors is None:
        self.enable_motors = False
      if self.use_external_yaw is None:
        self.use_external_yaw = False
    else:
      self.current_yaw = 0.
      self.kf_correction = 0.
      self.angle_corrections = [0.] * 2
      self.enable_motors = False
      self.use_external_yaw = False

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
      buff.write(_get_struct_2d().pack(_x.current_yaw, _x.kf_correction))
      buff.write(_get_struct_2d().pack(*self.angle_corrections))
      _x = self
      buff.write(_get_struct_2B().pack(_x.enable_motors, _x.use_external_yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.current_yaw, _x.kf_correction,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 16
      self.angle_corrections = _get_struct_2d().unpack(str[start:end])
      _x = self
      start = end
      end += 2
      (_x.enable_motors, _x.use_external_yaw,) = _get_struct_2B().unpack(str[start:end])
      self.enable_motors = bool(self.enable_motors)
      self.use_external_yaw = bool(self.use_external_yaw)
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
      buff.write(_get_struct_2d().pack(_x.current_yaw, _x.kf_correction))
      buff.write(self.angle_corrections.tostring())
      _x = self
      buff.write(_get_struct_2B().pack(_x.enable_motors, _x.use_external_yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.current_yaw, _x.kf_correction,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 16
      self.angle_corrections = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=2)
      _x = self
      start = end
      end += 2
      (_x.enable_motors, _x.use_external_yaw,) = _get_struct_2B().unpack(str[start:end])
      self.enable_motors = bool(self.enable_motors)
      self.use_external_yaw = bool(self.use_external_yaw)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2d = None
def _get_struct_2d():
    global _struct_2d
    if _struct_2d is None:
        _struct_2d = struct.Struct("<2d")
    return _struct_2d
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
