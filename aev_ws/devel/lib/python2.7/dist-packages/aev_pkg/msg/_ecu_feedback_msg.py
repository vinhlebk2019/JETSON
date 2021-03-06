# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from aev_pkg/ecu_feedback_msg.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ecu_feedback_msg(genpy.Message):
  _md5sum = "4501f6c5918ccf5a041a7524a20b1561"
  _type = "aev_pkg/ecu_feedback_msg"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint32 	msg_counter
uint8 feedbackSpeed_b1
uint8 feedbackSpeed_b2
uint8 feedbackSpeed_b3 
uint8 feedbackSpeed_b4 
uint8 	acceleratorLevel
bool acceleratorSwitch
bool brakeSwitch
bool movingDirection
uint8 	turnSignal
bool 	horn
bool 	frontLight
"""
  __slots__ = ['msg_counter','feedbackSpeed_b1','feedbackSpeed_b2','feedbackSpeed_b3','feedbackSpeed_b4','acceleratorLevel','acceleratorSwitch','brakeSwitch','movingDirection','turnSignal','horn','frontLight']
  _slot_types = ['uint32','uint8','uint8','uint8','uint8','uint8','bool','bool','bool','uint8','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       msg_counter,feedbackSpeed_b1,feedbackSpeed_b2,feedbackSpeed_b3,feedbackSpeed_b4,acceleratorLevel,acceleratorSwitch,brakeSwitch,movingDirection,turnSignal,horn,frontLight

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ecu_feedback_msg, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.msg_counter is None:
        self.msg_counter = 0
      if self.feedbackSpeed_b1 is None:
        self.feedbackSpeed_b1 = 0
      if self.feedbackSpeed_b2 is None:
        self.feedbackSpeed_b2 = 0
      if self.feedbackSpeed_b3 is None:
        self.feedbackSpeed_b3 = 0
      if self.feedbackSpeed_b4 is None:
        self.feedbackSpeed_b4 = 0
      if self.acceleratorLevel is None:
        self.acceleratorLevel = 0
      if self.acceleratorSwitch is None:
        self.acceleratorSwitch = False
      if self.brakeSwitch is None:
        self.brakeSwitch = False
      if self.movingDirection is None:
        self.movingDirection = False
      if self.turnSignal is None:
        self.turnSignal = 0
      if self.horn is None:
        self.horn = False
      if self.frontLight is None:
        self.frontLight = False
    else:
      self.msg_counter = 0
      self.feedbackSpeed_b1 = 0
      self.feedbackSpeed_b2 = 0
      self.feedbackSpeed_b3 = 0
      self.feedbackSpeed_b4 = 0
      self.acceleratorLevel = 0
      self.acceleratorSwitch = False
      self.brakeSwitch = False
      self.movingDirection = False
      self.turnSignal = 0
      self.horn = False
      self.frontLight = False

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
      buff.write(_get_struct_I11B().pack(_x.msg_counter, _x.feedbackSpeed_b1, _x.feedbackSpeed_b2, _x.feedbackSpeed_b3, _x.feedbackSpeed_b4, _x.acceleratorLevel, _x.acceleratorSwitch, _x.brakeSwitch, _x.movingDirection, _x.turnSignal, _x.horn, _x.frontLight))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 15
      (_x.msg_counter, _x.feedbackSpeed_b1, _x.feedbackSpeed_b2, _x.feedbackSpeed_b3, _x.feedbackSpeed_b4, _x.acceleratorLevel, _x.acceleratorSwitch, _x.brakeSwitch, _x.movingDirection, _x.turnSignal, _x.horn, _x.frontLight,) = _get_struct_I11B().unpack(str[start:end])
      self.acceleratorSwitch = bool(self.acceleratorSwitch)
      self.brakeSwitch = bool(self.brakeSwitch)
      self.movingDirection = bool(self.movingDirection)
      self.horn = bool(self.horn)
      self.frontLight = bool(self.frontLight)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_I11B().pack(_x.msg_counter, _x.feedbackSpeed_b1, _x.feedbackSpeed_b2, _x.feedbackSpeed_b3, _x.feedbackSpeed_b4, _x.acceleratorLevel, _x.acceleratorSwitch, _x.brakeSwitch, _x.movingDirection, _x.turnSignal, _x.horn, _x.frontLight))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 15
      (_x.msg_counter, _x.feedbackSpeed_b1, _x.feedbackSpeed_b2, _x.feedbackSpeed_b3, _x.feedbackSpeed_b4, _x.acceleratorLevel, _x.acceleratorSwitch, _x.brakeSwitch, _x.movingDirection, _x.turnSignal, _x.horn, _x.frontLight,) = _get_struct_I11B().unpack(str[start:end])
      self.acceleratorSwitch = bool(self.acceleratorSwitch)
      self.brakeSwitch = bool(self.brakeSwitch)
      self.movingDirection = bool(self.movingDirection)
      self.horn = bool(self.horn)
      self.frontLight = bool(self.frontLight)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_I11B = None
def _get_struct_I11B():
    global _struct_I11B
    if _struct_I11B is None:
        _struct_I11B = struct.Struct("<I11B")
    return _struct_I11B
