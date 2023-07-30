import hl2ss_schema
from enum import IntEnum
import time

import zenoh


class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class Locator(metaclass=SingletonMeta):
    _session = None
    _session_config = None

    def get_session(self, config=None):
        """
        get the zenoh session
        """
        if config is not None and not isinstance(config, dict):
            raise ValueError("Zenoh config needs to be provided as dictionary")

        if self._session_config is not None and config is not None:
            if self._session_config != config:
                raise ValueError("Existing Zenoh session with different config")
        if self._session is not None:
            return self._session
        zenoh.init_logger()
        zcfg = zenoh.Config.from_obj(config)
        self._session = zenoh.open(zcfg)
        self._session_config = config
        return self._session



# Duration and Time are from rclpy (https://github.com/ros2/rclpy/tree/ca1d26d6ce36414425f205ea0675aeb6cc969b46)

CONVERSION_CONSTANT = 10 ** 9
S_TO_NS = 1000 * 1000 * 1000


class ClockType(IntEnum):
    """
    Enum for clock type.

    This enum matches the one defined in rcl/time.h
    """

    ROS_TIME = 1
    SYSTEM_TIME = 2
    STEADY_TIME = 3


class Duration:
    """A period between two time points, with nanosecond precision."""

    def __init__(self, *, seconds=0, nanoseconds=0):
        """
        Create an instance of :class:`Duration`, combined from given seconds and nanoseconds.

        :param seconds: Time span seconds, if any, fractional part will be included.
        :param nanoseconds: Time span nanoseconds, if any, fractional part will be discarded.
        """
        total_nanoseconds = int(seconds * S_TO_NS)
        total_nanoseconds += int(nanoseconds)
        # missing overflow check
        self._value = total_nanoseconds

    @property
    def nanoseconds(self):
        return self._value

    def __repr__(self):
        return 'Duration(nanoseconds={0})'.format(self.nanoseconds)

    def __eq__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Duration(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare duration with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Duration):
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented

    def to_msg(self):
        """
        Get duration as :class:`builtin_interfaces.msg.Duration`.

        :returns: duration as message
        :rtype: builtin_interfaces.msg.Duration
        """
        seconds, nanoseconds = divmod(self.nanoseconds, S_TO_NS)
        return hl2ss_schema.Duration(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg):
        """
        Create an instance of :class:`Duration` from a duration message.

        :param msg: An instance of :class:`builtin_interfaces.msg.Duration`.
        """
        if not isinstance(msg, hl2ss_schema.Duration):
            raise TypeError('Must pass a builtin_interfaces.msg.Duration object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)


class Time:

    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total_nanoseconds = int(seconds * CONVERSION_CONSTANT)
        total_nanoseconds += int(nanoseconds)
        try:
            self._value = total_nanoseconds
        except OverflowError as e:
            raise OverflowError(
                'Total nanoseconds value is too large to store in C time point.') from e
        self._clock_type = clock_type

    @property
    def nanoseconds(self):
        return self._value

    def seconds_nanoseconds(self):
        """
        Get time as separate seconds and nanoseconds components.

        :returns: 2-tuple seconds and nanoseconds
        :rtype: tuple(int, int)
        """
        nanoseconds = self.nanoseconds
        return nanoseconds // CONVERSION_CONSTANT, nanoseconds % CONVERSION_CONSTANT

    @property
    def clock_type(self):
        return self._clock_type

    def __repr__(self):
        return 'Time(nanoseconds={0}, clock_type={1})'.format(
            self.nanoseconds, self.clock_type.name)

    def __add__(self, other):
        if isinstance(other, Duration):
            try:
                return Time(
                    nanoseconds=(self.nanoseconds + other.nanoseconds),
                    clock_type=self.clock_type)
            except OverflowError as e:
                raise OverflowError('Addition leads to overflow in C storage.') from e
        else:
            return NotImplemented

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't subtract times with different clock types")
            try:
                return Duration(nanoseconds=(self.nanoseconds - other.nanoseconds))
            except ValueError as e:
                raise ValueError('Subtraction leads to negative duration.') from e
        if isinstance(other, Duration):
            try:
                return Time(
                    nanoseconds=(self.nanoseconds - other.nanoseconds),
                    clock_type=self.clock_type)
            except ValueError as e:
                raise ValueError('Subtraction leads to negative time.') from e
        else:
            return NotImplemented

    def __eq__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds == other.nanoseconds
        # Raise instead of returning NotImplemented to prevent comparison with invalid types,
        # e.g. ints.
        # Otherwise `Time(nanoseconds=5) == 5` will return False instead of raising, and this
        # could lead to hard-to-find bugs.
        raise TypeError("Can't compare time with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds < other.nanoseconds
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds <= other.nanoseconds
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds > other.nanoseconds
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Time):
            if self.clock_type != other.clock_type:
                raise TypeError("Can't compare times with different clock types")
            return self.nanoseconds >= other.nanoseconds
        return NotImplemented

    def to_msg(self):
        seconds, nanoseconds = self.seconds_nanoseconds()
        return hl2ss_schema.Time(sec=seconds, nanosec=nanoseconds)

    @classmethod
    def from_msg(cls, msg, clock_type=ClockType.ROS_TIME):
        if not isinstance(msg, hl2ss_schema.Time):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec, clock_type=clock_type)

