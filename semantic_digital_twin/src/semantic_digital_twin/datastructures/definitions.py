from enum import Enum, auto


class JointStateType(Enum): ...


class GripperState(JointStateType):
    OPEN = auto()
    CLOSE = auto()
    MEDIUM = auto()


class TorsoState(JointStateType):
    HIGH = auto()
    MID = auto()
    LOW = auto()


class StaticJointState(JointStateType):
    PARK = auto()


class RoomEnum(Enum):
    KITCHEN = auto()
    LIVINGROOM = auto()
    OFFICE = auto()
    DINING_ROOM = auto()
    PREPERATION_ROOM = auto()


class ChallengeMode(str, Enum):
    GPSR = "gpsr"
    PP = "pp"
    FD = "fd"