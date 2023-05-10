from enum import Enum


class DeviceID(Enum):
    """The ID of a device on the Reach Bravo 7."""

    LINEAR_JAWS = 0x01
    ROTATE_END_EFFECTOR = 0x02
    BEND_FOREARM = 0x03
    ROTATE_ELBOW = 0x04
    BEND_ELBOW = 0x05
    BEND_SHOULDER = 0x06
    ROTATE_BASE = 0x07
    ALL_JOINTS = 0xFF
