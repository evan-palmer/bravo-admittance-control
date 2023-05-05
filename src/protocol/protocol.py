from enum import Enum

from cobs import cobs
from crc import Calculator, Configuration


class PacketID(Enum):
    MODE = 0x01
    VELOCITY = 0x02
    POSITION = 0x03
    CURRENT = 0x05
    RELATIVE_POSITION = 0x0E
    INDEXED_POSITION = 0x0D
    REQUEST = 0x60
    SERIAL_NUMBER = 0x61
    MODEL_NUMBER = 0x62
    TEMPERATURE = 0x66
    SOFTWARE_VERSION = 0x6C
    KM_END_POS = 0xA1
    KM_END_VEL = 0xA2
    KM_END_VEL_LOCAL = 0xCB
    KM_BOX_OBSTACLE_02 = 0xA5
    KM_BOX_OBSTACLE_03 = 0xA6
    KM_BOX_OBSTACLE_04 = 0xA7
    KM_BOX_OBSTACLE_05 = 0xA8
    KM_CYLINDER_OBSTACLE_02 = 0xAB
    KM_CYLINDER_OBSTACLE_03 = 0xAC
    KM_CYLINDER_OBSTACLE_04 = 0xAD
    KM_CYLINDER_OBSTACLE_05 = 0xAE
    VOLTAGE = 0x90
    SAVE = 0x50
    HEARTBEAT_FREQUENCY = 0x92
    HEARTBEAT_SET = 0x91
    POSITION_LIMITS = 0x10
    VELOCITY_LIMITS = 0x11
    CURRENT_LIMITS = 0x12
    ATI_FT_READING = 0xD8
    BOOTLOADER = 0xFF
    VOLTAGE_THRESHOLD_PARAMETERS = 0x99


class DeviceID(Enum):
    LINEAR_JAWS = 0x01
    ROTATE_END_EFFECTOR = 0x02
    BEND_ELBOW = 0x03
    BEND_SHOULDER = 0x04
    ROTATE_BASE = 0x05
    ALL_JOINTS = 0xFF


class Packet:
    def __init__(
        self, device_id: DeviceID, packet_id: PacketID, data: bytearray
    ) -> None:
        self.device_id = device_id
        self.packet_id = packet_id

        crc_config = Configuration(
            width=8,
            polynomial=0x4D,
            init_value=0x00,
            final_xor_value=0xFF,
            reverse_input=True,
            reverse_output=True,
        )

        self._crc_calculator = Calculator(crc_config)

    def encode(self) -> bytearray:
        ...

    @classmethod
    def decode(cls, data: bytearray) -> Packet:
        ...
