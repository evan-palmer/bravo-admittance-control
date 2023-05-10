import struct
import sys
import time

sys.path.append("..")

from driver import BravoDriver
from protocol import PacketID, DeviceID, Packet


if __name__ == "__main__":
    bravo = BravoDriver()

    bravo.connect()

    def my_hopefully_functional_cb(packet: Packet):
        print(packet.data)
        print(struct.unpack("<f", packet.data))

    def what_is_going_on_cb(packet: Packet):
        print(struct.unpack("<ffffff", packet.data))

    bravo.attach_callback(PacketID.ATI_FT_READING, what_is_going_on_cb)

    request = Packet(
        DeviceID.FORCE_TORQUE_SENSOR,
        PacketID.REQUEST,
        bytes([PacketID.ATI_FT_READING.value]),
    )

    tare = Packet(
        DeviceID.FORCE_TORQUE_SENSOR,
        PacketID.ATI_FT_READING,
        struct.pack(">ffffff", 0, 0, 0, 0, 0, 0),
    )

    bravo.send(tare)

    while True:
        bravo.send(request)
        time.sleep(1.0)
