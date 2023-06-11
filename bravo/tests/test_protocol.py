import sys

import pytest  # noqa

sys.path.append("..")

from protocol import DeviceID, Packet, PacketID  # noqa


def test_packet_encoding() -> None:
    """Test that the packet encoding properly encodes serial data."""
    expected_encoding = bytes([0x06, 0x03, 0x60, 0x01, 0x05, 0x52, 0x00])

    packet = Packet(
        DeviceID.LINEAR_JAWS, PacketID.REQUEST, bytes([PacketID.POSITION.value])
    )

    assert expected_encoding == packet.encode()


def test_data_decoding() -> None:
    """Test that the serial data decoding properly decodes the serial data."""
    encoded_data = bytes([0x09, 0x01, 0x02, 0x03, 0x04, 0x01, 0xFF, 0x08, 0x5D, 0x00])
    decoded_data = bytes([0x01, 0x02, 0x03, 0x04])

    assert decoded_data == Packet.decode(encoded_data).data
