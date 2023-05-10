import atexit
import socket
import sys
import threading
from typing import Callable

sys.path.append("..")

from protocol import PacketID, Packet  # noqa


class BravoDriver:
    """Low-level interface for sending and receiving serial data from the Bravo 7."""

    def __init__(self, ip: str = "192.168.2.4", port: int = 6789) -> None:
        """Create a new driver.

        Args:
            ip: The IP address of the Bravo 7. Defaults to "192.168.2.4".
            port: The port to connect with the Bravo 7 over. Defaults to 6789.
        """
        self.address = (ip, port)
        self.callbacks: dict[PacketID, list[Callable]] = {}
        self._running = False

        # Configure a new socket with the Bravo
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0)

        self.poll_t = threading.Thread(target=self._poll)
        self.poll_t.setDaemon(True)

        atexit.register(self.disconnect)

    def connect(self) -> None:
        """Connect the driver to the Bravo 7."""
        self._running = True
        self.poll_t.run()

    def disconnect(self) -> None:
        """Disconnect the driver from the Bravo 7."""
        self._running = False
        self.poll_t.join()

    def send(self, packet: Packet):
        """Send a packet to the Bravo 7.

        Args:
            packet: The serial packet to send.
        """
        self.sock.sendto(packet.encode(), self.address)

    def attach_callback(self, packet_id: PacketID, callback: Callable) -> None:
        """Bind a callback to the given packet type.

        Args:
            packet_id: The ID of the packet that, when received, should signal the
                callback.
            callback: The callback to execute when a packet with the given ID is
                received.
        """
        if packet_id not in self.callbacks:
            self.callbacks[packet_id] = []

        self.callbacks[packet_id].append(callback)

    def _poll(self) -> None:
        """Poll the socket for new data and call the registered callbacks."""
        while self._running:
            try:
                read_data, _ = self.sock.recvfrom(256)
            except BaseException:
                ...
            else:
                if read_data == b"":
                    continue

                # TODO(evan): See what data is sent and process it
