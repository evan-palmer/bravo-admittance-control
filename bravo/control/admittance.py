import atexit
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

sys.path.append("..")

from driver import BravoDriver  # noqa
from protocol import DeviceID, Packet, PacketID  # noqa


@dataclass
class Dynamics:
    """Encapsulates the dynamic parameters for the Bravo 7."""

    Md: np.ndarray
    Kp: np.ndarray
    Kd: np.ndarray
    Kf: np.ndarray


class AdmittanceController:
    """A demonstration of admittance control with the Reach Bravo 7 manipulator."""

    def __init__(self, dynamics: Dynamics) -> None:
        """Create a new admittance controller instance."""
        self._bravo = BravoDriver()
        self._running = False
        self.dynamics = dynamics

        # Attach the packet callbacks
        self._bravo.attach_callback(PacketID.ATI_FT_READING, self._save_ft_readings_cb)

        self.request_ft_readings_t = threading.Thread(target=self._request_ft_readings)
        self.request_ft_readings_t.setDaemon(True)

        atexit.register(self.disable)

    def enable(self) -> None:
        """Enable admittance control."""
        self._running = True
        self._bravo.connect()
        self.request_ft_readings_t.start()

        # Tare the FT sensor on startup
        self.tare_ft_sensor()

    def disable(self) -> None:
        """Disable admittance control."""
        self._running = False
        self._bravo.disconnect()
        self.request_ft_readings_t.join()

    def _request_ft_readings(self) -> None:
        """Poll the Bravo 7 force-torque sensor readings."""
        while self._running:
            request = Packet(
                DeviceID.FORCE_TORQUE_SENSOR,
                PacketID.REQUEST,
                bytes([PacketID.ATI_FT_READING.value]),
            )

            self._bravo.send(request)

            # Poll the sensor at a frequency of 100 Hz
            time.sleep(0.01)

    def _save_ft_readings_cb(self, packet: Packet) -> None:
        """Save the force-torque sensor readings for use by the admittance controller.

        Args:
            packet: The force-torque sensor reading packet.
        """
        # We are using Windows for this demonstration, so the packets received use
        # little-endian ordering.
        print(struct.unpack("<ffffff", packet.data))

    def tare_ft_sensor(self) -> None:
        """Tare the Bravo 7 force-torque sensor."""
        tare = Packet(
            DeviceID.FORCE_TORQUE_SENSOR,
            PacketID.ATI_FT_READING,
            struct.pack(">ffffff", 0, 0, 0, 0, 0, 0),
        )
        self._bravo.send(tare)


if __name__ == "__main__":
    controller = AdmittanceController()

    # Enable the controller
    controller.enable()

    # Let the controller do its thing
    while True:
        ...
