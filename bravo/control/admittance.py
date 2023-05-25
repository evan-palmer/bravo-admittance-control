import atexit
import struct
import sys
import threading
import time
from argparse import ArgumentParser, Namespace

import kinpy as kp
import numpy as np
import yaml  # type: ignore

sys.path.append("..")

from driver import BravoDriver  # noqa
from protocol import DeviceID, Packet, PacketID  # noqa


class AdmittanceController:
    """A demonstration of admittance control with the Reach Bravo 7 manipulator."""

    def __init__(
        self,
        Md: np.ndarray,
        Kp: np.ndarray,
        Kd: np.ndarray,
        Kf: np.ndarray,
        B: np.ndarray,
        fd: np.ndarray,
        xd: np.ndarray,
        serial_chain: kp.chain.chain.SerialChain,
    ) -> None:
        """Create a new admittance controller.

        Args:
            Md: The manipulator's virtual mass matrix.
            Kp: The proportional gain.
            Kd: The derivative gain.
            Kf: The force gain.
            B: The selection matrix used to dictate which axes are active during
                control.
            fd: The desired force to apply.
            xd: The desired end-effector pose.
            serial_chain: A serial chain built from the Bravo 7 URDF.
        """
        self._bravo = BravoDriver()
        self._running = False

        self.Md = Md  # Mass matrix
        self.Kp = Kp  # Proportional gain
        self.Kd = Kd  # Derivative gain
        self.Kf = Kf  # Force gain
        self.B = B  # Selection matrix
        self.fd = fd  # Desired forces
        self.xd = xd  # Desired end-effector pose
        self.serial_chain = serial_chain

        # Robot state
        self.joint_positions = np.array([0.0] * 7)
        self.joint_velocities = np.array([0.0] * 7)
        self.forces = np.array([0.0] * 6)

        # Attach the packet callbacks
        self._bravo.attach_callback(PacketID.ATI_FT_READING, self._read_ft_readings_cb)

        # Request the selected packets from the Bravo 7 devices
        self.request_readings_t = threading.Thread(
            target=self._request_packets,
            args=(
                {
                    DeviceID.FORCE_TORQUE_SENSOR: PacketID.ATI_FT_READING,
                    DeviceID.ALL_JOINTS: PacketID.POSITION,
                    DeviceID.ALL_JOINTS: PacketID.VELOCITY,
                },
            ),
        )
        self.request_readings_t.setDaemon(True)

        # Run the controller in it's own thread
        self.controller_t = threading.Thread(target=self._run_controller)
        self.controller_t.setDaemon(True)

        atexit.register(self.disable)

    def enable(self) -> None:
        """Enable admittance control."""
        self._running = True
        self._bravo.connect()
        self.request_readings_t.start()
        self.controller_t.start()

        # Tare the FT sensor on startup
        self.tare_ft_sensor()

    def disable(self) -> None:
        """Disable admittance control."""
        self._running = False
        self._bravo.disconnect()
        self.controller_t.join()
        self.request_readings_t.join()

    def _request_packets(self, packets: dict[DeviceID, PacketID]) -> None:
        """Request packets from the Bravo 7.

        Args:
            packets: A dictionary with the packets to request from the Bravo.
        """
        while self._running:
            for device_id, packet_id in packets.items():
                request = Packet(device_id, PacketID.REQUEST, bytes([packet_id.value]))
                self._bravo.send(request)

                # Delay a bit between packet requests
                time.sleep(0.01)

    def _read_ft_readings_cb(self, packet: Packet) -> None:
        """Read the force-torque sensor readings.

        Args:
            packet: The force-torque sensor reading packet.
        """
        # TODO(evan): Transform this from the FT-sensor frame to the EE frame
        forces = np.array(struct.unpack("<ffffff", packet.data))

    def _read_joint_positions_cb(self, packet: Packet) -> None:
        """Read the joint positions from the Bravo 7.

        Args:
            packet: The joint position packet.
        """
        position: float = struct.unpack("<f", packet.data)[0]

        # The jaws are a linear joint; convert from mm to m
        if packet.device_id == 0:
            position *= 0.001

        # Save the joint positions at the same index as their ID
        self.joint_positions[packet.device_id.value] = position

    def _read_joint_velocities_cb(self, packet: Packet) -> None:
        """Read the joint velocities from the Bravo 7.

        Args:
            packet: The joint velocity packet.
        """
        velocity: float = struct.unpack("<f", packet.data)[0]

        # The jaws are a linear joint; convert from mm/s to m/s
        if packet.device_id == 0:
            velocity *= 0.001

        # Save the joint velocities at the same index as their ID
        self.joint_velocities[packet.device_id.value] = velocity

    def tare_ft_sensor(self) -> None:
        """Tare the Bravo 7 force-torque sensor."""
        tare = Packet(
            DeviceID.FORCE_TORQUE_SENSOR,
            PacketID.ATI_FT_READING,
            struct.pack(">ffffff", 0, 0, 0, 0, 0, 0),
        )
        self._bravo.send(tare)

    def _run_controller(self) -> None:
        while self._running:
            # Calculate the Jacobian and it's time derivative
            jc = self.serial_chain.jacobian(self.joint_positions)

            # Get the current end-effector position
            xe = self.serial_chain.forward_kinematics(self.joint_positions)


def create_admittance_controller_from_file(
    config_fp: str, urdf_fp: str
) -> AdmittanceController:
    """Create an admittance controller from a configuration file.

    Args:
        config_fp: The full path to the configuration file to use.
        urdf_fp: The full path to the Bravo 7 URDF file.

    Returns:
        An admittance controller.
    """
    # Get the necessary parameters from the configuration file
    with open(config_fp, "r") as config_f:
        config = yaml.safe_load(config_f)

        # Get the dynamics and gains
        Md = config["Md"]
        Kp = config["Kp"]
        Kd = config["Kd"]
        Kf = config["Kf"]
        B = config["B"]
        fd = config["fd"]
        xd = config["xd"]

    # Get the serial chain using the Bravo 7 URDF
    with open(urdf_fp, "r") as urdf_f:
        serial_chain = kp.build_serial_chain_from_urdf(urdf_f, "ee_link")

    return AdmittanceController(Md, Kp, Kd, Kf, B, fd, xd, serial_chain)


def get_args() -> Namespace:
    """Get the script arguments.

    Returns:
        The parsed argument namespace.
    """
    parser = ArgumentParser()

    parser.add_argument(
        "config", type=str, help="The full path to the controller configuration file."
    )
    parser.add_argument(
        "bravo_urdf", type=str, help="The full path to the Bravo 7 URDF."
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()

    controller = create_admittance_controller_from_file(args.config, args.bravo_urdf)

    # Enable the controller
    controller.enable()

    # Let the controller do its thing
    while True:
        try:
            ...
        except KeyboardInterrupt:
            controller.disable()
