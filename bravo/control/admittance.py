import atexit
import struct
import sys
import threading
import time
from argparse import ArgumentParser, Namespace
from collections import deque
from typing import Any

import kinpy as kp
import numpy as np
import yaml  # type: ignore
from scipy.spatial.transform import Rotation as R

sys.path.append("..")

from driver import BravoDriver  # noqa
from protocol import DeviceID, Packet, PacketID  # noqa
from utils import FileLogger, init_logger, transform_forces  # noqa


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
        serial_chain: Any,
        joint_limits: np.ndarray = np.array(
            [0.0873, 0.0873, 0.0873, 0.0873, 0.0873, 0.0873, 0.01]
        ),
        deadband: tuple[float, float] = (-0.07, 0.07),
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
            joint_limits: The maximum velocity of each joint.
            deadband: Clamp all velocity commands in this range to zero.
        """
        self._bravo = BravoDriver()
        self._running = False
        self.logger = init_logger("AdmittanceController")
        self._file_logger = FileLogger()

        self.Md = Md  # Mass matrix
        self.Kp = Kp  # Proportional gain
        self.Kd = Kd  # Derivative gain
        self.Kf = Kf  # Force gain
        self.B = B  # Selection matrix
        self.fd = fd  # Desired forces
        self.xd = xd  # Desired end-effector pose
        self.serial_chain = serial_chain
        self.joint_limits = joint_limits
        self.deadband = deadband

        # Robot state
        self.joint_positions = np.array([0.0] * 7)
        self.joint_velocities = np.array([0.0] * 7)
        self.forces = np.array([0.0] * 6)
        self.forces_buffer: deque[np.ndarray] = deque(maxlen=20)

        # Attach the packet callbacks
        self._bravo.attach_callback(PacketID.ATI_FT_READING, self._read_ft_readings_cb)
        self._bravo.attach_callback(PacketID.VELOCITY, self._read_joint_velocities_cb)
        self._bravo.attach_callback(PacketID.POSITION, self._read_joint_positions_cb)

        # Request the selected packets from the Bravo 7 devices
        self.request_readings_t = threading.Thread(
            target=self._request_packets,
            args=(
                {
                    PacketID.ATI_FT_READING: DeviceID.FORCE_TORQUE_SENSOR,
                    PacketID.POSITION: DeviceID.ALL_JOINTS,
                    PacketID.VELOCITY: DeviceID.ALL_JOINTS,
                },
            ),
        )
        self.request_readings_t.daemon = True

        # Run the controller in it's own thread
        self.controller_t = threading.Thread(target=self._run_controller)
        self.controller_t.daemon = True

        atexit.register(self.disable)

    def enable(self) -> None:
        """Enable admittance control."""
        self._running = True
        self._bravo.connect()
        self.request_readings_t.start()
        self.controller_t.start()

        self.logger.warning("Admittance control has been enabled.")

        # Tare the FT sensor on startup
        self.tare_ft_sensor()

    def disable(self) -> None:
        """Disable admittance control."""
        self._running = False
        self._bravo.disconnect()
        self.controller_t.join()
        self.request_readings_t.join()

        self.logger.warning("Admittance control has been disabled.")

    def _request_packets(self, packets: dict[PacketID, DeviceID]) -> None:
        """Request packets from the Bravo 7.

        Args:
            packets: A dictionary with the packets to request from the Bravo.
        """
        while self._running:
            for packet_id, device_id in packets.items():
                request = Packet(device_id, PacketID.REQUEST, bytes([packet_id.value]))
                self._bravo.send(request)

                # Delay a bit between packet requests
                time.sleep(0.01)

    def _read_ft_readings_cb(self, packet: Packet) -> None:
        """Read the force-torque sensor readings.

        Args:
            packet: The force-torque sensor reading packet.
        """
        # NOTE: These are saved in the FT-sensor frame
        self.forces = np.array(struct.unpack("<ffffff", packet.data))

    def _read_joint_positions_cb(self, packet: Packet) -> None:
        """Read the joint positions from the Bravo 7.

        Args:
            packet: The joint position packet.
        """
        position: float = struct.unpack("<f", packet.data)[0]

        # The jaws are a linear joint; convert from mm to m
        if packet.device_id == DeviceID.LINEAR_JAWS:
            position *= 0.001

        # Save the joint positions at the same index as their ID
        self.joint_positions[packet.device_id.value - 1] = position

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
        self.joint_velocities[packet.device_id.value - 1] = velocity

    def tare_ft_sensor(self) -> None:
        """Tare the Bravo 7 force-torque sensor."""
        self.logger.info("Taring the force-torque sensor readings.")
        tare = Packet(
            DeviceID.FORCE_TORQUE_SENSOR,
            PacketID.ATI_FT_READING,
            struct.pack(">ffffff", 0, 0, 0, 0, 0, 0),
        )
        self._bravo.send(tare)

    def _run_controller(self) -> None:
        """Run the parallel admittance/pose controller."""
        while self._running:
            # Calculate the Jacobian
            jc = self.serial_chain.jacobian(self.joint_positions[::-1])

            # Get the current end-effector pose in the base frame as a numpy array
            xe_transform = self.serial_chain.forward_kinematics(
                self.joint_positions[::-1]
            )
            xe_rot = R.from_quat(xe_transform.rot)
            xe = np.array([*xe_transform.pos, *xe_rot.as_euler("xyz")])

            # Calculate the end effector velocity in the base frame
            ve = jc @ np.array(self.joint_velocities[::-1])

            # Transform the FT-sensor reading from the sensor frame to the EE frame
            forces = transform_forces(
                self.forces,
                np.array([0, 0, 0.041275]),
                R.from_euler(
                    "xyz",
                    [0] * 3,  # The force transformation is just a translation
                ),
            )

            # Apply a running average filter over the force readings
            self.forces_buffer.append(forces)
            forces_buffer_arr = np.array([reading for reading in self.forces_buffer])
            forces_filtered = np.cumsum(forces_buffer_arr, axis=1)[0] / len(
                self.forces_buffer
            )

            # Get the desired velocities given the reference position and force
            vd = (
                np.linalg.pinv(jc)
                @ np.linalg.pinv(self.Md)
                @ (
                    -self.Kd @ ve
                    + self.Kp
                    @ (self.xd - xe + self.Kf @ (self.fd - self.B @ forces_filtered))
                )
            )

            # Clamp the joint velocities to the joint limits
            vd = np.array(
                [
                    max(-self.joint_limits[i], min(self.joint_limits[i], vel))
                    for i, vel in enumerate(vd)
                ]
            )

            # Create a deadband zone for the Bravo to help reduce chatter
            vd = np.array(
                [
                    0 if self.deadband[0] <= vel <= self.deadband[1] else vel
                    for vel in vd
                ]
            )

            self._file_logger(
                time.time(),
                self.joint_positions.round(3),
                self.joint_velocities.round(3),
                forces.round(3),
                vd,
            )

            # We need to send the velocity to each joint individually. The arm doesn't
            # like when we send them using the ALL_JOINTS DeviceID.
            for i, vel in enumerate(vd[::-1]):
                self._bravo.send(
                    Packet(DeviceID(i + 1), PacketID.VELOCITY, struct.pack(">f", vel))
                )


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
        Md = np.array(config["Md"])
        Kp = np.array(config["Kp"])
        Kd = np.array(config["Kd"])
        Kf = np.array(config["Kf"])
        B = np.array(config["B"])
        fd = np.array(config["fd"])
        xd = np.array(config["xd"])
        joint_limits = np.array(config["joint_limits"])
        deadband: tuple[float, float] = tuple(config["deadband"])  # type: ignore

    # Get the serial chain using the Bravo 7 URDF
    serial_chain = kp.build_serial_chain_from_urdf(open(urdf_fp).read(), "ee_link")

    return AdmittanceController(
        Md,
        Kp,
        Kd,
        Kf,
        B,
        fd,
        xd,
        serial_chain,
        joint_limits=joint_limits,
        deadband=deadband,
    )


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
            time.sleep(0.01)
        except KeyboardInterrupt:
            controller.disable()
            exit()
