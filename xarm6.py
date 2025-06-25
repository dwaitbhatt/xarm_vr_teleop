import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from xarm.wrapper import XArmAPI


class XArmControl:
    """
    Wrapper class around XArmAPI to provide easy access to 
    end-effector/gripper setter and getter functions 
    Default is to use SI units: meters, radian, etc.

    TODO: Add joint velocity control functions
    """
    def __init__(self, ip=None, mode=0, simulated=False, tcp_z_offset=0, object_to_grip="green_can"):
        self.arm = XArmAPI(ip, is_radian=True)
        self.arm.set_simulation_robot(simulated)
        self.arm.motion_enable(enable=True)
        self.arm.set_tcp_offset([0, 0, tcp_z_offset, 0, 0, 0])

        _, self.init_angles =  self.arm.get_initial_point()
        self.init_angles = np.array(self.init_angles) / 180 * np.pi

        self.init_position = self.get_eef_position()

        self.mode = mode
        self.reset()
        
        # Setting gripper qpos limits for each object
        gripper_qpos_limits = {
            "red_cube_40mm": {"max": 850, "min": 385},
            "green_cube_40mm": {"max": 850, "min": 385},
            "green_cube_55mm": {"max": 850, "min": 500},
            "red_box_raspi": {"max": 850, "min": 230},
            "green_can": {"max": 850, "min": 625},
        }
        assert object_to_grip in gripper_qpos_limits.keys(), f"Object {object_to_grip} not supported"

        self.gripper_qpos_max = gripper_qpos_limits[object_to_grip]["max"]
        self.gripper_qpos_min = gripper_qpos_limits[object_to_grip]["min"]
        self.gripper_qpos = None
        self.arm.set_gripper_enable(True)           # Turn on gripper control
        self.open_gripper()

        assert self.arm.default_is_radian == True

    def close(self):
        self.reset()
        self.arm.disconnect()

    def reset(self, custom_init_position=None):
        """
            Reset arm back to zero position.
            NOTE: Resetting only works correctly in mode 0. Hence switching to mode 0, and back to self.mode once done.
        """
        # Reset to initial joints
        self.arm.set_mode(0)                        # 0: End-effector/Joint position control
        self.arm.set_state(state=0) 
        time.sleep(1)
        # self.arm.reset(wait=True)
        # TODO: Fix reset issue for non-zero initial position
        #       self.arm.reset goes to zero poition, which is outside safe boundary.
        #       Also we cannot set joint angles to initial, as it can lead to dangerous motion
        # self.set_eef_position(*self.init_position)
        if custom_init_position is not None:
            self.init_position = custom_init_position
        self.set_eef_pose(*self.init_position, np.pi, 0, 0, relative=False, speed=80)

        if self.mode != 0:
            # Set again required control mode
            self.arm.set_mode(self.mode)
            self.arm.set_state(0)
            time.sleep(1)

    def switch_mode(self, mode):
        self.arm.set_mode(mode)
        self.arm.set_state(state=0)
        time.sleep(1)
        self.mode = mode

    def set_yaw_and_open_gripper(self, angle):
        self.arm.set_position(yaw=angle, is_radian=True)
        self.arm.set_pause_time(2)
        self.set_gripper_qpos(600/10000)

    def set_eef_position(self, x, y, z):
        """
        Input cartesian positions x, y, z in meters, xArm uses mm by default
        """
        x, y, z = x*1000, y*1000, z*1000
        code = self.arm.set_position(x=x, y=y, z=z, speed=80, wait=True)

        if code != 0:
            raise ValueError(f"xArm error code: {code}")

    def set_eef_pose(self, x, y, z, roll, pitch, yaw, relative=True, speed=30):
        """
        Input cartesian positions x, y, z in meters, xArm uses mm by default
        """
        x, y, z = x*1000, y*1000, z*1000
        code = self.arm.set_position(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=speed, 
            is_radian=True, relative=relative, wait=True)

        if code != 0:
            raise ValueError(f"xArm error code: {code}")

    def get_eef_position(self):
        code, position = self.arm.get_position()
        if code == 0:
            x, y, z, _, _, _ = position
            return np.array([x, y, z]) / 1000.
        else:
            raise ValueError("xArm unable to read current end-effector position")

    def get_eef_quaternion(self):
        """
        Returns end-effector quaternion in [x, y, z, w] format
        TODO: Double check euler angle intrinsic/extrinsic
        """
        code, position = self.arm.get_position(is_radian=True)
        if code == 0:
            _, _, _, roll, pitch, yaw = position
            r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)

            quat = r.as_quat()
            if quat[3] < 0:
                quat = -quat
            return quat

    def get_qpos_qvel(self):
        # Only take the first 6 values for xArm6
        code, [qpos, qvel, _] = self.arm.get_joint_states(is_radian=True)
        if code == 0:
            return np.array(qpos[:6]), np.array(qvel[:6])
        else:
            raise ValueError("xArm unable to read current joint states")

    def get_gripper_qpos(self):
        """
            Fetch absolute gripper position in meters
        """
        code, pos = self.arm.get_gripper_position()
        if code == 0:
            return np.array([pos/10000])
        else:
            raise ValueError("xArm unable to read current gripper position")

    def set_gripper_qpos(self, qpos):
        """
            Set absolute gripper position with input qpos in meters, xArm uses mm/10 by default
        """
        # Setting gripper position stops arm motion. Only set if a new position is required.
        if self.gripper_qpos != qpos:
            code = self.arm.set_gripper_position(qpos * 10000, wait=False)
            if not (code in [0, None]):
                raise ValueError(f"xArm error code: {code}")
            self.gripper_qpos = qpos

    def open_gripper(self):
        self.set_gripper_qpos(self.gripper_qpos_max/10000)

    def close_gripper(self):
        self.set_gripper_qpos(self.gripper_qpos_min/10000)

    def is_gripper_closed(self):
        # Test if difference between gripper qpos and minimum is small
        is_closed = np.abs(self.get_gripper_qpos() - self.gripper_qpos_min/10000) < 50/10000
        # print("is_closed:", is_closed)
        return np.array([int(is_closed[0])])

    def get_gripper_pos(self):
        """
            Fetch gripper postion normalized to [-1, 1]
        """
        code, pos = self.arm.get_gripper_position()
        if code == 0:
            pos = np.clip(pos, self.gripper_qpos_min, self.gripper_qpos_max)
            pos = (pos - self.gripper_qpos_min) / (self.gripper_qpos_max - self.gripper_qpos_min) * 2 - 1
            return np.array([pos])
        else:
            raise ValueError("xArm unable to read current gripper position")

    def set_gripper_pos(self, action):
        """
            Set gripper action in [-1, 1]
            -1: open (qpos = gripper_qpos_max), 1: close (qpos = gripper_qpos_min)
        """
        assert -1 <= action <= 1
        gripper_qpos = (self.gripper_qpos_max - self.gripper_qpos_min) * (1 - action) / 2. \
            + self.gripper_qpos_min        
        code = self.set_gripper_qpos(gripper_qpos/10000)
        if not (code in [0, None]):
            raise ValueError(f"xArm error code: {code}")

    def set_servo_angle(self, angle=None, speed=None):
        code = self.arm.set_servo_angle(angle=angle, speed=speed, wait=True)
        if code != 0:
            raise ValueError(f"xArm error code: {code}")

    def set_joint_angles(self, jangles):
        """ 
            Note: jangles is in radians. 
               Also consider if this is safe to use, directly setting joint 
               angles to values far from current position could be dangerous.
        """
        assert len(jangles) == 7
        code = self.arm.set_servo_angle_j(jangles)
        if code != 0:
            raise ValueError(f"xArm error code: {code}")

    def set_joint_velocity(self, qvel, duration=0):

        assert len(qvel) == 6
        code = self.arm.vc_set_joint_velocity(
            list(qvel), is_radian=True, is_sync=True, duration=duration)

        # Sleep for duration for xarm to apply velocity control
        time.sleep(duration)
        if code != 0:
            raise ValueError(f"xArm error code: {code}")
