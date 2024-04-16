import numpy as np
import triad_openvr as vr
from devices.xarm6 import XArmControl
import time
import transforms3d as t3d
from vr_test import get_transforms
import matplotlib.pyplot as plt


def flush_controller_data(flush_count=50):
    pose_matrix = controller.get_pose_matrix()
    while flush_count > 0:
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        flush_count -= 1


def fetch_init_poses():
    global frame_1_to_2, init_ori_in_1, init_ori_in_2, init_controller_position, init_eef_pos

    flush_controller_data()
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms()

    pose_matrix = controller.get_pose_matrix()
    while pose_matrix is None:
        pose_matrix = controller.get_pose_matrix()

    init_controller_position = frame_1_to_2 @ pose_matrix[:3, 3]
    init_eef_pos = xarm.get_eef_position()


prev_pos = np.zeros(3)
error_sum = 0
pid_flush_count = 40
def get_eef_target_pos_ori(use_position_pid=False, smooth_pos=False, dummy_ori=False, dummy_pos=False, duration=None, rpy_mask=[1,1,1]):
    global prev_pos, error_sum, pid_flush_count

    pose_matrix = controller.get_pose_matrix()
    if pose_matrix is None:
        return None, None
    
    if dummy_ori:
        rpy = np.array([np.pi, 0, 0])
    else:
        curr_ori_in_1 = pose_matrix[:3, :3]
        curr_ori_in_2 = frame_1_to_2 @ curr_ori_in_1
        curr_ori_in_init =  init_ori_in_1.T @ curr_ori_in_1
        curr_ori_in_init_wrt_2 = init_ori_in_2 @ curr_ori_in_init @ init_ori_in_2.T
        rpy = list(t3d.euler.mat2euler(curr_ori_in_init_wrt_2))

    if not rpy_mask[0]:
        rpy[0] = np.pi
    if not rpy_mask[1]:
        rpy[1] = 0
    if not rpy_mask[2]:
        rpy[2] = 0

    if dummy_pos:
        rel_pos = init_eef_pos + np.array([duration/50, 0, 0])
    else:
        curr_pos_in_1 = pose_matrix[:3, 3]
        curr_pos_in_2 = frame_1_to_2 @ curr_pos_in_1
    
        curr_pos = curr_pos_in_2

        alpha = 0.92
        if smooth_pos:
            # Exponential moving average to smooth out VR controller's occasional jerky noise
            curr_pos = prev_pos * alpha + (1 - alpha) * curr_pos
            prev_pos = curr_pos

        Kp =  0.1
        Ki =  0.4 * 16
        if use_position_pid:
            curr_error = curr_pos - prev_pos
            error_sum += curr_error
            error_I = error_sum * (1/50)
            PID_position = Kp * curr_error + Ki * error_I
            
            prev_pos = PID_position

            if pid_flush_count == 0:
                curr_pos = PID_position
            else:
                if pid_flush_count == 1:
                    print("\nPID Flush: Okay, PID flush complete\n")
                else:
                    print("PID Flush: Don't move too fast yet!")
                pid_flush_count -= 1 
        
        rel_pos = init_eef_pos + (curr_pos - init_controller_position)
    
    return rel_pos, rpy


def recover_from_failure(start_pos, end_pos, maintain_rpy, use_position_pid):
    # start_point = np.concatenate([start_pos*1000, maintain_rpy])
    # end_point = np.concatenate([end_pos*1000, maintain_rpy])
    # code = xarm.arm.move_arc_lines([start_point, end_point], is_radian=True, wait=True)

    print(f"Recovering from failure! Hold")
    recovery_motion_size = 0.005
    step_size = recovery_motion_size/np.linalg.norm(start_pos - end_pos) 
    for t in np.arange(0, 1+step_size, step_size):
        loop_start_time = time.time()

        interm_pos = (1-t) * start_pos + t * end_pos
        code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([interm_pos*1000, maintain_rpy]), input_is_radian=True, return_is_radian=True)
        if code != 0:
            continue
        
        # curr_jangs = 0.5*np.array(curr_jangs[:-1] + [0])
        # code = xarm.arm.set_servo_angle(angle=curr_jangs, wait=True, timeout=xarm_action_duration)
        code = xarm.arm.set_servo_angle_j(angles=curr_jangs, is_radian=True)
        controller.trigger_haptic_pulse()

        loop_duration = time.time()-loop_start_time
        # print(f"Loop duration: {loop_duration}")
        if loop_duration < xarm_action_duration:
            time.sleep(xarm_action_duration - loop_duration)

    rel_pos = None
    while rel_pos is None:
        rel_pos, rpy = get_eef_target_pos_ori(dummy_ori=False, rpy_mask=[0,0,1], use_position_pid=use_position_pid)
    if np.linalg.norm(rel_pos - end_pos) > 0.01:
        recover_from_failure(end_pos, rel_pos, rpy, use_position_pid)


def robot_control_xarmapi(control_mode="joint_vel", use_position_pid=True):
    assert control_mode in ["eef_pos", "eef_pose_vel", "joint_vel", "joint_ang"]
    global prev_pos, xarm_action_duration

    xarm_speed_lims = np.array(xarm.arm.get_ft_sensor_config()[1][21])*0.9

    start_time = time.time()
    duration = 0 
    prev_posori = np.zeros(6)
    prev_jangs = np.zeros(6)
    first = True

    if control_mode in ["eef_pos", "eef_pose_vel"]:
        xarm_action_duration = 0.3
    elif control_mode in ["joint_vel"]:
        xarm_action_duration = 0.2
    elif control_mode in ["joint_ang"]:
        xarm_action_duration = 0.05

    trajectory = []
    gripper_close_times = []
    gripper_open_times = []
    loop_count = 0
    recover_ikfail_from_pos = None
    recover_oob_from_pos = None
    gripper_closed = False
    rel_pos = None
    
    fetch_init_poses()
    # prev_pos = init_eef_pos
    # while duration < 10:
    while True:
        loop_start_time = time.time()

        if not rel_pos is None:
            prev_valid_pos = rel_pos
        rel_pos, rpy = get_eef_target_pos_ori(dummy_ori=False, rpy_mask=[0,0,1], use_position_pid=use_position_pid)
        if rel_pos is None:
            controller.trigger_haptic_pulse()
            recover_oob_from_pos = prev_valid_pos
            continue
        if recover_oob_from_pos is not None and np.linalg.norm(rel_pos-recover_oob_from_pos) > 0.01:
            # recover_from_failure(recover_oob_from_pos, rel_pos, rpy, use_position_pid)
            # print("Recovered from Out of Bounds failure")
            recover_oob_from_pos = None
            print("\nController traveled too far while out of bounds for detection. Exiting to avoid potentially unsafe situation")
            break

        trajectory.append(rel_pos)

        # Setting eef pose with different control modes 
        if control_mode == "eef_pos":
            xarm.set_eef_position(*rel_pos)

        elif control_mode == "eef_pose":
            xarm.set_eef_pose(rel_pos[0], rel_pos[1], rel_pos[2])

        elif control_mode == "eef_pose_vel":
            curr_posori = np.concatenate([rel_pos*1000, rpy])
            curr_vel =  curr_posori - prev_posori
            prev_posori = curr_posori
            if first:
                first = False
                continue
            curr_vel = curr_vel.clip(-xarm_speed_lims, xarm_speed_lims)
            print(f"Setting {curr_vel}")
            code = xarm.arm.vc_set_cartesian_velocity(curr_vel, duration=xarm_action_duration, is_tool_coord=True)
            if code != 0:
                print(f"ERROR executing action, code: {code}")

        elif control_mode == "joint_vel":
            code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([rel_pos*1000, rpy]), input_is_radian=True, return_is_radian=True)
            if code != 0:
                print(f"IK Failed, skipping")
                continue
            curr_jangs = np.array(curr_jangs)
            curr_jvels = (curr_jangs[:6] - prev_jangs[:6]) * 1.5
            prev_jangs = curr_jangs
            if first:
                first = False
                continue
            code = xarm.set_joint_velocity(curr_jvels, duration=xarm_action_duration)
        
        elif control_mode == "joint_ang":
            code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([rel_pos*1000, rpy]), input_is_radian=True, return_is_radian=True)
            if code != 0:
                print(f"IK Failed, skipping")
                controller.trigger_haptic_pulse()
                recover_ikfail_from_pos = rel_pos
                continue

            if recover_ikfail_from_pos is not None:
                recover_from_failure(recover_ikfail_from_pos, rel_pos, rpy, use_position_pid)
                print("Recovered from IK failure")
                recover_ikfail_from_pos = None
                continue

            # curr_jangs = 0.5*np.array(curr_jangs)
            # curr_jangs = 0.5*np.array(curr_jangs[:-1] + [0])
            # code = xarm.arm.set_servo_angle(angle=curr_jangs, wait=True, timeout=xarm_action_duration)
            code = xarm.arm.set_servo_angle_j(angles=curr_jangs, is_radian=True)
            if code != 0:
                print(f"Execution failed, error code {code}")
                if code == 1:
                    break
                continue

        # Gripper action using trigger button
        controller_inputs = controller.get_controller_inputs()
        if controller_inputs['trigger'] > 0 and not gripper_closed:
            gripper_close_times.append(loop_count)
            xarm.close_gripper()
            gripper_closed = True
            print(f"Closing gripper")
        if controller_inputs['trigger'] == 0 and gripper_closed:
            gripper_open_times.append(loop_count)
            xarm.open_gripper()
            gripper_closed = False
            print(f"Opening gripper")
        loop_count += 1
        
        if controller_inputs["menu_button"]:
            break

        loop_duration = time.time()-loop_start_time
        # print(f"Loop duration: {loop_duration}")
        if loop_duration < xarm_action_duration:
            time.sleep(xarm_action_duration - loop_duration)
        duration += xarm_action_duration
    xarm.reset()
    xarm.close()

    plt.figure("VR/EEF Trajectory")
    plt.plot(trajectory, label=['x','y','z'])
    plt.vlines(gripper_close_times, -0.5, 0.5, 'r', linestyles='dashed', label="Close gripper triggered")
    plt.vlines(gripper_open_times, -0.5, 0.5, 'g', linestyles='dashed', label="Open gripper triggered")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    xarm_control_mode = "joint_ang"
    simulated = False

    if xarm_control_mode == "eef_pos":
        mode = 0
    elif xarm_control_mode == "joint_ang":
        mode = 1
    elif xarm_control_mode == "joint_vel":
        mode = 4
        simulated = True
    elif xarm_control_mode == "eef_pose_vel":
        mode = 5
        simulated = True

    v = vr.triad_openvr()
    controller = v.devices["controller_1"]
    xarm = XArmControl(
        ip="192.168.1.242", 
        mode=mode,
        simulated=simulated,
        tcp_z_offset=145
    )
  
    robot_control_xarmapi(control_mode=xarm_control_mode, use_position_pid=True)
    # test_robot_IK()
