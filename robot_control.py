import numpy as np
import triad_openvr as vr
import time
import transforms3d as t3d
from vr_test import get_transforms
import matplotlib.pyplot as plt

# For custom wrapper over xArm6 Python API
# export PYTHONPATH=$PYTHONPATH:/home/erl-tianyu/dwait_local_repo/erl_xArm/
import sys
sys.path.append("/home/erl-tianyu/dwait_local_repo/erl_xArm/")
from devices.xarm6 import XArmControl


def flush_controller_data(flush_count=50):
    pose_matrix = controller.get_pose_matrix()
    while flush_count > 0:
        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        flush_count -= 1


def fetch_init_poses():
    global frame_1_to_2, init_ori_in_1, init_ori_in_2, init_controller_position, init_eef_pos, init_jangs

    flush_controller_data()
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms()

    pose_matrix = controller.get_pose_matrix()
    while pose_matrix is None:
        pose_matrix = controller.get_pose_matrix()

    init_controller_position = frame_1_to_2 @ pose_matrix[:3, 3]
    init_eef_pos = xarm.get_eef_position()
    
    code = 1
    while code != 0:
        code, init_jangs = xarm.arm.get_servo_angle(is_radian=True)


prev_rel_eef_pos = np.zeros(3)
rel_vr_pos_error_sum = 0
jang_error_sum = 0
pid_flush_count = 40
def get_eef_target_pos_ori(use_position_pid=False, ema_smooth_pos=False, use_jang_pid=False, dummy_ori=False, dummy_pos=False, duration=None, rpy_mask=[1,1,1]):
    global prev_rel_eef_pos, rel_vr_pos_error_sum, prev_actual_robot_jang, jang_error_sum, pid_flush_count

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
        rel_vr_pos = init_eef_pos + np.array([duration/control_freq, 0, 0])
    else:
        curr_pos_in_1 = pose_matrix[:3, 3]
        curr_pos_in_2 = frame_1_to_2 @ curr_pos_in_1
    
        curr_pos = curr_pos_in_2
        rel_vr_pos = curr_pos - init_controller_position

        alpha = 0.92
        if ema_smooth_pos:
            # Exponential moving average to smooth out VR controller's occasional jerky noise
            curr_pos = prev_pos * alpha + (1 - alpha) * curr_pos
            prev_pos = curr_pos

        Kp_pos =  0.1
        Ki_pos =  0.4 * 16
        if use_position_pid:
            # Relative VR position is the target relative eef position
            rel_vr_pos_error = rel_vr_pos - prev_rel_eef_pos
            rel_vr_pos_error_sum += rel_vr_pos_error
            error_I = rel_vr_pos_error_sum * (1/control_freq)
            PID_rel_vr_position = Kp_pos * rel_vr_pos_error + Ki_pos * error_I
            # prev_rel_eef_pos = PID_rel_vr_position

        # For safety, using 0.5 times the VR controller's motion for the target eef position.
        eef_target_pos_PID = init_eef_pos + PID_rel_vr_position * 0.5

        Kp_jang =  0.1
        Ki_jang =  0.4 * 16
        PID_jang = None
        jang_code = 0
        if use_jang_pid:    
            jang_code, target_jang = xarm.arm.get_inverse_kinematics(np.concatenate([eef_target_pos_PID*1000, rpy]), input_is_radian=True, return_is_radian=True)
            if jang_code == 0:
                curr_jang_error = target_jang - prev_actual_robot_jang
                jang_error_sum += curr_jang_error
                error_I = jang_error_sum * (1/control_freq)
                PID_jang = Kp_jang * curr_jang_error + Ki_jang * error_I
                
                # prev_actual_robot_jang = PID_jang
            else:
                print(f"IK failed, code: {jang_code}")
                    
    return eef_target_pos_PID, rpy, PID_jang, jang_code


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


def robot_control_xarmapi(control_mode="joint_vel", use_position_pid=True, use_jang_pid=False):
    assert control_mode in ["eef_pos", "eef_pose_vel", "joint_vel", "joint_ang"]
    global xarm_action_duration, prev_actual_robot_jang, prev_rel_eef_pos

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
    prev_actual_robot_jang = np.array(init_jangs)

    while True:
        loop_start_time = time.time()

        if not (rel_pos is None):
            prev_valid_pos = rel_pos
        rel_pos, rpy, PID_jang, jang_code = get_eef_target_pos_ori(dummy_ori=False, rpy_mask=[0,0,1], use_position_pid=use_position_pid, use_jang_pid=use_jang_pid)
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

        if use_jang_pid:
            curr_jangs = PID_jang
            code = jang_code
        else:
            code, curr_jangs = xarm.arm.get_inverse_kinematics(np.concatenate([rel_pos*1000, rpy]), input_is_radian=True, return_is_radian=True)
        
        if control_mode == "joint_vel":
            if code != 0:
                print(f"PID Calc Failed, skipping")
                continue
            curr_jangs = np.array(curr_jangs)
            curr_jvels = (curr_jangs[:6] - prev_jangs[:6]) * 1.5
            prev_jangs = curr_jangs
            if first:
                first = False
                continue
            code = xarm.set_joint_velocity(curr_jvels, duration=xarm_action_duration)
        
        elif control_mode == "joint_ang":
            if curr_jangs is None:
                print(f"")
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
            # print(f"\nSetting {np.array(curr_jangs)*180/np.pi}\n")
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

        prev_rel_eef_pos = xarm.get_eef_position() - init_eef_pos
        code = 1
        while code != 0 and use_jang_pid:
            code, prev_actual_robot_jang = xarm.arm.get_servo_angle(is_radian=True)
            prev_actual_robot_jang = np.array(prev_actual_robot_jang)

        loop_duration = time.time()-loop_start_time
        # print(f"Loop duration: {loop_duration} = {1/loop_duration} Hz")
        # if loop_duration < xarm_action_duration:
        #     time.sleep(xarm_action_duration - loop_duration)
        # duration += xarm_action_duration
        if loop_duration < 1/control_freq:
            time.sleep(1/control_freq - loop_duration)
        duration += 1/control_freq
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
    simulated = True
    control_freq = 50

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
