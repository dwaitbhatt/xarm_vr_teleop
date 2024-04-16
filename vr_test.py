import triad_openvr as vr
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import transforms3d as t3d
import sys

v = vr.triad_openvr()
controller = v.devices["controller_1"]


def get_3d_traj_fig_axs_lines(plot_name="Controller Trajectory", figsize=(6,6)):
    # Trajectory plots
    fig_traj = plt.figure(plot_name, figsize=figsize)
    ax = fig_traj.add_subplot(111, projection='3d')
    ax.view_init(elev=0, azim=0)  # Elevation = 0, Azimuth = -90

    # Initialize the trajectory line
    trajectory, = ax.plot([], [], [], color='b', alpha=0.3, label="Trajectory")
    smooth_traj, = ax.plot([], [], [], color='r', label="Smoothened Trajectory")   # Running average to smooth out jerks
    fig_traj.legend()

    return fig_traj, ax, (trajectory, smooth_traj)


def get_2d_fig_axs_lines(plot_vel=False):
    # Position and velocity plots
    fig_2d = plt.figure("Controller Position and Velocity", figsize=(10,8))
    fig2d_manager = plt.get_current_fig_manager()
    fig2d_manager.window.wm_geometry("+900+0")

    ncols = 1
    if plot_vel:
        ncols = 2
    gs = fig_2d.add_gridspec(3,ncols)
    
    ax_posx = fig_2d.add_subplot(gs[0,0])
    ax_posx.set_title('Position in X')
    ax_posx.set_ylabel('X Position')

    ax_posy = fig_2d.add_subplot(gs[1,0])
    ax_posy.set_title('Position in Y')
    ax_posy.set_ylabel('Y Position')

    ax_posz = fig_2d.add_subplot(gs[2,0])
    ax_posz.set_title('Position in Z')
    ax_posz.set_ylabel('Z Position')
    ax_posz.set_xlabel('Time')

    # Initialize position lines
    pos_x, = ax_posx.plot([], [], color='r', alpha=0.6)
    pos_y, = ax_posy.plot([], [], color='g', alpha=0.6)
    pos_z, = ax_posz.plot([], [], color='b', alpha=0.6)
    smooth_pos_x, = ax_posx.plot([], [], color='r')
    smooth_pos_y, = ax_posy.plot([], [], color='g')
    smooth_pos_z, = ax_posz.plot([], [], color='b')

    ret_list = [fig_2d, 
                (ax_posx, ax_posy, ax_posz), 
                (pos_x, pos_y, pos_z, smooth_pos_x, smooth_pos_y, smooth_pos_z),
                None, None]

    if plot_vel:
        ax_velx = fig_2d.add_subplot(gs[0,1])
        ax_velx.set_title('Velocity in X')
        ax_velx.set_ylabel('X Velocity')

        ax_vely = fig_2d.add_subplot(gs[1,1])
        ax_vely.set_title('Velocity in Y')
        ax_vely.set_ylabel('Y Velocity')

        ax_velz = fig_2d.add_subplot(gs[2,1])
        ax_velz.set_title('Velocity in Z')
        ax_velz.set_ylabel('Z Velocity')
        ax_velz.set_xlabel('Time')

        # Initialize velocity lines
        vel_x, = ax_velx.plot([], [], color='r', alpha=0.6)
        vel_y, = ax_vely.plot([], [], color='g', alpha=0.6)
        vel_z, = ax_velz.plot([], [], color='b', alpha=0.6)
        smooth_vel_x, = ax_velx.plot([], [], color='r')
        smooth_vel_y, = ax_vely.plot([], [], color='g')
        smooth_vel_z, = ax_velz.plot([], [], color='b')

        ret_list[-2:] = [(ax_velx, ax_vely, ax_velz),
                         (vel_x, vel_y, vel_z, smooth_vel_x, smooth_vel_y, smooth_vel_z)]

    return ret_list


def update_2d_plots(normal_points, smoothened_points, time_steps, axs, lines):
    ax_x, ax_y, ax_z = axs
    line_x, line_y, line_z, smooth_line_x, smooth_line_y, smooth_line_z = lines

    xs, ys, zs = zip(*normal_points)
    line_x.set_data(time_steps, xs)
    line_y.set_data(time_steps, ys)
    line_z.set_data(time_steps, zs)
    
    xs_vs, ys_vs, zs_vs = zip(*smoothened_points)
    smooth_line_x.set_data(time_steps, xs_vs)
    smooth_line_y.set_data(time_steps, ys_vs)
    smooth_line_z.set_data(time_steps, zs_vs)

    ax_x.relim()
    ax_x.autoscale_view()
    ax_y.relim()
    ax_y.autoscale_view()
    ax_z.relim()
    ax_z.autoscale_view()


def update_3d_plot(normal_points, smoothened_points, ax, lines, consider_frame_arrows=False):
    line, smooth_line = lines

    xs, ys, zs = zip(*normal_points)
    line.set_data(xs, ys)
    line.set_3d_properties(zs)

    xs_s, ys_s, zs_s = zip(*smoothened_points)
    smooth_line.set_data(xs_s, ys_s)
    smooth_line.set_3d_properties(zs_s)

    # Dynamically update trajectory axis limits
    comb_xs = xs + xs_s
    comb_ys = ys + ys_s
    comb_zs = zs + zs_s
    max_range = max(max(comb_xs) - min(comb_xs), max(comb_ys) - min(comb_ys), max(comb_zs) - min(comb_zs)) / 2
    if consider_frame_arrows:
        max_range = max(0.1, max_range + 0.1)
    mid_x = (max(comb_xs) + min(comb_xs)) / 2
    mid_y = (max(comb_ys) + min(comb_ys)) / 2
    mid_z = (max(comb_zs) + min(comb_zs)) / 2
    ax.set_xlim([mid_x - max_range, mid_x + max_range])
    ax.set_ylim([mid_y - max_range, mid_y + max_range])
    ax.set_zlim([mid_z - max_range, mid_z + max_range])


def visualize_trajectory(plot_2d=True, plot_3d=False, plot_vel=False):
    if not (plot_3d or plot_2d):
        return

    if plot_2d and (plot_vel or plot_3d):
        freq = 10 # Hz
    elif plot_2d:
        freq = 15 # Hz
    else:
        freq = 50 # Hz

    if plot_3d:
        fig_traj, ax_3d, lines_3d = get_3d_traj_fig_axs_lines()
    if plot_2d:
        fig_2d, pos_axs, pos_lines, vel_axs, vel_lines = get_2d_fig_axs_lines(plot_vel)
        
    # List to store the trajectory and velocities
    trajectory_points = []
    smooth_traj_points = []
    PID_pos_points = []
    velocity_points = [np.zeros(3)]
    smooth_vel_points = [np.zeros(3)]
    
    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms(vive_base_direction="perpendicular")

    # Get the initial position
    initial_pose = controller.get_pose_matrix()
    while initial_pose is None:
        initial_pose = controller.get_pose_matrix()

    initial_position = frame_1_to_2 @ initial_pose[:3, 3]
    trajectory_points.append(initial_position)
    smooth_traj_points.append(initial_position)
    PID_pos_points.append(initial_position)
    last_position = initial_position

    if plot_3d:
        # Set initial axis limits
        axis_range = 1.0  # Initial range for each axis
        ax_3d.set_xlim([initial_position[0] - axis_range, initial_position[0] + axis_range])
        ax_3d.set_ylim([initial_position[1] - axis_range, initial_position[1] + axis_range])
        ax_3d.set_zlim([initial_position[2] - axis_range, initial_position[2] + axis_range])

    alpha_traj = 0.92
    alpha_vel = 0.8
    time_step = 0
    time_steps = [time_step]
    time.sleep(1/freq)
    elapsed_time = 1/freq
    
    smooth_vel_scaling = 2
    error_sum = 0
    prev_error = 0

    # Found experimentally as per Ziegler–Nichols method
    Ku = 1
    Tu = 1/16

    Kp =  0.1 * Ku     # 0.6 * Ku      # 0.45 * Ku      
    Ki =  0.4 * Ku/Tu  # 1.2 * Ku/Tu   # 0.54 * Ku/Tu   
    Kd =  0             # 3 * Ku*Tu/40  # 0              

    while True:
        start_time = time.time()

        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue

        current_position = frame_1_to_2 @ pose_matrix[:3, 3]
        current_velocity = (current_position - last_position) / (1/freq)
        EMA_position = alpha_traj * smooth_traj_points[-1] + (1-alpha_traj) * current_position
        EMA_velocity = alpha_vel * smooth_vel_points[-1] + (1-alpha_vel) * current_velocity

        # Older tracking approach using smooth vel
        # vel_EMA_implied_pos = smooth_vel_implied_pos_points[-1] + smooth_vel_scaling * EMA_velocity * (1/freq)

        # Tracking position using PID
        curr_error = current_position - PID_pos_points[-1]
        error_sum += curr_error
        error_I = error_sum * (1/freq)
        error_D = (curr_error - prev_error) / (1/freq)
        
        PID_position = Kp * curr_error + Ki * error_I + Kd * error_D
        prev_error = curr_error

        trajectory_points.append(current_position)
        smooth_traj_points.append(EMA_position)
        velocity_points.append(current_velocity)
        smooth_vel_points.append(EMA_velocity)
        PID_pos_points.append(PID_position)
        last_position = current_position

        time_step += max(elapsed_time, 1/freq)
        time_steps.append(time_step)

        if len(trajectory_points) > 100:
            trajectory_points.pop(0)
            smooth_traj_points.pop(0)
            velocity_points.pop(0)
            smooth_vel_points.pop(0)
            PID_pos_points.pop(0)
            time_steps.pop(0)


        if plot_2d:
            update_2d_plots(trajectory_points, PID_pos_points, time_steps, pos_axs, pos_lines)
            if plot_vel:
                update_2d_plots(velocity_points, np.array(smooth_vel_points)*smooth_vel_scaling, time_steps, vel_axs, vel_lines)

            plt.figure(fig_2d)
            plt.draw()
            plt.pause(0.001)  # A short pause to allow for plot updates

        if plot_3d:    
            # update_3d_plot(trajectory_points, smooth_traj_points, ax_3d, lines_3d)
            update_3d_plot(trajectory_points, PID_pos_points, ax_3d, lines_3d)
 
            plt.figure(fig_traj)
            plt.draw()
            plt.pause(0.001)  # A short pause to allow for plot updates

        # Wait to maintain approximately refresh rate of <freq> Hz
        elapsed_time = time.time() - start_time
        print(f"Plot update max freq: {1/elapsed_time} Hz")
        time.sleep(max(1/freq - elapsed_time, 0))


def get_transforms(vive_base_direction="perpendicular"):
    assert vive_base_direction in ["perpendicular", "parallel"]
    
    # The physical controller rotations are made wrt frame 2 (robot base frame). While the pose is returned in frame 1 (Vive Base station frame).
    if vive_base_direction == "parallel":
        # Change axes from frame 1 (z towards base station and x left) to frame 2 (x away from base station and z up)
        frame_1_to_2 = np.array([[0,  0,  -1], 
                                [-1,  0,  0], 
                                [0,  1,  0]])
        init_ori_in_1 = np.diag([-1, -1, 1])
    elif vive_base_direction == "perpendicular":
        # Change axes from frame 1 (z towards base station and x left) to frame 2 (x left and z up)
        frame_1_to_2 = np.array([[1,  0,  0], 
                                 [0,  0,  -1], 
                                 [0,  1,  0]])
        init_ori_in_1 = np.array([[0, 0, -1],
                                  [0, -1, 0],
                                  [-1, 0, 0]])

    # init_ori_in_1 = np.eye(3)
    init_ori_in_2 = frame_1_to_2 @ init_ori_in_1
    return frame_1_to_2, init_ori_in_1, init_ori_in_2


def visualize_orientation():
    fig = plt.figure("Controller Orientation", figsize=(15,8))
    ax = fig.add_subplot(111, projection='3d')

    # Set the initial view angle
    ax.view_init(elev=10, azim=20)  # Elevation = 0, Azimuth = 180

    # Initialize arrows for x, y, and z axes at the origin
    origin = [0, 0, 0]

    R = np.eye(3)
    fixed_arrow_x = ax.quiver(*origin, *R[:, 0], color='r', length=1.0, linestyle='dashed')
    fixed_arrow_y = ax.quiver(*origin, *R[:, 1], color='g', length=1.0, linestyle='dashed')
    fixed_arrow_z = ax.quiver(*origin, *R[:, 2], color='b', length=1.0, linestyle='dashed')

    arrow_x = ax.quiver(*origin, *R[:, 0], color='r', length=1.0)
    arrow_y = ax.quiver(*origin, *R[:, 1], color='g', length=1.0)
    arrow_z = ax.quiver(*origin, *R[:, 2], color='b', length=1.0)
    
    text_display = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

    # Set axis limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    plt.draw()
    plt.pause(0.001)  # A short pause to allow for plot updates

    pose_matrix = controller.get_pose_matrix()
    while pose_matrix is None:
        pose_matrix = controller.get_pose_matrix()

    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms(vive_base_direction="perpendicular")

    while True:
        start_time = time.time()

        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        
        curr_ori_in_1 = pose_matrix[:3, :3]
        curr_ori_in_2 = frame_1_to_2 @ curr_ori_in_1
        curr_ori_in_init =  init_ori_in_1.T @ curr_ori_in_1

        curr_ori_in_init_wrt_2 = init_ori_in_2 @ curr_ori_in_init @ init_ori_in_2.T

        plot_ori = curr_ori_in_init_wrt_2
        r, p, y = np.array(t3d.euler.mat2euler(plot_ori)) * 180/np.pi
        
        # Update arrows' orientations
        arrow_x.remove()
        arrow_y.remove()
        arrow_z.remove()
        arrow_x = ax.quiver(*origin, *plot_ori[:, 0], color='r', length=1.0)
        arrow_y = ax.quiver(*origin, *plot_ori[:, 1], color='g', length=1.0)
        arrow_z = ax.quiver(*origin, *plot_ori[:, 2], color='b', length=1.0)

        text_display.set_text(f"Roll: {r:.2f}, Pitch: {p:.2f}, Yaw: {y:.2f}")

        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # A short pause to allow for plot updates

        # Wait to maintain approximately 60Hz refresh rate
        elapsed_time = time.time() - start_time
        time.sleep(max(1/60 - elapsed_time, 0))


def visualize_pose():
    # Get fig and lines for trajectory
    fig_traj, ax_3d, lines_3d = get_3d_traj_fig_axs_lines(plot_name="Controller Pose", figsize=(15,8))
    
    # Set the initial view angle
    ax_3d.view_init(elev=10, azim=20)  # Elevation = 0, Azimuth = 180

    initial_pose = controller.get_pose_matrix()
    while initial_pose is None:
        initial_pose = controller.get_pose_matrix()

    frame_1_to_2, init_ori_in_1, init_ori_in_2 = get_transforms(vive_base_direction="perpendicular")
    initial_position = frame_1_to_2 @ initial_pose[:3, 3]

    relative_trajectory_points = []
    relative_PID_pos_points = []
    rel_pos = np.zeros(3)    
    relative_trajectory_points.append(rel_pos)
    relative_PID_pos_points.append(rel_pos)

    identity = np.eye(3)
    fixed_arrow_x = ax_3d.quiver(*rel_pos, *identity[:, 0], color='r', length=0.15, linestyle='dashed')
    fixed_arrow_y = ax_3d.quiver(*rel_pos, *identity[:, 1], color='g', length=0.15, linestyle='dashed')
    fixed_arrow_z = ax_3d.quiver(*rel_pos, *identity[:, 2], color='b', length=0.15, linestyle='dashed')

    arrow_x = ax_3d.quiver(*rel_pos, *identity[:, 0], color='r', length=0.15)
    arrow_y = ax_3d.quiver(*rel_pos, *identity[:, 1], color='g', length=0.15)
    arrow_z = ax_3d.quiver(*rel_pos, *identity[:, 2], color='b', length=0.15)
    
    text_display_rpy = ax_3d.text2D(0.05, 0.95, "", transform=ax_3d.transAxes)
    text_display_pos = ax_3d.text2D(0.05, 0.85, "", transform=ax_3d.transAxes)

    # Set axis limits
    ax_3d.set_xlim([-1, 1])
    ax_3d.set_ylim([-1, 1])
    ax_3d.set_zlim([-1, 1])

    plt.draw()
    plt.pause(0.001)  # A short pause to allow for plot updates

    Kp =  0.1
    Ki =  0.4 * 16
    error_sum = 0
    while True:
        start_time = time.time()

        pose_matrix = controller.get_pose_matrix()
        if pose_matrix is None:
            continue
        
        curr_ori_in_1 = pose_matrix[:3, :3]
        curr_ori_in_2 = frame_1_to_2 @ curr_ori_in_1
        curr_ori_in_init =  init_ori_in_1.T @ curr_ori_in_1
        curr_ori_in_init_wrt_2 = init_ori_in_2 @ curr_ori_in_init @ init_ori_in_2.T

        plot_ori = curr_ori_in_init_wrt_2
        r, p, y = np.array(t3d.euler.mat2euler(plot_ori)) * 180/np.pi

        current_position = frame_1_to_2 @ pose_matrix[:3, 3]
        rel_pos = current_position - initial_position
        
        # Tracking position using PID
        curr_error = rel_pos - relative_PID_pos_points[-1]
        error_sum += curr_error
        error_I = error_sum * (1/50)
        rel_PID_pos = Kp * curr_error + Ki * error_I
        
        print(np.linalg.norm(rel_pos - relative_trajectory_points[-1]))
        relative_trajectory_points.append(rel_pos)
        relative_PID_pos_points.append(rel_PID_pos)        
        if len(relative_trajectory_points) > 100:
            relative_trajectory_points.pop(0)
            relative_PID_pos_points.pop(0)

        # Update arrows' orientations
        arrow_x.remove()
        arrow_y.remove()
        arrow_z.remove()
        fixed_arrow_x.remove()
        fixed_arrow_y.remove()
        fixed_arrow_z.remove()
        arrow_x = ax_3d.quiver(*rel_pos, *plot_ori[:, 0], color='r', length=0.15)
        arrow_y = ax_3d.quiver(*rel_pos, *plot_ori[:, 1], color='g', length=0.15)
        arrow_z = ax_3d.quiver(*rel_pos, *plot_ori[:, 2], color='b', length=0.15)
        fixed_arrow_x = ax_3d.quiver(*rel_pos, *identity[:, 0], color='r', length=0.15, linestyle='dashed')
        fixed_arrow_y = ax_3d.quiver(*rel_pos, *identity[:, 1], color='g', length=0.15, linestyle='dashed')
        fixed_arrow_z = ax_3d.quiver(*rel_pos, *identity[:, 2], color='b', length=0.15, linestyle='dashed')

        text_display_rpy.set_text(f"Roll: {r:.2f}, Pitch: {p:.2f}, Yaw: {y:.2f}")
        text_display_pos.set_text(f"X:{rel_pos[0]}, Y:{rel_pos[1]}, Z:{rel_pos[2]}")
        
        update_3d_plot(relative_trajectory_points, relative_PID_pos_points, ax_3d, lines_3d, consider_frame_arrows=True)
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # A short pause to allow for plot updates

        if controller.get_controller_inputs()["menu_button"]:
            break
        
        # Wait to maintain approximately 60Hz refresh rate
        elapsed_time = time.time() - start_time
        time.sleep(max(1/60 - elapsed_time, 0))


def test_controller_buttons():
    v = vr.triad_openvr()
    controller = v.devices["controller_1"]

    tests = ["Press grip button",
             "Touch trackpad",
             "Press trackpad",
             "Trackpad directions",
             "Press menu button",
             "Press layer(?) button"]

    hz = 20
    seconds_per_test = 4
    for test in tests:
        print(f"\n\n{test}")
        history = []
        for i in range(hz*seconds_per_test):
            _, controller_state = controller.vr.getControllerState(controller.index)
            history.append(bin(controller_state.ulButtonPressed))
            time.sleep(1/hz)
            # print(f"state.ulButtonPressed: {bin(controller_state.ulButtonPressed)}")
        print(f"Last entry for \"{test}\": {history[-1]}")
        plt.figure(test)
        plt.plot(history)
        plt.show()



if __name__ == "__main__":    
    get_transforms()

    test_type = "pose"
    if len(sys.argv) > 1:
        test_type = sys.argv[-1]

    if test_type in ["traj3d", "traj"]:
        print("\nRunning 3D trajectory visualization\n")
        visualize_trajectory(plot_3d=True, plot_2d=False)
    elif test_type == "traj2d":
        print("\nRunning position and velocity visualization\n")
        visualize_trajectory(plot_3d=False, plot_2d=True)
    elif test_type == "trajall":
        print("\nRunning 3D trajectory and position/velocity visualization\n")
        visualize_trajectory(plot_3d=True, plot_2d=True)
    elif test_type == "ori":
        print("\nRunning orientation visualization\n")
        visualize_orientation()        
    elif test_type == "pose":
        print("\nRunning pose visualization\n")
        visualize_pose()
    elif test_type == "buttons":
        print("\nRunning controller button tests\n")
        test_controller_buttons()