import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D # For 3D trajectories
import argparse

def load_position_error_graph(df):
    plt.figure(figsize=(12, 6))
    df['pos_error_smoothed'] = df['pos_error'].rolling(window=50).mean()
    plt.plot(df['timestamp'].to_numpy(), df['pos_error_smoothed'].to_numpy(), label='Total Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Factor Graph Error')
    plt.title('Factor Graph Optimization Error Over Time')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig('position_error.pdf')


def trajectories_graph2d(df, num_robots):
    plt.figure(figsize=(10, 10))
    plt.plot(df['actual_load_y'].to_numpy(), -df['actual_load_x'].to_numpy(), 'g-', label='Actual Load Trajectory')

    for i in range(1, num_robots+1):
        plt.plot(df[f'drone{i}_pose_y'].to_numpy(), -df[f'drone{i}_pose_x'].to_numpy(), 'r:', label=f'Drone {i} Trajectory')

    plt.title('XY projection of 3D Trajectories of Load and Drone(s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig('traj_up.pdf')


    plt.figure(figsize=(10, 10))
    y_values = np.zeros_like(df['actual_load_x'].to_numpy())
    plt.plot(df['actual_load_x'].to_numpy(), y_values, 'g-', label='Actual Load Trajectory')

    for i in range(1, num_robots+1):
        plt.plot(df[f'drone{i}_pose_x'].to_numpy(), df[f'drone{i}_pose_z'].to_numpy(), 'r:', label=f'Drone {i} Trajectory')

    plt.title('XZ projection of 3D Trajectories of Load and Drone(s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig('traj_side.pdf')


def trajectories_graph(df, num_robots):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    #ax.plot(df['desired_load_x'].to_numpy(), df['desired_load_y'].to_numpy(), 'k--', label='Desired Load Trajectory')
    #ax.plot(df['attachment_point1_x'].to_numpy(), df['attachment_point1_y'].to_numpy(), 'b-', label='AP Load Trajectory')

    ax.plot(df['actual_load_x'].to_numpy(), df['actual_load_y'].to_numpy(), 'g-', label='Actual Load Trajectory')

    for i in range(1, num_robots+1):
        ax.plot(df[f'drone{i}_pose_x'].to_numpy(), df[f'drone{i}_pose_y'].to_numpy(), df[f'drone{i}_pose_z'].to_numpy(), 'r:', label=f'Drone {i} Trajectory')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectories of Load and Drone(s)')
    ax.legend()
    plt.tight_layout()
    plt.savefig('trajectories_3d.pdf')


def drone_control_graph(df, num_robots, robot_height):
    window_size = 50
    plt.figure(figsize=(12, 8))
    n_rows = 3 if robot_height else 2
    for i in range(1, num_robots + 1):
        df[f'u{i}_x_smoothed'] = df[f'drone{i}_control_x'].rolling(window=window_size).mean()
        df[f'u{i}_y_smoothed'] = df[f'drone{i}_control_y'].rolling(window=window_size).mean()
        df[f'u{i}_z_smoothed'] = df[f'drone{i}_control_z'].rolling(window=window_size).mean()

        plt.subplot(n_rows, num_robots, i)
        plt.plot(df['timestamp'].to_numpy(), df[f'u{i}_x_smoothed'].to_numpy(), label=f'Drone {i} Control X')
        plt.xlabel('Time (s)')
        plt.ylabel('Control X')
        plt.grid(True)
        plt.legend()

        plt.subplot(n_rows, num_robots, num_robots + i)
        plt.plot(df['timestamp'].to_numpy(), df[f'u{i}_y_smoothed'].to_numpy(), label=f'Drone {i} Control Y')
        plt.xlabel('Time (s)')
        plt.ylabel('Control Y')
        plt.grid(True)
        plt.legend()

        if (robot_height):
            plt.subplot(n_rows, num_robots, 2 * num_robots + i)
            plt.plot(df['timestamp'].to_numpy(), df[f'u{i}_z_smoothed'].to_numpy(), label=f'Drone {i} Control Z')
            plt.xlabel('Time (s)')
            plt.ylabel('Control Z')
            plt.grid(True)
            plt.legend()

    plt.tight_layout()
    plt.savefig('drone_controls.pdf')


def cable_tensions_graph(df, num_robots, load_ori, robot_height):
    plt.figure(figsize=(12, 8))
    for i in range(1, num_robots+1):
        plt.subplot(2, num_robots, i)
        # Change to distance between load and robot
        att_p_col = 'actual_load' if not load_ori else f'attachment_point{i}'
        if robot_height:
            df[f'dist_{i}'] = np.sqrt((df[f'drone{i}_pose_x'] - df[f'{att_p_col}_x'])**2  + (df[f'drone{i}_pose_y'] - df[f'{att_p_col}_y'])**2 + (df[f'drone{i}_pose_z'] - 0.2)**2)
        else:
            df[f'dist_{i}'] = np.sqrt((df[f'drone{i}_pose_x'] - df[f'{att_p_col}_x'])**2  + (df[f'drone{i}_pose_y'] - df[f'{att_p_col}_y'])**2)
        plt.plot(df['timestamp'].to_numpy(), df[f'dist_{i}'].to_numpy(), label=f'Drone {i} <--> Load distance')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (M)')
        #plt.title(f'Distance between Load and Drone {i}')
        plt.grid(True)
        plt.legend()

        plt.subplot(2, num_robots, num_robots + i)
        plt.plot(df['timestamp'].to_numpy(), df[f'cable{i}_tension'].to_numpy(), label=f'Cable {i} Tension')
        plt.xlabel('Time (s)')
        plt.ylabel('Tension (N)')
        #plt.title(f'Cable {i} Tension Over Time')
        plt.grid(True)
        plt.legend()
    
    plt.tight_layout()
    plt.savefig('cable_tensions.pdf')

def get_filename(num_robots, load_ori, robot_height = False):
    num_drones = ["one_drone", "two_drones", "three_drones", "four_drones"]
    filename = num_drones[num_robots - 1]
    filename += "_with_height" if robot_height else ""
    filename += "_with_ori" if load_ori else ""

    return filename
    
    # if (num_robots == 1 and not load_ori and not robot_height):
    #     return "one_drone_no_ori"
    
    # if (num_robots == 1 and load_ori and not robot_height):
    #     return "one_drone_with_ori"

    # if (num_robots == 2 and not load_ori and not robot_height):
    #     return "two_drones_no_ori"

    # if (num_robots == 2 and load_ori and not robot_height):
    #     return "three_drones_with_ori"

    # if (num_robots == 4 and load_ori and not robot_height):
    #     return "four_drones_with_ori"

    # if (num_robots == 3 and load_ori and not robot_height):
    #     return "three_drones_with_ori"

    # if (num_robots == 3 and load_ori and robot_height):
    #     return "three_drones_with_height_and_ori"

    # if (num_robots == 4 and load_ori and robot_height):
    #     return "four_drones_with_height_and_ori"

    # if (num_robots == 1 and load_ori and robot_height):
    #     return "one_drone_with_height_and_ori"

    # if (num_robots == 2 and load_ori and robot_height):
    #     return "two_drones_with_height_and_ori"

    # raise ValueError("Unknown combination of parameters")
    

parser = argparse.ArgumentParser(description="A simple program that reads a value from the command line.")
parser.add_argument("--num_robots", type=int, default=1, help="Number of robots.")
parser.add_argument("--load_ori", action="store_true", help="Where load had orientation.")
parser.add_argument("--robot_height", action="store_true", help="Where robot height was also optimized for.")
args = parser.parse_args()

print(f"Calculating metrics for the following config:\n Number of robots: {args.num_robots}\n Load orientation: {args.load_ori}\n Robot height: {args.robot_height}\n")

filename = get_filename(args.num_robots, args.load_ori, args.robot_height)
df = pd.read_csv( f'/home/maryia/legacy/experiments/metrics/{filename}.csv')

load_position_error_graph(df)
trajectories_graph(df, args.num_robots)
trajectories_graph2d(df, args.num_robots)
drone_control_graph(df, args.num_robots, args.robot_height)
cable_tensions_graph(df, args.num_robots, args.load_ori, args.robot_height)

plt.show()