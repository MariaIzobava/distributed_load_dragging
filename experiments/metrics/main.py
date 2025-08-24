import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D # For 3D trajectories
import argparse

def load_position_error_graph(df):
    df['error_x'] = df['desired_load_x'] - df['actual_load_x']
    df['error_y'] = df['desired_load_y'] - df['actual_load_y']
    df['position_error_magnitude'] = np.sqrt(df['error_x']**2 + df['error_y']**2)
    print(df['position_error_magnitude'])
    plt.figure(figsize=(12, 6))
    plt.plot(df['timestamp'].to_numpy(), df['position_error_magnitude'].to_numpy(), label='Load Position Error Magnitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Error Magnitude (m)')
    plt.title('Load Position Tracking Error Over Time')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig('load_position_error.png')



def trajectories_graph(df, num_robots):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    #ax.plot(df['desired_load_x'].to_numpy(), df['desired_load_y'].to_numpy(), 'k--', label='Desired Load Trajectory')
    ax.plot(df['actual_load_x'].to_numpy(), df['actual_load_y'].to_numpy(), 'b-', label='Actual Load Trajectory')

    for i in range(1, num_robots+1):
        ax.plot(df[f'drone{i}_pose_x'].to_numpy(), df[f'drone{i}_pose_y'].to_numpy(), df[f'drone{i}_pose_z'].to_numpy(), 'r:', label=f'Drone {i} Trajectory')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectories of Load and Drone(s)')
    ax.legend()
    plt.tight_layout()
    plt.savefig('trajectories_3d.png')


def drone_velocity_graph(df, num_robots):
    plt.figure(figsize=(12, 8))
    for i in range(1, num_robots + 1):
        plt.subplot(2, num_robots, i)
        plt.plot(df['timestamp'].to_numpy(), df[f'drone{i}_velocity_z'].to_numpy(), label=f'Drone {i} Velocity Z')
        plt.xlabel('Time (s)')
        plt.ylabel('Vertical Velocity (Z)')
        plt.grid(True)
        plt.legend()

        plt.subplot(2, num_robots, num_robots + i)
        plt.plot(df['timestamp'].to_numpy(), df[f'drone{i}_velocity_x'].to_numpy(), label=f'Drone {i} Velocity X')
        plt.plot(df['timestamp'].to_numpy(), df[f'drone{i}_velocity_y'].to_numpy(), label=f'Drone {i} Velocity Y')
        plt.xlabel('Time (s)')
        plt.ylabel('Horizontal Velocity')
        plt.grid(True)
        plt.legend()

    plt.tight_layout()
    plt.savefig('drone_velocities.png')


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
    plt.savefig('cable_tensions.png')

def get_filename(num_robots, load_ori):
    if (num_robots == 1 and not load_ori):
        return "one_drone_no_ori"
    
    if (num_robots == 1 and load_ori):
        return "one_drone_with_ori"

    if (num_robots == 2 and not load_ori):
        return "two_drones_no_ori"

    if (num_robots == 2 and load_ori):
        return "two_drones_with_ori"

    if (num_robots == 4 and load_ori):
        return "four_drones_with_ori"

    raise ValueError("Unknown combination of parameters")
    

parser = argparse.ArgumentParser(description="A simple program that reads a value from the command line.")
parser.add_argument("--num_robots", type=int, default=1, help="Number of robots.")
parser.add_argument("--load_ori", action="store_true", help="Where load had orientation.")
parser.add_argument("--robot_height", action="store_true", help="Where robot height was also optimized for.")
args = parser.parse_args()

print(f"Calculating metrics for the following config:\n Number of robots: {args.num_robots}\n Load orientation: {args.load_ori}\n Robot height: {args.robot_height}\n")

filename = get_filename(args.num_robots, args.load_ori)
df = pd.read_csv( f'/home/maryia/legacy/experiments/metrics/{filename}.csv')

#load_position_error_graph(df)
trajectories_graph(df, args.num_robots)
drone_velocity_graph(df, args.num_robots)
cable_tensions_graph(df, args.num_robots, args.load_ori, args.robot_height)

plt.show()