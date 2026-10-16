"""
 * @file visualize_torques.py
 * @brief Read and visualize torque data from joint_torques.csv
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-09-18
 *
 * @copyright Copyright (c) 2025
"""

import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

# Review diff test (round 2):
# This two-line comment should be visible in the code review details.

def read_torque_data(csv_file='joint_torques.csv'):
    """
    Read torque data from CSV file.
    Returns time array and torque array (rows: time steps, cols: joints).
    """
    if not os.path.isfile(csv_file):
        raise FileNotFoundError(f"CSV file not found: {csv_file}")

    times = []
    torques = []
    with open(csv_file, 'r', newline='') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header
        expected_cols = 30  # time + 29 joints
        if len(header) != expected_cols:
            print(f"[WARN] Expected {expected_cols} columns, found {len(header)}")

        for row in reader:
            if len(row) != expected_cols:
                print(f"[WARN] Skipping row with {len(row)} columns, expected {expected_cols}")
                continue
            try:
                times.append(float(row[0]))
                torques.append([float(x) for x in row[1:]])
            except ValueError as e:
                print(f"[WARN] Skipping invalid row: {e}")
                continue

    if not times:
        raise ValueError("No valid data found in CSV file")

    return np.array(times), np.array(torques)

def plot_torques(times, torques, output_file='torque_plot.png', joint_list=None, show_plot=False):
    """
    Plot torque data for each joint over time and save to file.
    Optionally display the plot if show_plot is True.
    """
    num_joints = torques.shape[1]
    plt.figure(figsize=(12, 8))
    if joint_list is None:
        for i in range(num_joints):
            plt.plot(times, torques[:, i], label=f'Joint {i}', linewidth=2)
    else:
        for i in joint_list:
            plt.plot(times, torques[:, i], label=f'Joint {i}', linewidth=2)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (N*m)')
    plt.title('Joint Torques vs Time')
    plt.legend(ncol=3, fontsize=8)  # Compact legend for 29 joints
    plt.grid(True)
    
    try:
        plt.savefig(output_file)
        print(f"[INFO] Plot saved to {output_file}")
    except Exception as e:
        print(f"[ERROR] Failed to save plot: {e}")
    
    if show_plot:
        try:
            plt.show()
        except Exception as e:
            print(f"[WARN] Failed to display plot (headless environment?): {e}")
    
    plt.close()


def main():
    mode = 4

    joint_list = None
    if mode == 0:
        joint_list = None
    elif mode == 1:
        joint_list = [0,1,2] # Waist
    elif mode == 2:
        joint_list = [3,4,5,6,7,8,9,
                  10,11,12,13,14,15,16] # Upper body
        joint_list = [7,8,9,
                  14,15,16] # Upper body
    elif mode == 3:
        joint_list = [17,18,19,20,
                  23,24,25,26] # Hips and knees
    elif mode == 4:
        joint_list = [21,22,
                  27,28] # Ankles


    try:
        times, torques = read_torque_data('/home/ubuntu/deep_robotics/rl_deploy_dr2/joint_torques.csv')
        print(f"[INFO] Read {len(times)} time steps, {torques.shape[1]} joints")
        plot_torques(times, torques, output_file='/home/ubuntu/deep_robotics/rl_deploy_dr2/torque_plot.png', joint_list=joint_list, show_plot=True)
    except Exception as e:
        print(f"[ERROR] Failed to process data: {e}")

if __name__ == "__main__":
    main()
