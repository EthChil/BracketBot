import plotly.graph_objs as go
import plotly.express as px
import os

import numpy as np
import sys

# Function to read the text file and extract the coordinates
def read_coordinates(filename):
    x, y, z = [], [], []

    with open(filename, 'r') as file:
        for line in file:
            coordinates = line.strip().split()
            x.append(float(coordinates[0]))
            y.append(float(coordinates[1]))
            z.append(float(coordinates[2]))

    return x, y, z

def read_keyframe_coordinates(filename):
    x, y, z = [], [], []

    with open(filename, 'r') as file:
        for line in file:
            keyframe_data = line.strip().split()
            x.append(float(keyframe_data[1]))
            y.append(float(keyframe_data[2]))
            z.append(float(keyframe_data[3]))

    return x, y, z

def rotate_points_90_degrees_x_axis(points):
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])

    rotated_points = np.dot(points, rotation_matrix)
    return rotated_points[:, 0], rotated_points[:, 1], rotated_points[:, 2]

def mirror_points_about_z_axis(points):
    points[:, 0] *= 1
    points[:, 1] *= -1
    return points[:, 0], points[:, 1], points[:, 2]

def mirror_points_about_x_axis(points):
    points[:, 0] *= -1
    return points[:, 0], points[:, 1], points[:, 2]

def rotate_points_3d(points, axis, angle_degrees):
    angle_rad = np.deg2rad(angle_degrees)

    # Rotation matrix for X-axis
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(angle_rad), -np.sin(angle_rad)],
                   [0, np.sin(angle_rad), np.cos(angle_rad)]])

    # Rotation matrix for Y-axis
    Ry = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                   [0, 1, 0],
                   [-np.sin(angle_rad), 0, np.cos(angle_rad)]])

    # Rotation matrix for Z-axis
    Rz = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                   [np.sin(angle_rad), np.cos(angle_rad), 0],
                   [0, 0, 1]])

    if axis == 'x':
        R = Rx
    elif axis == 'y':
        R = Ry
    elif axis == 'z':
        R = Rz
    else:
        raise ValueError("Invalid axis. Choose from 'x', 'y', or 'z'.")

    return np.dot(points, R.T)

def translate_points_3d(points, dx, dy, dz):

    translation_matrix = np.array([dx, dy, dz])
    translated_points = points + translation_matrix

    return translated_points

# Read the coordinates from the text file
x_data = np.load("../orbslam_independant_files_donttouch/ego_xs.npy")
y_data = np.load("../orbslam_independant_files_donttouch/ego_ys.npy")
theta_data = np.load( "../orbslam_independant_files_donttouch/ego_thetas.npy")
map_filename = '../orbslam_independant_files_donttouch/tum_map_pts_and_keyframes.txt'  # Replace this with the path to your text file
keyframe_filename = '../orbslam_independant_files_donttouch/tum_key_frame_trajectory.txt'

cutoff = 5

x, y, z = read_coordinates(map_filename)
x = x[:int(int(len(x))/cutoff)]
y = y[:int(int(len(y))/cutoff)]
z = z[:int(int(len(z))/cutoff)]

points = np.vstack((x, y, z)).T
x, y, z = mirror_points_about_z_axis(points)
points = np.vstack((x, y, z)).T
x, y, z = rotate_points_90_degrees_x_axis(points)
points = np.vstack((x, y, z)).T
x, y, z = mirror_points_about_x_axis(points)

points = np.vstack((x, y, z)).T
rotated_points= rotate_points_3d(points,'z', 45)
x, y, z = rotated_points[:, 0], rotated_points[:, 1], rotated_points[:, 2]

points = np.vstack((x, y, z)).T
translated_points = translate_points_3d(points,0.5,1,0)
x, y, z = translated_points[:, 0], translated_points[:, 1], translated_points[:, 2]





x_keyframes, y_keyframes, z_keyframes = read_keyframe_coordinates(keyframe_filename)
x_keyframes = x_keyframes[:int(int(len(x_keyframes))/cutoff)]
y_keyframes = y_keyframes[:int(int(len(y_keyframes))/cutoff)]
z_keyframes = z_keyframes[:int(int(len(z_keyframes))/cutoff)]

keyframe_points = np.vstack((x_keyframes, y_keyframes, z_keyframes)).T
x_keyframes, y_keyframes, z_keyframes = mirror_points_about_z_axis(keyframe_points)
keyframe_points = np.vstack((x_keyframes, y_keyframes, z_keyframes)).T
x_keyframes, y_keyframes, z_keyframes = rotate_points_90_degrees_x_axis(keyframe_points)
keyframe_points = np.vstack((x_keyframes, y_keyframes, z_keyframes)).T
x_keyframes, y_keyframes, z_keyframes = mirror_points_about_x_axis(keyframe_points)

keyframe_points = np.vstack((x_keyframes, y_keyframes, z_keyframes)).T
rotated_keyframes = rotate_points_3d(keyframe_points,'z', 45)
x_keyframes, y_keyframes, z_keyframes = rotated_keyframes[:, 0], rotated_keyframes[:, 1], rotated_keyframes[:, 2]

keyframe_points = np.vstack((x_keyframes, y_keyframes, z_keyframes)).T
translated_keyframes = translate_points_3d(keyframe_points,0.5,1,0)
x_keyframes, y_keyframes, z_keyframes = translated_keyframes[:, 0], translated_keyframes[:, 1], translated_keyframes[:, 2]






x_values, x_timestamps = x_data[:, 0], x_data[:, 1]
y_values, y_timestamps = y_data[:, 0], y_data[:, 1]
theta_values, theta_timestamps = theta_data[:, 0], theta_data[:, 1]

fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z, mode='markers', marker=dict(size=2), name='Points from Text File')])

fig.add_trace(go.Scatter3d(x=x_keyframes, y=y_keyframes, z=z_keyframes, mode='markers', marker=dict(size=2, color='red'), name='Keyframe Points'))
fig.add_trace(go.Scatter3d(x=y_values, y=x_values, z=np.zeros(len(y_values)), mode='markers', marker=dict(size=2), name='Ego Data'))
fig.update_layout(scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'), title='Combined 3D Points')

# Show the plot
fig.show()