#!/usr/bin/python

import os
import csv
import numpy
import seaborn
from pandas import DataFrame
from matplotlib.colors import ListedColormap

def load_data(dataset_folder):
    positions = []
    for file in numpy.unique(os.listdir(dataset_folder)):
        if file.endswith('_info.dat'):
            positions.append(extract_position(dataset_folder + file))

    file = open(dataset_folder + 'metadata.dat')
    for line in iter(file):
        if line.startswith('Second loop index: '):
            line = line.replace('Second loop index: ', '')
            second_loop_index = int(line)
        elif line.startswith('Loop 1 odometry difference: '):
            line = line.replace('Loop 1 odometry difference: ', '')
            loop1_odom_diff = [float(i) for i in line.split()]
        elif line.startswith('Loop 2 odometry difference: '):
            line = line.replace('Loop 2 odometry difference: ', '')
            loop2_odom_diff = [float(i) for i in line.split()]
    metadata = [second_loop_index, loop1_odom_diff, loop2_odom_diff]

    return positions, metadata

def extract_position(filepath):
    file = open(filepath)
    for line in iter(file):
        if line.startswith('Odometry: '):
            line = line.replace('Odometry: ', '')
            pose = [float(i) for i in line.split()]
            position = [pose[0], pose[1], pose[2]]

    return position

def create_path_figure(positions, metadata):
    seaborn.set_style('whitegrid')
    seaborn.plt.title('Path of the robot', fontsize=20)
    seaborn.plt.xlabel('X position (m)')
    seaborn.plt.ylabel('Y position (m)')
    seaborn.plt.axis('equal')
    seaborn.plt.grid(True)

    i_split = metadata[0]
    x = [i[0] for i in positions]
    y = [i[1] for i in positions]
    seaborn.plt.plot(x[:i_split], y[:i_split], 'b-*', label='Loop 1')
    seaborn.plt.plot(x[i_split:], y[i_split:], 'ro-', markersize = 4, label='Loop 2')
    seaborn.plt.legend()

    seaborn.plt.savefig('Data/paths_plot.pdf')
    # seaborn.plt.show()

def create_distances_matrix(positions, metadata):
    sample_counts = len(positions)
    distances_matrix = numpy.zeros(shape=(sample_counts, sample_counts))
    for i in range(0,sample_counts):
        for j in range(0,i):
            distance = get_distance(positions, metadata, j, i)
            distances_matrix[i,j] = distance
            distances_matrix[j,i] = distance

    numpy.save('Data/distances_matrix.npy', distances_matrix)

    return distances_matrix

# This take into account that last is far from first (loop too open)
def get_distance(positions, metadata, i1, i2):
    pos1 = numpy.array(positions[i1])
    pos2 = numpy.array(positions[i2])
    direct_distance = numpy.linalg.norm(pos2-pos1)

    loop1_last_position = numpy.array(positions[metadata[0]-1])
    loop1_position_diff = numpy.array(metadata[1][:3])

    loop2_last_position = numpy.array(positions[len(positions)-1])
    loop2_position_diff = numpy.array(metadata[2][:3])

    if i1 < metadata[0] and i2 < metadata[0]:
        extended_position = loop1_last_position + loop1_position_diff + positions[i1]
        loop_distance = numpy.linalg.norm(extended_position - pos2)
    elif i1 < metadata[0]/2 and i2 >= len(positions)-metadata[0]/2:
        extended_position = loop2_last_position + loop2_position_diff + positions[i1]
        loop_distance = numpy.linalg.norm(extended_position - pos2)
    elif i1 < metadata[0] and i2 >= metadata[0]:
        extended_position = loop1_last_position + loop1_position_diff + positions[i2]
        loop_distance = numpy.linalg.norm(extended_position - pos1)
    else:
        extended_position = loop2_last_position + loop2_position_diff + positions[i1]
        loop_distance = numpy.linalg.norm(extended_position - pos2)

    return min(direct_distance, loop_distance)

def main():
    print("Creating odom matrix and figures...")

    dataset_folder = '/media/D/Datasets/PlaceRecognition/SickBuilding/Output/'
    positions, metadata = load_data(dataset_folder)

    create_path_figure(positions, metadata)
    distances_matrix = create_distances_matrix(positions, metadata)

    print("Done !")


if __name__ == '__main__':
    main()
