#!/usr/bin/python

import os
import sys
import csv
import math
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
        elif line.startswith('Loop 1-1 odometry difference: '):
            line = line.replace('Loop 1-1 odometry difference: ', '')
            odom_diff_1_1 = [float(i) for i in line.split()]
        elif line.startswith('Loop 1-2 odometry difference: '):
            line = line.replace('Loop 1-2 odometry difference: ', '')
            odom_diff_1_2 = [float(i) for i in line.split()]
        elif line.startswith('Loop 2-2 odometry difference: '):
            line = line.replace('Loop 2-2 odometry difference: ', '')
            odom_diff_2_2 = [float(i) for i in line.split()]
        elif line.startswith('Loop 2-1 odometry difference: '):
            line = line.replace('Loop 2-1 odometry difference: ', '')
            odom_diff_2_1 = [float(i) for i in line.split()]
    # Where loops_odom_diff_matrix[0][1] represent diff between loop1 end and loop2 start
    loops_odom_diff_matrix = [[odom_diff_1_1, odom_diff_1_2], [odom_diff_2_1, odom_diff_2_2]]
    metadata = [second_loop_index, loops_odom_diff_matrix]

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
    # seaborn.plt.title('Path of the robot', fontsize=20)
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
    seaborn.plt.gca().invert_xaxis()

    seaborn.plt.savefig('Data/paths_plot.pdf')
    seaborn.plt.show()

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

def get_distance(positions, metadata, i1, i2):
    loop2_first_index = metadata[0]
    last_index = len(positions)-1
    loops_odom_diff = metadata[1]

    pos1 = numpy.array(positions[i1])
    pos2 = numpy.array(positions[i2])
    posLast1 = numpy.array(positions[loop2_first_index-1])
    posLast2 = numpy.array(positions[last_index])

    distance = numpy.linalg.norm(pos2-pos1)

    # Check if loop distance (in index counts) is shoter, take this distance instead
    direct_index_count = i2-i1
    if i1 < loop2_first_index and i2 < loop2_first_index:
        loop_index_count = (loop2_first_index - i2) + i1
        if loop_index_count < direct_index_count:
            distance = numpy.linalg.norm((posLast1 - pos2) + loops_odom_diff[0][0][0:3] + pos1)

    elif i1 >= loop2_first_index and i2 >= loop2_first_index:
        loop_index_count = (last_index + 1 - i2) + (i2 - loop2_first_index)
        if loop_index_count < direct_index_count:
            distance = numpy.linalg.norm((posLast2 - pos2) + loops_odom_diff[1][1][0:3] + pos1)

    elif i1 < loop2_first_index and i2 >= loop2_first_index:
        # TODO Here I am
        i1_ratio = float(i1 + 1) / float(loop2_first_index)
        i2_ratio = float(i2 - loop2_first_index + 1) / float(last_index - loop2_first_index + 1)
        gap_ratio = 2 / (last_index + 1) # = avg for one difference
        direct_ratio = math.fabs(i2_ratio - i1_ratio)
        loop_1_2_ratio = (1-i1_ratio) + (i2_ratio) + gap_ratio
        loop_2_1_ratio = (1-i2_ratio) + (i1_ratio) + gap_ratio

        if direct_ratio <= loop_1_2_ratio and direct_ratio <= loop_2_1_ratio:
            distance = numpy.linalg.norm(pos2-pos1) # Just repeated for better understanding
        elif loop_1_2_ratio <= direct_ratio and loop_1_2_ratio <= loop_2_1_ratio:
            distance = numpy.linalg.norm((posLast1 - pos1) + loops_odom_diff[0][1][0:3] + pos2)
        elif loop_2_1_ratio <= direct_ratio and loop_2_1_ratio <= loop_1_2_ratio:
            distance = numpy.linalg.norm((posLast1 - pos1) + loops_odom_diff[1][0][0:3] + pos2)

    else:
        print "you forgot one case bro..."

    return distance

def main(argv):
    print("Creating odom matrix and figures...")

    dataset_folder = '/media/D/Datasets/PlaceRecognition/SickBuilding/Output/'
    # dataset_folder = argv[0]
    positions, metadata = load_data(dataset_folder)

    # create_path_figure(positions, metadata)
    distances_matrix = create_distances_matrix(positions, metadata)

    print("Done !")


if __name__ == '__main__':
    main(sys.argv[1:])
