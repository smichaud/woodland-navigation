#!/usr/bin/python

import os
import sys
import csv
import math
import numpy

def load_odom(dataset_folder):
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

def load_scores_matrix(path_scores):
    reader=csv.reader(open(path_scores,"r"),delimiter=',')
    tmp_scores=list(reader)
    scores_matrix = numpy.array(tmp_scores).astype('float')

    return scores_matrix
