#!/usr/bin/python

import os
import sys
import csv
import math
import numpy

def create_distances_matrix(positions, metadata):
    sample_counts = len(positions)
    distances_matrix = numpy.zeros(shape=(sample_counts, sample_counts))
    for i in range(0,sample_counts):
        for j in range(0,i):
            distance = get_distance(positions, metadata, j, i)
            distances_matrix[i,j] = distance
            distances_matrix[j,i] = distance

    # numpy.save('Data/distances_matrix.npy', distances_matrix)

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
