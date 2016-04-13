#!/usr/bin/python

import csv
import time
import numpy

import seaborn
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from pandas import DataFrame

def matrices_to_list(distances_matrix, scores_matrix):
    # Create lists for index1, index2, distances (left-lower diago excl. diago)
    matrix_size = distances_matrix.shape[1]
    index1 = []
    index2 = []
    distances = []
    scores = []
    for i in range(0, matrix_size):
        for j in range(0,i):
            index1.extend([i])
            index2.extend([j])
            distances.extend([distances_matrix[i,j]])
            scores.extend([scores_matrix[i,j]])

    return index1, index2, distances, scores

def sort_by_distance(index1, index2, distances, scores):
    sorted_indexes = numpy.argsort(distances)
    sorted_index1 = [index1[i] for i in sorted_indexes]
    sorted_index2 = [index2[i] for i in sorted_indexes]
    sorted_distances = [distances[i] for i in sorted_indexes]
    sorted_scores = [scores[i] for i in sorted_indexes]

    return sorted_index1, sorted_index2, sorted_distances, sorted_scores

def create_no_fp_data(sorted_distances, sorted_scores):
    distances = []
    scores = []
    max_score = 0
    for i in range(len(sorted_distances)-1, 0, -1):
        distances.append(sorted_distances[i])
        if sorted_scores[i] > max_score:
            max_score = sorted_scores[i]

        scores.append(max_score)

    return distances, scores

def compute_results(sorted_distances, sorted_scores, score_threshold):
    # Compute values for distance = 0
    fn = 0
    fp = 0
    tn = 0
    tp = 0
    pairCounts = len(sorted_distances)
    for i in range(0, pairCounts):
        if sorted_scores[i] > score_threshold:
            fp = fp + 1
        else:
            tn = tn + 1

    result_distances = [0]
    result_fn = [0]
    result_fp = [fp]
    result_tn = [tn]
    result_tp = [0]

    # Loop for all pairs in ascending order by distance...
    # switch the old value for new value and record
    for i in range(0, pairCounts): # Basically the pair become "in range'
        if sorted_scores[i] > score_threshold:
            fp = fp - 1
            tp = tp + 1
        else:
            tn = tn - 1
            fn = fn + 1

        result_distances.extend([sorted_distances[i]]) # Could do it in one shot
        result_fn.extend([fn])
        result_fp.extend([fp])
        result_tn.extend([tn])
        result_tp.extend([tp])

    return result_distances, result_fn, result_fp, result_tn, result_tp

