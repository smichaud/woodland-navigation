#!/usr/bin/python

import csv
import time
import numpy

import seaborn
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from pandas import DataFrame

import processing

def paths(positions, metadata):
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

    seaborn.plt.savefig('Data/paths.pdf')
    seaborn.plt.show()

def distances_matrix(distances_matrix, second_loop_index):
    # seaborn.plt.title('Distances matrix', fontsize=20, y=1.1)

    dataframe = DataFrame(data=distances_matrix)
    cmap1 = ListedColormap(seaborn.color_palette("gist_heat_r",512))
    ax = seaborn.heatmap(dataframe, square=True, yticklabels=20, xticklabels=20, cmap=cmap1)
    seaborn.plt.ylabel('Sample index')
    seaborn.plt.xlabel('Sample index')
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top') 

    seaborn.plt.savefig('Data/distances_matrix.png', bbox_inches='tight')
    seaborn.plt.show()

def scores_matrix(scores_matrix, second_loop_index):
    # seaborn.plt.title('Scores matrix', fontsize=20, y=1.1)

    dataframe = DataFrame(data=scores_matrix)
    cmap1 = ListedColormap(seaborn.color_palette("gist_heat_r",512))
    ax = seaborn.heatmap(dataframe, square=True, yticklabels=20, xticklabels=20, cmap=cmap1)
    seaborn.plt.ylabel('Sample index')
    seaborn.plt.xlabel('Sample index')
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top') 

    seaborn.plt.savefig('Data/scores_matrix.png', bbox_inches='tight')
    seaborn.plt.show()

def distances_scores_plot(sorted_distances, sorted_scores, no_fp_distances, no_fp_scores):
    seaborn.set_style('whitegrid')
    # seaborn.plt.title('Score as function of distance')
    seaborn.plt.xlabel('Distance between scans (m)')
    seaborn.plt.ylabel('Place recognition score')
    seaborn.plt.grid(True)

    seaborn.plt.scatter(sorted_distances, sorted_scores, s=4, label='Pair of scans data points')
    seaborn.plt.plot(no_fp_distances, no_fp_scores, label='Score limit to avoid FP')
    
    seaborn.plt.legend()

    seaborn.plt.savefig('Data/distances_scores.pdf')
    seaborn.plt.show()

def recall_plot(sorted_distances, sorted_scores, score_thresholds):
    seaborn.set_style('whitegrid')
    # seaborn.plt.title('Recall rates', fontsize=20)
    seaborn.plt.xlabel('Maximum distance between scans to be considered as originating from the same place (m)')
    seaborn.plt.ylabel('Recall rate (%)')
    seaborn.plt.grid(True)

    for threshold in score_thresholds:
        result_distances, fn, fp, tn, tp = processing.compute_results(sorted_distances, sorted_scores, threshold)
        tp = numpy.array(tp, dtype=numpy.float)
        fn = numpy.array(fn, dtype=numpy.float)
        recall = numpy.divide(tp, numpy.add(tp, fn))*100

        seaborn.plt.plot(result_distances, recall, label='Score threshold = ' + str(threshold))

    seaborn.plt.legend()

    seaborn.plt.savefig('Data/recall.pdf')
    seaborn.plt.show()
