#!/usr/bin/python

import os
import sys

import data
import odom
import processing
import visualization
import util

def main(argv):
    # dataset_name = "building"
    # dataset_folder = "/media/D/Datasets/PlaceRecognition/SickBuilding/Output/"
    dataset_name = "forest"
    dataset_folder = "/media/D/Datasets/PlaceRecognition/SickForest/OutputMerged2/"
    # dataset_name = "velodyne"
    # dataset_folder = "/media/D/Datasets/PlaceRecognition/VelodyneForest/Output/"

    output_folder = os.path.dirname(os.path.realpath(__file__)) + '/Data/'
    util.rm_files(output_folder)

    print "Load data from :" + dataset_folder
    positions, metadata = data.load_odom(dataset_folder)

    distances_matrix = odom.create_distances_matrix(positions, metadata)
    scores_matrix = data.load_scores_matrix(dataset_folder + 'scores_matrix.csv')

    index1, index2, distances, scores = processing.matrices_to_list(distances_matrix, scores_matrix)
    sorted_index1, sorted_index2, sorted_distances, sorted_scores = processing.sort_by_distance(index1, index2, distances, scores)
    no_fp_distances, no_fp_scores = processing.create_no_fp_data(sorted_distances, sorted_scores)

    second_loop_index = metadata[0]
    visualization.paths(positions, metadata)
    visualization.distances_matrix(distances_matrix, second_loop_index)
    visualization.scores_matrix(scores_matrix, second_loop_index)
    visualization.distances_scores_plot(sorted_distances, sorted_scores, no_fp_distances, no_fp_scores)

    score_thresholds = [0.75, 0.5, 0.25, 0.1]
    visualization.recall_plot(sorted_distances, sorted_scores, score_thresholds)


    thesis_img_folder = "/home/smichaud/Workspace/ThesisMaster/Thesis/img/chap_slam/"
    util.add_prefix_to_files(output_folder, dataset_name + "_")
    util.copy_files(output_folder, thesis_img_folder)
    
    print "Job done !"


if __name__ == '__main__':
    main(sys.argv[1:])
