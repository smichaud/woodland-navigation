#!/usr/bin/python

import os,sys

def add_prefix_to_files(folder, prefix):
    for file in os.listdir(folder):
        current = folder + file
        new = folder + prefix + file
        # new = current.replace(prefix, '')
        os.rename(current, new)

def main(argv):
    folder = os.path.dirname(os.path.realpath(__file__)) + '/Data/'
    add_prefix_to_files(folder, argv[0])

if __name__ == '__main__':
    main(sys.argv[1:])
