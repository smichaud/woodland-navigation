#!/usr/bin/python

import os
import sys
import shutil

def add_prefix_to_files(folder, prefix):
    for file in os.listdir(folder):
        current = folder + file
        new = folder + prefix + file
        # new = current.replace(prefix, '')
        os.rename(current, new)

def copy_files(src_folder, dest_folder):
    src_files = os.listdir(src_folder)
    for file_name in src_files:
        full_file_name = os.path.join(src_folder, file_name)
        if (os.path.isfile(full_file_name)):
            shutil.copy(full_file_name, dest_folder)

def rm_files(folder):
    files = os.listdir(folder)
    for file_name in files:
        full_file_name = os.path.join(folder, file_name)
        if (os.path.isfile(full_file_name)):
            os.remove(full_file_name)

