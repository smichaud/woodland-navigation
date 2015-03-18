import os
import sys

def pcd_to_csv(complete_filename):
    file_path = os.path.split(complete_filename)[0] + '/'
    file_name = os.path.split(complete_filename)[1]
    file_base_name = os.path.splitext(complete_filename)[0]
    
    input_file = file_path + file_name
    output_file = file_base_name + '.csv'
    
    output_handle = open(output_file, "w")
    with open(input_file) as input_handle:
        for line in input_handle:
            if line[0].isdigit() or line[0] ==  '-':
                output_handle.write(line.replace(' ', ','))
            elif line.startswith('FIELDS '):
                output_handle.write(line.replace('FIELDS ', '').replace(' ', ','))
                
    input_handle.close()
    output_handle.close()
