import json
import copy


class SceneGenerationConfig:
    def __init__(self):
        self.output_pcd_file = ""
        self.output_blend_file = ""
        self.primitives = []

    def add_primitive(self, primitive):
        self.primitives.append(copy.deepcopy(primitive))

    def clear_primitives(self):
        self.primitives = []

    def write_json(self, filename="ConfigFiles/default.json"):
        with open(filename, 'w') as scene_file:
            self.output_dict = self.__dict__.copy()
            self.output_dict.update({'primitives':self.primitives}.copy())
            scene_file.write(json.dumps(self.output_dict, indent=4, sort_keys=True))

    def read_json(self, filename="ConfigFiles/default.json"):
        with open(filename,'r') as scene_file:
            input_dict = json.loads(scene_file.read())

            if 'output_pcd_file' in input_dict:
                self.output_pcd_file = input_dict['output_pcd_file']
            if 'output_blend_file' in input_dict:
                self.output_blend_file = input_dict['output_blend_file']
            if 'primitives' in input_dict:
                self.primitives = input_dict['primitives']