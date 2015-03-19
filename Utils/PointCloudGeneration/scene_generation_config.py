import json
import copy


class SceneGenerationConfig:
    output_pcd_file = ""
    output_blend_file = ""
    primitives = []

    def add_primitive(self, primitive):
        self.primitives.append(copy.deepcopy(primitive))

    def to_json(self, filename="ConfigFiles/default.json"):
        with open(filename, 'w') as scene_file:
            output_dict = self.__dict__.copy()
            output_dict.update({'primitives':self.primitives}.copy())
            scene_file.write(json.dumps(output_dict, indent=4, sort_keys=True))

    def from_json(self, filename="ConfigFiles/default.json"):
        with open(filename,'r') as scene_file:
            input_dict = json.loads(scene_file.read())

            if 'output_pcd_file' in input_dict:
                self.output_pcd_file = input_dict['output_pcd_file']
            if 'output_blend_file' in input_dict:
                self.output_blend_file = input_dict['output_blend_file']
            if 'primitives' in input_dict:
                self.primitives = input_dict['primitives']