import json
import copy

class Configuration:

    scenes = []

    def add_scene(self, scene):
        self.scenes.append(copy.deepcopy(scene))

    def to_json(self):
        return json.dumps([scene.__dict__ for scene in self.scenes])

    def from_json(self, json_string):
        self.scenes = json.loads(json_string)

    def write_json(filename):
        with open(filename, "w") as config_file:
            config_file.write(self.to_json())

    def read_json(filename):
        output_scene = SceneConfig()
        with open(filename,"r") as config_file:
            output_scene.from_json(config_file.read())

        return output_scene

