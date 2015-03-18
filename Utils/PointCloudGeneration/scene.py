import json

class Scene():

    cylinder_count = 0
    cylinder_radius = 0
    rotation_range = [-1, 1]
    translation_range = [-1.2, 1.2]

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def from_json(self, json_string):
        for key, value in json.loads(json_string).items():
            setattr(self, key, value)