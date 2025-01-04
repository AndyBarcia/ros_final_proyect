import py_trees

class FindDoors(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, name="FindDoors"):
        super(FindDoors, self).__init__(name)
        self.lidar_processor = lidar_processor
        self.door_locations = []

    def update(self):
        self.door_locations = self.lidar_processor.get_door_locations()
        if self.door_locations:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def get_door_locations(self):
        return self.door_locations