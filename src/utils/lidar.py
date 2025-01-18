import py_trees
import numpy as np
import math

class LidarProcessor:
    def __init__(self, robot_radius=0.2, safe_margin=0.01):
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key="safe_distances", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="sensor_angles", access=py_trees.common.Access.WRITE)

        self.scan = None
        self.safe_distances = {}
        self.door_locations = []
        self.robot_radius = robot_radius
        self.safe_margin = safe_margin
        self.sensor_angles = None

    def lidar_callback(self, scan):
        self.scan = scan
        self.safe_distances, self.sensor_angles = self.maximum_safe_distance()
        self.find_door_locations()

        self.blackboard.safe_distances = list(self.safe_distances)
        self.blackboard.sensor_angles = list(self.sensor_angles)

    def maximum_safe_distance(self):
        if self.scan is None:
            return None, None

        # The values recorded by the lidar sensor.
        sensor_values = np.array(self.scan.ranges)

        # The angle that each sensor makes with the forward direction in radians
        sensor_angles = np.linspace(self.scan.angle_min, self.scan.angle_max, len(sensor_values)+1)[:-1]

        # As per the documentation, technically we should ignore values outside the valid
        # range, but we are just going to replace them with the range values. This is specially
        # important to work in open spaces, because otherwise we would remove all infinity values. 
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
        sensor_values[sensor_values < self.scan.range_min] = self.scan.range_min
        sensor_values[sensor_values > self.scan.range_max] = self.scan.range_max

        # Create a matrix of all possible sensor values for all directions.
        per_direction_sensor_values = np.array([
            np.roll(sensor_values, -i) 
            for i in range(len(sensor_values))
        ])

        # Compute forward and side distances for each sensor in each direction.
        with np.errstate(invalid='ignore'):
            forward_distances = np.cos(sensor_angles) * per_direction_sensor_values
            side_distances = np.sin(sensor_angles) * per_direction_sensor_values

        # Identify potential collisions in each direction
        robot_radius = self.robot_radius + self.safe_margin
        collisions = (forward_distances >= 0) & (np.abs(side_distances) < robot_radius)    

        # The maximum safe distance across all directions, initially
        # set to infinity in the case no detection is found.
        max_safe_distances = np.full((len(sensor_angles),), float('inf'))

        for direction_idx in range(len(sensor_angles)):
            # Get collision mask for this specific direction
            direction_collisions = collisions[direction_idx]

            # If there are collisions in this direction
            if np.any(direction_collisions):
                # Compute the forward and side distances.
                collision_forward_distances = forward_distances[direction_idx][direction_collisions]
                collision_side_distances = side_distances[direction_idx][direction_collisions]
                # Compute the angle this collision would
                # make with the robot at the collision point.
                collision_angle = np.arcsin(collision_side_distances/robot_radius)

                # Determine the actual amount the robot could
                # advance until the detected point reached
                # the robot radius.
                with np.errstate(divide='ignore', invalid='ignore'):
                    sin_beta = collision_side_distances/robot_radius
                    safe_forward_distances = collision_forward_distances - np.where(
                        collision_angle != 0,
                        collision_side_distances*(np.sqrt(1-sin_beta**2))/sin_beta,
                        robot_radius
                    )

                # Find minimum forward distance for this direction
                min_dist = max(np.min(safe_forward_distances), 0.0)
                max_safe_distances[direction_idx] = min_dist
        
        return max_safe_distances, sensor_angles

    def find_door_locations(self):
        if self.scan is None:
            self.door_locations = []
            return
      
        threshold_door_width= 1.0  #in meters, minimum door width.
        threshold_gap = 0.4 #min distance between doors
        door_locations = [] #will contain all possible door locations
        max_distances = {}
        ranges = np.array(self.scan.ranges)
        angle_increment = self.scan.angle_increment
        angle_min = self.scan.angle_min
        num_directions = 36
        for i in range(num_directions):
            angle = angle_min + i * (2 * math.pi / num_directions)
            start_index = int((angle - angle_min) / angle_increment)
            end_index = int(start_index + (2*math.pi/num_directions)/angle_increment)

            if start_index > end_index:
                end_index=len(ranges)
            if end_index>= len(ranges):
                end_index= len(ranges) -1
            
            distances_in_range=ranges[start_index:end_index]
            distances_in_range = distances_in_range[distances_in_range!=np.inf]
            if (len(distances_in_range)==0):
                max_distances[angle] = 0
            else:
                max_distances[angle] = np.max(distances_in_range)
        angles = sorted(max_distances.keys())
        
        
        #We go through each angle looking for the possible doors
        i = 0
        while i < len(angles):
          angle = angles[i]
          #We consider a door when the max distance is greater than a threshold...
          if (max_distances[angle] > threshold_door_width):
            start_angle=angle
            end_angle = angle
            #We start to look for more distances that are greater than the threshold (which means that we are looking for a door)
            j = i+1
            while j < len(angles):
              next_angle = angles[j]
              #We only consider that the next angle belong to the same door when the max distance in that direction is also greater than a threshold
              if (max_distances[next_angle] > threshold_door_width) and (next_angle-end_angle < threshold_gap):
                  end_angle = next_angle
                  j = j+1
              else:
                  break
            
            #Once a door has been identified, we calculate the average angle
            average_angle= (start_angle+end_angle)/2.0
            door_locations.append(average_angle)
            i=j
          else:
            i=i+1

        self.door_locations = door_locations
    
    def get_door_locations(self):
       return self.door_locations
    
    def get_safe_distances(self):
        return self.safe_distances