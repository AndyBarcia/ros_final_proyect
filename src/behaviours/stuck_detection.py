import rospy
import py_trees
import numpy as np

class StuckDetection(py_trees.behaviour.Behaviour):
    def __init__(self, lidar_processor, robot_vel_pub, move_forward_behaviour, name="StuckDetection", stuck_threshold=0.1, check_interval=1):
        super(StuckDetection, self).__init__(name)
        self.lidar_processor = lidar_processor
        self.robot_vel_pub = robot_vel_pub
        self.stuck_threshold = stuck_threshold
        self.check_interval = check_interval
        self.last_check_time = None
        self.move_forward_behaviour=move_forward_behaviour
        self.last_scan_ranges = None
    def update(self):
      
        current_time = rospy.Time.now()

        if self.last_check_time is None:
          self.last_check_time=current_time
          self.last_scan_ranges = self.lidar_processor.get_previous_scan_ranges()
          if self.last_scan_ranges is None:
                self.lidar_processor.update_previous_scan_ranges() #we update the scan if we don't have one
                self.last_scan_ranges=self.lidar_processor.get_previous_scan_ranges()
          return py_trees.common.Status.RUNNING

        elapsed_time = (current_time- self.last_check_time).to_sec()
        
        if elapsed_time < self.check_interval:
            return py_trees.common.Status.RUNNING
        else:
            current_scan_ranges = self.lidar_processor.get_scan()
            if current_scan_ranges is None or self.last_scan_ranges is None:
                self.last_check_time = current_time
                if current_scan_ranges is not None:
                    self.lidar_processor.update_previous_scan_ranges()
                    self.last_scan_ranges = self.lidar_processor.get_previous_scan_ranges()
                return py_trees.common.Status.FAILURE

            current_scan_ranges=np.array(current_scan_ranges.ranges)
            
            diff = np.abs(current_scan_ranges-self.last_scan_ranges)
            
            #check for NaN values
            diff = diff[~np.isnan(diff)]
            
            if len(diff) == 0 or np.max(diff) < self.stuck_threshold: #if the difference of scan values is not significant, the robot is stuck
              self.last_check_time = current_time
              self.lidar_processor.update_previous_scan_ranges()
              self.last_scan_ranges = self.lidar_processor.get_previous_scan_ranges()
              return py_trees.common.Status.SUCCESS #we are stuck
            else:
                self.last_check_time = current_time
                self.lidar_processor.update_previous_scan_ranges()
                self.last_scan_ranges = self.lidar_processor.get_previous_scan_ranges()
                return py_trees.common.Status.FAILURE