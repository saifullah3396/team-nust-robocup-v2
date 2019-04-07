#!/usr/bin/python

import rospy
from visualization_publishers import BallInfoVisualizer
from visualization_publishers import GoalInfoVisualizer
from visualization_publishers import ObstaclesVisualizer
from visualization_publishers import TeamInfoVisualizer
from visualization_publishers import ObsLandmarkVisualizer
from visualization_publishers import FootstepsVisualizer


class VisualizationHandler:
    def __init__(self):
        self._visualizers = []
        self._visualizers.append(BallInfoVisualizer())
        self._visualizers.append(GoalInfoVisualizer())
        self._visualizers.append(ObstaclesVisualizer())
        self._visualizers.append(TeamInfoVisualizer())
        self._visualizers.append(ObsLandmarkVisualizer())
        self._visualizers.append(FootstepsVisualizer())
        pass

    def init(self):
        rospy.init_node('visualization_handler', anonymous=True)
        rate = rospy.Rate(10)
        for visualizer in self._visualizers:
            visualizer.init()
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        pass
        

if __name__ == '__main__':
    try:
        nh = VisualizationHandler()
        nh.init()
    except rospy.ROSInterruptException:
        nh.cleanup()
        pass
