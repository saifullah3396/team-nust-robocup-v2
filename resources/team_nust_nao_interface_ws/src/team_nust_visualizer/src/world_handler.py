#!/usr/bin/python

import rospy
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

class World:
    def __init__(self):
        self._publisher = rospy.Publisher('world_markers', MarkerArray, queue_size=10)
        self._markers = []
        self._ns = "world"
        self.init_markers()

    def init(self):
        rospy.init_node('world_publisher_node', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        self.update_markers()
        marker_array = MarkerArray()
        marker_array.markers = self._markers
        self._publisher.publish(marker_array)

    def init_markers(self):
        field_marker = Marker()
        field_marker.type = Marker.CUBE
        field_marker.action = Marker.ADD
        field_marker.pose.position.x = 0
        field_marker.pose.position.y = 0
        field_marker.pose.position.z = 0
        field_marker.pose.orientation.x = 0
        field_marker.pose.orientation.y = 0
        field_marker.pose.orientation.z = 0
        field_marker.scale.x = 11
        field_marker.scale.y = 8
        field_marker.scale.z = 0.1
        field_marker.color.r = 0
        field_marker.color.g = 0.3
        field_marker.color.b = 0
        field_marker.color.a = 1.0

        p1, p2, p3, p4 = Point(), Point(), Point(), Point()
        p1.x = 4.5
        p1.y = 3.0
        p1.z = 0.05
        p2.x = -4.5
        p2.y = 3.0
        p2.z = 0.05
        p3.x = -4.5
        p3.y = -3.0
        p3.z = 0.05
        p4.x = 4.5
        p4.y = -3.0
        p4.z = 0.05

        boundary_line_left = Marker()
        boundary_line_left.type = Marker.CUBE
        boundary_line_left.action = Marker.ADD
        boundary_line_left.scale.x = 9
        boundary_line_left.scale.y = 0.05
        boundary_line_left.scale.z = 0.01
        boundary_line_left.pose.position.x = 0.0
        boundary_line_left.pose.position.y = 3.0
        boundary_line_left.pose.position.z = 0.05
        boundary_line_left.pose.orientation.x = 0.0
        boundary_line_left.pose.orientation.y = 0.0
        boundary_line_left.pose.orientation.z = 0.0
        boundary_line_left.pose.orientation.w = 1
        boundary_line_left.color.r = 1.0
        boundary_line_left.color.g = 1.0
        boundary_line_left.color.b = 1.0
        boundary_line_left.color.a = 1.0

        boundary_line_right = Marker()
        boundary_line_right.type = Marker.CUBE
        boundary_line_right.action = Marker.ADD
        boundary_line_right.scale.x = 9
        boundary_line_right.scale.y = 0.05
        boundary_line_right.scale.z = 0.01
        boundary_line_right.pose.position.x = 0.0
        boundary_line_right.pose.position.y = -3.0
        boundary_line_right.pose.position.z = 0.05
        boundary_line_right.pose.orientation.x = 0.0
        boundary_line_right.pose.orientation.y = 0.0
        boundary_line_right.pose.orientation.z = 0.0
        boundary_line_right.pose.orientation.w = 1
        boundary_line_right.color.r = 1.0
        boundary_line_right.color.g = 1.0
        boundary_line_right.color.b = 1.0
        boundary_line_right.color.a = 1.0

        boundary_line_top = Marker()
        boundary_line_top.type = Marker.CUBE
        boundary_line_top.action = Marker.ADD
        boundary_line_top.scale.x = 0.05
        boundary_line_top.scale.y = 6.05
        boundary_line_top.scale.z = 0.01
        boundary_line_top.pose.position.x = 4.5
        boundary_line_top.pose.position.y = 0.0
        boundary_line_top.pose.position.z = 0.05
        boundary_line_top.pose.orientation.x = 0.0
        boundary_line_top.pose.orientation.y = 0.0
        boundary_line_top.pose.orientation.z = 0.0
        boundary_line_top.pose.orientation.w = 1
        boundary_line_top.color.r = 1.0
        boundary_line_top.color.g = 1.0
        boundary_line_top.color.b = 1.0
        boundary_line_top.color.a = 1.0

        boundary_line_bottom = Marker()
        boundary_line_bottom.type = Marker.CUBE
        boundary_line_bottom.action = Marker.ADD
        boundary_line_bottom.scale.x = 0.05
        boundary_line_bottom.scale.y = 6.05
        boundary_line_bottom.scale.z = 0.01
        boundary_line_bottom.pose.position.x = -4.5
        boundary_line_bottom.pose.position.y = 0.0
        boundary_line_bottom.pose.position.z = 0.05
        boundary_line_bottom.pose.orientation.x = 0.0
        boundary_line_bottom.pose.orientation.y = 0.0
        boundary_line_bottom.pose.orientation.z = 0.0
        boundary_line_bottom.pose.orientation.w = 1
        boundary_line_bottom.color.r = 1.0
        boundary_line_bottom.color.g = 1.0
        boundary_line_bottom.color.b = 1.0
        boundary_line_bottom.color.a = 1.0

        boundary_line_middle = Marker()
        boundary_line_middle.type = Marker.CUBE
        boundary_line_middle.action = Marker.ADD
        boundary_line_middle.scale.x = 0.05
        boundary_line_middle.scale.y = 6.0
        boundary_line_middle.scale.z = 0.01
        boundary_line_middle.pose.position.x = 0.0
        boundary_line_middle.pose.position.y = 0.0
        boundary_line_middle.pose.position.z = 0.05
        boundary_line_middle.pose.orientation.x = 0.0
        boundary_line_middle.pose.orientation.y = 0.0
        boundary_line_middle.pose.orientation.z = 0.0
        boundary_line_middle.pose.orientation.w = 1
        boundary_line_middle.color.r = 1.0
        boundary_line_middle.color.g = 1.0
        boundary_line_middle.color.b = 1.0
        boundary_line_middle.color.a = 1.0

        boundary_line_top_penalty_left = Marker()
        boundary_line_top_penalty_left.type = Marker.CUBE
        boundary_line_top_penalty_left.action = Marker.ADD
        boundary_line_top_penalty_left.scale.x = 0.65
        boundary_line_top_penalty_left.scale.y = 0.05
        boundary_line_top_penalty_left.scale.z = 0.01
        boundary_line_top_penalty_left.pose.position.x = 4.2
        boundary_line_top_penalty_left.pose.position.y = 1.1
        boundary_line_top_penalty_left.pose.position.z = 0.05
        boundary_line_top_penalty_left.pose.orientation.x = 0.0
        boundary_line_top_penalty_left.pose.orientation.y = 0.0
        boundary_line_top_penalty_left.pose.orientation.z = 0.0
        boundary_line_top_penalty_left.pose.orientation.w = 1
        boundary_line_top_penalty_left.color.r = 1.0
        boundary_line_top_penalty_left.color.g = 1.0
        boundary_line_top_penalty_left.color.b = 1.0
        boundary_line_top_penalty_left.color.a = 1.0

        boundary_line_top_penalty_mid = Marker()
        boundary_line_top_penalty_mid.type = Marker.CUBE
        boundary_line_top_penalty_mid.action = Marker.ADD
        boundary_line_top_penalty_mid.scale.x = 0.05
        boundary_line_top_penalty_mid.scale.y = 2.2
        boundary_line_top_penalty_mid.scale.z = 0.01
        boundary_line_top_penalty_mid.pose.position.x = 3.9
        boundary_line_top_penalty_mid.pose.position.y = 0.0
        boundary_line_top_penalty_mid.pose.position.z = 0.05
        boundary_line_top_penalty_mid.pose.orientation.x = 0.0
        boundary_line_top_penalty_mid.pose.orientation.y = 0.0
        boundary_line_top_penalty_mid.pose.orientation.z = 0.0
        boundary_line_top_penalty_mid.pose.orientation.w = 1
        boundary_line_top_penalty_mid.color.r = 1.0
        boundary_line_top_penalty_mid.color.g = 1.0
        boundary_line_top_penalty_mid.color.b = 1.0
        boundary_line_top_penalty_mid.color.a = 1.0

        boundary_line_top_penalty_right = Marker()
        boundary_line_top_penalty_right.type = Marker.CUBE
        boundary_line_top_penalty_right.action = Marker.ADD
        boundary_line_top_penalty_right.scale.x = 0.65
        boundary_line_top_penalty_right.scale.y = 0.05
        boundary_line_top_penalty_right.scale.z = 0.01
        boundary_line_top_penalty_right.pose.position.x = 4.2
        boundary_line_top_penalty_right.pose.position.y = -1.1
        boundary_line_top_penalty_right.pose.position.z = 0.05
        boundary_line_top_penalty_right.pose.orientation.x = 0.0
        boundary_line_top_penalty_right.pose.orientation.y = 0.0
        boundary_line_top_penalty_right.pose.orientation.z = 0.0
        boundary_line_top_penalty_right.pose.orientation.w = 1
        boundary_line_top_penalty_right.color.r = 1.0
        boundary_line_top_penalty_right.color.g = 1.0
        boundary_line_top_penalty_right.color.b = 1.0
        boundary_line_top_penalty_right.color.a = 1.0

        boundary_line_bottom_penalty_left = Marker()
        boundary_line_bottom_penalty_left.type = Marker.CUBE
        boundary_line_bottom_penalty_left.action = Marker.ADD
        boundary_line_bottom_penalty_left.scale.x = 0.65
        boundary_line_bottom_penalty_left.scale.y = 0.05
        boundary_line_bottom_penalty_left.scale.z = 0.01
        boundary_line_bottom_penalty_left.pose.position.x = -4.2
        boundary_line_bottom_penalty_left.pose.position.y = 1.1
        boundary_line_bottom_penalty_left.pose.position.z = 0.05
        boundary_line_bottom_penalty_left.pose.orientation.x = 0.0
        boundary_line_bottom_penalty_left.pose.orientation.y = 0.0
        boundary_line_bottom_penalty_left.pose.orientation.z = 0.0
        boundary_line_bottom_penalty_left.pose.orientation.w = 1
        boundary_line_bottom_penalty_left.color.r = 1.0
        boundary_line_bottom_penalty_left.color.g = 1.0
        boundary_line_bottom_penalty_left.color.b = 1.0
        boundary_line_bottom_penalty_left.color.a = 1.0

        boundary_line_bottom_penalty_mid = Marker()
        boundary_line_bottom_penalty_mid.type = Marker.CUBE
        boundary_line_bottom_penalty_mid.action = Marker.ADD
        boundary_line_bottom_penalty_mid.scale.x = 0.05
        boundary_line_bottom_penalty_mid.scale.y = 2.2
        boundary_line_bottom_penalty_mid.scale.z = 0.01
        boundary_line_bottom_penalty_mid.pose.position.x = -3.9
        boundary_line_bottom_penalty_mid.pose.position.y = 0.0
        boundary_line_bottom_penalty_mid.pose.position.z = 0.05
        boundary_line_bottom_penalty_mid.pose.orientation.x = 0.0
        boundary_line_bottom_penalty_mid.pose.orientation.y = 0.0
        boundary_line_bottom_penalty_mid.pose.orientation.z = 0.0
        boundary_line_bottom_penalty_mid.pose.orientation.w = 1
        boundary_line_bottom_penalty_mid.color.r = 1.0
        boundary_line_bottom_penalty_mid.color.g = 1.0
        boundary_line_bottom_penalty_mid.color.b = 1.0
        boundary_line_bottom_penalty_mid.color.a = 1.0

        boundary_line_bottom_penalty_right = Marker()
        boundary_line_bottom_penalty_right.type = Marker.CUBE
        boundary_line_bottom_penalty_right.action = Marker.ADD
        boundary_line_bottom_penalty_right.scale.x = 0.65
        boundary_line_bottom_penalty_right.scale.y = 0.05
        boundary_line_bottom_penalty_right.scale.z = 0.01
        boundary_line_bottom_penalty_right.pose.position.x = -4.2
        boundary_line_bottom_penalty_right.pose.position.y = -1.1
        boundary_line_bottom_penalty_right.pose.position.z = 0.05
        boundary_line_bottom_penalty_right.pose.orientation.x = 0.0
        boundary_line_bottom_penalty_right.pose.orientation.y = 0.0
        boundary_line_bottom_penalty_right.pose.orientation.z = 0.0
        boundary_line_bottom_penalty_right.pose.orientation.w = 1
        boundary_line_bottom_penalty_right.color.r = 1.0
        boundary_line_bottom_penalty_right.color.g = 1.0
        boundary_line_bottom_penalty_right.color.b = 1.0
        boundary_line_bottom_penalty_right.color.a = 1.0

        field_circle_outer = Marker()
        field_circle_outer.type = Marker.CYLINDER
        field_circle_outer.action = Marker.ADD
        field_circle_outer.scale.x = 1.525
        field_circle_outer.scale.y = 1.525
        field_circle_outer.scale.z = 0.01
        field_circle_outer.pose.position.x = 0.0
        field_circle_outer.pose.position.y = 0.0
        field_circle_outer.pose.position.z = 0.05
        field_circle_outer.pose.orientation.x = 0.0
        field_circle_outer.pose.orientation.y = 0.0
        field_circle_outer.pose.orientation.z = 0.0
        field_circle_outer.pose.orientation.w = 1
        field_circle_outer.color.r = 1.0
        field_circle_outer.color.g = 1.0
        field_circle_outer.color.b = 1.0
        field_circle_outer.color.a = 1.0

        field_circle_inner = Marker()
        field_circle_inner.type = Marker.CYLINDER
        field_circle_inner.action = Marker.ADD
        field_circle_inner.scale.x = 1.475
        field_circle_inner.scale.y = 1.475
        field_circle_inner.scale.z = 0.01
        field_circle_inner.pose.position.x = 0.0
        field_circle_inner.pose.position.y = 0.0
        field_circle_inner.pose.position.z = 0.051
        field_circle_inner.pose.orientation.x = 0.0
        field_circle_inner.pose.orientation.y = 0.0
        field_circle_inner.pose.orientation.z = 0.0
        field_circle_inner.pose.orientation.w = 1
        field_circle_inner.color.r = 0.0
        field_circle_inner.color.g = 0.3
        field_circle_inner.color.b = 0.0
        field_circle_inner.color.a = 1.0

        field_circle_inner_line = Marker()
        field_circle_inner_line.type = Marker.CUBE
        field_circle_inner_line.action = Marker.ADD
        field_circle_inner_line.scale.x = 0.05
        field_circle_inner_line.scale.y = 1.5
        field_circle_inner_line.scale.z = 0.01
        field_circle_inner_line.pose.position.x = 0.0
        field_circle_inner_line.pose.position.y = 0.0
        field_circle_inner_line.pose.position.z = 0.052
        field_circle_inner_line.pose.orientation.x = 0.0
        field_circle_inner_line.pose.orientation.y = 0.0
        field_circle_inner_line.pose.orientation.z = 0.0
        field_circle_inner_line.pose.orientation.w = 1
        field_circle_inner_line.color.r = 1.0
        field_circle_inner_line.color.g = 1.0
        field_circle_inner_line.color.b = 1.0
        field_circle_inner_line.color.a = 1.0

        goal_cylinder_top_left = Marker()
        goal_cylinder_top_left.type = Marker.CYLINDER
        goal_cylinder_top_left.action = Marker.ADD
        goal_cylinder_top_left.scale.x = 0.1
        goal_cylinder_top_left.scale.y = 0.1
        goal_cylinder_top_left.scale.z = 0.9
        goal_cylinder_top_left.pose.position.x = 4.5
        goal_cylinder_top_left.pose.position.y = 0.8
        goal_cylinder_top_left.pose.position.z = 0.05 + 0.45
        goal_cylinder_top_left.pose.orientation.x = 0.0
        goal_cylinder_top_left.pose.orientation.y = 0.0
        goal_cylinder_top_left.pose.orientation.z = 0.0
        goal_cylinder_top_left.pose.orientation.w = 1
        goal_cylinder_top_left.color.r = 1.0
        goal_cylinder_top_left.color.g = 1.0
        goal_cylinder_top_left.color.b = 1.0
        goal_cylinder_top_left.color.a = 1.0

        goal_cylinder_top_mid = Marker()
        goal_cylinder_top_mid.type = Marker.CYLINDER
        goal_cylinder_top_mid.action = Marker.ADD
        goal_cylinder_top_mid.scale.x = 0.1
        goal_cylinder_top_mid.scale.y = 0.1
        goal_cylinder_top_mid.scale.z = 1.5
        goal_cylinder_top_mid.pose.position.x = 4.5
        goal_cylinder_top_mid.pose.position.y = 0.0
        goal_cylinder_top_mid.pose.position.z = 0.05 + 0.85
        goal_cylinder_top_mid.pose.orientation.x = 1.0
        goal_cylinder_top_mid.pose.orientation.y = 0.0
        goal_cylinder_top_mid.pose.orientation.z = 0.0
        goal_cylinder_top_mid.pose.orientation.w = 1
        goal_cylinder_top_mid.color.r = 1.0
        goal_cylinder_top_mid.color.g = 1.0
        goal_cylinder_top_mid.color.b = 1.0
        goal_cylinder_top_mid.color.a = 1.0

        goal_cylinder_top_right = Marker()
        goal_cylinder_top_right.type = Marker.CYLINDER
        goal_cylinder_top_right.action = Marker.ADD
        goal_cylinder_top_right.scale.x = 0.1
        goal_cylinder_top_right.scale.y = 0.1
        goal_cylinder_top_right.scale.z = 0.9
        goal_cylinder_top_right.pose.position.x = 4.5
        goal_cylinder_top_right.pose.position.y = -0.8
        goal_cylinder_top_right.pose.position.z = 0.05 + 0.45
        goal_cylinder_top_right.pose.orientation.x = 0.0
        goal_cylinder_top_right.pose.orientation.y = 0.0
        goal_cylinder_top_right.pose.orientation.z = 0.0
        goal_cylinder_top_right.pose.orientation.w = 1
        goal_cylinder_top_right.color.r = 1.0
        goal_cylinder_top_right.color.g = 1.0
        goal_cylinder_top_right.color.b = 1.0
        goal_cylinder_top_right.color.a = 1.0

        goal_cylinder_bottom_left = Marker()
        goal_cylinder_bottom_left.type = Marker.CYLINDER
        goal_cylinder_bottom_left.action = Marker.ADD
        goal_cylinder_bottom_left.scale.x = 0.1
        goal_cylinder_bottom_left.scale.y = 0.1
        goal_cylinder_bottom_left.scale.z = 0.9
        goal_cylinder_bottom_left.pose.position.x = -4.5
        goal_cylinder_bottom_left.pose.position.y = 0.8
        goal_cylinder_bottom_left.pose.position.z = 0.05 + 0.45
        goal_cylinder_bottom_left.pose.orientation.x = 0.0
        goal_cylinder_bottom_left.pose.orientation.y = 0.0
        goal_cylinder_bottom_left.pose.orientation.z = 0.0
        goal_cylinder_bottom_left.pose.orientation.w = 1
        goal_cylinder_bottom_left.color.r = 1.0
        goal_cylinder_bottom_left.color.g = 1.0
        goal_cylinder_bottom_left.color.b = 1.0
        goal_cylinder_bottom_left.color.a = 1.0

        goal_cylinder_bottom_mid = Marker()
        goal_cylinder_bottom_mid.type = Marker.CYLINDER
        goal_cylinder_bottom_mid.action = Marker.ADD
        goal_cylinder_bottom_mid.scale.x = 0.1
        goal_cylinder_bottom_mid.scale.y = 0.1
        goal_cylinder_bottom_mid.scale.z = 1.5
        goal_cylinder_bottom_mid.pose.position.x = -4.5
        goal_cylinder_bottom_mid.pose.position.y = 0.0
        goal_cylinder_bottom_mid.pose.position.z = 0.05 + 0.85
        goal_cylinder_bottom_mid.pose.orientation.x = 1.0
        goal_cylinder_bottom_mid.pose.orientation.y = 0.0
        goal_cylinder_bottom_mid.pose.orientation.z = 0.0
        goal_cylinder_bottom_mid.pose.orientation.w = 1
        goal_cylinder_bottom_mid.color.r = 1.0
        goal_cylinder_bottom_mid.color.g = 1.0
        goal_cylinder_bottom_mid.color.b = 1.0
        goal_cylinder_bottom_mid.color.a = 1.0

        goal_cylinder_bottom_right = Marker()
        goal_cylinder_bottom_right.type = Marker.CYLINDER
        goal_cylinder_bottom_right.action = Marker.ADD
        goal_cylinder_bottom_right.scale.x = 0.1
        goal_cylinder_bottom_right.scale.y = 0.1
        goal_cylinder_bottom_right.scale.z = 0.9
        goal_cylinder_bottom_right.pose.position.x = -4.5
        goal_cylinder_bottom_right.pose.position.y = -0.8
        goal_cylinder_bottom_right.pose.position.z = 0.05 + 0.45
        goal_cylinder_bottom_right.pose.orientation.x = 0.0
        goal_cylinder_bottom_right.pose.orientation.y = 0.0
        goal_cylinder_bottom_right.pose.orientation.z = 0.0
        goal_cylinder_bottom_right.pose.orientation.w = 1
        goal_cylinder_bottom_right.color.r = 1.0
        goal_cylinder_bottom_right.color.g = 1.0
        goal_cylinder_bottom_right.color.b = 1.0
        goal_cylinder_bottom_right.color.a = 1.0

        top_penalty_marker = Marker()
        top_penalty_marker.type = Marker.CYLINDER
        top_penalty_marker.action = Marker.ADD
        top_penalty_marker.scale.x = 0.1
        top_penalty_marker.scale.y = 0.1
        top_penalty_marker.scale.z = 0.01
        top_penalty_marker.pose.position.x = 3.2
        top_penalty_marker.pose.position.y = 0.0
        top_penalty_marker.pose.position.z = 0.05
        top_penalty_marker.pose.orientation.x = 0.0
        top_penalty_marker.pose.orientation.y = 0.0
        top_penalty_marker.pose.orientation.z = 0.0
        top_penalty_marker.pose.orientation.w = 1
        top_penalty_marker.color.r = 1.0
        top_penalty_marker.color.g = 1.0
        top_penalty_marker.color.b = 1.0
        top_penalty_marker.color.a = 1.0

        bottom_penalty_marker = Marker()
        bottom_penalty_marker.type = Marker.CYLINDER
        bottom_penalty_marker.action = Marker.ADD
        bottom_penalty_marker.scale.x = 0.1
        bottom_penalty_marker.scale.y = 0.1
        bottom_penalty_marker.scale.z = 0.01
        bottom_penalty_marker.pose.position.x = -3.2
        bottom_penalty_marker.pose.position.y = 0.0
        bottom_penalty_marker.pose.position.z = 0.05
        bottom_penalty_marker.pose.orientation.x = 0.0
        bottom_penalty_marker.pose.orientation.y = 0.0
        bottom_penalty_marker.pose.orientation.z = 0.0
        bottom_penalty_marker.pose.orientation.w = 1
        bottom_penalty_marker.color.r = 1.0
        bottom_penalty_marker.color.g = 1.0
        bottom_penalty_marker.color.b = 1.0
        bottom_penalty_marker.color.a = 1.0

        self._markers.append(field_marker)
        self._markers.append(boundary_line_left)
        self._markers.append(boundary_line_right)
        self._markers.append(boundary_line_top)
        self._markers.append(boundary_line_bottom)
        self._markers.append(boundary_line_middle)
        self._markers.append(boundary_line_top_penalty_left)
        self._markers.append(boundary_line_top_penalty_mid)
        self._markers.append(boundary_line_top_penalty_right)
        self._markers.append(boundary_line_bottom_penalty_left)
        self._markers.append(boundary_line_bottom_penalty_mid)
        self._markers.append(boundary_line_bottom_penalty_right)
        self._markers.append(field_circle_inner)
        self._markers.append(field_circle_outer)
        self._markers.append(field_circle_inner_line)
        self._markers.append(goal_cylinder_top_left)
        self._markers.append(goal_cylinder_top_mid)
        self._markers.append(goal_cylinder_top_right)
        self._markers.append(goal_cylinder_bottom_left)
        self._markers.append(goal_cylinder_bottom_mid)
        self._markers.append(goal_cylinder_bottom_right)
        self._markers.append(top_penalty_marker)
        self._markers.append(bottom_penalty_marker)
        for idx, marker in enumerate(self._markers):
            marker.header.frame_id = "world_frame"
            marker.ns = self._ns
            marker.id = idx

    def update_markers(self):
        duration = rospy.Duration()
        time = rospy.Time.now()
        for idx, marker in enumerate(self._markers):
            marker.lifetime = duration
            marker.header.stamp = time


if __name__ == '__main__':
    try:
        world = World()
        world.init()
    except rospy.ROSInterruptException as e:
        print e
        pass
