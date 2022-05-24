#! /usr/bin/env python3

import rospy;
import numpy as np
import rospy
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import LaserScan;
from visualization_msgs.msg import Marker


class LaserNode:

    def __init__(self):

        self.laserScan = LaserScan();
        self.marker = Marker();
        self.pub = rospy.Publisher("point_positions", Marker, queue_size=1);
        self.rate = rospy.Rate(25);
        self.inital_time = rospy.Time.now();

        rospy.Subscriber("scan", LaserScan, self.laser_callback);

    def laser_callback(self, data):

        self.laserScan = data;
        # print(type(self.laserScan.ranges));

    def run(self):

        while not rospy.is_shutdown():
            self.pub.publish(self.create_marker());
            self.rate.sleep();
            # print(self.laserScan.header.stamp);

    def create_marker(self):
        now = rospy.Time.now()

        t = (now - self.inital_time).to_sec()
        marker = Marker()

        marker.header.frame_id = self.laserScan.header.frame_id;
        marker.header.stamp = self.laserScan.header.stamp;

        marker.type = Marker.POINTS

        marker.color.a = 1.0

        marker.color.r, marker.color.g, marker.color.b = (1., 0., 0.)

        marker.scale.x = 0.5
        marker.scale.y = 0.5

        for i in range(len(self.laserScan.ranges)):

            r = self.laserScan.ranges[i];

            if (r >= self.laserScan.range_min and r <= self.laserScan.range_max):
                p = Point();

                fi = self.laserScan.angle_min + i * self.laserScan.angle_increment;
                p.x = r * math.cos(fi);
                p.y = r * math.sin(fi);

                marker.points.append(p);

        return marker;


if __name__ == '__main__':

    rospy.init_node('ellipsis_points');

    try:
        laserNode = LaserNode();
        laserNode.run();
    except rospy.ROSInterruptException:
        pass;
