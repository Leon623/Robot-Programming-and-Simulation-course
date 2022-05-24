#! /usr/bin/env python3

import rospy;
import numpy as np
import rospy
import math
import tf2_ros;
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import LaserScan;
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Quaternion,PoseStamped, QuaternionStamped
from tf_conversions import transformations;
from tf2_geometry_msgs import do_transform_pose


class LaserNode:

    def __init__(self):

        self.global_frame = rospy.get_param('~global_frame', 'map');
        self.accumulate_points = rospy.get_param('~accumulate_points',False);
        self.accumulate_every_n = rospy.get_param('~accumulate_every_n',50);
        self.n = 0;

        self.tf_buffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tf_buffer);
        self.theta_transform = 0;
        self.x_tranfsorm = 0;
        self.y_transform = 0;
        self.laserScan = LaserScan();
        self.tf_bcaster = tf2_ros.TransformBroadcaster();
        self.clear_marker = False;
        self.marker = Marker();

        self.pub = rospy.Publisher("point_positions", Marker, queue_size=1);
        self.rate = rospy.Rate(25);
        self.inital_time = rospy.Time.now();

        rospy.Subscriber("scan", LaserScan, self.laser_callback);

        print("Starting the laser scan visualizer node.");
        print("global_frame: {}, accumulate_points: {}, accumulate_every_n: {}".format(self.global_frame,self.accumulate_points,self.accumulate_every_n));

    def laser_callback(self, data):

        self.n+=1;

        if(self.laserScan.header.stamp < self.marker.header.stamp):

            self.marker.header.stamp = self.laserScan.header.stamp;
            self.marker.points.clear();
            self.tf_buffer.clear();

        if(self.accumulate_points):

            if(self.n >= self.accumulate_every_n):

                self.n = 0;

            else: return;

        else:
            self.marker.points.clear();


        self.laserScan = data;
        source_frame = self.laserScan.header.frame_id;

        pose_obs = PoseStamped()

        try:

            tf_map_laser = self.tf_buffer.lookup_transform(self.global_frame, source_frame,rospy.Time(0));
            rotation = tf_map_laser.transform.rotation;
            pose_map_laser = do_transform_pose(pose_obs, tf_map_laser).pose;
            z = 2*math.atan2(rotation.z,rotation.w);

            self.theta_transform = z;
            self.x_tranfsorm = pose_map_laser.position.x;
            self.y_transform = pose_map_laser.position.y;

            #print(z)
            #print(pose_map_laser.position.x)
            #print(pose_map_laser.position.y)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:

            rospy.logwarn(ex);

        return None;
    def create_marker(self):

        if(not self.accumulate_points):

            self.marker.points.clear();

        self.marker.type = Marker.POINTS
        self.marker.color.a = 1.0
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (1., 0, 0.)
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.header.frame_id = self.global_frame;
        marker = self.marker;

        #marker.header.stamp = self.laserScan.header.stamp;

        for i in range(len(self.laserScan.ranges)):

            r = self.laserScan.ranges[i];

            if (r >= self.laserScan.range_min and r <= self.laserScan.range_max):
                p = Point();

                fi = self.laserScan.angle_min + i * self.laserScan.angle_increment + self.theta_transform;
                p.x = r * math.cos(fi) + self.x_tranfsorm;
                p.y = r * math.sin(fi) + self.y_transform;

                marker.points.append(p);

        return marker;

    def run(self):

        while not rospy.is_shutdown():
            self.pub.publish(self.create_marker());
            self.rate.sleep();

if __name__ == '__main__':

    rospy.init_node('ellipsis_points');

    try:
        laserNode = LaserNode();
        laserNode.run();
    except rospy.ROSInterruptException:
        pass;
