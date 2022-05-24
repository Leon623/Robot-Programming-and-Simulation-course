#!/usr/bin/env python3

import rospy;
from tf2_msgs.msg import TFMessage;
from geometry_msgs.msg import TransformStamped;
from geometry_msgs.msg import Point;
from visualization_msgs.msg import Marker;

class Trajectory_visualizer():

    def __init__(self):

        self.frame_id = rospy.get_param('~frame_id','map');
        self.child_frame_id = rospy.get_param('~child_id','pioneer/base_link');
        self.sub = rospy.Subscriber('tf',TFMessage,self.tf_callback)
        self.tf_msg = TFMessage();
        
        self.marker = Marker();
        self.marker.color.a = 1.0;
        self.marker.scale.x = 0.5;
        #self.marker.scale.y = 0.5;
        self.marker.color.r, self.marker.color.g, self.marker.color.b = (0, 1., 0.);
        self.marker.type = Marker.LINE_STRIP;
        self.marker.pose.orientation.w = 1.0;
        
        self.marker.header.stamp = rospy.Time(0);
        self.pub = rospy.Publisher("robot_position", Marker, queue_size=1);
        self.rate = rospy.Rate(1);

    def run(self):

        while not rospy.is_shutdown():

            self.pub.publish(self.marker);
            self.rate.sleep();
            #print("test")

    def tf_callback(self,data):

        self.tf_msg = data;

        for x in self.tf_msg.transforms:

            if (x.child_frame_id == self.child_frame_id and x.header.frame_id == self.frame_id):

                if (x.header.stamp < self.marker.header.stamp):
                    self.marker.points.clear();
                    print("Timestamp has jumpedbackwards, clearing the trajectory.");

                robot_translation = x.transform.translation;
                p = Point();
                p.x = robot_translation.x;
                p.y = robot_translation.y;
                p.z = robot_translation.z;

                self.marker.header.frame_id = x.header.frame_id;
                self.marker.header.stamp = x.header.stamp;
                self.marker.points.append(p);

if __name__ == '__main__':

    rospy.init_node("my_node");

    try:
        tv = Trajectory_visualizer();
        tv.run();
    except rospy.ROSInterruptException:
        pass;
