#! /usr/bin/env python3

import rosbag;
import math;
import sys;


class Turtle:

    def __init__(self, pose_x, pose_y, pose_time):
        self.pose_x = pose_x;
        self.pose_y = pose_y;
        self.pose_time = pose_time;
        self.covered_distance = 0;
        self.time_duration = self.pose_time[-1] - self.pose_time[0];

    def calculate_covered_distance(self):
        for i in range(len(self.pose_x) - 1):
            self.covered_distance += math.sqrt(
                (self.pose_x[i] - self.pose_x[i + 1]) ** 2 + (self.pose_y[i] - self.pose_y[i + 1]) ** 2);

        return self.covered_distance;

    def calculate_avg_velocity(self):
        return self.covered_distance / self.time_duration;


if __name__ == '__main__':

    bag_name = sys.argv[1];
    bag = rosbag.Bag(bag_name);

    # initializing data
    marko_pose_x, marko_pose_y, marko_pose_time, turtle1_pose_x, turtle1_pose_y, turtle1_pose_time = ([] for i in
                                                                                                      range(6))
    written_messages = 0;
    runner_pose_topic = "/Marko/pose";
    chaser_pose_topic = "/turtle1/pose";

    # collecting data
    with rosbag.Bag('processed_chase.bag', 'w') as outbag:
        for (topic, msg, t) in bag.read_messages():

            if (topic == runner_pose_topic):

                marko_pose_x.append(msg.x);
                marko_pose_y.append(msg.y);
                marko_pose_time.append(t.to_time());
                outbag.write('/runner/pose', msg, t);
                written_messages += 1;

            elif (topic == chaser_pose_topic):

                turtle1_pose_x.append(msg.x);
                turtle1_pose_y.append(msg.y);
                turtle1_pose_time.append(t.to_time());
                outbag.write('chaser/pose', msg, t);
                written_messages += 1;

        # creating turtle objects
        chaser = Turtle(turtle1_pose_x, turtle1_pose_y, turtle1_pose_time);
        runner = Turtle(marko_pose_x, marko_pose_y, marko_pose_time);

        # calculation and printing
        print("Processing input bagfile: {}".format(bag_name));
        print("Runner");
        print("  Covered distance: {}".format(runner.calculate_covered_distance()))
        print("  Average velocity: {}".format(runner.calculate_avg_velocity()));
        print("Chaser");
        print("  Covered distance: {}".format(chaser.calculate_covered_distance()));
        print("  Average velocity: {}".format(chaser.calculate_avg_velocity()));
        print("Chase duration: {}".format(chaser.time_duration));
        print("Wrote {} messages to processed_chase.bag".format(written_messages));
