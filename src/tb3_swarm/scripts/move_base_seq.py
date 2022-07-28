#!/usr/bin/env python
__author__ = 'santosh'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from calculate_distance_traveled import CalculateDistanceTraveled
from std_msgs.msg import Float64
import json


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        # Reading JSON file
        with open('/home/santosh/workspaces/uwb_swarm_localization/src/tb3_swarm/scripts/text.txt') as f:
            data = f.read()
        js = json.loads(data)
        # Get the task
        _task = rospy.get_param('move_base_seq/to_do_task')
        # points
        self.points_seq = js[_task]
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq = list()
        for i in range(0, len(self.points_seq)):
            yaweulerangles_seq.append(self.points_seq[str(i)][3])
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        self.robot1_total_distance = 0.0
        rospy.Subscriber('/robot1/moved_distance', Float64, self.mvd_distance)
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [self.points_seq[str(i)][0:3] for i in range(0, len(self.points_seq))]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #Create action client
        self.client = actionlib.SimpleActionClient('/robot1/move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" "+str(self.points_seq[str(self.goal_cnt)][4])+" is now being processed by the Action Server...")

    # def feedback_cb(self, feedback):
    #     #To print current pose at each feedback:
    #     #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
    #     rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.points_seq[str(self.goal_cnt)][4])+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.points_seq[str(self.goal_cnt-1)][4])+" reached")
            if self.goal_cnt< len(self.pose_seq):
                cdt1 = CalculateDistanceTraveled(robot_namespace="robot1")
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                # self.robot1_total_distance += cdt1.getTotalDistance()
                self.client.send_goal(next_goal, self.done_cb, self.active_cb)
                # self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)1

            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.loginfo(" Robot1 total distance = "+ str(self.robot1_total_distance))
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        cdt1 = CalculateDistanceTraveled(robot_namespace="robot1")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        # self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        self.client.send_goal(goal, self.done_cb, self.active_cb)
        # self.robot1_total_distance += cdt1.getTotalDistance()
        print("Robot1 distance" + str(self.robot1_total_distance))
        rospy.spin()

    def mvd_distance(self, data):
        self.robot1_total_distance = data

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
