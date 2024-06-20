#!/usr/bin/env python3
import rospy
import math
import numpy as np
import time
import os

from pedsim_metrics import metrics 

#import interactive_marker_twist_server 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse

class NavMetrics:
    def __init__(self):
    ## Initialize the node
        self.agents_list = []
        self.robot_list = []
        self.robot_goal = None
        self.metrics_to_compute = {}
        self.metrics_lists = {}

    ## Get parameters 
    # TODO: set parameters in the launch file
        #self.mode = 2 #rospy.get_param('~mode', 2) # 1: start with the service, 2: start when the robot goal is received
        # frequency of the metrics computation, it should be lower than the frequency of the simulation
        # if it is 0.0, the metrics are computed at the same frequency as the simulation
        self.freq = 0.0 #rospy.get_param('~frequency', 0.0) 
        self.result_file_path = 'metrics.txt' #rospy.get_param('~result_file', 'metrics')
        self.exp_tag = '1' #rospy.get_param('~experiment_tag', '1')
        self.use_navgoal_to_start = True #rospy.get_param('~use_navgoal_to_start', True)
            
    ## Initialize the subscribers
        # Agents subscriber 
        self.sub_agents = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.human_callback)
        # Robot state subscriber 
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.robot_callback)
        self.sub_goal = rospy.Subscriber("/clicked_point", PointStamped, self.goal_callback)

    ## Subscriber call back functions
    def human_callback(self, agents): 
        self.agents = agents

    def robot_callback(self, odom):
        self.robot = odom

    def goal_callback(self, msg): 
        self.robot_goal = msg
        if self.use_navgoal_to_start:
            rospy.loginfo("Goal received! Metrics node started recording!")
            self.use_navgoal_to_start = False
            self.recording = True

    def recording_service(self, request):
        response = TriggerResponse()
        response.success = True
        if self.recording:
            rospy.loginfo("Hunav evaluator stopping recording!")
            self.recording = False
            response.message = 'Hunav recording stopped'
            self.compute_metrics()
        else:
            rospy.loginfo("Hunav evaluator started recording!")
            self.recording = True
            response.message = 'Hunav recording started'
        return response

    def timer_end_callback(self, event):
        if self.init:
            secs = (rospy.Time.now() - self.last_time).to_sec()
            if secs >= self.time_period:
                self.recording = False
                rospy.loginfo("Hunav evaluator stopping recording!")
                self.compute_metrics()

    def timer_record_callback(self, event):
        if self.recording and self.init:
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)

    def compute_metrics(self):
        if not self.check_data():
            rospy.loginfo("Data not collected. Not computing metrics.")
            return
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        rospy.loginfo("Collected %i messages of agents and %i of robot" % (agents_size, robot_size))
        rospy.loginfo("Computing metrics...")

        self.metrics_lists['time_stamps'] = metrics.get_time_stamps(self.agents_list, self.robot_list)
        for m in self.metrics_to_compute.keys():
            metric = metrics.metrics[m](self.agents_list, self.robot_list)
            self.metrics_to_compute[m] = metric[0]
            if len(metric) > 1:
                self.metrics_lists[m] = metric[1]

        rospy.loginfo('Metrics computed:')
        self.store_metrics(self.result_file_path)

        rospy.signal_shutdown('Metrics computation completed')

    def store_metrics(self, result_file):
        list_file = result_file
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.exp_tag) + '.txt'

        file_was_created = os.path.exists(result_file)
        with open(result_file, 'a+') as file:
            if not file_was_created:
                file.write('experiment_tag\t')
                for m in self.metrics_to_compute.keys():
                    file.write(m + '\t')
                file.write('\n')
            file.write(self.exp_tag + '\t')
            for v in self.metrics_to_compute.values():
                file.write(str(v) + '\t')
            file.write('\n')

        with open(list_file, 'w') as file2:
            for m in self.metrics_lists.keys():
                file2.write(m + '\t')
            file2.write('\n')
            length = len(self.metrics_lists['time_stamps'])
            for i in range(length):
                for m in self.metrics_lists.keys():
                    v = self.metrics_lists[m]
                    file2.write(str(v[i]) + '\t')
                file2.write('\n')

    def check_data(self):
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        if agents_size == 0 and robot_size == 0:
            return False
        if abs(agents_size - robot_size) != 0:
            while len(self.agents_list) > len(self.robot_list):
                self.agents_list.pop()
            while len(self.robot_list) > len(self.agents_list):
                self.robot_list.pop()
        return True

if __name__ == '__main__':
    rospy.init_node('nav_metrics', anonymous=True)
    try:
        nav_metrics= NavMetrics()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass