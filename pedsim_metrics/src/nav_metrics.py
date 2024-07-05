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
from pedsim_msgs.msg import AgentStates
from std_msgs.msg import Bool

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
        self.freq = 10 #rospy.get_param('~frequency', 0.0) 
        self.result_file_path = os.path.abspath('../tiago_ws_ros1/results/metrics') #rospy.get_param('~result_file', 'metrics')
        self.exp_tag = '1' #rospy.get_param('~experiment_tag', '1')
        self.use_navgoal_to_start = True #rospy.get_param('~use_navgoal_to_start', True)

        if self.use_navgoal_to_start == True:
            self.recording = False
        else:
            self.recording = True
        self.time_period = 1000  # seconds # Time period for the metrics computation
        self.last_time = rospy.Time.now()
        self.init = False # This will turn to True when the agents are received
        self.end_timer = rospy.Timer(rospy.Duration(1.0), self.timer_end_callback) # Timer interval 1.0 seconds. 
        
        # Load the metrics to compute
        for m in metrics.metrics.keys():
            param_name = 'nav_metrics_node/ros__parameters/metrics/' + m
            if rospy.has_param(param_name):
                ok = rospy.get_param(param_name, True)
                if ok:
                    self.metrics_to_compute[m] = 0.0
        rospy.loginfo('Metrics to compute: %s', self.metrics_to_compute)

        # If the recording frequency is not 0.0, start the recording timer, otherwise recording is directory done in the subscriber callback
        if self.freq > 0.0:
                self.agents = AgentStates()
                self.robot = Odometry()
                self.record_timer = rospy.Timer(rospy.Duration(1.0 / self.freq), self.timer_record_callback) 
        
    ## Initialize the subscribers
        # Agents subscriber 
        self.sub_agents = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.human_callback)
        # Robot state subscriber 
        self.sub_odom = rospy.Subscriber('/pedsim_simulator/robot_position', Odometry, self.robot_callback)
        self.sub_goal = rospy.Subscriber("/clicked_point", PointStamped, self.goal_callback)
        self.sub_goal_status = rospy.Subscriber("/goal_status", Bool, self.goal_status_callback)

    ## Subscriber call back functions
    def human_callback(self, agents): 
        self.init  = True
        #self.last_time = rospy.Time.now() # I don't understand why this line was here
        if self.recording ==True:
            if self.freq == 0.0:
                self.agents_list.append(agents)
            else:
                self.agents = agents
        #rospy.loginfo("Agents received!")

    def robot_callback(self, odom):
        self.init = True
        #self.last_time = rospy.Time.now()
        if self.recording == True:
            if self.freq == 0.0:
                self.robot_list.append(odom)
            else:
                self.robot = odom
        #rospy.loginfo("Robot received!")

    def goal_callback(self, msg): 
        self.robot_goal = msg
        if self.use_navgoal_to_start:
            rospy.loginfo("Goal received! Metrics node started recording!")
            self.use_navgoal_to_start = False
            self.recording = True

    def goal_status_callback(self, msg):
        if msg.data:  # If goal_reached is True
            rospy.loginfo("Pedsim evaluator stopping recording because the goal is reached")
            self.recording = False
            self.compute_metrics()
        else:
            rospy.loginfo("Goal is not reached...")

    # def recording_service(self, request):
    #     response = TriggerResponse()
    #     response.success = True
    #     if self.recording:
    #         rospy.loginfo("Pedsim evaluator stopping recording via service!")
    #         self.recording = False
    #         response.message = 'Hunav recording stopped'
    #         self.compute_metrics()
    #     else:
    #         rospy.loginfo("Hunav evaluator started recording!")
    #         self.recording = True
    #         response.message = 'Hunav recording started'
    #     return response

    def timer_end_callback(self, event):
        if self.init:
            secs = (rospy.Time.now() - self.last_time).to_sec()
            #rospy.loginfo("self.last_time: %s", self.last_time)
            rospy.loginfo("Time elapsed: %f" % secs)
            if secs >= self.time_period:
                self.recording = False
                rospy.loginfo("Timer ended. Pedsim evaluator stopping recording!")
                self.compute_metrics()

    def timer_record_callback(self, event):
        if self.recording and self.init:
            self.agents_list.append(self.agents)
            rospy.loginfo("Recording agents data")
            self.robot_list.append(self.robot)
            rospy.loginfo("Recording robot data")

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
            metric = metrics.metrics[m](self.agents_list, self.robot_list) # Retreive the metric function from the metrics dictionary, and calculate the metric
            self.metrics_to_compute[m] = metric[0] 
            if len(metric) > 1:
                self.metrics_lists[m] = metric[1]

        self.store_metrics(self.result_file_path)

        rospy.signal_shutdown('Metrics computation completed')

    def store_metrics(self, result_file):
        rospy.loginfo('Current working directory: %s', os.getcwd())
        list_file = result_file
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        
        rospy.loginfo('Storing metrics in %s' % list_file)
        rospy.loginfo('Storing step metrics in %s', list_file)

        file_was_created = os.path.exists(result_file)
        rospy.loginfo('file_was_created: %s', file_was_created)
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
            for i in range(length): # for all time stamps
                for m in self.metrics_lists.keys(): # for all keys 
                    v = self.metrics_lists[m] # v is the list of values of that key
                    #print(self.metrics_lists)
                    #print(v)
                    file2.write(str(v[i]) + '\t') # write for that time stamp
                file2.write('\n')
        rospy.loginfo('Metrics stored!')

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