#!/usr/bin/env python3
import rospy
import math
import copy
import numpy as np

# from hunav_msgs.msg import Agents
# from hunav_msgs.msg import Agent
from pedsim_msgs.msg import AgentStates # Note: replaced the above two lines to this
from geometry_msgs.msg import Pose
#from hunav_evaluator.sfm import SFM # Todo: replace this with the pedsim version

robot_radius = 0.35
agent_radius = 0.3

def euclidean_distance(pose, pose1):
    return math.sqrt((pose.position.x - pose1.position.x)**2 + (pose.position.y - pose1.position.y)**2)

def get_group_center(agents_i, group_id, distance):
    
    group = []
    for agent in agents_i:
        if agent.group_id == group_id:
            pose = Pose()
            pose.position.x = agent.position.position.x + (distance * math.cos(agent.yaw))
            pose.position.y = agent.position.position.y + (distance * math.sin(agent.yaw))
            group.append(pose)

    interaction_center = Pose()    
    for p in group:
        interaction_center.position.x += p.position.x
        interaction_center.position.y += p.position.y
    
    interaction_center.position.x = float(interaction_center.position.x/len(group))
    interaction_center.position.y = float(interaction_center.position.y/len(group))
    return interaction_center

def indicator_function(norm, k):
    if k == 'intimate':
        if norm < 0.45:
            return 1
        else:
            return 0
    elif k == 'personal':
        if norm >= 0.45 and norm < 1.2:
            return 1
        else:
            return 0
    elif k == 'social':
        if norm >= 1.2 and norm < 3.6:
            return 1
        else:
            return 0
    elif k == 'public':
        if norm >= 3.6:
            return 1
        else:
            return 0
    else:
        return 0
    
def get_time_stamps(agents, robot):
    time_list = []
    t0 = agents[0].header.stamp
    for a in agents: 
        t = a.header.stamp
        dur = t - t0
        s = float(dur.secs + dur.nsecs / 1e9)
        time_list.append(s)
    return time_list

def total_time(agents, robot):
    t2 = agents[-1].header.stamp
    t1 = agents[0].header.stamp
    dur = t2 - t1
    secs = float(dur.secs + dur.nsecs / 1e9)
    print('\nTime_to_reach_goal computed: %.2f secs' % secs)
    return [secs]

def robot_path_length(agents, robot):
    path_length = 0.0
    for i in range(len(robot)-1):
        path_length += euclidean_distance(robot[i+1].pose.pose, robot[i].pose.pose)
    print('Path_length computed: %.2f m' % path_length)
    return [path_length]

def cumulative_heading_changes(agents, robot):
    chc_list = [0.0]
    chc = 0
    for i in range(len(robot) - 1):
        norm = normalize_angle(robot[i].pose.pose.orientation.z - robot[i+1].pose.pose.orientation.z)
        if norm < 0.0:
            norm *= -1
        chc += norm
        chc_list.append(norm)
    print('Cumulative_heading_changes: %.2f rads' % chc)
    return [chc, chc_list]

def normalize_angle(ang):
    while (ang <= -math.pi): 
      ang += 2 * math.pi
    while (ang > math.pi):
      ang -= 2 * math.pi
    return ang

def avg_closest_person(agents, robot):
    min_dist_list = []
    avg_dist = 0
    for i in range(len(robot)):
        min_dist = 10000 
        for agent in agents[i].agent_states:
            d = euclidean_distance(robot[i].pose.pose, agent.pose) - robot_radius - agent_radius
            if(d < min_dist):
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        if(len(agents[i].agent_states)>0):
            avg_dist += min_dist
            min_dist_list.append(min_dist)

    avg_dist = avg_dist/len(robot)
    print('Average_closest_person: %.2f m' % avg_dist)
    return [avg_dist, min_dist_list]

def minimum_distance_to_people(agents, robot):
    min_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agent_states:
            d = euclidean_distance(robot[i].pose.pose, agent.pose) - robot_radius - agent_radius
            if d<0.0:
                d = 0.0
            min_distance.append(d) 
    
    min_dist = min(min_distance)
    
    print('Minimum_distance_to_people: %.2f m' % min_dist)

    return [min_dist]

def maximum_distance_to_people(agents, robot):
    max_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].agent_states:
            max_distance.append(euclidean_distance(robot[i].pose.pose, agent.pose) )- robot_radius - agent_radius
    
    max_dist = max(max_distance)
    
    print('Maximum_distance_to_people: %.2f m' % max_dist)

    return [max_dist]

def space_intrusions(agents, robot, k):
    space_intrusions = 0
    space_intrusions_list = [0] * len(robot)

    for i in range(len(robot)):
        min_dist = 10000
        for agent in agents[i].agent_states:
            d = euclidean_distance(robot[i].pose.pose, agent.pose) - robot_radius - agent_radius
            if d < min_dist:
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1
            space_intrusions_list[i]=1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0
    print('percentage: %.2f' % percentage)
    #print('space_intrusions_list: %s' % space_intrusions_list)

    return percentage, space_intrusions_list

def intimate_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'intimate')
    print('Intimate_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]

def personal_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'personal')
    print('Personal_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]
    
def social_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'social')
    print('Social_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]

def detect_groups(agents):
    group_ids = []
    for a in agents[0].agent_states:
        if(a.group_id != -1 and ((a.group_id in group_ids) == False)):
            group_ids.append(a.group_id)

    return group_ids


def group_space_intrusions(agents, robot, k):
    group_ids = detect_groups(agents)
    if(len(group_ids)==0):
        return [0.0]

    d=1.5
    space_intrusions = 0
    group_list = [0] * len(robot)
    for i in range(len(robot)):
        min_dist = 10000
        for id in group_ids:
            group_center = get_group_center(agents[i].agent_states, id, d)
            dist = euclidean_distance(robot[i].position, group_center.position) - robot[i].radius
            if dist < min_dist:
                min_dist = dist
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1
            group_list[i] = 1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return [percentage, group_list]

def group_intimate_space_intrusions(agents, robot):
    r = group_space_intrusions(agents, robot, 'intimate')
    print('Group_intimate_space_intrusions: %.2f %% of the total time' % r[0])
    return r

def group_personal_space_intrusions(agents, robot):
    r =  group_space_intrusions(agents, robot, 'personal')
    print('Group_personal_space_intrusions: %.2f %% of the total time' % r[0])
    return r

def group_social_space_intrusions(agents, robot):
    r =  group_space_intrusions(agents, robot, 'social')
    print('Group_social_space_intrusions: %.2f %% of the total time' % r[0])
    return r          

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.

# def collisions(agents, robot):
#     robot_coll_list = [0] * len(robot)
#     person_coll_list = [0] * len(robot)
#     robot_collisions = 0
#     person_collisions = 0

#     for i in range(len(robot)):
#         for agent in agents[i].agents:

#             if euclidean_distance(robot[i].position, agent.position) - robot[i].radius - agent.radius < 0.02:
                
#                 # Robot's angle
#                 nrx = (robot[i].position.position.x - agent.position.position.x) * math.cos(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.sin(agent.yaw)
#                 nry = -(robot[i].position.position.x - agent.position.position.x) * math.sin(agent.yaw) + (robot[i].position.position.y - agent.position.position.y) * math.cos(agent.yaw)
#                 alpha = math.atan2(nry, nrx)

#                 # Agent's angle
#                 nrx = (agent.position.position.x - robot[i].position.position.x) * math.cos(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.sin(robot[i].yaw)
#                 nry = -(agent.position.position.x - robot[i].position.position.x) * math.sin(robot[i].yaw) + (agent.position.position.y - robot[i].position.position.y) * math.cos(robot[i].yaw)
#                 alpha2 = math.atan2(nrx, nry)

#                 if abs(alpha) < abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
#                     robot_collisions += 1
#                     robot_coll_list[i] = 1
#                 elif abs(alpha) > abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
#                     person_collisions += 1
#                     person_coll_list[i] = 1
#                 elif abs(alpha) < abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
#                     #person_collision += 1
#                     robot_collisions += 1
#                     robot_coll_list[i] = 1
#                 elif abs(alpha) > abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
#                     #robot_collision += 1
#                     person_collisions += 1
#                     person_coll_list[i] = 1
#                 elif abs(alpha) == abs(alpha2) and robot[i].linear_vel == agent.linear_vel:
#                     robot_collisions += 1
#                     person_collisions += 1
#                     robot_coll_list[i] = 1
#                     person_coll_list[i] = 1

#     return robot_collisions, person_collisions, robot_coll_list, person_coll_list

# def robot_on_person_collision(agents, robot):

#     collision = collisions(agents, robot)
#     print('Robot_on_person_collision: %i ' % collision[0])

#     return [collision[0], collision[2]]

# def person_on_robot_collision(agents, robot):
    
#     collision = collisions(agents, robot)
#     print('Person_on_robot_collision: %i' % collision[1])

#     return [collision[1], collision[3]]


# def time_not_moving(agents, robot):
    
#     not_moving = [0]*len(robot)
#     time_step = total_time(agents, robot)[0]/len(agents)

#     count = 0
#     for index, r in enumerate(robot):
#         if(r.linear_vel < 0.01 and abs(r.angular_vel < 0.02)):
#             count=count+1
#             not_moving[index]=1
#     time_stopped = time_step*count
#     print('Time stopped: %i secs' % time_stopped)
#     return [time_stopped, not_moving]


# def goal_reached(agents, robot):
#     mind = 0.0
#     if(len(robot[-1].goals)):
#         for g in robot[-1].goals:
#             d = euclidean_distance(robot[-1].position, g) - robot[-1].goal_radius
#             if d<mind:
#                 return [True]
#     return [False]
   

# def final_goal_distance(agents, robot):
#     min_dist = 10000
#     if(len(robot[-1].goals)):
#         for g in robot[-1].goals:
#             d = euclidean_distance(robot[-1].position, g)
#             if d<min_dist:
#                 min_dist = d
#     return [min_dist]
    


# def minimum_goal_distance(agents, robot):
#     min_dist = 10000
#     for r in robot:
#         if(len(r.goals)):
#             for g in r.goals:
#                 d = euclidean_distance(r.position, g)
#                 if d<min_dist:
#                     min_dist = d
#     return [min_dist]

# SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
#ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University

def avg_robot_linear_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.twist.twist.linear.x)
        speed += r.twist.twist.linear.x

    speed = speed / len(robot)

    print('Average_robot_speed: %.2f m/s' % speed)
    return [speed, speed_list]

def avg_robot_angular_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.twist.twist.angular.z)
        speed += np.abs(r.twist.twist.angular.z)

    speed = speed / len(robot)

    print('Average_robot_speed: %.2f rad/s' % speed)
    return [speed, speed_list]



# def avg_acceleration(agents, robot):
#     acceleration = 0
#     acceleration_list = [0.0]
#     for i in range(len(robot) - 1):
#         dv = robot[i+1].linear_vel - robot[i].linear_vel
#         tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
#         ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
#         dur = (tf - ti).to_msg()
#         dt = float(dur.sec + dur.nanosec/1e9)
#         if dt != 0.0:
#             accel = dv/dt
#             acceleration += np.abs(accel)
#             acceleration_list.append(accel)
#         else:
#             acceleration_list.append(0.0)

#     acceleration = acceleration / len(robot)

#     print('Average_robot_acceleration: %.5f m/s^2' % acceleration)

#     return [acceleration, acceleration_list]


# def avg_overacceleration(agents, robot):
#     jerk = 0
#     jerk_list = [0.0]
#     for i in range(len(robot) - 1):
#         dv = robot[i+1].linear_vel - robot[i].linear_vel
#         tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
#         ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
#         dur = (tf - ti).to_msg()
#         dt = float(dur.sec + dur.nanosec/1e9)
#         if dt != 0.0:
#             acceleration = dv/dt
#             jerk += np.abs(acceleration/dt)
#             jerk_list.append(acceleration/dt)
#         else:
#             jerk_list.append(0.0)

#     jerk = jerk / len(robot)

#     print('Average_robot_jerk(over_acceleration): %.5f m/s^3' % jerk)

#     return [jerk, jerk_list]

## Skip group-aware policy and social force model metrics for now

metrics = {
    # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
    # igation Behaviors to Optimal RRT Planners,” International Journal of
    # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
    'time_to_reach_goal': total_time,
    'path_length': robot_path_length,
    'cumulative_heading_changes': cumulative_heading_changes,
    'avg_distance_to_closest_person': avg_closest_person,
    'minimum_distance_to_people': minimum_distance_to_people,
    'maximum_distance_to_people': maximum_distance_to_people,
    'intimate_space_intrusions': intimate_space_intrusions,
    'personal_space_intrusions': personal_space_intrusions,
    'social_space_intrusions': social_space_intrusions,
    # 'group_intimate_space_intrusions': group_intimate_space_intrusions, # TODO: these require group information
    # 'group_personal_space_intrusions': group_personal_space_intrusions,
    # 'group_social_space_intrusions': group_social_space_intrusions,
    # N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh,
    # M. Hussein, A. W. Gupta, M. Kapadia, and M. V ́azquez, “Sean 2.0:
    # Formalizing and generating social situations for robot navigation,”
    # IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–
    # 11 054, 2022
    #   - 'Total Path Length' (meters): similar to 'path_length'
    #   - 'Path Irregularity': (radians): total rotations in the robot's 
    #       traveled path greater than the total rotations in the search-based 
    #       path from the starting pose.
    #   - 'Path Efficiency': (meters): ratio between robot's traveled path and 
    #       geodesic distance of the search-based path from the starting pose.

    # true when the robot's final pose is within a specified distance of the goal. 
    # The final distance threshold is easily adjustable by the user, but defaults 
    # to 1.2m.
    #'completed': goal_reached,
    #(meters): the closest the robot passes to the target position.
    #'minimum_distance_to_target': minimum_goal_distance,  
    #(meters): distance between the last robot position and the target position.
    #'final_distance_to_target': final_goal_distance, 
    #   - 'Robot on Person Personal Distance Violation': number of times a robot 
    # approaches a person within the personal distance of the robot.
    # Similar to 'personal_space_intrusions'
    #   - 'Person on Robot Personal Distance Violation': number of times a person 
    # approaches the robot within the personal distance of the robot.
    #   - 'Intimate Distance Violation': number of times the robot approached within 
    # the intimate distance of a person.
    #   - 'Person on Robot Intimate Distance Violation': number of times a person 
    # approaches the robot within the intimate distance of the robot.
    #'robot_on_person_collision': robot_on_person_collision,
    #'person_on_robot_collision': person_on_robot_collision,
    #'time_not_moving': time_not_moving,
    # TODO: 'static_obstacle_collision': static_obs_collision,
    # number of times the robot collides with a static obstacle.

    # SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
    #ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University
    'avg_robot_linear_speed': avg_robot_linear_speed,
    'avg_robot_angular_speed': avg_robot_angular_speed,
    # 'avg_acceleration': avg_acceleration,
    # 'avg_overacceleration': avg_overacceleration,

    # Learning a Group-Aware Policy for Robot Navigation
    # Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2
    # 'avg_pedestrian_velocity': avg_pedestrian_velocity,
    # 'avg_closest_pedestrian_velocity': avg_closest_pedestrian_velocity,

    # # metrics based on Social Force Model employed in different papers
    # 'social_force_on_agents': social_force_on_agents,
    # 'social_force_on_robot': social_force_on_robot,
    # 'social_work': social_work,
    # 'obstacle_force_on_robot': obstacle_force_on_robot,
    # 'obstacle_force_on_agents': obstacle_force_on_agents,
}