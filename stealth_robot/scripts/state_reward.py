#!/usr/bin/env python3

from math import sqrt, floor

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rl_msgs.msg import RLStateReward
from actions import StealthActions


""" define states and rewards """

class StateReward:
    def __init__(self, goal_x, goal_y, cloud, step_dist):
        """ Constructor for class to assign states and rewards """
        self.actor = StealthActions(distance=step_dist)
        self.action: int

        self.cloud = cloud

        # total visibility from searchlights
        self.visibility: float

        # get robot pose with AMCL
        self.amcl_pose = rospy.Subscriber("/amcl_pose",
                PoseWithCovarianceStamped, self.pose_callback)

        # goal coordinates
        self.goal_x = goal_x
        self.goal_y = goal_y

        # coordinates of robot (raw, used for searchlights)
        self.robot_x: float
        self.robot_y: float

        # coordinates of robot (truncated)
        self.state_x: int
        self.state_y: int

    def set_action(self, action):
        # set chosen action as class attribute
        self.action = action

    def pose_callback(self, msg):
        """ Update robot position (x and y coords) locally """
        # print("pose callback")

        # raw coordinate values from AMCL
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # print(f"pose: {self.robot_x}, {self.robot_y}")

        # truncated values to use for state
        self.state_x = floor(self.robot_x)
        self.state_y = floor(self.robot_y)

        # print(f"current pos: {self.robot_x}, {self.robot_y}")

    def dist_from_goal(self):
        """ Get Euclidean distance of robot from goal

        Use raw values for precision (reward is a float anyway!)
        """
        dist_x = self.robot_x - self.goal_x
        dist_y = self.robot_y - self.goal_y

        return sqrt(dist_x**2 + dist_y**2)

    def is_at_goal(self):
        """ Define whether the robot is at the goal """
        # distance within the goal point to be accepted
        accept_radius = 1.0

        # Euclidean distance of robot from goal
        robot_dist = self.dist_from_goal()

        return robot_dist <= accept_radius

    def apply_action(self, action):
        """ Convert [0,1,2,3] to [up, down, left, right] """
        if action == 0:
            self.actor.up()
        elif action == 1:
            self.actor.down()
        elif action == 2:
            self.actor.left()
        elif action == 3:
            self.actor.right()

    def get_new_state(self):
        """ Get robot's new state (used in message construction) """
        # apply chosen action
        # (will trigger an action_callback() to update pos and state)
        self.apply_action(self.action)

        # state defined as 2D coordinate
        return [self.state_x, self.state_y]

    def is_detected(self):
        """ Determine whether transition (new state) is a 'bad' terminal state

        'bad' = detected by searchlights
        """
        threshold = 0.3
        return self.visibility >= threshold

    def is_terminal(self):
        """ Determine whether current state is a terminal state

        i.e. either goal reached or detected by searchlight
        """
        return self.is_at_goal() or self.is_detected()

    def calc_reward(self):
        """ Calculate reward based on searchlights and goal state 

        Contributing factors:
        - how close we are in range of searchlight
        - Euclidean distance from the goal
        """

        # similar to tworooms.cc: return 0 on goal, negative otherwise
        goal_reward = 0  # max reward for reaching goal

        goal_dist = self.dist_from_goal()  # distance of robot from goal
        # print(f"dist from goal: {goal_dist}")

        # weights for overall reward
        w1 = 70         # visibility weight
        w2 = 5          # goal distance weight (when at goal)
        w3 = 50         # goal distance weight (when not terminal)
        w4 = 100        # detection penalty weight

        # reward is weighted sum of detection/goal reward + goal distance
        reward = 0


        # if self.is_terminal():
        if self.is_at_goal():
            # goal state reached
            reward = goal_reward - w2 * goal_dist
            rospy.loginfo("goal reached")
        elif self.is_detected():
            reward = -w4 * self.visibility + w3 / goal_dist
            rospy.loginfo("detected by searchlight")
        else:
            reward = -(w1 * self.visibility + w3 * goal_dist)

        return reward

    def construct_msg(self):
        """ Construct RLStateReward message """

        msg = RLStateReward()

        # get new state
        msg.state = self.get_new_state()

        # get robot visibility from searchlights
        self.visibility = self.cloud.total_robot_visibility([self.robot_x,
                                                             self.robot_y])

        # is it terminal?
        msg.terminal = self.is_terminal()

        # get reward for new state
        msg.reward = self.calc_reward()

        rospy.loginfo(f"distance = {self.dist_from_goal():5.3f}, visibility = {self.visibility:4.3f}, reward = {msg.reward:5.3f}")

        # return message for publishing in environment
        return msg
