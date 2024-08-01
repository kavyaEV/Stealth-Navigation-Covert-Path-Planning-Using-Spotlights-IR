#!/usr/bin/env python3

from math import floor

import rospy
import rl_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

from publishers import StateRewardPublisher
from subscribers import ActionSubscriber
from state_reward import StateReward


class StealthEnvironment:
    def __init__(self, map_name, goal_difficulty, cloud):
        """ Initialise stealth environment publishers and subscribers """
        self.cloud = cloud

        # map name (for searchlights and initial pose)
        self.map_name = map_name
        # difficulty for goal point
        self.goal_difficulty = goal_difficulty

        # set initial robot pose
        initial_x, initial_y = self.set_initial_pose()

        """ set up publishers (to send messages to agent) """
        # agent's new state, reward received on time step, and whether the
        # transition is terminal or not
        self.sr_pub = StateRewardPublisher()
        rospy.loginfo("StateRewardPublisher set up")

        """ Publish goal """
        for i in range(10):
            try:
                self.goal_x, self.goal_y = self.set_goal()
                rospy.loginfo("Goal set")
            except Exception as e:
                print(e)
                rospy.sleep(i+1)
                continue
            break

        """ set up state and reward calculator """
        # step distance based on map name
        step_dists = {
                'lgfloor': 0.25,
                'map1': 0.12,
                'floorplan': 0.5,
                'simpler-maze': 0.3
                }

        self.sr = StateReward(self.goal_x,
                              self.goal_y,
                              self.cloud,
                              step_dists[map_name])

        rospy.loginfo("StateReward calculator initialised")

        """ set up subscribers (to receive messages from agent) """
        # environment action selected by agent
        self.action_sub = ActionSubscriber(callback=self.action_callback)

        rospy.sleep(1.0)

        if self.action_sub.sub.get_num_connections() == 0:
            rospy.logerr("Failed to subscribe to rl_action")
        else:
            rospy.loginfo("ActionSubscriber initialised")

        """ publish initial state and reward """
        # set to true once robot has reached a terminal state 
        # (tell runner to reset environment)
        self.end_episode = False

        initial_sr = rl_msgs.msg.RLStateReward()
        initial_sr.state = [floor(initial_x), floor(initial_y)]
        initial_sr.reward = -100
        initial_sr.terminal = False

        # wait for subscribers before publishing initial state and reward
        while not self.sr_pub.check_subscribers():
            rospy.sleep(1.0)

        self.sr_pub.publish(initial_sr)
        rospy.loginfo("Published initial state")

    def set_initial_pose(self):
        poses_and_maps = {
                "lgfloor": [14.447601088757134,
                            14.887156483118229,
                            0.0,
                            0.0,
                            0.0,
                            0.44514310982090366,
                            0.8954594417275273],
                "map1": [9.60427188873291,
                         10.102672576904297,
                         0.0,
                         0.0,
                         0.0,
                         0.010407704152367553,
                         0.9999458383803979],
                "floorplan": [20.238670349121094,
                              12.45450210571289,
                              0.0,
                              0.0,
                              0.0,
                              0.0009688652001724583,
                              0.9999995306500018],
                "simpler-maze": [7.462601088757134,
                                7.778601088757134,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                1.0]
        }

        covariances_and_maps = {
                "lgfloor": [0.24765850578262416, 0.0003599694450997504, 0.0, 0.0,
                            0.0, 0.0, 0.0003599694450997504, 0.24290098925786197,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.06716567243247112],
                "map1": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.06853892326654787],
                "floorplan": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.06853892326654787],
                "simpler-maze": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.06853892326654787]
        }

        params = poses_and_maps[self.map_name]

        initial_pose = PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "map"

        initial_pose.pose.pose.position.x = params[0]
        initial_pose.pose.pose.position.y = params[1]
        initial_pose.pose.pose.position.z = params[2]

        initial_pose.pose.pose.orientation.x = params[3]
        initial_pose.pose.pose.orientation.y = params[4]
        initial_pose.pose.pose.orientation.z = params[5]
        initial_pose.pose.pose.orientation.w = params[6]

        initial_pose.pose.covariance = covariances_and_maps[self.map_name]

        pose_publisher = rospy.Publisher("/initialpose",
                                         PoseWithCovarianceStamped,
                                         queue_size=10)

        while pose_publisher.get_num_connections() == 0:
            rospy.sleep(1.0)

        pose_publisher.publish(initial_pose)

        return initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y


    def action_callback(self, msg):
        # action = message from agent
        self.sr.set_action(msg.action)

        # calculate reward, construct, and publish RLStateReward message
        state_reward = self.sr.construct_msg()
        self.sr_pub.publish(state_reward)

        # check if in a terminal state: if so, end the episode
        if self.sr.is_terminal():
            self.end_episode = True

    def set_goal(self):
        # easy, medium, and hard goals for each map
        goals_and_maps = {
                           "lgfloor": [
                               [16.04461669921875, 16.123353958129883],
                               [14.997982025146484, 18.965513229370117],
                               [11.457938194274902, 24.07845687866211],
                               ],
                           "map1": [
                               [7.9813737869262695,  8.11137580871582],
                               [7.1499223709106445, 10.475198745727539],
                               [6.481032371520996, 4.276494979858398],
                               ],
                           "floorplan": [
                               [19.04546546936035, 8.22945785522461],
                               [28.05337905883789, 16.2329158782959],
                               [8.050286293029785, 14.638769149780273],
                               ],
                           "simpler-maze": [
                               [21.975339889526367/2, 15.896149635314941/2],
                               [12.21079158782959/2, 25.425504684448242/2],
                               [12.21079158782959/2, 25.425504684448242/2],
                               ]
                        }

        goal = PointStamped()
        goal.header.frame_id = "map"

        # get goal point based on map name and difficulty
        goal.point.x = goals_and_maps[self.map_name][self.goal_difficulty][0]
        goal.point.y = goals_and_maps[self.map_name][self.goal_difficulty][1]

        goal_publisher = rospy.Publisher("/clicked_point", PointStamped, queue_size=10)

        while goal_publisher.get_num_connections() == 0:
            rospy.sleep(1.0)

        goal_publisher.publish(goal)

        return goal.point.x, goal.point.y
