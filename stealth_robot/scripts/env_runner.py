#!/usr/bin/env python3

from sys import argv, exit

import rospy
import roslaunch
import rl_msgs.msg
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid

from publishers import EnvDescPublisher
from environment import StealthEnvironment
from searchlight import Searchlight, SearchlightCloud, Map

""" Node to run reinforcement learning environment

restart environment at the end of an episode (robot reaches terminal state
"""

class EnvRunner:
    def __init__(self):
        """ formulate and send RLEnvDescription message

        only needs to be done once: at the start of the learning session
        """

        rospy.loginfo("EnvRunner constructor")

        self.map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        rospy.loginfo("Got map data")
        print(self.map.info)

        self.env_desc_pub = EnvDescPublisher()
        self.rate = rospy.Rate(10)

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.map_name: str
        self.goal_difficulty: int  # 0 = easy, 1 = medium, 2 = hard

    def pub_env_description(self):
        msg = rl_msgs.msg.RLEnvDescription()

        msg.title = "Environment: Stealth"
        msg.stochastic = True
        msg.episodic = True

        msg.max_reward = 0  # goal reward
        msg.num_actions = 4  # up, down, left, right

        while not self.env_desc_pub.check_subscribers():
            rospy.loginfo("Checking for agent subscriber")
            rospy.sleep(1.0)

        self.env_desc_pub.publish(msg)
        rospy.loginfo("Published environment description")

    def run_env(self):
        self.env = StealthEnvironment(self.map_name,
                                      self.goal_difficulty,
                                      self.cloud)

    def start_nodes(self):
        """ launch amcl, rviz, and stage """

        amcl_node = roslaunch.core.Node(package="amcl",
                                        node_type="amcl",
                                        name="amcl",
                                        args="scan:=base_scan")


        stage_node = roslaunch.core.Node(package="stage_ros",
                                         node_type="stageros",
                                         name="stage",
                                         args=f"--clock $(find stealth_robot)/maps/{self.map_name}.world")

        rviz_node = roslaunch.core.Node(package="rviz",
                                        node_type="rviz",
                                        name="rviz",
                                        args=f"-d $(find stealth_robot)/config/{self.map_name}.rviz")

        self.stage = self.launch.launch(stage_node)
        self.amcl = self.launch.launch(amcl_node)
        self.rviz = self.launch.launch(rviz_node)

        rospy.loginfo("started amcl, rviz, stage")

    def stop_processes(self):
        self.rviz.stop()
        self.amcl.stop()
        self.stage.stop()

        rospy.loginfo("stopped amcl, rviz, stage")

    def init_searchlights(self):
        occupancy_map = Map(data=self.map.data,
                            width=self.map.info.width,
                            height=self.map.info.height)
        searchlights = []

        # dict of all map names and searchlight configurations
        maps_and_searchlights = {
                "lgfloor": [[(20.5, 4.5), 6.5, 45, 45, 75],
                            [(19.6, 19), 4.4, 30, 30, 240],
                            [(10, 25), 9, 30, 30, 270],
                            [(15, 18), 9, 30, 30, 155]],
                "map1": [[(5.4, 16), 6, 30, 30, 270],
                         [(15, 8.6), 3, 90, 90, 90],
                         [(7.6, 3.2), 5, 50, 50, 90]],
                "floorplan": [[(20, 9), 9, 30, 30, 270],
                              [(35, 15), 12, 50, 50, 135],
                              [(5, 7), 7, 30, 30, 0]],
                "simpler-maze": [[(4.9/2, 5.5/2), 20/2, 60, 60, 0],
                                [(8.7/2, 20/2), 15/2, 60, 60, 180],
                                [(20/2, 12/2), 10/2, 60, 60, 270],
                                [(22/2, 20/2), 5/2, 60, 60, 225]]
        }

        # construct Searchlight objects according to map name
        for params in maps_and_searchlights[self.map_name]:
            searchlights.append(Searchlight(occupancy_map=occupancy_map,
                                            position=params[0],
                                            detection_range=params[1],
                                            beamWidth=params[2],
                                            cutoffAngle=params[3],
                                            direction=params[4]))

        self.cloud = SearchlightCloud(searchlights)


if __name__ == '__main__':
    rospy.init_node("env_runner", anonymous=True)
    runner = EnvRunner()

    # publish environment description at the start
    runner.pub_env_description()

    try:
        runner.map_name = argv[1]
        runner.goal_difficulty = int(argv[2])
    except IndexError:
        rospy.logerr("Usage: {argv[0]} [map_name] [goal_difficulty]")
        exit(1)


    while not rospy.is_shutdown():
        # new environment every episode
        runner.start_nodes()
        runner.init_searchlights()
        runner.cloud.publish()
        rospy.sleep(4.0)
        runner.run_env()

        while not runner.env.end_episode:
            # keep running environment until terminal state reached
            rospy.sleep(1.0)

        rospy.loginfo(f"final position: {runner.env.sr.robot_x}, {runner.env.sr.robot_y}")
        rospy.loginfo(f"distance from goal: {runner.env.sr.dist_from_goal()}")

        runner.stop_processes()
