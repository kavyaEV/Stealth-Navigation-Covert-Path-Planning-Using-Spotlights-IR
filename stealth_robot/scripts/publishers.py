#!/usr/bin/env python3

import rospy
import rl_msgs.msg

""" rl_msgs publishers """


class RLPublisher:
    def __init__(self, topic, msg_type):
        self.pub = rospy.Publisher(topic, msg_type, queue_size=10)

    def publish(self, msg):
        self.pub.publish(msg)

    def check_subscribers(self):
        """ Check if there are subscribers publishing """
        return self.pub.get_num_connections() > 0


class EnvDescPublisher(RLPublisher):
    """ Environment -> agent: describe environment """

    def __init__(self):
        super().__init__("rl_env/rl_env_description",
                         rl_msgs.msg.RLEnvDescription)


class SeedExpPublisher(RLPublisher):
    """ Environment -> agent: provide full experience seed (s,a,s',r) for
    agent's learning """

    def __init__(self):
        super().__init__("rl_env/rl_seed",
                         rl_msgs.msg.RLEnvSeedExperience)


class StateRewardPublisher(RLPublisher):
    """ Environment -> agent: agent's new state, reward received on time step,
    and whether the transition is terminal or not """

    def __init__(self):
        super().__init__("rl_env/rl_state_reward",
                         rl_msgs.msg.RLStateReward)


class ActionPublisher(RLPublisher):
    """ Agent -> environment: selected environment action """

    def __init__(self):
        super().__init__("rl_agent/rl_action",
                         rl_msgs.msg.RLAction)


class ExpInfoPublisher(RLPublisher):
    """ Agent -> environment: information on results of latest episode """

    def __init__(self):
        super().__init__("rl_agent/rl_experiment_info",
                         rl_msgs.msg.RLExperimentInfo)
