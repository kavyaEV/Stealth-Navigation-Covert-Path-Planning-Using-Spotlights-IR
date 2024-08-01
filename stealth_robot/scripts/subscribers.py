#!/usr/bin/env python3

import rospy
import rl_msgs.msg

""" rl_msgs publishers """


def common_callback(data):
    # return data for parsing in other modules
    print("common_callback: ", data)
    return data


class RLSubscriber:
    def __init__(self, topic, msg_type, callback=common_callback):
        # if no callback argument provided, use common callback function
        self.sub = rospy.Subscriber(topic, msg_type, callback)


class ActionSubscriber(RLSubscriber):
    """ Environment: get action selected by agent """

    def __init__(self, callback=common_callback):
        super().__init__("rl_agent/rl_action",
                         rl_msgs.msg.RLAction, callback)


class ExpInfoSubscriber(RLSubscriber):
    """ Environment: get information on results of latest episode """

    def __init__(self, callback=common_callback):
        super().__init__("rl_agent/rl_experiment_info",
                         rl_msgs.msg.RLExperimentInfo, callback)


class EnvDescSubscriber(RLSubscriber):
    """ Agent: get description of environment """

    def __init__(self, callback=common_callback):
        super().__init__("rl_env/rl_env_description",
                         rl_msgs.msg.RLEnvDescription, callback)


class SeedExpSubscriber(RLSubscriber):
    """ Agent: get experience seed (s,a,s',r) """

    def __init__(self, callback=common_callback):
        super().__init__("rl_env/rl_seed",
                         rl_msgs.msg.RLEnvSeedExperience, callback)


class StateRewardSubscriber(RLSubscriber):
    """ Agent: get new state, reward, and whether transition is terminal """

    def __init__(self, callback=common_callback):
        super().__init__("rl_env/rl_state_reward",
                         rl_msgs.msg.RLStateReward, callback)
