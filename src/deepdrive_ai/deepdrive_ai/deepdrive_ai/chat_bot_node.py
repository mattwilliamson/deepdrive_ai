#!/usr/bin/env python3

import rclpy

from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from deepdrive_ai.states import ListenFSM
from deepdrive_ai.states import LlamaState
from deepdrive_ai.states import SpeakState


class ChatBot:

    def __init__(self) -> None:
        # self.declare_parameter("greeting", "Hello there!")
        # self.greeting = self.chunk = self.get_parameter("greeting").get_parameter_value().string_value
        self.greeting = "Hello there!"

        # create a state machine
        self.sm = StateMachine(outcomes=[CANCEL, ABORT])

        # add states
        self.sm.add_state("GREETING", SpeakState(), transitions={SUCCEED: "LISTENING", ABORT: ABORT, CANCEL: CANCEL})
        self.sm.add_state("LISTENING", ListenFSM(), transitions={SUCCEED: "RESPONDING", ABORT: ABORT, CANCEL: CANCEL})
        self.sm.add_state("RESPONDING", LlamaState(), transitions={SUCCEED: "LOG", ABORT: ABORT, CANCEL: CANCEL})
        self.sm.add_state("LOG", CbState([SUCCEED], self.log_response), transitions={SUCCEED: "SPEAKING"})
        self.sm.add_state("SPEAKING", SpeakState(), transitions={SUCCEED: "LISTENING", ABORT: ABORT, CANCEL: CANCEL})

        YasminViewerPub("DEEPDRIVE_AI", self.sm)

    def start(self) -> None:
        blackboard = Blackboard()
        blackboard.tts = self.greeting
        self.sm(blackboard)

    def log_response(self, blackboard: Blackboard) -> str:
        YasminNode.get_instance().get_logger().info(f"Response: {blackboard.tts}")
        return SUCCEED


def main():
    rclpy.init()
    chatbot = ChatBot()
    chatbot.start()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
