# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard
from llama_msgs.action import GenerateResponse



class LlamaState(ActionState):

    def __init__(self) -> None:

        super().__init__(
            GenerateResponse, "/llama/generate_response",
            self.create_llama_goal,
            result_handler=self.handle_result
        )

    def create_llama_goal(self, blackboard: Blackboard) -> GenerateResponse.Goal:
        goal = GenerateResponse.Goal()

#         promtp = f"""<|im_start|>system
# You are an A. I. assistant that follows instruction extremely well. Help as much as you can, but be polite and brief. Your name is 'Deep Drive'. You live inside a 4 wheeled robot that can drive around the home and see things. You have a wide angle camera. a stereo depth camera, a two dimentional lidar, motion sensors.<|im_end|>
# <|im_start|>user
# {blackboard.stt}<|im_end|>
# <|im_start|>assistant
# """
        
        # Phi3
#         promtp = f"""<|system|>
# Below is an instruction that describes a task. Write a response that appropriately completes the request<|end|>
# <|user|>
# Hello<|end|>
# <|assistant|>
# Hello<|end|>"""
        
        # Starling/OpenChat
        # GPT4 User: {prompt}<|end_of_turn|>GPT4 Assistant:
        promtp = f"""GPT4 User: Below is an instruction that describes a task. Write a response that appropriately completes the request<|end_of_turn|>
GPT4 Assistant: Hi there! How can I help you today?<|end_of_turn|>
GPT4 User: Please check your battery level<|end_of_turn|>
GPT4 Assistant: The battery level is at 80%<|end_of_turn|>
GPT4 User: {blackboard.stt}<|end_of_turn|>"""

        
        # Llama3
#         promtp = f"""<|begin_of_text|><|start_header_id|>system<|end_header_id|>

# Below is an instruction that describes a task. Write a response that appropriately completes the request<|eot_id|><|start_header_id|>user<|end_header_id|>

# Hello<|eot_id|><|start_header_id|>assistant<|end_header_id|>

# Hello<|eot_id|><|start_header_id|>user<|end_header_id|>"""

        goal.prompt = promtp
        goal.reset = True
        return goal

    def handle_result(
        self,
        blackboard: Blackboard,
        result: GenerateResponse.Result
    ) -> str:
        blackboard.tts = result.response.text.strip()
        return SUCCEED
