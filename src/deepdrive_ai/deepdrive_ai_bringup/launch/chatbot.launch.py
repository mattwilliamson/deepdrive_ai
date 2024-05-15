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


import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from llama_bringup.utils import create_llama_launch
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "whisper.launch.py")
        ),
        launch_arguments={
            "launch_audio_capturer": LaunchConfiguration("launch_audio_capturer", default=False),
            # "model_repo": "distil-whisper/distil-large-v3-ggml",
            # "model_filename": "ggml-distil-large-v3.bin",
            # More accurate?
            # "model_filename": "ggml-distil-large-v3.fp32.bin",
        }.items(),
            
    )

    # llama_cmd = create_llama_launch(
    #     use_llava=True,
    #     n_ctx=2048,
    #     n_batch=256,
    #     n_gpu_layers=99,
    #     n_threads=1,
    #     n_predict=-1,

    #     model_repo="remyxai/stablelm-zephyr-3B_localmentor",
    #     model_filename="ggml-model-q4_k_m.gguf",

    #     stopping_words=["<|im_end|>"],
    #     debug=True
    # )

    llama_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.phi-3.launch.py")
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.llama3.launch.py")

            # Uncensored:
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.kappa-3.launch.py")
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.phi-3-dolphin.launch.py")
            # Fits in memory:
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.llama3-dolphin.launch.py")
            # Slow, doesn't fit into memory
            # os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.mixtral-dolphin.launch.py")
            os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "llm.starling.launch.py")

            
        ),
        launch_arguments={
            "launch_audio_capturer": LaunchConfiguration("launch_audio_capturer", default=False)
        }.items(),
    )

    audio_player_cmd = Node(
        package="audio_common",
        executable="audio_player_node",
        name="player_node",
        namespace="audio",
        output="both",
        remappings=[("audio", "out")],
        condition=IfCondition(PythonExpression([LaunchConfiguration("launch_audio_player", default=False)])),
    )

    tts_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("deepdrive_ai_bringup"), "launch", "tts.launch.py")
        ),
        launch_arguments={
            # "launch_audio_capturer": LaunchConfiguration("launch_audio_capturer", default=False)
        }.items(),
    )

    chatbot_node_cmd = Node(
        package="deepdrive_ai",
        executable="chat_bot_node",
        output="both",
        parameters=[{
            "greeting": LaunchConfiguration("greeting", default="Hello there!")
        }],
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
        # parameters=[{
        #     "host": "0.0.0.0",
        #     "port": "5000",
        # }],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(whisper_cmd)
    ld.add_action(llama_cmd)
    ld.add_action(audio_player_cmd)
    ld.add_action(tts_node_cmd)
    ld.add_action(chatbot_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
