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

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                "deepdrive_voice_bringup"), "launch", "whisper.launch.py")),
        launch_arguments={
            "launch_audio_capturer": LaunchConfiguration("launch_audio_capturer", default=False)
        }.items(),
    )

    llama_cmd = create_llama_launch(
        n_ctx=2048,
        n_batch=256,
        n_gpu_layers=33,
        n_threads=1,
        n_predict=-1,

        model_repo="remyxai/stablelm-zephyr-3B_localmentor",
        model_filename="ggml-model-q4_k_m.gguf",

        stopping_words=["<|im_end|>"],
        debug=False
    )

    # llama_cmd = create_llama_launch(
    #         n_ctx=2048,
    #         n_batch=8,
    #         n_gpu_layers=99,
    #         n_threads=1,
    #         n_predict=2048,

    #         model_repo="lmstudio-community/Meta-Llama-3-8B-Instruct-GGUF",
    #         model_filename="Meta-Llama-3-8B-Instruct-Q4_K_M.gguf",

    #         prefix="\n<|start_header_id|>user<|end_header_id|>\n\n",
    #         suffix="<|eot_id|><|start_header_id|>assistant<|end_header_id|>",
    #         stopping_words=["<|eot_id|>"],

    #         system_prompt_type="llama3"
    #     )
    
    # llama_cmd = create_llama_launch(
    #         n_ctx=2048,
    #         n_batch=8,
    #         n_gpu_layers=99,
    #         n_threads=1,
    #         n_predict=2048,

    #         model_repo="TheBloke/OpenHermes-2.5-neural-chat-v3-3-Slerp-GGUF",
    #         model_filename="openhermes-2.5-neural-chat-v3-3-slerp.Q4_K_M.gguf",

    #         prefix="\n<|im_start|>user\n",
    #         suffix="<|im_end|>\n<|im_start|>assistant\n",
    #         stopping_words=["<|im_end|>"],

    #         system_prompt_type="ChatML"
    #     )

    audio_player_cmd = Node(
        package="audio_common",
        executable="audio_player_node",
        name="player_node",
        namespace="audio",
        output="both",
        remappings=[("audio", "out")],
        condition=IfCondition(PythonExpression(
                [LaunchConfiguration("launch_audio_player", default=False)]))
    )

    tts_node_cmd = Node(
        package="tts_ros",
        executable="tts_node",
        output="both",
        parameters=[{
            "device": "cuda",
            # "chunk": 4096,
            "model": "tts_models/en/ljspeech/vits--neon",
            # ("chunk", 4096),
            # ("frame_id", ""),

            # ("model", "tts_models/en/ljspeech/vits"),
            # ("speaker_wav", ""),
            # ("device", "cpu")
        }],
        remappings=[("audio", "/audio/out")]
    )

    # tts_node_cmd = Node(
    #     package="tts_ros",
    #     executable="tts_node",
    #     output="both",
    #     parameters=[{
    #         "device": "cuda",
    #         # ("model", "tts_models/en/ljspeech/vits"),
    #         "model": "tts_models/en/ljspeech/vits--neon",
    #         # "model": "tts_models/en/jenny/jenny",
    #         "model": "tts_models/en/ljspeech/vits",
    #         # "model": "tts_models/en/ljspeech/vits--neon",
    #     }],
    #     remappings=[("audio", "/audio/out")]
    # )

    chatbot_node_cmd = Node(
        package="chatbot_ros",
        executable="chat_bot_node",
        output="both",
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
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
