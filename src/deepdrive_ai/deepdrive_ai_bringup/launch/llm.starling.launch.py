# MIT License

# Copyright (c) 2024  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    return LaunchDescription([
        create_llama_launch(
            n_ctx=8 * 1024,
            n_batch=8,
            n_gpu_layers=33,
            n_threads=1,
            n_predict=2048,

            model_repo="TheBloke/Starling-LM-7B-alpha-GGUF",
            # model_filename="starling-lm-7b-alpha.Q4_K_M.gguf", # medium, balanced quality - recommended 6.87 GB
            model_filename="starling-lm-7b-alpha.Q5_K_M.gguf", # large, very low quality loss - recommended 7.63 GB

            # GPT4 User: {prompt}<|end_of_turn|>GPT4 Assistant:
            prefix="GPT4 Correct User: ",
            suffix="<|end_of_turn|>GPT4 Correct Assistant:",
            stopping_words=["<|end_of_turn|>", "<|end_of_turn|"],

            system_prompt_type="openchat"
        )
    ])
