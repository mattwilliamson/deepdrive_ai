.PHONY: all clean docker docker_run

docker:
	sudo docker build -t deepdrive_ai .

docker_run: docker
	sudo docker run -it --rm --runtime=nvidia --gpus all \
		--network host \
		-v $(CURDIR)/.cache/huggingface:/root/.cache/huggingface \
		-v $(CURDIR)/.local:/root/.local/ \
		-v $(CURDIR)/src:/root/ros2_ws/src \
		--name deepdrive_ai \
		deepdrive_ai bash
		# ros2 launch llama_bringup marcoroni.launch.py


