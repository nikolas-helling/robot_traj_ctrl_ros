#!/bin/bash

# Check if the user provided a task name
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <task_name>"
    echo "Available tasks: task1, task2"
    exit 1
fi

# Assign the task name from the command line argument
TASK_NAME=$1

# Validate the task name
if [[ "$TASK_NAME" != "task1" && "$TASK_NAME" != "task2" ]]; then
    echo "Error: Unknown task '$TASK_NAME'."
    echo "Available tasks: task1, task2"
    exit 1
fi

# Set the name of the docker image you want to run
DOCKER_IMAGE_NAME=docker_project_traj_track

# Set name of the container
CONTAINER_NAME=docker_project_traj_track

# GUI configurations
xhost local:root
XAUTH=/tmp/.docker.xauth

# Run docker container
sudo docker run -it --rm \
    --name=$CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --env="XAUTHORITY=$XAUTH" \
    --privileged \
    --net=host \
    $DOCKER_IMAGE_NAME \
    $TASK_NAME
#    --gpus all \

# Inform user of successful execution
echo "Container [$DOCKER_IMAGE_NAME($TASK_NAME)] run successfully!"


