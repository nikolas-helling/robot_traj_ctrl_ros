#!/bin/bash

# check for task name
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <task_name>"
    echo "Available tasks: task1, task2"
    exit 1
fi
TASK_NAME=$1

if [[ "$TASK_NAME" != "task1" && "$TASK_NAME" != "task2" ]]; then
    echo "Error: Unknown task '$TASK_NAME'."
    echo "Available tasks: task1, task2"
    exit 1
fi

# set the name of docker image
DOCKER_IMAGE_NAME=docker_project_traj_track

# set name of container
CONTAINER_NAME=docker_project_traj_track

# GUI configurations
xhost local:root
XAUTH=/tmp/.docker.xauth

# run docker container
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

# inform of of successful execution
echo "Container [$DOCKER_IMAGE_NAME($TASK_NAME)] run successfully!"


