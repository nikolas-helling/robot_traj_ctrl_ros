#!/bin/bash

# declare workspace path for easier syntax
WORKSPACE_PATH=/CMR_ws

# check the first argument to determine which application to run
if [ "$1" == "task1" ]; then
    echo "Starting task1 ..."
    exec $WORKSPACE_PATH/start_task1.sh

elif [ "$1" == "task2" ]; then
    echo "Starting task2 ..."
    exec $WORKSPACE_PATH/start_task2.sh

else
    echo "Error: Please specify 'task1' or 'task2' to run the respective application."
    exit 1
fi
