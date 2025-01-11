# pull docker image for the base installation of ROS melodic
FROM ros:melodic-ros-base

RUN mkdir -p /root/.config/terminator
RUN touch /root/.config/terminator/config

# create and set working directory (catkin workspace folder) in the root of the container structure
RUN mkdir -p /robot_traj_ctrl_ws/src
WORKDIR /robot_traj_ctrl_ws

# install system and ROS dependencies using rosdep
RUN apt-get update && \ 
    apt-get install -y \
    # fundamental dependencies
    apt-utils \
    terminator \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    dbus-x11 \
    x11-apps \
    x11-xserver-utils \
    # fundamental python dependencies
    python-pip \
    python-rosdep \
    python-dev \
    python-pyqt5 \
    python-tk \
    libjpeg-dev \
    zlib1g-dev && \
    # initialize rosdep if it hasn't been initialized yet
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && \
    # update rosdep database
    rosdep update && \
    # install ROS dependencies using rosdep
    rosdep install --from-paths src --ignore-src -r -y && \
    # clean up for smaller images
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# copy the source code and the start scripts into the container
COPY ./src /robot_traj_ctrl_ws/src
COPY ./start_docker/entrypoint.sh /robot_traj_ctrl_ws/
COPY ./start_docker/start_task1.sh /robot_traj_ctrl_ws/
COPY ./start_docker/start_task2.sh /robot_traj_ctrl_ws/

# make the start bash files executable
RUN chmod +x /robot_traj_ctrl_ws/start_task1.sh && \
    chmod +x /robot_traj_ctrl_ws/start_task2.sh && \
    chmod +x /robot_traj_ctrl_ws/entrypoint.sh

# install extra Python dependencies using pip3
COPY py_requirements.txt /robot_traj_ctrl_ws/
RUN pip install -r /robot_traj_ctrl_ws/py_requirements.txt && \
    rm -rf /root/.cache/pip

# source ROS environment and build catkin workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# setup container entrypoint
ENTRYPOINT ["/robot_traj_ctrl_ws/entrypoint.sh"]

# specify the default command to run when starting the container
CMD ["/bin/bash"]
