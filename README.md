# ROS Project (CMR course)

## Overview

This project implements a trajectory tracking controller for a car-like (Ackermann steering) mobile robot
using standard ROS libraries, PID controllers and different tyre models.

## Prerequisites

- Docker installed on your machine.
- Git installed on your machine.

## Repository Structure

```
.
├── .gitignore
├── .dockerignore
├── Dockerfile
├── README.md
├── py_requirements.txt
├── start_docker
│   ├── entrypoint.sh
│   ├── start_task1.sh
│   ├── start_task2.sh
│   └── run_docker.sh
└── src
    └── (...actual ROS nodes)
```

## Clone the GitHub Repository

Clone the project repository:

```
git clone https://github.com/lellosburello/CMR_project_docker.git
```

Go to the project repository folder:

```
cd CMR_project_docker
```

## Building the Docker image

To build the Docker image, navigate to the root of the project directory (`/CMR_project_docker`), where the Dockerfile is located, and run:

```
sudo docker build -t docker_project_traj_track .
```

## Running the project tasks

Now, go first to the `/start_docker` folder before running the bash scripts for executing the tasks:

```
cd start_docker
```

### Running task1 (bicycle kinematic model)

To run the first task (task1), the trajectory tracking simulation with the bicycle kinematic model, run:

```
./run_docker.sh task1
```

### Running task2 (dynamic model - linear tyre model)

To run the second task (task2), the trajectory tracking simulation using the dynamic model with linear tyre model, run:

```
./run_docker.sh task2
```
