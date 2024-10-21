# Trajectory Tracking for Mobile Robot (ROS project for CMR)

This project implements a trajectory tracking controller for a car-like (Ackermann steering) mobile robot
using standard ROS libraries and PID controllers. The controllers are designed both on kinematic models and dynamic models using different tyre models.

## Prerequisites

### General Requirements
- **Docker**: [Install Docker](https://docs.docker.com/get-docker/)
- **Git**: [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

### Windows Only

- **WSL2 Backend**: Required for running Linux-based Docker containers on Windows.
  - [Install WSL2](https://docs.microsoft.com/en-us/windows/wsl/install)
  - Hardware virtualization must be enabled in the BIOS/UEFI settings of your Windows machine
- **VcXsrv**: Recommended X server for running GUI applications (gazebo, rviz, ...) inside ROS-based containers.
  - [Install VcXsrv](https://sourceforge.net/projects/vcxsrv/)
  - **Launch Instructions**:
    1. After installation, search and launch **XLaunch**.
    2. Select **Multiple windows** and click **Next**.
    3. Choose **Start no client** and click **Next**.
    4. Ensure **Disable access control** is checked to allow connections from your WSL environment, then click **Next**.
    5. Click **Finish** to start the VcXsrv server.

## Repository Structure

```
.
├── .gitignore
├── .gitattributes
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

## Initial steps

Clone the project repository:

```
git clone https://github.com/lellosburello/CMR_project_docker.git
```

Go to the project repository folder:

```
cd CMR_project_docker
```
Start the WSL environment (Windows only):
```
wsl
```

## Building the Docker image

To build the Docker image, navigate to the root of the project directory (`/CMR_project_docker`), where the Dockerfile is located, and run:

```
sudo docker build -t docker_project_traj_track .
```

## Running the Docker container (project tasks)

Now, go to the `/start_docker` folder before running the bash scripts for executing the tasks:

```
cd start_docker
```

Make the start script executable:
```
chmod +x run_docker.sh
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
