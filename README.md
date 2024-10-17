# ROS Project (CMR course)

## Overview

This project implements a trajectory tracking controller for a car-like (Akermann steering) mobile robot
using standard ROS libraries, PID controllers and different tyre models.

## Prerequisites

- Docker installed on your machine.
- Git installed on your machine (for cloning the repository).

## Repository Structure

.
├── .gitignore
├── .dockerignore
├── Dockerfile
├── README.md
├── start_docker
└── entrypoint.sh
└── start_task1.sh
└── start_task2.sh
└── run_docker.sh
└── src
└── (...actual ROS nodes)

## Building the Docker Image

To build the Docker image, navigate to the root of the project directory and run:

````bash
docker build -t your_docker_image_name .

## Move to the right directory

Cd to the right directory before running the bash scripts for executing the tasks:

```bash
cd start_docker

## Running task1

To run the first task (task1) run:

```bash
./run_docker.sh task1

## Running task2

To run the first task (task2) run:

```bash
./run_docker.sh task2
````
