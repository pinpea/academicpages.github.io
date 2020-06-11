---
title: "ROS GitHub Actions with Docker Example"
date: 2020-03-02
permalink: /posts/2020/ros-actions
tags:
  - ROS
  - Docker
  - GitHub Actions
excerpt: "A practical demo of using Docker to create a build in GitHub Actions"
---

## Automated build with Docker and GitHub Actions

[More on GitHub Actions](https://help.github.com/en/actions)

[Setup GitHub Actions](https://help.github.com/en/actions/configuring-and-managing-workflows/configuring-a-workflow#creating-a-workflow-file)

## Demo Workflow File Using Docker

```yaml
name: Waypoint_Planner_Docker_Image_CI

on:
  push:
    branches: [master]
  pull_request:
    branches: [master]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build the Docker image
        run: docker build . --file Dockerfile --tag kinetic-waypoint-planner
```

This build uses the Dockerfile at the root of the repo:

```docker
FROM osrf/ros:kinetic-desktop-full AS waypoint-planner

# SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

#ENV PIP_REQ_FILE "${CATKIN_WS_LOCATION}/src/${PACKAGE_NAME}/src/ros_node/requirements.txt"
ENV ROSINSTALL_FILE "/root/home/dependencies.rosinstall"
ENV CATKIN_OPTIONS "/root/home/catkin.options"
ENV ROS_PARALLEL_JOBS "-j8 -l6"
ENV ROS_DISTRO="kinetic"

# Set the python path manually to include /usr/-/python2.7/dist-packages
# as this is where apt-get installs python packages.
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages:/usr/local/lib/python2.7/dist-packages

RUN echo " Installing catkin dependencies" \
    && apt-get update -qq && apt-get install -y \
    git\
    vim \
    nano \
    python-pip \
    python-catkin-tools

FROM waypoint-planner
# RUN echo "Set up Python" \
#     && pip install wheel \
#     && python setup.py bdist_wheel

ENV PACKAGE_NAME "waypoint_planner"
ENV CATKIN_WS_LOCATION "/root/home/catkin_ws"
ENV ROS_SOURCE="/opt/ros/${ROS_DISTRO}/setup.bash"
ENV source_ws='source "${CATKIN_WS_LOCATION}/devel/setup.bash"'
ENV source_ros='source ${ROS_SOURCE}'
ENV USERNAME myNewUserName

#Add new sudo user
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # Replace 1000 with your user/group id
    usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME

RUN /bin/bash -c " mkdir -p  ${CATKIN_WS_LOCATION}/src/"
COPY ./ ${CATKIN_WS_LOCATION}/src/${PACKAGE_NAME}
COPY ./ros_entrypoint.sh /ros_entrypoint.sh

RUN /bin/bash -c " ls ${CATKIN_WS_LOCATION}/src/"

RUN /bin/bash -c " echo 'Initialising catkin_ws' \
    echo 'ROS DISTRO' ${ROS_DISTRO} \
    && $source_ros \
    && cd ${CATKIN_WS_LOCATION} \
    && catkin init"

RUN /bin/bash -c "echo 'Installing package dependencies' \
    && $source_ros  \
    && cd ${CATKIN_WS_LOCATION} \
    && rosdep install --from-paths src --ignore-src -r -y"

# RUN echo "Installing Python requirements located at ${PIP_REQ_FILE} " \
#     && pip install -r ${PIP_REQ_FILE}

RUN /bin/bash -c " echo 'Build Package' \
    && $source_ros  \
    && cd ${CATKIN_WS_LOCATION} \
    && catkin build ${PACKAGE_NAME}"


```

## Adding a badge to README.md

```
https://github.com/<OWNER>/<REPOSITORY>/workflows/<WORKFLOW_NAME>/badge.svg
```

where name is a parameter the actions file, in this case Waypoint_Planner_Docker_Image_CI.
