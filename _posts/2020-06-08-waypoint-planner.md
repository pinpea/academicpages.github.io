---
title: "Waypoint Planner"
date: 2020-02-15
permalink: /posts/2020/02/waypoint-planner/
tags:
  - ROS
  - C++
excerpt: " This post contains an overview for a waypoint planner I wrote in ROS to give goals for move_base or other planners.."
---

[Source available here](https://github.com/pinpea/waypoint_planner)

This ROS node uses Interactive Markers and a Dynamic Reconfigure server to publish waypoints (as a PoseArray on /waypoint_planner/waypoint_poses). In my current work, I use this node as the input to a [SMACH](http://wiki.ros.org/smach) and [move_base_flex](http://wiki.ros.org/move_base_flex) setup for a ground robot to follow the published waypoints whilst observing the environment for radiation.

## Basic Usage

```
# clone into your catkin_ws/src, build and source
roslaunch waypoint_planner waypoint_planner.launch
```

The launch file gives options to launch rviz and rqt gui.
when launching the rqt gui, choose options in 'Plugins/Configuration/Dynamic Reconfigure' to edit dynamic parameters.

Note, this project can be built with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)

<figure class="align-center">
  <img src="{{ site.url }}{{ site.baseurl }}/images/waypoint_planner.png " alt="">
</figure>

## Dynamic Parameters

- The number of interactive "chess" markers can be increased, decreased, but must be greater than 0.
- The number of inner loops can be varied to vary coverage.
- The outer markers can be frozen in place.
- The inner loops (and waypoints) can be frozen in place, whilst outer markers move
- A transform to a specified frame can be published

<figure class="align-center">
  <img src="{{ site.url }}{{ site.baseurl }}/images/dynamic_reconfig_options.png" alt="">
</figure>

## Demo Video

The following video shows a demo using the Dynamic Reconfigure server to change parameters, sucha as the number of Interactive Markers.

<!-- <figure class="align-center"> -->

<div class="embed-container">
  <iframe
    src="{{ site.url }}{{ site.baseurl }}/images/waypoint_3_annotated.mp4?autoplay=1&loop=1"
    type="video/mp4"
    width="600"
    height="400"
    frameborder="0"
    allowfullscreen="">
  </iframe>
</div>

<!-- </figure> -->

<!-- <figure class="align-center">
<video width="540" height="310" controls>
  <source src="{{ site.url }}{{ site.baseurl }}/images/waypoint_3_annotated.mp4" type="video/mp4">
</video>
</figure> -->

## Freezing waypoints using a service

A service can be called that freezes/unfreezes the markers.

```
rosservice call /waypoint_planner/follow_waypoints "data: true"
```

## Published Topics

- /waypoint_planner/centroid_position
- /waypoint_planner/feedback
- /waypoint_planner/follow_waypoints
- /waypoint_planner/inner_loops
- /waypoint_planner/outer_loop
- /waypoint_planner/update
- /waypoint_planner/update_full
- /waypoint_planner/waypoint_poses
