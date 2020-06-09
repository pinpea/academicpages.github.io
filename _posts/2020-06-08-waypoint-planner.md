---
title: "Waypoint Planner"
date: 2020-02-15
permalink: /posts/2020/02/waypoint-planner/
tags:
  - cool posts
  - category1
  - category2
excerpt: " This is a sample blog post. Lorem ipsum I can't remember the rest of lorem ipsum and don't have an internet connection right now. Testing testing testing this blog post. Blog posts are cool."
---

https://github.com/pinpea/waypoint_planner

## Basic Usage

<pre>
# clone into your catkin_ws and build
roslaunch waypoint_planner waypoint_planner.launch
</pre>

<pre>
.post-title {
  margin: 0 0 5px;
  font-weight: bold;
  font-size: 38px;
  line-height: 1.2;
  and here's a line of some really, really, really, really long text, just to see how the PRE tag handles it and to find out how it overflows;
}
</pre>

The launch file gives options to launch rviz and rqt gui.
when launching the rqt gui, choose options in 'Plugins/Configuration/Dynamic Reconfigure' to edit dynamic parameters.

Note, this project can be built with catkin tools, see https://catkin-tools.readthedocs.io/en/latest/

<img src="./documentation/waypoint_planner.png" width="400">

## Dynamic Parameters

- The number of interactive "chess" markers can be increased, decreased, but must be greater than 0.
- The number of inner loops can be varied to vary coverage.
- The outer markers can be frozen in place.
- The inner loops (and waypoints) can be frozen in place, whilst outer markers move
- A transform to a specified frame can be published

<img src="./documentation/dynamic_reconfig_options.png" width="600">

## Freezing waypoints using a service

A service can be called that freezes/unfreezes the markers.

```
rosservice call /waypoint_planner/follow_waypoints "data: true"
```

## Topics

- /waypoint_planner/centroid_position
- /waypoint_planner/feedback
- /waypoint_planner/follow_waypoints
- /waypoint_planner/inner_loops
- /waypoint_planner/outer_loop
- /waypoint_planner/update
- /waypoint_planner/update_full
- /waypoint_planner/waypoint_poses
