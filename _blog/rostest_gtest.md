---
title: "Demo of using gtest and rostest"
collection: blog
type: "Demo of using gtest and rostest"
permalink: /blog/gtest-and-ros
excerpt: " "

venue: ""
date: 2015-08-14
location: ""
---

## Setup

I am using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/index.html) to build this project. See their [cheat sheet](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html) for setting up a project and using catkin build etc.

## gtest and rostest

Google Test (gtest) is for writing unit tests, their is more information, and a couple of examples on [the ROS Wiki](http://wiki.ros.org/gtest). See [here](http://cheezyworld.com/wp-content/uploads/2010/12/PlainGoogleQuickTestReferenceGuide1.pdf) for a cheat sheet of the available tests.

[rostest](http://wiki.ros.org/rostest) is used to perform integration testing and to test more complicated behavious, such as testing across multiple nodes.

I found some more realistic use cases from [this working example](https://github.com/gocarlos/ros_unit_tests_example) and the [roscomm repository](https://github.com/ros/ros_comm/tree/ebd9e491e71947889eb81089306698775ab5d2a2/test/test_roscpp/test).

## Minimal working example

My project structure looks like this:

```txt
└── src
    └── waypoint_planner
        ├── CMakeLists.txt
        ├── Dockerfile
        ├── documentation
        │   └── rostest_gtest.md
        ├── include
        │   └── waypoint_planner
        │       ├── waypoint_planner.h
        │       └── waypoint_planner.hpp
        ├── launch
        │   └── waypoint_planner.launch
        ├── package.xml
        ├── README.md
        ├── rviz
        │   └── rviz_waypoint_planner.cfg.rviz
        ├── src
        │   └── waypoint_planner_node.cpp
        └── test
            ├── launch
            │   └── test_waypoint_planner.launch
            └── test_waypoint_planner.cpp
```

I am testing one package (waypoint_planner). My code for this project is in C++ and I have tried to follow the conventions for writing a roscpp package. My tests are placed in the `test/` folder.

I began with a minimal example using gtest (test_waypoint_planner.cpp):

```cpp
#include <stdexcept> // for std::runtime_error
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(WaypointTest, myworkcell_core_framework)
{
    ASSERT_TRUE(true);
}
```

This does not yet perform any tests interfacing with my ROS node.

### Building the project

I set up my CMakesList.txt for my package to build as a C++ library (see [here](https://docs.ros.org/api/catkin/html/howto/format1/building_libraries.html)) as follows:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  interactive_markers
  dynamic_reconfigure
  visualization_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS  include
  CATKIN_DEPENDS roscpp interactive_markers  visualization_msgs tf  dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
 src/waypoint_planner_node.cpp
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

To build the tests, I added the following lines:

```cmake
if (CATKIN_ENABLE_TESTING)
find_package(catkin REQUIRED COMPONENTS rostest roscpp interactive_markers visualization_msgs tf)
add_rostest_gtest(waypoint_planner_test test/launch/test_waypoint_planner.launch test/test_waypoint_planner.cpp)
  target_link_libraries(waypoint_planner_test ${catkin_LIBRARIES} )
endif()

```

The `find_package_components` was important to give the test file access to roscpp and other required libraries (without this line I had errors relating to `undefined reference to`ros::init` meaning the ros.h library was not found).

To build the package I used:

```bash
# Without tests
catkin build

# With tests
catkin build waypoint_planner --catkin-make-args run_tests  #for just the waypoint_planner package

catkin build  --catkin-make-args run_tests #for all packages and tests

```

To run the test I used:

```bash
rostest waypoint_planner test_waypoint_planner.launch
```

Where my launch file was located in test/launch and contained the following:

```xml
<?xml version="1.0"?>
<launch>
    <test test-name="unit_test_node" pkg="waypoint_planner" type="waypoint_planner_test"/>
</launch>
```

Which gave the following test result (saved at ~/.ros/log/):

```txt
[==========] Running 1 test from 1 test case.
[----------] Global test environment set-up.
[----------] 1 test from WaypointTest
[ RUN      ] WaypointTest.myworkcell_core_framework
[       OK ] WaypointTest.myworkcell_core_framework (0 ms)
[----------] 1 test from WaypointTest (0 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (0 ms total)
[  PASSED  ] 1 test.
```

## Interfacing with my ROS node

As usual, I find myself doing a more complicated example rather than starting simply and building up....

Here is my first test example:

```cpp
#include <stdexcept> // for std::runtime_error
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "waypoint_planner/waypoint_planner.h"

TEST(WaypointTest, myworkcell_core_framework)
{
    ASSERT_TRUE(true);
}

TEST(WaypointTest, CreateMarkersTest){
    ros::NodeHandle nh;
    WaypointPlanner WaypointPlanner(&nh);
    WaypointPlanner.initializePublishers();
    server.reset(new interactive_markers::InteractiveMarkerServer("waypoint_planner", "", false));
    int n =4;
    std::vector<std::string> markers;
    markers = WaypointPlanner.createMarkers(n);

    EXPECT_EQ(n,markers.size());
}

void spinThread()
{
  ros::NodeHandle nh;
  ros::spin();

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "waypoint_tests");
    boost::thread spin_thread(&spinThread);
    int result= RUN_ALL_TESTS();
    ros::shutdown();
    spin_thread.join();
    return result;
}
```

There are a few things going on here. Since I built my package as a library, I can include my header file as follows:

```cpp
#include "waypoint_planner/waypoint_planner.h"
```

A simpler example of integrating with ROS is as follows (from [gtest Wiki](http://wiki.ros.org/gtest)):

```cpp
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_talker");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
```

However, my package interfaces with the interactive_markers library, which uses boost shared pointers. And gave errors as follows:

```bash
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
```

This error pointed me to [here](https://answers.ros.org/question/326853/boost-mutex-error-when-using-simpleactionclient-with-stdunique_ptr/) and [here](https://docs.ros.org/groovy/api/actionlib/html/simple__client__wait__test_8cpp_source.html), to give the following:

```cpp
void spinThread()
{
  ros::NodeHandle nh;
  ros::spin();

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "waypoint_tests");
    boost::thread spin_thread(&spinThread);
    int result= RUN_ALL_TESTS();
    ros::shutdown();
    spin_thread.join();
    return result;
}
```

Now, looking at the test itself:

```cpp
TEST(WaypointTest, CreateMarkersTest){
    ros::NodeHandle nh;
    WaypointPlanner WaypointPlanner(&nh);
    WaypointPlanner.initializePublishers();
    server.reset(new interactive_markers::InteractiveMarkerServer("waypoint_planner", "", false));

    int n =4;
    std::vector<std::string> markers;
    markers = WaypointPlanner.createMarkers(n);
    EXPECT_EQ(n,markers.size());
}

```

The actual test occurs on the last four lines: I am testing where my function that creates and names interactive markers has created the number of markers I request.

The lines above are the setup, I had the error:

```bash
ASSERTION FAILED
    file = /opt/ros/indigo/include/ros/publisher.h
    line = 102
    cond = false
    message =
[FATAL] Call to publish() on an invalid Publisher
```

Until I made my initializePublishers() function (which sets up `marker_publisher = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10, true);`
) a public member function in my class and called it directly.

I will look into why this was further.

Now when I run the test, I sill get a warning related to the boost library, but the tests run and give me the following output:

```bash
[ROSTEST]-----------------------------------------------------------------------

[waypoint_planner.rosunit-unit_test_node/myworkcell_core_framework][passed]
[waypoint_planner.rosunit-unit_test_node/CreateMarkersTest][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

## Automated tesing

Travis and GitHub Actions

Adding a badge to README.md

```
https://github.com/<OWNER>/<REPOSITORY>/workflows/<WORKFLOW_NAME>/badge.svg
```

where name is a parameter the actions file
