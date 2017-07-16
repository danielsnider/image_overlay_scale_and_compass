# image_overlay [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__image_overlay__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__image_overlay__ubuntu_xenial_amd64__binary)

A package that will buffer `move_base` goals until instructed to navigate to all waypoints in sequence.

Full documentation on wiki: [http://wiki.ros.org/image_overlay](http://wiki.ros.org/image_overlay)

![image_overlay](https://github.com/danielsnider/image_overlay/blob/master/readme_images/image_overlay_rviz.png "rviz")

## Installation

```
  $ sudo apt-get install ros-kinetic-image-overlay
```

## Usage

To set waypoints you can either publish a ROS `PoseWithCovarianceStamped` message to the `/initialpose` topic directly or use RViz’s tool "2D Pose Estimate" to click anywhere. To visualize the waypoints as pink arrows in RViz, configure RViz to display the topic `/current_waypoints` which is published by `image_overlay` and must be subscribed to in Rviz as a `PoseAarray` type.

To initiate waypoint following send a "path ready" message.

```
  $ rostopic pub /path_ready std_msgs/Empty -1
```

**Full documentation on wiki: [http://wiki.ros.org/image_overlay](http://wiki.ros.org/image_overlay)**
