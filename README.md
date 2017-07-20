# image_overlay_scale_and_compass [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__image_overlay_scale_and_compass__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__image_overlay_scale_and_compass__ubuntu_xenial_amd64__binary)

Add an indication of scale and compass to images.

![image_overlay_scale_and_compass](http://wiki.ros.org/image_overlay_scale_and_compass?action=AttachFile&do=get&target=mars.png "mars")

Full documentation on wiki: [http://wiki.ros.org/image_overlay_scale_and_compass](http://wiki.ros.org/image_overlay_scale_and_compass)

## Quick Start

1. Install:

```
$ sudo apt-get install ros-kinetic-image-overlay-compass-and-scale
```
2. Launch node:

```
$ roslaunch image_overlay_scale_and_compass overlay.launch
```

3. Publish heading and scale values

```
$ rostopic pub /heading std_msgs/Float32 45
$ rostopic pub /scale std_msgs/Float32 133
```

4. View resulting image

```
$ rqt_image_view /science/overlay/compressed
```

## Command Line Interface (CLI)

Invoke once using Command Line Interface (CLI) and save to disk instead of pulishing to ROS.

```
$ roscd image_overlay_scale_and_compass
$ ./src/image_overlay_scale_and_compass/image_overlay.py --input-image ~/mars.png --heading 45 --scale-text 133 --output-file output.png
```

### CLI Options
```
Usage: image_overlay.py [OPTIONS]

Options:
--input-image TEXT  Path to input image file  [required]
--heading FLOAT     Current heading relative to north in degrees  [required]
--scale-text FLOAT  The value to be displayed on the right of the scale bar
                    in centemeters  [required]
--output-file TEXT  Output filename to save result to
                    [default='output.png']
--help              Show this message and exit.
```

**Full documentation on wiki: [http://wiki.ros.org/image_overlay_scale_and_compass](http://wiki.ros.org/image_overlay_scale_and_compass)**
