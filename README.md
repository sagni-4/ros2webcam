How this packages work?

Python3 package:mango/webcam_driver.py


This package leverages OpenCV to read a webcam, draw the current time in seconds on the image and then publish the image to topic called /webcam/image_raw. The image is published using the Best_Effort QoS.


C++ package:blue_filter/src/color_filter.cpp


This package reads the image topic /webcam/image_raw published from the python package, identify a color blue in the image, and change it to a Green color.

How to run these files:

-Clone the three packages (mango, blue_filter and webcam_nodes_launch) into your workspace ->preferably to ros2ws/src

-build the packages using colcon build or (optionally colcon build --packages-select packagesName)

-source your workspace with source install/setup.bash

Finally, launch the files using->ros2 launch webcam_nodes_launch webcam_nodes_launch.launch.py

these launches two windows 'Origional Image' and 'Blue->Green'.

Result: any blue color shown on the original window should converted to green on the Blue->Green window

The result should look like the following:
