How to use:

Follow this (http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages) to add the dylan_msc/obj.h message type to your package.

for C++: #include <dylan_msc/obj.h>

for Python: from dylan_msc.msg import obj

dylan_msc/obj has 4 fields:

uint32 index (frame index)

geometry_msgs/Point centroid (centroid of segmented cluster)

geometry_msgs/Point min (smallest x, y, z values)

geometry_msgs/Point max (largest x, y, z values)

Segmented point cloud data is published to: /points2 (msg type: sensor_msgs/PointCloud2)

Centroids are published to: /mark2 (msg type: visualization_msgs/Marker)

Centroid + bounding box information is published to: /plot2 (msg type: dylan_msc/obj.h)

Note: frame of published data is not global frame.

To run: 

Open a terminal window

run: roscore

Open a new terminal window

run: rviz

Open a new terminal window

run: roslaunch dylan_msc main.launch

In rviz:

Change fixed frame in global options from map to camera link

Click add -> camera.
add -> pointcloud2
add -> pointcloud2
add -> marker
