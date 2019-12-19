How to use:

Follow this (http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages) to add the dylan_msc/obj.h message type to your package.

for C++: #include <dylan_msc/obj.h>

for Python: from dylan_msc.msg import obj

Segmented point cloud data is published to: /points2

Centroids are published to: /mark2

Centroid + bounding box information is published to: /plot2

Note: frame of published data is not global frame.
