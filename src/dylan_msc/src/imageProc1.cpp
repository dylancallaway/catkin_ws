#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <dylan_msc/obj.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h> // Required for pcl::fromROSMsg
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

PCLCloud::Ptr cloud(new PCLCloud), cloud_f(new PCLCloud), cloud_filtered(new PCLCloud);
PCLCloud::Ptr cloud_msg(new PCLCloud);

ros::Publisher pcl_pub, marker_pub, plotter_pub;

// Function called for each frame received
void frame_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *cloud); // Convert PointCloud2 ROS msg to PCL PointCloud
    // std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

    // Passthrough filter
    // Removes points outside of a certain range
    pcl::PassThrough<pcl::PointXYZ> pass; // create filter object
    pass.setInputCloud(cloud);            // set incoming cloud as input to filter
    pass.setFilterFieldName("z");         // filter on z axis (depth)
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*cloud_filtered); // returns cloud_filtered, the cloud with points outside of limits removed
    // Repeat for y axis (height)
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, 0.2);
    pass.filter(*cloud_filtered);
    // std::cout << "PointCloud after distance filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // Estimate planar model of floor and remove it
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // planar inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.02); // maximum euclidean distance between points considered to be planar

    int i = 0, nr_points = (int)cloud_filtered->points.size(); // get size of cloud_filtered in number of points
    // Filter out largest planar model until cloud_filtered is reduced to XX% (see code) of original cloud_filtered
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    bool check_val = 1;
    while (check_val && cloud_filtered->points.size() > 0.5 * nr_points)
    // while (cloud_filtered->points.size() > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        static float delta_check = 0.2f;
        check_val = check_val && (coefficients->values[0] > -delta_check && coefficients->values[0] < delta_check);
        check_val = check_val && (coefficients->values[1] > -1.0f - delta_check && coefficients->values[1] < -1.0f + delta_check);
        check_val = check_val && (coefficients->values[2] > -delta_check && coefficients->values[2] < delta_check);
        // check_val = check_val && (coefficients->values[3] >= -0.2 && coefficients->values[3] <= 0.2);
        // A  = 0
        // B = -1
        // C = 0
        // D can be anything
        // eqn of x-z plane is B = D
        if (!check_val)
        {
            continue;
        }

        // // info about removed planes
        // std::cout << "Coeffs: " << coefficients->values[0] << " "
        //           << coefficients->values[1] << " "
        //           << coefficients->values[2] << " "
        //           << coefficients->values[3] << " "
        //           << inliers->indices.size() << "\n"
        //           << std::endl;

        // If no planar inliers exist
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    // std::cout << "PointCloud after plane filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(300);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    PCLCloud::Ptr cluster_sum(new PCLCloud);
    pcl::PointXYZ min_pt, max_pt, cent_pt;
    int ind = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PCLCloud::Ptr cloud_cluster(new PCLCloud);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        pcl::computeCentroid(*cloud_cluster, cent_pt);

        *cluster_sum += *cloud_cluster;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        // std::cout << "Centroid of cluster: " << cent_pt << "\n"
        //   << std::endl;

        visualization_msgs::Marker cent_marker;
        cent_marker.pose.position.x = cent_pt.x;
        cent_marker.pose.position.y = cent_pt.y;
        cent_marker.pose.position.z = cent_pt.z;
        cent_marker.type = visualization_msgs::Marker::SPHERE;
        cent_marker.header.stamp = ros::Time::now();
        cent_marker.header.frame_id = msg->header.frame_id;
        cent_marker.ns = "centroids";
        cent_marker.id = ind;
        cent_marker.action = visualization_msgs::Marker::ADD;
        cent_marker.scale.x = 0.05;
        cent_marker.scale.y = 0.05;
        cent_marker.scale.z = 0.05;
        cent_marker.color.r = 0.0f;
        cent_marker.color.g = 1.0f;
        cent_marker.color.b = 0.0f;
        cent_marker.color.a = 0.75f;
        cent_marker.lifetime = ros::Duration(1.0);
        marker_pub.publish(cent_marker);

        // make rectangular marker as large as bounding box and put on centroid

        // publish xyz, size, index data to new topic and plot with Python

        dylan_msc::obj plot_obj;

        plot_obj.index = ind;

        plot_obj.centroid.x = cent_pt.x;
        plot_obj.centroid.y = cent_pt.y;
        plot_obj.centroid.z = cent_pt.z;

        plot_obj.min.x = min_pt.x;
        plot_obj.min.y = min_pt.y;
        plot_obj.min.z = min_pt.z;

        plot_obj.max.x = max_pt.x;
        plot_obj.max.y = max_pt.y;
        plot_obj.max.z = max_pt.z;

        // std_msgs::Int64 test;
        // test.data = 25;
        plotter_pub.publish(plot_obj);

        // test/setup camera in lab room or room above floor

        // fix near vision issues

        // get everything working on turtelbot

        // pose stuff

        // bayesian filter

        ind++;
    }

    ind = 0;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cluster_sum, output);
    output.header = msg->header;
    pcl_pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imageProc1");
    ros::NodeHandle node;

    pcl_pub = node.advertise<PCLCloud>("/points2", 10);
    marker_pub = node.advertise<visualization_msgs::Marker>("/mark2", 10);
    plotter_pub = node.advertise<dylan_msc::obj>("/plot2", 10);
    // plotter_pub = node.advertise<std_msgs::Int64>("/plot2", 10);

    ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 5, frame_cb);

    ros::spin();

    return 0;
}
