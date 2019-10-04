#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

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

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

PCLCloud::Ptr cloud(new PCLCloud), cloud_f(new PCLCloud), cloud_filtered(new PCLCloud);
PCLCloud::Ptr cloud_msg(new PCLCloud);
ros::Publisher pcl_pub, marker_pub;

void frame_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *cloud); // Convert PointCloud2 ROS msg to PCL PointCloud
    // std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    // vg.filter(*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
    *cloud_filtered = *cloud;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.75 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
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

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(2000);
    ec.setMaxClusterSize(100000);
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
        // std::cout << "Centroid of cluster: " << cent_pt.x << "\n"
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
        ind++;
        cent_marker.action = visualization_msgs::Marker::ADD;
        cent_marker.scale.x = .1;
        cent_marker.scale.y = .1;
        cent_marker.scale.z = .1;
        cent_marker.color.r = 0.0f;
        cent_marker.color.g = 1.0f;
        cent_marker.color.b = 0.0f;
        cent_marker.color.a = 1.0;
        // ros::Duration quick_decay(1.0);
        cent_marker.lifetime = ros::Duration(1.0);
        marker_pub.publish(cent_marker);
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
    ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 1, frame_cb);

    pcl_pub = node.advertise<PCLCloud>("points2", 10);
    marker_pub = node.advertise<visualization_msgs::Marker>("mark2", 10);

    ros::spin();

    return 0;
}
