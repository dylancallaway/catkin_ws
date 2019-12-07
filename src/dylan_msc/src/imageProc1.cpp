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
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
// #include <Eigen/StdVector>

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

PCLCloud::Ptr cloud(new PCLCloud), cloud_f(new PCLCloud), cloud_filtered(new PCLCloud);
PCLCloud::Ptr cloud_msg(new PCLCloud);

ros::Publisher pcl_pub, marker_pub, plotter_pub;

float t0 = 0.0f;

uint64_t frame_index = 0;

Eigen::MatrixXf A(6, 6), Q(6, 6), u(6, 1), H(3, 6), R(3, 3);
Eigen::MatrixXf x_p(6, 1), x_m(3, 1), z(3, 1);
Eigen::MatrixXf P_p(6, 6), P_m(6, 6);

std::vector<Eigen::MatrixXf> x_ps, x_ms, x_ms_last, zs, P_ps, P_ms, zs_orig, x_ms_orig;

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
    pass.setFilterLimits(0.0f, 1.0f);
    pass.filter(*cloud_filtered); // returns cloud_filtered, the cloud with points outside of limits removed
    // Repeat for y axis (height)
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0f, 1.0f);
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
    seg.setDistanceThreshold(0.01f); // maximum euclidean distance between points considered to be planar

    int nr_points = (int)cloud_filtered->points.size(); // get size of cloud_filtered in number of points
    // Filter out largest planar model until cloud_filtered is reduced to XX% (see code) of original cloud_filtered
    bool check_val = 1;
    uint8_t check_sum = 0;
    uint8_t filter_index = 0;
    while (filter_index < 10) // TODO make this better...
    {
        // std::cout << filter_index << "\n"
        //   << std::endl;
        filter_index += 1;
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        // If no planar inliers exist
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        check_val = 1;
        static float delta_check = 0.2f;
        check_val = check_val && (coefficients->values[0] > -delta_check && coefficients->values[0] < delta_check);
        check_val = check_val && (coefficients->values[1] > -1.0f - delta_check && coefficients->values[1] < -1.0f + delta_check);
        check_val = check_val && (coefficients->values[2] > -delta_check && coefficients->values[2] < delta_check);
        // A  = 0
        // B = -1
        // C = 0
        // D can be anything

        if (!check_val)
        {
            continue;
        }

        // info about plane
        // std::cout << "Coeffs: " << coefficients->values[0] << " "
        //           << coefficients->values[1] << " "
        //           << coefficients->values[2] << " "
        //           << coefficients->values[3] << " "
        //           << inliers->indices.size() << " "
        //           << check_val << "\n"
        //           << std::endl;

        // check_sum += 1;
        // std::cout << check_sum << "\n"
        //           << std::endl;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        // extract.setNegative(false);

        // Get the points associated with the planar surface
        // extract.filter(*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points.\n"
        //   << std::endl;

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
    ec.setClusterTolerance(0.02f);
    ec.setMinClusterSize(250);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    PCLCloud::Ptr cluster_sum(new PCLCloud);
    pcl::PointXYZ min_pt, max_pt, cent_pt;
    float t1 = float(std::clock()) / float(CLOCKS_PER_SEC);
    float dt = t1 - t0;
    int num_clusters = 0;
    static int last_num_clusters = 0;
    // std::cout << dt << "\n"
    //   << std::endl;

    zs.clear();
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

        // if (cent_pt.y > 0.1f)
        // {
        // continue;
        // }

        z(0) = cent_pt.x;
        z(1) = cent_pt.y;
        z(2) = cent_pt.z;
        zs.push_back(z);

        *cluster_sum += *cloud_cluster;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        // std::cout << "Centroid of cluster: " << cent_pt << "\n"
        //   << std::endl;

        // visualization_msgs::Marker kal_marker;
        // kal_marker.pose.position.x = x_ms[i](0);
        // kal_marker.pose.position.y = x_ms[i](1);
        // kal_marker.pose.position.z = x_ms[i](2);
        // kal_marker.type = visualization_msgs::Marker::SPHERE;
        // kal_marker.header.stamp = ros::Time::now();
        // kal_marker.header.frame_id = msg->header.frame_id;
        // kal_marker.ns = "centroids";
        // kal_marker.id = i + 100;
        // kal_marker.action = visualization_msgs::Marker::ADD;
        // kal_marker.scale.x = 0.02;
        // kal_marker.scale.y = 0.02;
        // kal_marker.scale.z = 0.02;
        // kal_marker.color.r = 0.0f;
        // kal_marker.color.g = 0.0f;
        // kal_marker.color.b = 1.0f;
        // kal_marker.color.a = 1.0f;
        // kal_marker.lifetime = ros::Duration(0.75f);
        // marker_pub.publish(kal_marker);

        // dylan_msc::obj plot_obj;

        // plot_obj.index = i;

        // plot_obj.centroid.x = cent_pt.x;
        // plot_obj.centroid.y = cent_pt.y;
        // plot_obj.centroid.z = cent_pt.z;

        // plot_obj.min.x = min_pt.x;
        // plot_obj.min.y = min_pt.y;
        // plot_obj.min.z = min_pt.z;

        // plot_obj.max.x = max_pt.x;
        // plot_obj.max.y = max_pt.y;
        // plot_obj.max.z = max_pt.z;

        // plotter_pub.publish(plot_obj);

        num_clusters += 1;
    }

    // plot raw measurements
    for (int i = 0; i < num_clusters; i++)
    {
        visualization_msgs::Marker cent_marker;
        cent_marker.pose.position.x = zs[i](0);
        cent_marker.pose.position.y = zs[i](1);
        cent_marker.pose.position.z = zs[i](2);
        cent_marker.type = visualization_msgs::Marker::SPHERE;
        cent_marker.header.stamp = ros::Time::now();
        cent_marker.header.frame_id = msg->header.frame_id;
        cent_marker.ns = "centroids";
        cent_marker.id = i;
        cent_marker.action = visualization_msgs::Marker::ADD;
        cent_marker.scale.x = 0.05;
        cent_marker.scale.y = 0.05;
        cent_marker.scale.z = 0.05;
        cent_marker.color.r = 0.0f;
        cent_marker.color.g = 1.0f;
        cent_marker.color.b = 0.0f;
        cent_marker.color.a = 0.25f;
        cent_marker.lifetime = ros::Duration(0.75f);
        marker_pub.publish(cent_marker);
    }

    // filtering stuff
    if (frame_index < 10)
    {
        x_ms.clear();
        x_ps.clear();
        P_ms.clear();
        P_ps.clear();
        for (int i = 0; i < num_clusters; i++)
        {
            x_m(0) = zs[i](0);
            x_m(1) = zs[i](1);
            x_m(2) = zs[i](2);
            // x_m(3) = 0.0f;
            // x_m(4) = 0.0f;
            // x_m(5) = 0.0f;
            x_ms.push_back(x_m);
            x_ps.push_back(x_m);

            P_m << 0.1f * Eigen::MatrixXf::Identity(6, 6);
            P_ms.push_back(P_m);
            P_ps.push_back(P_m);
        }
    }
    else
    {
        std::cout << "\nFrame: " << frame_index << "\n";

        zs_orig = zs;                                     // store raw measurements
        Eigen::MatrixXf dists(num_clusters, x_ms.size()); // euc distance matrix (rows are measurements, cols are estimates)
        for (int j = 0; j < x_ms.size(); j++)             // for jth x_m (estimate)
        {
            for (int i = 0; i < num_clusters; i++) // for ith z (measurement)
            {
                float dist = (zs_orig[i] - x_ms[j]).norm(); // add H back here
                dists(i, j) = dist;                         // euc distance between zs[i] and x_ms[j]
            }
        }

        //-----------------------------------------------------------

        // check if meas has multiple est matches
        // happens if meas is removed (obj is removed)
        // x_ms.size() = num_clusters + num_objs_removed
        if (num_clusters < x_ms.size())
        {
            // find columnwise minimum of dists matrix
            Eigen::MatrixXf min_dists(1, x_ms.size());           // vec of minimum distance to nearest meas for each est
            Eigen::Matrix<int, -1, -1> min_inds(2, x_ms.size()); // mat of indices of nearest meas for each est
            int i_min, j_min;
            for (int j = 0; j < x_ms.size(); j++) // for each est
            {
                min_dists(j) = dists.col(j).minCoeff(&i_min, &j_min); // find min dist
                min_inds(0, j) = i_min;                               // min dist ind i
                min_inds(1, j) = j;                                   // min dist ind j
            }

            std::cout << "Dists:\n"
                      << dists << "\n\n"
                      << "Min Dists: " << min_dists << "\n\n"
                      << "Min Inds:\n"
                      << min_inds << "\n\n";

            Eigen::Matrix<int, -1, -1> meas_dup(x_ms.size(), x_ms.size());
            Eigen::MatrixXf dup_dists(x_ms.size(), x_ms.size());
            meas_dup.setConstant(-1);
            dup_dists.setConstant(100.0f);
            for (int j = 0; j < x_ms.size(); j++) // for each est
            {
                // get indices of each duplicate j
                for (int jj = j + 1; jj < x_ms.size(); jj++) // for each est except for previous est
                {
                    if (min_inds(0, j) == min_inds(0, jj)) // if current meas index matches another meas index (one meas matches to mult ests)
                    {
                        // j and jj are the measurement indices that have duplicates
                        // are also the indices of the estimates that the duplicate measurements got matched to
                        meas_dup(j, j) = j;
                        meas_dup(j, jj) = jj;
                        dup_dists(j, j) = dists(j, j);
                        dup_dists(j, jj) = dists(j, jj);
                    }
                }
            }

            std::cout << "Meas Dup:\n"
                      << meas_dup << "\n\n"
                      << "Dup Dists:\n"
                      << dup_dists << "\n\n";

            Eigen::Matrix<int, -1, -1> min_dup_ind(x_ms.size(), 1);
            min_dup_ind.setConstant(-1);
            for (int j = 0; j < x_ms.size(); j++) // for each est
            {
                // this checks if the whole meas_dup row is -1 (no dups found)
                bool no_dup_check = 1;
                for (int jj = 0; jj < x_ms.size(); jj++) // for each est
                {
                    no_dup_check = no_dup_check && meas_dup(j, jj) == -1;
                }
                if (no_dup_check == 0) // if a row with dups is found
                {
                    int i_min, j_min;
                    float min_dup_dist = dup_dists.row(j).minCoeff(&i_min, &j_min); // row wise minimum of rows with dups
                    min_dup_ind(j) = j_min;                                         // this is the est index to keep of all the dups of z(j)
                }
            }

            std::cout << "Min Dup Ind:\n"
                      << min_dup_ind << "\n\n";

            x_ms.clear();
            for (int j = 0; j < x_ms.size(); j++) // for each est
            {
                for (int jj = 0; jj < x_ms.size(); jj++) // for each est
                {
                    if (meas_dup(j, jj) == min_dup_ind(j) && meas_dup(j, jj) != -1)
                    {
                        x_ms.push_back(x_ms_orig[j]);
                    }
                }
            }
        }
        // --------------------------------------------------------------
        else if (num_clusters > x_ms.size())
        {
            // check if est has multiple meas matches
            // happens if meas is added (obj is added)
            // num_clusters = x_ms.size() + num_objs_added

            // find rowwise minimum of dists matrix
            Eigen::MatrixXf min_dists(num_clusters, 1);           // vec of minimum distance to nearest meas for each est
            Eigen::Matrix<int, -1, -1> min_inds(2, num_clusters); // mat of indices of nearest meas for each est
            int i_min, j_min;
            for (int i = 0; i < num_clusters; i++) // for each est
            {
                min_dists(i) = dists.row(i).minCoeff(&i_min, &j_min); // find min dist
                min_inds(0, i) = i;                                   // min dist ind i
                min_inds(1, i) = j_min;                               // min dist ind j
            }

            std::cout << "Dists:\n"
                      << dists << "\n\n"
                      << "Min Dists:\n"
                      << min_dists << "\n\n"
                      << "Min Inds:\n"
                      << min_inds << "\n\n";
        }

        // // this is all if num_clusters < x_m
        // // ie happens when removing an object from FOV
        // Eigen::Matrix<int, -1, -1> dup_inds(2, x_ms.size()); // indices of duplicate matches (ie. multiple estimates match to the same measurement)
        // dup_inds.setConstant(-1);
        // for (int j = 0; j < x_ms.size(); j++) // for each est
        // {
        //     for (int jj = j + 1; jj < x_ms.size(); jj++) // for each est except for previous est
        //     {
        //         if (min_inds(0, j) == min_inds(0, jj)) // if est has the same min inds (if it matches more than one meas)
        //         {
        //             dup_inds(0, j) = min_inds(0, j); // set dup_inds to meas inds for that match
        //             dup_inds(0, jj) = min_inds(0, jj);
        //             dup_inds(1, j) = j; // set dup_inds to est inds for that match
        //             dup_inds(1, jj) = jj;
        //         }
        //     }
        // }

        // x_ms_orig = x_ms;                               // store original x_ms
        // Eigen::MatrixXf dup_dists(1, x_ms_orig.size()); // distances of duped matches
        // int j_min = 100.0f, j_max = 100.0f;
        // int i_max = -1, j_max = -1;
        // for (int j = 0; j < x_ms.size(); j++) // for each est
        // {
        //     dup_dists.setConstant(-1.0f);
        //     if (dup_inds(0, j) == -1 && dup_inds(1, j) == -1) // if that est was not duped
        //     {
        //         continue;
        //     }
        //     else // if it was a duped est (primarily)
        //     {
        //         for (int jj = j + 1; jj < x_ms_orig.size(); jj++) // for each est other than previous est
        //         {
        //             if (dup_inds(0, jj) == dup_inds(0, j)) // find the other match dupes of the same meas
        //             {
        //                 dup_dists(j) = min_dists(j); // set dup_dists to min dist on those matches
        //                 dup_dists(jj) = min_dists(jj);
        //             }
        //         }
        //         // bool no_dup = 1; // var for checking if the meas was in fact duped
        //         // for (int i = 0; i < dup_dists.size(); i++)
        //         // {
        //         // no_dup = no_dup && dup_dists(i) == -1.0f;
        //         // }
        //         // if (no_dup == false)
        //         // {
        //         float min_dup_dist = dup_dists.minCoeff(&i_min, &j_min); // just getting j_min (est with smallest euc distance)
        //         float max_dup_dist = dup_dists.maxCoeff(&i_max, &j_max); // just getting j_max (est with largest euc distance)
        //         for (int j = 0; j < x_ms.size(); j++)                    // for each est
        //         {

        //         }
        //         x_ms.erase(x_ms.begin() + j_max); // pop the largest distance est
        //         std::cout << dup_dists << "\n"
        //                   << j_max << "\n";
        //         // }
        //     }
        // }

        //           << dup_inds << "\n\n";

        // if (num_clusters < x_ms.size()) // if we lose a cluster same as if (j[any] == j[any])
        // {
        //     for (int j = 0; j < x_ms.size(); j++)
        //     {
        //         for (int jj = 0; jj < x_ms.size(); jj++)
        //         {
        //             if (i_m[j] == i_m[jj])
        //             {
        //             }
        //         }
        //     }
        // }

        // std::vector<int> i_un; // vector of indices of UNmatched zs to x_ms
        // for (int i = 0; i < num_clusters; i++)
        // {
        //     i_un.push_back(i); // create vector i_un = {0, 1, 2, 3, ...}
        // }

        // for (int j = 0; j < i_m.size(); j++) // for each matched index
        // {
        //     for (int i = 0; i < i_un.size(); i++) // for each cluster index (i_un.size() = num_clusters on first iteration)
        //     {
        //         if (i_un[i] == i_m[j]) // if an unmatched cluster index == an already matched cluster index
        //         {
        //             i_un.erase(i_un.begin() + i); // erase the unmatched cluster index from the unmatched indices vector
        //         }
        //         if (i >= i_un.size() - 1) // if the erased index was the last index in the vector
        //         {
        //             break;
        //         }
        //     }
        // }

        // // debug printing
        // std::cout << "i_m\n";
        // for (int i = 0; i < i_m.size(); i++)
        // {
        //     std::cout << i_m[i] << "\n";
        // }

        // std::cout << "i_un\n";
        // for (int i = 0; i < i_un.size(); i++)
        // {
        //     std::cout << i_un[i] << "*****\n";
        // }

        // std::cout << "\n";

        // reorder zs based on i_m and i_un

        // for (int i = 0; i < i_un.size(); i++)
        // {
        // x_ms.push_back(zs_orig[i_un[i]]);
        // }

        // std::cout << "z: "
        //           << "\n"
        //           << zs[0]
        //           << "\n"
        //           << "x_m: "
        //           << x_ms[0]
        //           << "\n";

        // --------------------------------------------

        // kalman filter
        // -------------

        // x_p = x_ps[i];
        // x_m = x_ms[i];
        // P_p = P_ps[i];
        // P_m = P_ms[i];

        // // How to know Q, R, and H?
        // // Q keep ratio on Q but scale it down
        // // Q diags relate directly to state
        // // Q off diags relate one state to another
        // // H extracts only the direct measurements

        // Prediction update
        A << 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

        // x_p = A * x_m + u;
        // P_p = A * P_m * A.transpose() + Q;
        // x_ps[i] = x_p;
        // P_ps[i] = P_p;

        // // Measurement update
        // P_m = (P_p.inverse() + H.transpose() * R.inverse() * H).inverse();
        // x_m = x_p + P_m * H.transpose() * R.inverse() * (z - H * x_p);
        // P_ms[i] = P_m;
        // x_ms[i] = x_m;
        // // -------------

        // std::cout << "Frame Index: " << frame_index << "\n"
        //           << "Cluster Index: " << i << "\n"
        //           << "z:\n"
        //           << z << "\n"
        //           << "x_m:\n"
        //           << x_m << "\n"
        //           << "P_m:\n"
        //           << P_m << "\n";

        //fake kalman filter for debug
        x_ms = zs;
    }
    // --------------------------------------------

    t0 = t1;
    last_num_clusters = num_clusters;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cluster_sum, output);
    output.header = msg->header;
    pcl_pub.publish(output);
    frame_index += 1;
    std::cout << "\n";
}

int main(int argc, char **argv)
{
    // static matrices
    u << 0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f;

    Q << 0.025f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f,
        0.05f, 0.025f, 0.05f, 0.05f, 0.05f, 0.05f,
        0.05f, 0.05f, 0.025f, 0.05f, 0.05f, 0.05f,
        0.05f, 0.05f, 0.05f, 0.1f, 0.05f, 0.05f,
        0.05f, 0.05f, 0.05f, 0.05f, 0.1f, 0.05f,
        0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.1f;

    R << 0.5f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f,
        0.0f, 0.0f, 0.5f;

    H << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;

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
