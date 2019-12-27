/**
 * @file: lidar_perception.h
 * @author: Z.H
 * @date: 2019.05.9
 */
#pragma once

//C++
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <mutex>
#include <omp.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include "Eigen/Dense"

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/UInt32.h"

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>


#define PI_OVER_180 (0.0174532925)//π ÷ 180

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudConstPtr;


class LidarPerception
{
public:

	LidarPerception();

	~LidarPerception();

    Eigen::Affine3f getTransformMat(const std::vector<double> &params);

    void init();

    void subLeftCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void subMidCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void subRightCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void conditionFilter(const PointCloudPtr &filter_cloud);

    void groundSegmenting(const PointCloudPtr &center_cloud);

    // make grid map
    void makeGridMap(const PointCloudPtr &cloudtogrid);
    
    void mainLoop();

private:

	//ros interface
	ros::NodeHandle nh_;
    ros::Subscriber left_lidar_sub_;
    ros::Subscriber mid_lidar_sub_;
    ros::Subscriber right_lidar_sub_;
	ros::Publisher center_lidar_pub_;
    ros::Publisher center_lidar_noground_pub_;
    ros::Publisher grid_map_pub_;
    ros::Publisher left_grid_map_pub_;
    ros::Publisher right_grid_map_pub_;
    ros::Publisher heart_pub_;

	//topic
	std::string left_lidar_topic_;
    std::string mid_lidar_topic_;
	std::string right_lidar_topic_;
    std::string center_lidar_topic_;
    std::string center_lidar_noground_topic_;
    std::string grid_map_topic_;
    std::string heart_topic_;

    PointCloudPtr left_cloud_;
    PointCloudPtr mid_cloud_;
    PointCloudPtr right_cloud_;
    PointCloudPtr center_cloud_;
    PointCloudPtr left_transfor_cloud_ptr_;
    PointCloudPtr mid_transfor_cloud_ptr_;
    PointCloudPtr right_transfor_cloud_ptr_;
    
    PointCloudPtr point_cloud_;
    PointCloudPtr seed_cloud_;
    PointCloudPtr ground_cloud_;
    PointCloudPtr no_ground_cloud_;


    //roi zone
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_; 
    double z_max_;

    //map_roi
    int map_x_min_;
    int map_x_max_;
    int map_y_min_;
    int map_y_max_;
    double map_resolution_;
    double car_width_;
    double car_length_;
    
    //condition filter range
    double x_filter_min = -20.0;
    double x_filter_max = 80.0;
    double y_filter_min = -15.0;
    double y_filter_max = 15.0;
    double z_filter_min = -1.5;
    double z_filter_max = 4;

    //plane ground filter params
    Eigen::MatrixXf normal_;
    float d_, th_dist_d_;
   
    //lidar calibration parameter
    std::vector<double> left_trans_params_;
    std::vector<double> mid_trans_params_;
    std::vector<double> right_trans_params_;
    Eigen::Affine3f left_transform_mat_;
    Eigen::Affine3f mid_transform_mat_;
    Eigen::Affine3f right_transform_mat_;

    
};
