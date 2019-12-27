/**
 * @file: lidar_perception.cpp
 * @author: Z.H
 * @date: 2019.05.9
 */
#include "lidar_perception/lidar_perception.h"

static bool point_cmp(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.z < b.z;
}

LidarPerception::LidarPerception():			 
									mid_transfor_cloud_ptr_(new PointCloud()),
					               left_transfor_cloud_ptr_(new PointCloud()),
					              right_transfor_cloud_ptr_(new PointCloud()),
				  seed_cloud_(new PointCloud()), mid_cloud_(new PointCloud()),
			    left_cloud_(new PointCloud()), right_cloud_(new PointCloud()),
			  point_cloud_(new PointCloud()), center_cloud_(new PointCloud()),
		  ground_cloud_(new PointCloud()), no_ground_cloud_(new PointCloud()){
	ROS_INFO("[Z.H]begin to lidar_perception!");
}

LidarPerception::~LidarPerception() {
	ros::shutdown();
}

Eigen::Affine3f LidarPerception::getTransformMat(const std::vector<double> &params)
{
	Eigen::Affine3f result = Eigen::Affine3f::Identity();

	result.translation() << params[0], params[1], params[2];

	result.rotate (Eigen::AngleAxisf(params[3] * PI_OVER_180, Eigen::Vector3f::UnitX())
	             * Eigen::AngleAxisf(params[4] * PI_OVER_180, Eigen::Vector3f::UnitY())
	             * Eigen::AngleAxisf(params[5] * PI_OVER_180, Eigen::Vector3f::UnitZ()));

	return result;
}

void LidarPerception::init()
{
	nh_.getParam("/lidar_perception/left_lidar_topic", left_lidar_topic_);
	nh_.getParam("/lidar_perception/mid_lidar_topic", mid_lidar_topic_);
	nh_.getParam("/lidar_perception/right_lidar_topic", right_lidar_topic_);
	nh_.getParam("/lidar_perception/center_lidar_topic", center_lidar_topic_);
	nh_.getParam("/lidar_perception/center_lidar_noground_topic", center_lidar_noground_topic_);
	nh_.getParam("/lidar_perception/grid_map_topic", grid_map_topic_);
	nh_.getParam("/lidar_perception/heart_topic", heart_topic_);

	nh_.getParam("/lidar_perception/roi/x_min", x_min_);
	nh_.getParam("/lidar_perception/roi/y_min", y_min_);
	nh_.getParam("/lidar_perception/roi/z_min", z_min_);
    nh_.getParam("/lidar_perception/roi/x_max", x_max_);
    nh_.getParam("/lidar_perception/roi/y_max", y_max_);
    nh_.getParam("/lidar_perception/roi/z_max", z_max_);

    nh_.getParam("/lidar_perception/map_resolution", map_resolution_);
    nh_.getParam("/lidar_perception/car_info/car_width", car_width_);
    nh_.getParam("/lidar_perception/car_info/car_length", car_length_);

	nh_.param("/lidar_perception/trans_params/left", 
		                left_trans_params_, std::vector<double>(0));
	nh_.param("/lidar_perception/trans_params/mid", 
		                mid_trans_params_, std::vector<double>(0));
	nh_.param("/lidar_perception/trans_params/right", 
		               right_trans_params_, std::vector<double>(0));

	if(left_trans_params_.empty() || right_trans_params_.empty() || mid_trans_params_.empty())
    	ROS_ERROR("[Z.H]lidar trans_params load failure!!!");

    left_lidar_sub_ = nh_.subscribe(left_lidar_topic_, 1, 
    	              &LidarPerception::subLeftCallback, this);
    mid_lidar_sub_ = nh_.subscribe(mid_lidar_topic_, 1, 
    	              &LidarPerception::subMidCallback, this);
    right_lidar_sub_ = nh_.subscribe(right_lidar_topic_, 1, 
    	              &LidarPerception::subRightCallback, this);

    center_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(center_lidar_topic_, 10, this);
    center_lidar_noground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(center_lidar_noground_topic_, 10, this);
    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_map_topic_, 10, this);
    left_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/left/gridmap", 10, this);
    right_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/right/gridmap", 10, this);
    heart_pub_ = nh_.advertise<std_msgs::UInt32>(heart_topic_, 10, this);
    

    // get transform_mat
	left_transform_mat_ = getTransformMat(left_trans_params_);
	mid_transform_mat_ = getTransformMat(mid_trans_params_);
	right_transform_mat_ = getTransformMat(right_trans_params_);
	std::cout << "left_transform_mat:" << std::endl << left_transform_mat_.matrix() << std::endl;
	std::cout << "mid_transform_mat:" << std::endl << mid_transform_mat_.matrix() << std::endl;
	std::cout << "right_transform_mat:" << std::endl << right_transform_mat_.matrix() << std::endl;

	//set map roi value
	//In the coordinate system conversion, the lidar coordinate system should be converted to the Car coordinate system of the local vehicle, x to the right, y to the front.
	map_x_min_ = 0;
	map_x_max_ = (int)((y_max_ - y_min_)/map_resolution_);
	map_y_min_ = 0;
	map_y_max_ = (int)((x_max_ - x_min_)/map_resolution_);
}


void LidarPerception::subLeftCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *left_cloud_);
    //ROS_INFO("left point size %ld ", left_cloud_->points.size());
}

void LidarPerception::subMidCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *mid_cloud_);
    //ROS_INFO("mid point size %ld ", mid_cloud_->points.size());
}

void LidarPerception::subRightCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *right_cloud_);
    //ROS_INFO("right point size %ld ", right_cloud_->points.size());
}

void LidarPerception::conditionFilter(const PointCloudPtr &filter_cloud)
{
	if(!filter_cloud->points.empty()) 
	{
		for(size_t i = 0; i < filter_cloud->points.size(); ++i) 
		{
			if(filter_cloud->points[i].x < x_filter_min || filter_cloud->points[i].x > x_filter_max ||
			   filter_cloud->points[i].y < y_filter_min || filter_cloud->points[i].y > y_filter_max ||
			   filter_cloud->points[i].z < z_filter_min || filter_cloud->points[i].z > z_filter_max ||
			   								  !pcl_isfinite(filter_cloud->points[i].x) || 
			   								  !pcl_isfinite(filter_cloud->points[i].y) ||
			   								  !pcl_isfinite(filter_cloud->points[i].z)) {
				continue;
			}
			else if((filter_cloud->points[i].x <= 0.15) && (filter_cloud->points[i].x >= (-car_length_- 0.15)) &&
					(filter_cloud->points[i].y <= (car_width_/2 + 0.15)) && (filter_cloud->points[i].y >= (- car_width_/2 - 0.15))) {
				continue;
			}
			else {
				center_cloud_->points.push_back(filter_cloud->points[i]);
			}
		}	
	}
}

void LidarPerception::groundSegmenting(const PointCloudPtr &center_cloud)
{
	if(!center_cloud->points.empty()) 
	{
		//1.Publish fusion to center pointcloud
		sensor_msgs::PointCloud2 centerMsg;
        pcl::toROSMsg(*center_cloud, centerMsg);
        centerMsg.header.frame_id = "base_link";
        centerMsg.header.stamp = ros::Time::now();        
        center_lidar_pub_.publish(centerMsg);

        //2.sort on Z-axis value.
		std::sort(center_cloud->points.begin(), center_cloud->points.end(), point_cmp);
		
		//3.extract init ground seeds.
	    //LPR is the mean of low point representative
	    int cnt = 0;
	    double sum = 0;
	    //4.calculate the mean height value.
	    for (size_t i = 0; i < center_cloud->points.size() && cnt < 25; ++i) {
	        sum += center_cloud->points[i].z;
	        ++cnt;    
	    }
	    double lpr_height = (cnt != 0 ? sum / cnt : 0); // in case divide by 0
	    seed_cloud_->clear();
	    ground_cloud_->clear();
	    //5.iterate pointcloud, filter those height is less than lpr.height+th_seeds_
	    #pragma omp parallel for
	    for (size_t i = 0; i < center_cloud->points.size(); ++i)
	    {
	        if (center_cloud->points[i].z < lpr_height + 1.2)
	        {
	            seed_cloud_->points.push_back(center_cloud->points[i]);
	        }
	    }
	    pcl::copyPointCloud(*seed_cloud_, *ground_cloud_); //copy
	   	//ROS_INFO("[Z.H]seed_cloud size: %ld ", seed_cloud_->points.size());
	 
	    //6.ground plane fitter mainloop
	    for (size_t i = 0; i < 4; ++i)//num_iter = 4
	    {
	        //create covarian matrix in single pass.
		    Eigen::Matrix3f cov;
		    Eigen::Vector4f pc_mean;
		    pcl::computeMeanAndCovarianceMatrix(*ground_cloud_, cov, pc_mean);
		    //Singular Value Decomposition: SVD
		    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
		    //use the least singular vector as normal
		    normal_ = (svd.matrixU().col(2));
		    // mean ground seeds value
		    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
		    //according to normal.T*[x,y,z] = -d
		    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
		    //set distance threhold to `th_dist - d`
		    th_dist_d_ = 0.3 - d_;
		    ground_cloud_->clear();
		    no_ground_cloud_->clear();
	        //pointcloud to matrix
	        Eigen::MatrixXf points(center_cloud->points.size(), 3);
	        int j = 0;
	        #pragma omp for
	        for (const auto &p : center_cloud->points)
	    	{
	            points.row(j++) << p.x, p.y, p.z;
	        }
	        //ground plane model
	        Eigen::VectorXf result = points * normal_;
	        //threshold filter
	        #pragma omp for
	        for (size_t r = 0; r < result.rows(); ++r)
	        {
	            if (result[r] < th_dist_d_) {
	                ground_cloud_->points.push_back(center_cloud->points[r]);// means ground
	            }
	            else {
	                no_ground_cloud_->points.push_back(center_cloud->points[r]);// means not ground 
	            }
	        }
	    }
	    ROS_INFO("[Z.H]center_ground_cloud size: %d", ground_cloud_->points.size());
	    ROS_INFO("[Z.H]center_noground_cloud size: %d", no_ground_cloud_->points.size());
	   
	    //7.Publish no ground points
	    sensor_msgs::PointCloud2 groundless_msg;
	    pcl::toROSMsg(*no_ground_cloud_, groundless_msg);
	    groundless_msg.header.frame_id = "base_link";
        groundless_msg.header.stamp = ros::Time::now();  
	    center_lidar_noground_pub_.publish(groundless_msg);  
    }
}


void LidarPerception::makeGridMap(const PointCloudPtr &cloudtogrid)
{
	//ROS_INFO("begin to make grid map!!!");
	nav_msgs::OccupancyGrid map;
	nav_msgs::OccupancyGrid map_left;
	nav_msgs::OccupancyGrid map_right;
    
	map.header.frame_id = "base_link";
  	map.header.stamp = ros::Time::now();
  	map_left.header.frame_id = "base_link";
  	map_left.header.stamp = ros::Time::now();
  	map_right.header.frame_id = "base_link";
  	map_right.header.stamp = ros::Time::now();

  	int map_left_size = ((int)(car_width_*1.5)/map_resolution_) * ((int)(car_length_*4)/map_resolution_);//(1.5 * car_width_) * (3 * car_length_) m 
  	map_left.info.resolution = map_resolution_;
	map_left.info.width = ((int)(car_width_*1.5)/map_resolution_);
	map_left.info.height = ((int)(car_length_*4)/map_resolution_);
	map_left.info.origin.position.x = (-((int)(car_width_*1.5)) - (car_width_/2));
	map_left.info.origin.position.y = (-((int)(car_length_*2)));
	map_left.info.origin.position.z = 0;
	map_left.data.resize(map_left_size, 0);
	// std::cout << "AAA :" << map_left_size << "; " << map_left.info.width << "; " << map_left.info.height 
	//           << "; " << map_left.info.origin.position.x << "; " << map_left.info.origin.position.y << std::endl;

	int map_right_size = ((int)(car_width_*1.5)/map_resolution_) * ((int)(car_length_*4)/map_resolution_);//(1.5 * car_width_) * (3 * car_length_) m 
  	map_right.info.resolution = map_resolution_;
	map_right.info.width = ((int)(car_width_*1.5)/map_resolution_);
	map_right.info.height = ((int)(car_length_*4)/map_resolution_);
	map_right.info.origin.position.x = (0 + (car_width_/2));
	map_right.info.origin.position.y = (-((int)(car_length_*2)));
	map_right.info.origin.position.z = 0;
	map_right.data.resize(map_right_size, 0);

	int map_width = (int)(map_x_max_ - map_x_min_);//80 pix
	int map_height = (int)(map_y_max_ - map_y_min_);//240 pix
	int map_size = map_width * map_height;
	map.info.resolution = map_resolution_;
	map.info.width = map_width;
	map.info.height = map_height;
	map.info.origin.position.x = y_min_;
	map.info.origin.position.y = x_min_;//In the coordinate system conversion, the lidar coordinate system should be converted to the Car coordinate system of the local vehicle, x to the right, y to the front.
	map.info.origin.position.z = 0;
	map.data.resize(map_size, 0);

	for(size_t i = 0; i < cloudtogrid->points.size(); i++) 
	{
		//make gridmap
		if(cloudtogrid->points[i].x >= x_min_ && cloudtogrid->points[i].x <= x_max_ &&
		   cloudtogrid->points[i].y >= y_min_ && cloudtogrid->points[i].y <= y_max_ &&
		   cloudtogrid->points[i].z >= z_min_ && cloudtogrid->points[i].z <= z_max_)
		{
			int map_x = (int)(-(cloudtogrid->points[i].y - y_max_)/map_resolution_);
			int map_y = (int)((cloudtogrid->points[i].x - x_min_)/map_resolution_);
			int index = map_y * map_width + map_x;
			if(index < map_size && index >= 0 && map_x >= 0 && map_y >= 0) {
				map.data[index] += 1; 
			}
		}
		//make left gridmap
		if(cloudtogrid->points[i].x >= (-((int)(car_length_*2))) && cloudtogrid->points[i].x <= (car_length_*2) &&
		   cloudtogrid->points[i].y >= (car_width_/2) && cloudtogrid->points[i].y <= ((car_width_/2) + (int)(car_width_*1.5)) &&
		   cloudtogrid->points[i].z >= z_min_ && cloudtogrid->points[i].z <= z_max_)
		{
			int map_left_x = (int)(-(cloudtogrid->points[i].y - ((car_width_/2) + (int)(car_width_*1.5)))/map_resolution_);
			int map_left_y = (int)((cloudtogrid->points[i].x - (-((int)(car_length_*2))))/map_resolution_);
			int index = map_left_y * ((int)(car_width_*1.5)/map_resolution_) + map_left_x;
			if(index < map_left_size && index >= 0 && map_left_x >= 0 && map_left_y >= 0) {
				map_left.data[index] += 1; 
			}
		}
		//make right gridmap
		if(cloudtogrid->points[i].x >= (-((int)(car_length_*2))) && cloudtogrid->points[i].x <= (car_length_*2) &&
		   cloudtogrid->points[i].y >= (-(car_width_/2) - ((int)(car_width_*1.5))) && cloudtogrid->points[i].y <= -(car_width_/2) &&
		   cloudtogrid->points[i].z >= z_min_ && cloudtogrid->points[i].z <= z_max_)
		{
			int map_right_x = (int)(-(cloudtogrid->points[i].y - (-(car_width_/2)))/map_resolution_);
			int map_right_y = (int)((cloudtogrid->points[i].x - (-((int)(car_length_*2))))/map_resolution_);
			int index = map_right_y * ((int)(car_width_*1.5)/map_resolution_) + map_right_x;
			if(index < map_right_size && index >= 0 && map_right_x >= 0 && map_right_y >= 0) {
				map_right.data[index] += 1; 
			}
		}	
	}

	/*
	static double old_map[80*240] = {0};
	static double old_map_right[16*72] = {0};	
	static double old_map_left[16*72] = {0};
	
	for(size_t j = 0; j < map_size; j++)
	{
		map.data[j] = map.data[j] * 0.5 + old_map[j] * 0.5;
		old_map[j] = map.data[j];
	}
	for(size_t j = 0; j < map_left_size; j++)
	{
		map_left.data[j] = map_left.data[j] * 0.5 + old_map_left[j] * 0.5;
		old_map_left[j] = map_left.data[j];
	}
	for(size_t j = 0; j < map_right_size; j++)
	{
		map_right.data[j] = map_right.data[j] * 0.5 + old_map_right[j] * 0.5;
		old_map_right[j] = map_right.data[j];
	}
	for(size_t j = 0; j < map_size; j++)
    {
    	//calculate the num of pointcloud threshold
    	double  num = 0;
    	if((j/map_width) <= 20 && (j/map_width) >= 0)
    		num = 8.0;
    	else if((j/map_width) < 140)
    		num = -0.0583 * (j/map_width) + 9.1666;
    	else
    		num = 1.0;
    	//judge occupied or not
        if(map.data[j] >= num) {
			map.data[j] = 100;          
        }
        else {
        	map.data[j] = 0;
        }
    }*/


	int j = 0;
    for(int i = 0; i < map_height; i++)
    {
    	double  num = 0;
    	double  slop = (5.0 - 1.0)/(20-100);
    	
    	if(i <= 20 && i >= 0)
    		num = 5.0;
    	else if(i < 100)
    		num = slop * (i - 100) + 1.0;
    	else
    		num = 1.0; 

    	for(int k = 0; k < map_width; k++)
    	{
	        if(map.data[j] >= num) 
	        {
				map.data[j] = 100;          
	        }
	        else 
	        {
	        	map.data[j] = 0;
	        }
	        
	        j++;
    	}

    }
	
    for(size_t j = 0; j < map_left_size; j++)
    {
        if(map_left.data[j] >= 3) {
			map_left.data[j] = 100;          
        }
        else {
        	map_left.data[j] = 0;
        }
    }

	   
    for(size_t j = 0; j < map_right_size; j++)
    {
        if(map_right.data[j] >= 3) {
			map_right.data[j] = 100;          
        }
        else {
        	map_right.data[j] = 0;
        }
    }

	//gridmap publish
	grid_map_pub_.publish(map);
	left_grid_map_pub_.publish(map_left);
	right_grid_map_pub_.publish(map_right);
}

void LidarPerception::mainLoop()
{
    ros::Rate loop_rate(25);

    while(nh_.ok()) {

        ros::spinOnce();
        ros::Time begin = ros::Time::now(); 

        //1.converting the pointcloud of the left and right lidar coordinate system to car body coordinate system
	    pcl::transformPointCloud(*left_cloud_, *left_transfor_cloud_ptr_, left_transform_mat_);
	    pcl::transformPointCloud(*mid_cloud_, *mid_transfor_cloud_ptr_, mid_transform_mat_);
	    pcl::transformPointCloud(*right_cloud_, *right_transfor_cloud_ptr_, right_transform_mat_);

	    point_cloud_->clear();
	    point_cloud_->points.insert(point_cloud_->points.end(),
	    				  left_transfor_cloud_ptr_->points.begin(),left_transfor_cloud_ptr_->points.end());
	    point_cloud_->points.insert(point_cloud_->points.end(),
	    				  mid_transfor_cloud_ptr_->points.begin(),mid_transfor_cloud_ptr_->points.end());
    	point_cloud_->points.insert(point_cloud_->points.end(),
    		             right_transfor_cloud_ptr_->points.begin(),right_transfor_cloud_ptr_->points.end());
		ROS_INFO("[Z.H]point_cloud size: %ld ", point_cloud_->points.size());
		
	    //2.condition filter
	    center_cloud_->clear();
	    conditionFilter(point_cloud_);
	    ROS_INFO("[Z.H]center_cloud size: %ld ", center_cloud_->points.size());

	    //3.ground segmenting
	    groundSegmenting(center_cloud_);
		
	    //4.constructing 2-D grid map and publish
        makeGridMap(no_ground_cloud_);
        
        ROS_INFO("[Lidar Perception] use %f s.\n\n", (ros::Time::now() - begin).toSec());  
        loop_rate.sleep();           
    }
}

