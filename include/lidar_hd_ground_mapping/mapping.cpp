/*
 * mapping.cpp
 *
 *  Created on: 2017/10/06
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#include "mapping.h"

//STD
#include <ios>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <type_traits>
#include <pthread.h>
#include <signal.h>
#include <chrono>
#include <algorithm>
#include <functional>
#include <mutex>
#include <thread>

//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
//for test/demos
//#include <rviz_visual_tools/rviz_visual_tools.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/filters/convolution.h>
//#include <pcl/filters/radius_outlier_removal.h>

//BOOST
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>

//OMP
//#include <omp.h>

#define DEBUGGING_MSG 0
#if DEBUGGING_MSG
#define DEBUG_POINT(p) (std::cout << "Debug point " << p << " at function: " << std::string(__func__) << ", line: " << __LINE__ << std::endl)
#else
#define DEBUG_POINT(p)
#endif
#if DEBUGGING_MSG
#define DEBUG_MSG(m) m
#else
#define DEBUG_MSG(m)
#endif

struct RingComparator {
	bool operator()(velodyne_pointcloud::PointXYZIR& val1, velodyne_pointcloud::PointXYZIR& val2) const {
		return (val1.ring < val2.ring);
	}
};

std::function<bool(const velodyne_pointcloud::PointXYZIR*&, const velodyne_pointcloud::PointXYZIR*& )> ringSortFn =
		[](auto&& val1, auto&& val2) ->bool { return (val1->ring < val2->ring); };

//Defines the FAST_SWAP macro
/**
 * @def FAST_SWAP
 * \brief Very fast swapping of two variables
 * If not defined, we'll do it
 */
#ifndef FAST_SWAP
#define FAST_SWAP(a,b) ( ((a) == (b)) || ( ((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)) ) )
#endif

enum {
    EXTRA_SIDE_POINTS = 4,
	MAX_SCAN_POINTS = 20000, //HDL-64E is about 131xxx points, less than 2100 points per ring, more than enough
};
/**
 * \brief Window for median filter calculation
 */
unsigned int median_window[MAX_SCAN_POINTS + EXTRA_SIDE_POINTS + 1] = {0};


typedef unsigned int elem_type;
static elem_type medianfilter3_fast(elem_type arr[])
{
    register elem_type a = arr[0];
    register elem_type b = arr[1];
    register elem_type c = arr[2];
    elem_type retVal = 0;


    if ((a < b && b > c && a < c) || (a > b && b < c && a > c)) {
        retVal = c;
    } else if ((a < b && b > c && a > c) || (a > b && b < c && a < c)) {
        retVal = a;
    } else {
        retVal = b;
    }

    return (retVal);
}

static elem_type medianfilter5_fast(elem_type arr[])
{
    register elem_type a = arr[0];
    register elem_type b = arr[1];
    register elem_type c = arr[2];
    register elem_type d = arr[3];
    register elem_type e = arr[4];
    elem_type retVal = 0;

    //(1) Make sure data is ordered such a<d, b<e, a<d
    //this takes 3 comparisons
    if ( a > b) {
        FAST_SWAP(a,b);
    }
    if ( d > e) {
        FAST_SWAP(d,e);
    }
    if ( a > d) {
        FAST_SWAP(a,d);
        FAST_SWAP(b,e);
    }

    //(2) select the median
    if ( c > b) { //4th comparison
        if ( b < d ) { //5th comparison
            retVal = std::min(c,d);
        } else {
            retVal = std::min(b,e);
        }
    } else {
        if ( c > d ) { //6th comparison
            retVal = std::min(c,e);
        } else {
            retVal = std::min(b,d);
        }
    }
    return (retVal);
}

static void medianfilter_3_fast(unsigned int *data_filter, unsigned int start, unsigned int end)
{
    register unsigned int *range = NULL;
    register unsigned int* ptr = NULL;
    unsigned int i = 0;

    //fail-safe checks
    if (!data_filter) {
        return;
    }

    //initialize the pointers
    range = (unsigned int*)&data_filter[start];
    ptr = (unsigned int*)&median_window[0];

    //initialise the median window
    *ptr++ = *(unsigned int*)range; //(start-1) = [start]
    for (i = start; i <= end; i++)  {    //start..end
        *ptr++ = *range++;
    }
    range = (unsigned int*)&data_filter[end]; //go back to [end]
    *ptr++ = *range; //(end+1) = [end]

    //start computing the median values
    range = (unsigned int*)&data_filter[start];

    ptr = (unsigned int*)&median_window[0];
    for (i = start; i <= end; i++)  {
        *range++ = medianfilter3_fast(ptr);
        ptr++;
    }
}

static void medianfilter_5_fast(unsigned int *data_filter, unsigned int start, unsigned int end)
{
    register unsigned int *range = (unsigned int*)NULL;
    register unsigned int* ptr = (unsigned int*)NULL;
    unsigned int i = 0;

    //fail-safe checks
    if (!data_filter) {
        return;
    }

    //initialize the pointers
    range = (unsigned int*)&data_filter[start];
    ptr = (unsigned int*)&median_window[0];

    //initialize the median window
    *ptr++ = *range; //(start-2) = [start]
    *ptr++ = *range; //(start-1) = [start]
    for (i = start; i <= end; i++)  {    //start..end
        *ptr++ = *range++;
    }
    //range -= 2; //go back to [end]
    range = (unsigned int*)&data_filter[end]; //go back to [end]
    *ptr++ = *range; //(end+1) = [end]
    *ptr++ = *range; //(end+2) = [end]

    //start computing the median values
    range = (unsigned int*)&data_filter[start];
    ptr = (unsigned int*)&median_window[0];
    for (i = start; i <= end; i++)  {
        *range++ = medianfilter5_fast(ptr);
        ptr++;
    }
}

namespace ground_mapping {
	Mapping* Mapping::instance = NULL;

	const int Mapping::WINDOW_SIZE = 2000; //1000
	const int Mapping::WIDTH=4096; //4096
	const int Mapping::HEIGHT=4096; //4096
	const double Mapping::XMAX=250.0; //250
	const double Mapping::XMIN=-250.0; //-250
	const double Mapping::YMAX=250.0; //250
	const double Mapping::YMIN=-250.0; //-250
	const double Mapping::XDIFF = (Mapping::XMAX-Mapping::XMIN);
	const double Mapping::YDIFF = (Mapping::YMAX-Mapping::YMIN);
	const double Mapping::HMAX=200.0;
	const double Mapping::HMIN=-800.0;
	const double Mapping::CELL_SIZE = 1.0;
    const double Mapping::MAX_DISTANCE = 5.0;
	const double Mapping::DELTA_Z_THRESHOLD = 0.08;
	const unsigned int Mapping::DELTA_Z_SAMPLES = 10;
	const unsigned int Mapping::MAX_DEPTH_BUFFERS=4;
	const double Mapping::LOOP_RATE=100.0;

Mapping::Mapping() :
            loop_rate_(ros::Rate(LOOP_RATE))

#if (USE_OCTREE_SEARCH)
		, octree_resolution_(0.1)
		, octree_(octree_resolution_)
#endif
	{
                setInstance();
                hidden_win_id_ = -1;
        #if (USE_OCTREE_SEARCH)
                octree_.setEpsilon(0.001);
        #endif
	}

	Mapping::~Mapping()
	{
		deInit();
	}

	void Mapping::setInstance()
	{
		instance = this;
	}

	Mapping::Mapping(ros::NodeHandle *nh) : node_handle_(*nh),
                                            loop_rate_(ros::Rate(LOOP_RATE))
    //init(int argc, char **argv)
	{

		waiting_for_tf_ = false;
		switch_dm_ = false;
		update_distance_ = 0.0;
		stop_program_ = 0;
		zoom_ = 0.0;
		redisplay_ = 0;
		first_display_ = 1;
		lidar_pose_ = std::vector<double>(6,0.0);
		prev_xo_ = 0.0;
		prev_yo_ = 0.0;
		prev_zo_ = 0.0;
		delta_z_ = 0.0;
		avg_delta_z_ = 0.0;
		delta_z_count_ = 0;
		total_running_time_ = 0.0;
		loops_count_ = 0;

		saving_file_ = false;
		publishing_obst_cloud_ = false;
		publishing_ground_cloud_ = false;

		loadAllParameters();

		min_height_offset_ = min_height_;
		max_height_offset_ = max_height_;

		//TF related
		tf_listener_  = new tf::TransformListener();

		//subscribers
		vel_subscriber_ = nodehandle_.subscribe(input_pointcloud_topic_, 10, &Mapping::velodyneCallback, this);
		//publishers
		depth_ground_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(output_pointcloud_topic_, 10);

		if (generate_obstacle_data_) {
			obstacle_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(obstacle_pointcloud_topic_, 10);
		}
		if (generate_ground_data_) {
			ground_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(ground_pointcloud_topic_, 10);
		}

		//creates the point array for parallel processing
		point_array_ = new pcl::PointXYZI[WINDOW_SIZE*WINDOW_SIZE];
		if (!point_array_) {
			ROS_ERROR_STREAM("Failed allocating " << WINDOW_SIZE*WINDOW_SIZE*sizeof(pcl::PointXYZI) << " bytes for point_array");
			exit(1);
		}


		pcl_ground_ptr_ = pcl::PointCloud<pcl::PointXYZI>::ConstPtr(&pcl_depth_ground_, &DoNotFree<pcl::PointCloud<pcl::PointXYZI>>);
	}

	void Mapping::deInit()
	{
		std::cout << "Ending program" << std::endl;
		stop_program_ = true;

		//Wait for threads to end
		if (publishing_ground_cloud_) {
			publish_ground_thread_.join();
		}
		if (publishing_obst_cloud_) {
			publish_obst_thread_.join();
		}
		if (save_height_map_ && saving_file_) {
			record_thread_.join();
		}
		//ros_thread_.join();

		//delete parameters
		nodehandle_.deleteParam("/ground_mapping/min_height");
		nodehandle_.deleteParam("/ground_mapping/max_height");
		nodehandle_.deleteParam("/ground_mapping/input_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/output_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/source_tf");
		nodehandle_.deleteParam("/ground_mapping/target_tf");

        nodehandle_.deleteParam("/ground_mapping/obstacle_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/ground_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/lidar_pose");

        nodehandle_.deleteParam("/ground_mapping/generate_obstacle_data");
		nodehandle_.deleteParam("/ground_mapping/generate_ground_data");
		nodehandle_.deleteParam("/ground_mapping/obstacle_height_threshold");

        nodehandle_.deleteParam("/ground_mapping/filter_input_points");
		nodehandle_.deleteParam("/ground_mapping/filter_input_rings");
		nodehandle_.deleteParam("/ground_mapping/filter_height_limit");
		nodehandle_.deleteParam("/ground_mapping/hide_opengl_window");
		nodehandle_.deleteParam("/ground_mapping/max_update_distance");
		nodehandle_.deleteParam("/ground_mapping/merge_with_past_frame");
		nodehandle_.deleteParam("/ground_mapping/accumulate_height");
		nodehandle_.deleteParam("/ground_mapping/map_loader_mode");
		nodehandle_.deleteParam("/ground_mapping/save_height_map");
		nodehandle_.deleteParam("/ground_mapping/height_map_filename_prefix");
		nodehandle_.deleteParam("/ground_mapping/save_every_scan");
	}

	void Mapping::loadAllParameters()
	{
		min_height_ = getParam(nodehandle_, "/ground_mapping/min_height", HMIN);
		max_height_ = getParam(nodehandle_, "/ground_mapping/max_height", HMAX);
		input_pointcloud_topic_  = getParam(nodehandle_, "/ground_mapping/input_pointcloud_topic", std::string("velodyne_points"));
		output_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/output_pointcloud_topic", std::string("points_raw_ground"));

        obstacle_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/obstacle_pointcloud_topic", std::string("points_obstacles"));
		ground_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/ground_pointcloud_topic", std::string("points_ground"));
		if (!(nodehandle_.getParam("/ground_mapping/lidar_pose", lidar_pose_))) {
			lidar_pose_ = std::vector<double>(6,0.0);
		}
		source_tf_  = getParam(nodehandle_, "/ground_mapping/source_tf", std::string("world"));
		target_tf_ = getParam(nodehandle_, "/ground_mapping/target_tf", std::string("velodyne"));

        generate_obstacle_data_ = getParam(nodehandle_, "/ground_mapping/generate_obstacle_data", false);
		generate_ground_data_ = getParam(nodehandle_, "/ground_mapping/generate_ground_data", false);
		obstacle_height_threshold_ = getParam(nodehandle_, "/ground_mapping/obstacle_height_threshold", 0.1);

        filter_input_points_ = getParam(nodehandle_, "/ground_mapping/filter_input_points", false);
		filter_input_rings_ = getParam(nodehandle_, "/ground_mapping/filter_input_rings", -1);
		filter_by_height_ = getParam(nodehandle_, "/ground_mapping/filter_by_height", false);
		filter_height_limit_ = getParam(nodehandle_, "/ground_mapping/filter_height_limit", HMIN);

        max_update_distance_ = getParam(nodehandle_, "/ground_mapping/max_update_distance", -1.0);
		merge_with_past_frame_ = getParam(nodehandle_, "/ground_mapping/merge_with_past_frame", false);
		accumulate_height_ = getParam(nodehandle_, "/ground_mapping/accumulate_height", true);
		map_loader_mode_ = getParam(nodehandle_, "/ground_mapping/map_loader_mode", false);
		save_height_map_ = getParam(nodehandle_, "/ground_mapping/save_height_map", false);
		height_map_filename_prefix_ = getParam(nodehandle_, "/ground_mapping/height_map_filename_prefix", std::string("/tmp/points_ground"));
		save_every_scan_ = getParam(nodehandle_, "/ground_mapping/save_every_scan", false);

		//make sure it is not running in save map and map loading at the same time
		//if so, deactivates map loading
		if (map_loader_mode_ && save_height_map_) {
			map_loader_mode_ = false;
		}

		std::cout << "-----------------------------------------------------------------" << std::endl;
		std::cout << "Running with parameters:" << std::endl;
		std::cout << "min_height: " << min_height_ << std::endl;
		std::cout << "max_height: " << max_height_ << std::endl;
		std::cout << "input_pointcloud_topic: " << input_pointcloud_topic_ << std::endl;
		std::cout << "output_pointcloud_topic: " << output_pointcloud_topic_ << std::endl;

        std::cout << "obstacle_pointcloud_topic: " << obstacle_pointcloud_topic_ << std::endl;
		std::cout << "ground_pointcloud_topic: " << ground_pointcloud_topic_ << std::endl;
		std::cout << "lidar_pose: ";
		for (auto p: lidar_pose_) {
		  std::cout << p << ' ';
		}
		std::cout << std::endl;
		std::cout << "source_tf: " << source_tf_ << std::endl;
		std::cout << "target_tf: " << target_tf_ << std::endl;

        std::cout << "generate_obstacle_data: " << generate_obstacle_data_ << std::endl;
		std::cout << "generate_ground_data: " << generate_ground_data_ << std::endl;
		std::cout << "obstacle_height_threshold: " << obstacle_height_threshold_ << std::endl;

        std::cout << "filter_input_points: " << filter_input_points_ << std::endl;
		std::cout << "filter_input_rings: " << filter_input_rings_ << std::endl;
		std::cout << "filter_by_height: " << filter_by_height_ << std::endl;
		std::cout << "filter_height_limit: " << filter_height_limit_ << std::endl;

        std::cout << "max_update_distance: " << max_update_distance_ << std::endl;
		std::cout << "merge_with_past_frame: " << merge_with_past_frame_ << std::endl;
		std::cout << "accumulate_height: " << accumulate_height_ << std::endl;
		std::cout << "map_loader_mode: " << map_loader_mode_ << std::endl;
		std::cout << "save_height_map: " << save_height_map_ << std::endl;
		std::cout << "height_map_filename_prefix: " << height_map_filename_prefix_ << std::endl;
		std::cout << "save_every_scan: " << save_every_scan_ << std::endl;
		std::cout << "-----------------------------------------------------------------" << std::endl;
	}

	void Mapping::obsSegmentation()
	{
		pcl_obstacles_.clear();
		pcl_ground_.clear();
		auto start_obs = std::chrono::high_resolution_clock::now();

		double offset_factor = (max_height_offset_-min_height_offset_);
		//double offset_factor_pos = (max_height_offset_+min_height_offset_);
		//double offset_factor_neg = (max_height_offset_-min_height_offset_);
		//double glortho_transpose_z = -offset_factor_pos/offset_factor_neg;
		//double glortho_inv_factor_z = -offset_factor_neg/2.0;
		for (auto p : pcl_out_.points) {
			int u = (p.x-xo_)*(WIDTH/XDIFF)+WINDOW_SIZE/2;
			int v = (p.y-yo_)*(HEIGHT/YDIFF)+WINDOW_SIZE/2;
			if(u<0||u>=WINDOW_SIZE)continue;
			if(v<0||v>=WINDOW_SIZE)continue;
			double depth = depthmap_[current_dm_].buffer2[v*WINDOW_SIZE+u];
			if (depth > -1.0 && depth < 1.0) {
				double height= depth*offset_factor + min_height_offset_ + -depthmap_[current_dm_].center_z;
				height += obstacle_height_threshold_;
				double distance=p.x*p.x+p.y*p.y+p.z*p.z;
				if(distance<3*3)continue;
				//printf("point(%f,%f,%f)->(%d,%d), depth:%f, height:%f\n", p.x, p.y, p.z, u, v, depth, height);
				if (p.z > height) {
					pcl_obstacles_.points.push_back(p);
				} else {
					pcl_ground_.points.push_back(p);
				}
			}
		}

		auto elapsed_obs = std::chrono::high_resolution_clock::now() - start_obs;
		long long microseconds_obs = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_obs).count();
		std::cout << "(obstacle/ground) computed in " <<(double)microseconds_obs/1000000.0 << " seconds" <<  std::endl;

		publishing_obst_cloud_ = false;
	}

	bool Mapping::checkSwitchDM(int &xc, int &yc, int &x1, int &y1)
	{
		bool dm_changed = false;
		bool over_framebuf = ((x1 < 0) || (y1 < 0) || (x1>=WIDTH-WINDOW_SIZE) || (y1>=HEIGHT-WINDOW_SIZE));

		if (redisplay_ && ((max_update_distance_ > 0.0 && update_distance_ > max_update_distance_) ||
				           (max_update_distance_ <= 0.0 && over_framebuf) || (!accumulate_height_)) ) {
			prev_dm_ = current_dm_;
			current_dm_++;
			if(current_dm_ >= dm_num_)current_dm_=0;

//			if (map_loader_mode_) {
//				loadImage(xo_, yo_, zo_, false);
//			} else {
				depthmap_[current_dm_].center_x=xo_;
				depthmap_[current_dm_].center_y=yo_;
				depthmap_[current_dm_].center_z=-zo_;
//			}

			dm_changed=true;
			std::cout << "*******************************************************" << std::endl;
			std::cout << "current_dm_= " << current_dm_ << std::endl;
			update_distance_ = 0.0;
			switch_dm_ = true;

			xc = (xo_-depthmap_[current_dm_].center_x)*(WIDTH/XDIFF)+WIDTH/2;
			yc = (yo_-depthmap_[current_dm_].center_y)*(HEIGHT/YDIFF)+HEIGHT/2;
			x1=xc-WINDOW_SIZE/2;
			y1=yc-WINDOW_SIZE/2;
		}

		return dm_changed;
	}

	void Mapping::filterPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &inCloud,
            					   pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& outCloud,
								   bool filter_range, int filter_rings, bool filter_height) {
        enum {
            M_TO_MM = 1000,
        };
        std::vector<std::vector<std::pair<double, unsigned int> > > range_matrix;
        //double range = 0.0;
        void
        (*median3)(unsigned int *, unsigned int, unsigned int) = &medianfilter_3_fast;//to avoid unused function warning
        void
        (*median5)(unsigned int *, unsigned int, unsigned int) = &medianfilter_5_fast;//to avoid unused function warning
        (void) median3;
        (void) median5;

        //copy input cloud
        outCloud = *inCloud;
        //if filtering requested
        if (filter_range) {
            //sort data by ring number
            std::sort(outCloud.points.begin(), outCloud.points.end(), RingComparator());
        }
        if (filter_rings > 0) {
            for (auto &p: outCloud.points) {
                if (p.ring <= (filter_rings - 1)) {
                    p.x = 0;
                    p.y = 0;
                    p.z = 0;
                }
            }
        }
        if (filter_height) {
            for (auto &p: outCloud.points) {
                if (p.z < filter_height_limit_) {
                    p.x = 0;
                    p.y = 0;
                    p.z = 0;
                }
            }
        }

        if (filter_range) {
            const double HEIGHT_ERROR = 1.0;
            for (unsigned int i = 1; i < outCloud.points.size() - 1; i++) {
                auto p = outCloud.points[i];
                auto q = outCloud.points[i - 1];
                auto r = outCloud.points[i + 1];
                if (p.ring == q.ring && p.ring == r.ring) {
                    if (p.z <= -1.0 * HEIGHT_ERROR &&
                        (fabs(p.z - q.z) > HEIGHT_ERROR && fabs(p.z - r.z) > HEIGHT_ERROR)) {
                        p.z = (q.z + r.z) / 2.0;
                    }
                }
            }
        }
    }

    void Mapping::velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg)
	{
		global_start_time_ = std::chrono::high_resolution_clock::now();
		auto start = global_start_time_;
		static ros::Time prev_time;
		double delta_t = 0.0;
		static double old_dist = 0.0;
		double dist = 0.0;
		int count = 0;
		const int MAX_WAIT = 11;
		const double EPSILON_Z = 0.001;

		if (stop_program_) {
			return;
		}

		pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pt_aux;
	    //filter with median filter over range data per ring
		filterPointCloud(msg, pt_aux, filter_input_points_, filter_input_rings_, filter_by_height_);
		pcl::copyPointCloud(pt_aux, prev_points_);

		std_msgs::Header header;
		pcl_conversions::fromPCL(msg->header,header);
		delta_t = std::fabs(header.stamp.toSec() - prev_time.toSec());
		DEBUG_MSG(std::cout << "header: " << std::setprecision(8) << header.stamp.toSec() << ", ROS time: " << ros::Time::now().toSec() << ", time diff.: " << delta_t << std::endl);

		tf::StampedTransform transform;
		tf::StampedTransform lidar_transform;
		try
		{
			ros::Time now = ros::Time(0);
			tf_listener_->waitForTransform(source_tf_, target_tf_, now, ros::Duration(10.0));
			tf_listener_->lookupTransform(source_tf_, target_tf_, now, transform);
		}
		catch (tf::TransformException& ex)
		{
			ROS_WARN("%s", ex.what());
		}

		if(prev_points_.points.size()>0){
//			waiting_for_tf_ = true;
//			while (waiting_for_tf_ && count < MAX_WAIT) {
//				try {
//					tf_listener_->waitForTransform(source_tf_,target_tf_, header.stamp/*prev_time*/, ros::Duration(0.1));
//					tf_listener_->lookupTransform(source_tf_,target_tf_, header.stamp/*prev_time*/, transform);
//					waiting_for_tf_ = false;
//				} catch (tf::TransformException &ex) {
//					ROS_WARN("%s",ex.what());
//					waiting_for_tf_ = true;
//					count++;
//					continue;
//				}
//			}
//			if (waiting_for_tf_) {
//				return;
//			}
			lidar_transform.setOrigin(tf::Vector3(lidar_pose_[0], lidar_pose_[1], lidar_pose_[2]));

			prev_xo_ = xo_;
			prev_yo_ = yo_;
			prev_zo_ = zo_;
			xo_=transform.getOrigin().x();
			yo_=transform.getOrigin().y();
			zo_=transform.getOrigin().z();
			transform.getBasis().getRPY(roll_, pitch_, yaw_);

			delta_z_ = zo_ - prev_zo_;
			if (fabs(delta_z_) < EPSILON_Z) {
				delta_z_ = 0.0;
			}
			double diff = delta_z_ - avg_delta_z_;
			if (delta_z_count_ < DELTA_Z_SAMPLES) {
				delta_z_count_++;
			}
			avg_delta_z_ = avg_delta_z_ + diff/(double)delta_z_count_;

//			min_height_offset_ = (int)(-zo_ - lidar_transform.getOrigin().z() + min_height_ + 0.5); //rounded
//			max_height_offset_ = (int)(-zo_ - lidar_transform.getOrigin().z() + max_height_ + 0.5); //rounded
//			min_height_offset_ = (min_height_offset_ <= 0.0) ? min_height_offset_ : min_height_; //for GlOrtho
//			max_height_offset_ = (max_height_offset_ >= 0.0) ? max_height_offset_ : max_height_; //for GlOrtho
			dist = sqrt((xo_-depthmap_[current_dm_].center_x)*(xo_-depthmap_[current_dm_].center_x) +
						(yo_-depthmap_[current_dm_].center_y)*(yo_-depthmap_[current_dm_].center_y) +
						(-zo_-depthmap_[current_dm_].center_z)*(-zo_-depthmap_[current_dm_].center_z));
			double dd = sqrt((xo_ - prev_xo_)*(xo_ - prev_xo_) + (yo_ - prev_yo_)*(yo_ - prev_yo_) + (zo_ - prev_zo_)*(zo_ - prev_zo_));
			double delta_d = std::fabs(dd - old_dist);
			old_dist = dd;
			if (!first_display_ && delta_t >= 0.1) {
				update_distance_ = dist;
				speed_ = delta_d / delta_t;
			}

			//and apply the transform
			{
				std::lock_guard<std::mutex> guard(ptcloud_access_);
				pcl_out_.clear();
				pcl_ros::transformPointCloud(prev_points_, pcl_out_, transform);

	//			pcl_out_.header=prev_points_.header;
	//			pcl_out_.header.frame_id=source_tf_;
				pcl_out_.header=msg->header;
				pcl_out_.header.frame_id=source_tf_;
			}
		}
		//pcl::copyPointCloud(pt_aux, prev_points_);
		prev_time =header.stamp;

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "(velodyneCallback) computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;
		redisplay_++;
		DEBUG_MSG(std::cout << "redisplay count " << redisplay_ << std::endl);
	}
	// %EndTag(CALLBACK)%

	template<class T>
	std::string Mapping::timeToStr(T ros_t)
	{
	    (void)ros_t;
	    std::stringstream msg;
	    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S.%f");
	    msg.imbue(std::locale(msg.getloc(),f));
	    msg << now;
	    return msg.str();
	}

	template <typename T>
	T Mapping::getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
	{
	  T value;
	  if (nh.hasParam(param_name))
	  {
		nh.getParam(param_name, value);
	  }
	  else		setInstance();
		hidden_win_id_ = -1;
#if (USE_OCTREE_SEARCH)
		octree_.setEpsilon(0.001);
#endif
	  {
		ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
		value = default_value;
	  }
	  return value;
	}
}