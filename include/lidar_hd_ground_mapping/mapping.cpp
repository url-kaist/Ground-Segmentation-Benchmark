// %Tag(FULLTEXT)%
/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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

//OpenGL
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <GL/glext.h>

//BOOST
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>

//OMP
//#include <omp.h>

/*
void signalHandler(int signum)
{
   if (signum == SIGINT) {
	   printf("End program by kbd interrupt!\n");
	   ground_mapping::Mapping::instance->stop_program_ = 1;
   }
}
*/

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

void GLAPIENTRY
MessageCallback( GLenum source,
                 GLenum type,
                 GLuint id,
                 GLenum severity,
                 GLsizei length,
                 const GLchar* message,
                 const void* userParam )
{
  fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
           ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            type, severity, message );
}

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
	const double Mapping::GRID_LENGTH_AUTO = -1.0; //-1 means auto compute
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

	void Mapping::init(int argc, char **argv)
	{
		nodehandle_ = ros::NodeHandle("~");

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
		pbo_index_ = 0;
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
		if (generate_grid_data_) {
			grid_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(grid_pointcloud_topic_, 10);
		}
		if (generate_obstacle_data_) {
			obstacle_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(obstacle_pointcloud_topic_, 10);
		}
		if (generate_ground_data_) {
			ground_pointcloud_pub_ = nodehandle_.advertise< pcl::PointCloud<pcl::PointXYZI > >(ground_pointcloud_topic_, 10);
		}

		//creates the grid
		if (generate_grid_data_) {
			if (grid_width_ == GRID_LENGTH_AUTO || grid_height_ == GRID_LENGTH_AUTO) {
				grid_width_ = ((double)WINDOW_SIZE-XDIFF)*(XDIFF/(double)WIDTH)*2;
				grid_height_ = ((double)WINDOW_SIZE-YDIFF)*(YDIFF/(double)HEIGHT)*2;
				grid_.resize(grid_width_, grid_height_, grid_cell_size_);
				std::cout << "grid cells: " << grid_.grid().size() << std::endl;
			}
		}

		//creates the point array for parallel processing
		point_array_ = new pcl::PointXYZI[WINDOW_SIZE*WINDOW_SIZE];
		if (!point_array_) {
			ROS_ERROR_STREAM("Failed allocating " << WINDOW_SIZE*WINDOW_SIZE*sizeof(pcl::PointXYZI) << " bytes for point_array");
			exit(1);
		}

		//buffer_ = (GLfloat *)malloc(WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat));
		buffer_ = (GLfloat *)malloc(WIDTH*HEIGHT*sizeof(GLfloat));
		if (!buffer_) {
			ROS_ERROR_STREAM("Error allocating " << WIDTH*HEIGHT*sizeof(GLfloat) << " bytes");
			exit(1);
		}

//		#pragma omp parallel
//		{
//			#pragma omp single
//			{
//				max_threads_ = omp_get_num_threads();
//				pcl_ground_aux_array_ = new pcl::PointCloud<pcl::PointXYZI>[max_threads_];
//			}
//		}

		pcl_ground_ptr_ = pcl::PointCloud<pcl::PointXYZI>::ConstPtr(&pcl_depth_ground_, &DoNotFree<pcl::PointCloud<pcl::PointXYZI>>);
		glutFunctions(argc, argv);

		//spin the ros-related thread
		//ros_thread_ = boost::thread(&Mapping::rosLoop, this);
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
		nodehandle_.deleteParam("/ground_mapping/grid_cell_size");
		nodehandle_.deleteParam("/ground_mapping/grid_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/obstacle_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/ground_pointcloud_topic");
		nodehandle_.deleteParam("/ground_mapping/lidar_pose");
		nodehandle_.deleteParam("/ground_mapping/grid_width");
		nodehandle_.deleteParam("/ground_mapping/grid_height");
		nodehandle_.deleteParam("/ground_mapping/generate_grid_data");
		nodehandle_.deleteParam("/ground_mapping/generate_obstacle_data");
		nodehandle_.deleteParam("/ground_mapping/generate_ground_data");
		nodehandle_.deleteParam("/ground_mapping/obstacle_height_threshold");
		nodehandle_.deleteParam("/ground_mapping/filter_grid_points");
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

		//close window if shown
		if (hidden_win_id_ > -1) {
			glutDestroyWindow(hidden_win_id_);
		}

		//free buffers
		glDeleteBuffersARB(PBO_COUNT, readPBOIDs_);
		glDeleteBuffersARB(1, &writePBOIDs_);
		glDeleteBuffersARB(1, &displayPBOIDs_);
		for (unsigned int i = 0; i < MAX_DEPTH_BUFFERS; i++) {
			if (depthmap_[i].buffer2) {
				free(depthmap_[i].buffer2);
			}
			glDeleteFramebuffers(1, &depthmap_[i].fb);
			depthmap_[i].fb = 0;
			glDeleteRenderbuffers(1, &depthmap_[i].rb);
			depthmap_[i].rb = 0;
			glDeleteTextures(1, &depthmap_[i].cb);
			depthmap_[i].cb = 0;
		}
		if (buffer_) {
			free(buffer_);
		}
		if (point_array_) {
			delete[] point_array_;
		}

		//stop GLUT loop
		glutLeaveMainLoop();
	}

	void Mapping::loadAllParameters()
	{
		min_height_ = getParam(nodehandle_, "/ground_mapping/min_height", HMIN);
		max_height_ = getParam(nodehandle_, "/ground_mapping/max_height", HMAX);
		input_pointcloud_topic_  = getParam(nodehandle_, "/ground_mapping/input_pointcloud_topic", std::string("velodyne_points"));
		output_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/output_pointcloud_topic", std::string("points_raw_ground"));
		grid_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/grid_pointcloud_topic", std::string("points_ground_grid"));
		obstacle_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/obstacle_pointcloud_topic", std::string("points_obstacles"));
		ground_pointcloud_topic_ = getParam(nodehandle_, "/ground_mapping/ground_pointcloud_topic", std::string("points_ground"));
		if (!(nodehandle_.getParam("/ground_mapping/lidar_pose", lidar_pose_))) {
			lidar_pose_ = std::vector<double>(6,0.0);
		}
		source_tf_  = getParam(nodehandle_, "/ground_mapping/source_tf", std::string("world"));
		target_tf_ = getParam(nodehandle_, "/ground_mapping/target_tf", std::string("velodyne"));
		grid_cell_size_ = getParam(nodehandle_, "/ground_mapping/grid_cell_size", CELL_SIZE);
		grid_width_ = getParam(nodehandle_, "/ground_mapping/grid_width", GRID_LENGTH_AUTO);
		grid_height_ = getParam(nodehandle_, "/ground_mapping/grid_height", GRID_LENGTH_AUTO);
		generate_grid_data_ = getParam(nodehandle_, "/ground_mapping/generate_grid_data", false);
		generate_obstacle_data_ = getParam(nodehandle_, "/ground_mapping/generate_obstacle_data", false);
		generate_ground_data_ = getParam(nodehandle_, "/ground_mapping/generate_ground_data", false);
		obstacle_height_threshold_ = getParam(nodehandle_, "/ground_mapping/obstacle_height_threshold", 0.1);
		filter_grid_points_ = getParam(nodehandle_, "/ground_mapping/filter_grid_points", false);
		filter_input_points_ = getParam(nodehandle_, "/ground_mapping/filter_input_points", false);
		filter_input_rings_ = getParam(nodehandle_, "/ground_mapping/filter_input_rings", -1);
		filter_by_height_ = getParam(nodehandle_, "/ground_mapping/filter_by_height", false);
		filter_height_limit_ = getParam(nodehandle_, "/ground_mapping/filter_height_limit", HMIN);
		hide_opengl_window_ = getParam(nodehandle_, "/ground_mapping/hide_opengl_window", true);
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
		std::cout << "grid_pointcloud_topic: " << grid_pointcloud_topic_ << std::endl;
		std::cout << "obstacle_pointcloud_topic: " << obstacle_pointcloud_topic_ << std::endl;
		std::cout << "ground_pointcloud_topic: " << ground_pointcloud_topic_ << std::endl;
		std::cout << "lidar_pose: ";
		for (auto p: lidar_pose_) {
		  std::cout << p << ' ';
		}
		std::cout << std::endl;
		std::cout << "source_tf: " << source_tf_ << std::endl;
		std::cout << "target_tf: " << target_tf_ << std::endl;
		std::cout << "grid_cell_size: " << grid_cell_size_ << std::endl;
		std::cout << "grid_width: " << grid_width_ << std::endl;
		std::cout << "grid_height: " << grid_height_ << std::endl;
		std::cout << "generate_grid_data: " << generate_grid_data_ << std::endl;
		std::cout << "generate_obstacle_data: " << generate_obstacle_data_ << std::endl;
		std::cout << "generate_ground_data: " << generate_ground_data_ << std::endl;
		std::cout << "obstacle_height_threshold: " << obstacle_height_threshold_ << std::endl;
		std::cout << "filter_grid_points: " << filter_grid_points_ << std::endl;
		std::cout << "filter_input_points: " << filter_input_points_ << std::endl;
		std::cout << "filter_input_rings: " << filter_input_rings_ << std::endl;
		std::cout << "filter_by_height: " << filter_by_height_ << std::endl;
		std::cout << "filter_height_limit: " << filter_height_limit_ << std::endl;
		std::cout << "hide_opengl_window: " << hide_opengl_window_ << std::endl;
		std::cout << "max_update_distance: " << max_update_distance_ << std::endl;
		std::cout << "merge_with_past_frame: " << merge_with_past_frame_ << std::endl;
		std::cout << "accumulate_height: " << accumulate_height_ << std::endl;
		std::cout << "map_loader_mode: " << map_loader_mode_ << std::endl;
		std::cout << "save_height_map: " << save_height_map_ << std::endl;
		std::cout << "height_map_filename_prefix: " << height_map_filename_prefix_ << std::endl;
		std::cout << "save_every_scan: " << save_every_scan_ << std::endl;
		std::cout << "-----------------------------------------------------------------" << std::endl;
	}

	void Mapping::initDepthBuffers()
	{
		//Check for GL extensions support
		if (!(isGLExtensionSupported("GL_ARB_framebuffer_object"))) {
			ROS_ERROR("'GL_ARB_framebuffer_object' extension is not supported");
			exit(1);
		}
		ROS_INFO("'GL_ARB_framebuffer_object' extension is supported");

		if (!(isGLExtensionSupported("GL_ARB_pixel_buffer_object"))) {
			ROS_ERROR("'GL_ARB_pixel_buffer_object' extension is not supported");
			exit(1);
		}
		ROS_INFO("'GL_ARB_pixel_buffer_object' extension is supported");

		dm_num_=MAX_DEPTH_BUFFERS;
		for (unsigned int i = 0; i < MAX_DEPTH_BUFFERS; i++) {
			init_fb(i);
		}
		current_dm_=1;
		prev_dm_=0;

		// create pixel buffer objects (PBO)
		// glBufferDataARB with NULL pointer reserves only memory space.
		//read(pack) PBOs
		glGenBuffersARB(PBO_COUNT, readPBOIDs_);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, readPBOIDs_[0]);
		glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, WIDTH*HEIGHT*sizeof(GLfloat), 0, GL_STREAM_READ_ARB);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, readPBOIDs_[1]);
		glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, WIDTH*HEIGHT*sizeof(GLfloat), 0, GL_STREAM_READ_ARB);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
		//write(unpack) PBOs
		glGenBuffersARB(1, &writePBOIDs_);
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, writePBOIDs_);
		glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, WIDTH*HEIGHT*sizeof(GLfloat), 0, GL_STREAM_DRAW_ARB);
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
		//write(unpack) PBOs for display
		glGenBuffersARB(1, &displayPBOIDs_);
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, displayPBOIDs_);
		glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, WIDTH*WIDTH*sizeof(GLfloat), 0, GL_STREAM_DRAW_ARB);
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
	}

	void Mapping::init_fb(unsigned int id)
	{
		//Generate the Texture(color) buffer
		glGenTextures(1, &(depthmap_[id].cb));
		glBindTexture(GL_TEXTURE_2D, depthmap_[id].cb);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA, GL_FLOAT, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT , WIDTH, HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
		glBindTexture(GL_TEXTURE_2D, 0);//reset

		//Generate the Frame buffer
		glGenFramebuffersEXT(1, &(depthmap_[id].fb));
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, depthmap_[id].fb);

		//Generate the Render buffer
		glGenRenderbuffers(1, &(depthmap_[id].rb));
		glBindRenderbuffer(GL_RENDERBUFFER, depthmap_[id].rb);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, WIDTH, HEIGHT);
		glBindRenderbufferEXT(GL_RENDERBUFFER, 0);//reset

		//Attach the render buffer
		//glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER, depthmap_[id].rb);

		//Attach the texture buffer
		//glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, depthmap_[id].cb,0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, depthmap_[id].cb, 0);

		//Reset, return to normal pixel processing
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

        bool status = checkFramebufferStatus(depthmap_[id].fb);
		if (!status) {
			ROS_ERROR("fbo status failed");
			exit(1);
		}

		depthmap_[id].buffer2 = (GLfloat *)malloc(WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat));
		if (!depthmap_[id].buffer2) {
			ROS_ERROR_STREAM("Error allocating " << WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat) << " bytes");
			exit(1);
		}
		depthmap_[id].x1 = 0;
		depthmap_[id].y1 = 0;
		depthmap_[id].x2 = WINDOW_SIZE;
		depthmap_[id].y2 = WINDOW_SIZE;
	}


	/*リサイズ*/////////////////////////////////////////////////////////////////////
	void Mapping::resize(int w, int h)
	{
		//not implemented
	}


	/*キーボード入力*//////////////////////////////////////////////////////////////////////////
	void Mapping::keyboard(unsigned char key, int x, int y)
	{
		if(key=='h') {
			hide_opengl_window_ = !hide_opengl_window_;
		} else if (key=='q' || key==27) {
			stop_program_ = 1;
		}
		//  glutPostRedisplay();
	}

	/*矢印キー入力*//////////////////////////////////////////////////////////////////////////
	void Mapping::special_key(int key, int x, int y)
	{
		glutPostRedisplay();
	}

	//Mouse wheel operation
	void Mapping::mouse_wheel(int wheel, int direction, int x, int y)
	{
		wheel=0;
		if (direction==-1) {
			zoom_ -= 0.5;
		}
		else if (direction==+1) {
			zoom_ += 0.5;
		}

		glutPostRedisplay();
	}

	void Mapping::mouse(int button, int state, int x, int y)
	{
	   // Wheel reports as button 3(scroll up) and button 4(scroll down)
	   if ((button == 3) || (button == 4)) {// It's a wheel event
		   // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		   if (state == GLUT_UP) {
			   zoom_ -= 0.5;
		   } else {
			   zoom_ += 0.5;
		   }
	   } else{  // normal button event
		   ;
	   }
	}

	void Mapping::loadDepthImage(const std::string filename)
	{
		//try reading depth image file
		std::string target_filename = filename;
		FILE *fp;
		fp=fopen(target_filename.c_str(),"r");
		if (!fp) {
			ROS_ERROR_STREAM("Failed opening \"" << target_filename.c_str() << "\" file for reading");
			return;
		}
		int width = 0;
		int height = 0;
		int nelem = fscanf(fp,"PF\n%d %d\n1.0\n", &width, &height);
		if (nelem < 2 || width != WIDTH || height != HEIGHT) {
			ROS_ERROR_STREAM("Image dimensions (" << width << "," << height << ") are invalid. Expecting (" << WIDTH << "," << HEIGHT << ")");
			fclose(fp);
			return;
		}
		size_t count = 0;
		count = fread(buffer_, sizeof(GLfloat), WIDTH*HEIGHT, fp);
		fclose(fp);

		if (count < WIDTH*HEIGHT) {
			ROS_ERROR_STREAM("Failed reading image data. Read " << (int)count << " elements, expected " << WIDTH*HEIGHT);
			return;
		}
	}

	void Mapping::saveDepthImage(GLfloat *buffer_src, const std::string filename, bool asPFM)
	{
		FILE *fp;

		if (!buffer_src) {
			return;
		}

		fp=fopen(filename.c_str(),"w");

		if (asPFM) {
			fprintf(fp,"PF\n%d %d\n1.0\n", WIDTH, HEIGHT);
			fwrite(buffer_src, sizeof(GLfloat), WIDTH*HEIGHT, fp);
		} else {
			GLfloat min_val = -1.0; //1.0e8;
			GLfloat max_val = 1.0; //-1.0e8;
//			for (int i = 0; i < WIDTH*HEIGHT; i++) {
//				if (buffer_src[i] > max_val) {
//					max_val = buffer_src[i];
//				}
//				if (buffer_src[i] < min_val) {
//					min_val = buffer_src[i];
//				}
//			}
			//save the PGM file (16 bits)
			const int max_scale = 65535;
			const float epsilon = 1.0e-8;
			float value = 0.0;
			unsigned short int valInt = 0;
//			const float offset_factor = (max_height_offset_-min_height_offset_);
//			float center_z = 0.0;
//			if (!accumulate_height_) {
//				center_z = depthmap_[current_dm_].center_z;
//			} else {
//				center_z = depthmap_[prev_dm_].center_z;
//			}
			fprintf(fp,"P5\n%d %d\n%d\n", WIDTH, HEIGHT, max_scale);
			for (int i = 0; i < WIDTH*HEIGHT; i++) {
				value = buffer_src[i];
				if (buffer_src[i] <= -1.0 || buffer_src[i] >= 1.0) {
					value = 0.0; //buffer_src[i];
				} else {
					//value = (value*offset_factor) + min_height_offset_ + -center_z;  //this is to recover real height values
					value = max_scale - (((value - min_val))/(max_val - min_val))*max_scale;
				}
				valInt = (unsigned short int)value;
				valInt = (valInt & 0xFF)<<8 | ((valInt >> 8) & 0xFF);
				fwrite((char*)&valInt, 2, 1, fp);
			}

		}
		fclose(fp);
	}

	void Mapping::loadImage(double curr_x, double curr_y, double curr_z, bool reload)
	{
		if (!buffer_) {
			return;
		}
		auto start = std::chrono::high_resolution_clock::now();

		//if first load and/o reloading (read again only if necessary)
		if (reload || hd_map_info_.size() == 0) {
			hd_map_info_.clear();

			//load the map info file
			std::ifstream mapfile;
			MapInfo info;
			mapfile.open(std::string(height_map_filename_prefix_ + ".map").c_str(), std::ios::in);
			if (!mapfile.is_open()) {
				ROS_ERROR_STREAM("Failed opening file \"" << std::string(height_map_filename_prefix_ + ".map").c_str() << "\" for reading\n");
				return;
			}
			char delim;
			while (!mapfile.eof()) {
				mapfile >> info.x >> delim >> info.y >>  delim >> info.z >> delim >> info.yaw >> delim >> info.pitch >> delim >> info.roll >> delim;
				//mapfile >> info.x >> delim >> info.y >>  delim >> info.z >> delim;
				std::getline(mapfile, info.image_file);
				hd_map_info_.push_back(info);
			}
			mapfile.close();
		}

		//find closest map to current coordinates
		double min_dist = 1.0e8;
		int index = -1;
		double dist = 0.0;
		MapInfo info;
		for (unsigned int i = 0; i < hd_map_info_.size(); i++) {
			info = hd_map_info_[i];
			dist = sqrt((curr_x - info.x)*(curr_x - info.x) + (curr_y - info.y)*(curr_y - info.y) + (curr_z - info.z)*(curr_z - info.z));
			if (dist < min_dist) {
				min_dist = dist;
				index = i;
			}
		}
		//found nothing or map info file is empty
		if (index < 0) {
			ROS_ERROR("Failed locating closest map");
			return;
		}

		//try reading depth image file
		loadDepthImage(hd_map_info_[index].image_file);

		//Load pixels into depth buffer
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		glViewport(0, 0, WIDTH, HEIGHT);
		glLoadIdentity();

		int oldMatrixMode;
		glGetIntegerv(GL_MATRIX_MODE,&oldMatrixMode);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(XMIN,XMAX,YMIN,YMAX, min_height_offset_, max_height_offset_); //world coords

		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);
//		glClear(GL_DEPTH_BUFFER_BIT);
//		glEnable(GL_DEPTH_TEST);
//		glDepthFunc(GL_LESS);

		//draw depth
		//glDrawPixels(WIDTH,HEIGHT, GL_DEPTH_COMPONENT, GL_FLOAT, buffer_);
		//Using write(unpack) PBO
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, writePBOIDs_);
		glDrawPixels(WIDTH,HEIGHT, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, writePBOIDs_);
        // map the buffer object into client's memory
        // Note that glMapBufferARB() causes sync issue.
        // If GPU is working with this buffer, glMapBufferARB() will wait(stall)
        // for GPU to finish its job. To avoid waiting (stall), you can call
        // first glBufferDataARB() with NULL pointer before glMapBufferARB().
        // If you do that, the previous data in PBO will be discarded and
        // glMapBufferARB() returns a new allocated pointer immediately
        // even if GPU is still working with the previous data.
        glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, WIDTH*HEIGHT*sizeof(GLfloat), 0, GL_STREAM_DRAW_ARB);
        GLubyte* ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, GL_WRITE_ONLY_ARB);
        if(ptr)
        {
            // update data directly on the mapped buffer
            memcpy(ptr, buffer_, WIDTH*HEIGHT*sizeof(GLfloat));
            glUnmapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB); // release pointer to mapping buffer
        }

		glFlush();
		glMatrixMode(oldMatrixMode);
		glViewport(0,0,WINDOW_SIZE,WINDOW_SIZE);//viewport[0], viewport[1], viewport[2], viewport[3]);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

		//update current depth map center
		depthmap_[current_dm_].center_x=hd_map_info_[index].x;
		depthmap_[current_dm_].center_y=hd_map_info_[index].y;
		depthmap_[current_dm_].center_z=hd_map_info_[index].z;

		std::cout << "Loaded depth map \"" << hd_map_info_[index].image_file.c_str() << "\" center=(" << hd_map_info_[index].x << "," << hd_map_info_[index].y << "," << hd_map_info_[index].z << ")" << std::endl;

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "(loadImage) computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;

        // it is good idea to release PBOs with ID 0 after use.
        // Once bound with 0, all pixel operations behave normal ways.
        glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
	}

	void Mapping::saveImage()
	{
		GLfloat *buffer_src;

//		if (accumulate_height_) {
//			buffer_src = buffer_;
//		} else {
//			buffer_src = depthmap_[current_dm_].buffer2;
//		}
		buffer_src = buffer_;
		if (!buffer_src) {
			return;
		}

		saving_file_ = true;

		memcpy(buffer_, depthmap_[prev_dm_/*current_dm_*/].buffer2, WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat));

		auto start = std::chrono::high_resolution_clock::now();

//		//read the full depth buffer data
//		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[prev_dm_].fb);
//		glReadPixels(0,0,WIDTH,HEIGHT,GL_DEPTH_COMPONENT, GL_FLOAT, buffer_);
//		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);

		//generate the filename with date/time
		std::string target_filename = height_map_filename_prefix_;
		target_filename += std::string("_") + timeToStr(ros::WallTime::now());
		target_filename += std::string(".pfm");

		saveDepthImage(buffer_src, target_filename, true);

		//update the map info file
		std::ofstream mapfile;
		mapfile.open (std::string(height_map_filename_prefix_ + ".map").c_str(), std::ios::app);
		if (accumulate_height_) {
			mapfile << xo_ << "," << yo_ << "," << zo_ << "," << yaw_ << "," << pitch_ << "," << roll_ << "," << target_filename << std::endl;
		} else {
			mapfile << depthmap_[prev_dm_].center_x << "," << depthmap_[prev_dm_].center_y << "," << depthmap_[prev_dm_].center_z << "," << yaw_ << "," << pitch_ << "," << roll_ << "," << target_filename << std::endl;
		}
		mapfile.close();
//		FILE *fp;
//		std::string target_filename_date = timeToStr(ros::WallTime::now());
//		std::string filename = std::string("/tmp/depth_image_") + target_filename_date + std::string(".csv");
//		fp=fopen(filename.c_str(),"w");
//		for(int j=0;j<WINDOW_SIZE;j++){
//			for(int i=0;i<WINDOW_SIZE;i++){
//				if (depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i] > -1.0 &&
//					depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i] < 1.0) {
//					fprintf(fp,"%d,%d,%f\n",i,j,depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i]);
//				}
//			}
//			fprintf(fp,"\n");
//		}
//		fclose(fp);
//
//		std::ofstream logfile;
//		filename = std::string("/tmp/pcl_out_") + target_filename_date + std::string(".csv");
//		logfile.open(filename.c_str());
//		for (auto p : pcl_out_.points) {
//			logfile << p.x << "," << p.y << "," << p.z << std::endl;
//		}
//		logfile.close();
//
//		filename = std::string("/tmp/pcl_ground_aux") + target_filename_date + std::string(".csv");
//		logfile.open(filename.c_str());
//		for (auto p : pcl_ground_aux_.points) {
//			logfile << p.x << "," << p.y << "," << p.z << std::endl;
//		}
//		logfile.close();
//
//		filename = std::string("/tmp/info") + target_filename_date + std::string(".csv");
//		logfile.open(filename.c_str());
//		logfile << xo_ << "," << yo_ << "," << zo_ << "," << depthmap_[current_dm_].center_x << "," << depthmap_[current_dm_].center_y << "," << depthmap_[current_dm_].center_z << std::endl;
//		logfile.close();

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "(saveImage) computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;

		saving_file_ = false;
	}

	void Mapping::displayDepthData()
	{
		//initialize
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);//reset
		glEnable(GL_DEPTH_TEST);
//		if (avg_delta_z_ < DELTA_Z_THRESHOLD && delta_z_count_ >= DELTA_Z_SAMPLES) {
//			glDepthFunc(GL_GREATER);
//		} else {
//			glDepthFunc(GL_LESS);
//		}
		glDepthFunc(GL_LESS);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

		//draw depth
		int oldMatrixMode;
		glGetIntegerv(GL_MATRIX_MODE,&oldMatrixMode);
		glPushMatrix();
		////glDrawPixels(WINDOW_SIZE,WINDOW_SIZE, GL_LUMINANCE, GL_FLOAT, buffer_);
		//glDrawPixels(WINDOW_SIZE,WINDOW_SIZE, GL_LUMINANCE, GL_FLOAT, depthmap_[current_dm_].buffer2);
		//Using write(unpack) PBO for display
		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, displayPBOIDs_);
		glDrawPixels(WINDOW_SIZE,WINDOW_SIZE, GL_LUMINANCE, GL_FLOAT, 0);

		glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, displayPBOIDs_);
        // map the buffer object into client's memory
        // Note that glMapBufferARB() causes sync issue.
        // If GPU is working with this buffer, glMapBufferARB() will wait(stall)
        // for GPU to finish its job. To avoid waiting (stall), you can call
        // first glBufferDataARB() with NULL pointer before glMapBufferARB().
        // If you do that, the previous data in PBO will be discarded and
        // glMapBufferARB() returns a new allocated pointer immediately
        // even if GPU is still working with the previous data.
        glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat), 0, GL_STREAM_DRAW_ARB);
        GLubyte* ptr = (GLubyte*)glMapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, GL_WRITE_ONLY_ARB);
        if(ptr)
        {
            // update data directly on the mapped buffer
            memcpy(ptr, depthmap_[current_dm_].buffer2, WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat));
            glUnmapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB); // release pointer to mapping buffer
        }
		glPopMatrix();
		glMatrixMode(oldMatrixMode);
		glClear(GL_DEPTH_BUFFER_BIT);

		glViewport(0,0,WINDOW_SIZE,WINDOW_SIZE);
		glLoadIdentity();

		//mapping from world to window
		glOrtho(XMIN*((double)WINDOW_SIZE/WIDTH),XMAX*((double)WINDOW_SIZE/WIDTH),
				YMIN*((double)WINDOW_SIZE/HEIGHT),YMAX*((double)WINDOW_SIZE/HEIGHT),
				-1.0, 1.0);

		//draw the robot as a red point
		glPointSize(3);
		glBegin(GL_POINTS);
		glVertex2f(0,0);
		glEnd();
		glFlush();

        // it is good idea to release PBOs with ID 0 after use.
        // Once bound with 0, all pixel operations behave normal ways.
        glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

		/* 画面表示 */
		glutSwapBuffers();
	}

	void Mapping::genGroundPointCloud()
	{
		//clear the pointcloud and grid
		pcl_depth_ground_.clear();
		if (generate_grid_data_) {
			grid_.clear();
		}
		auto start_obs = std::chrono::high_resolution_clock::now();

		pcl::PointCloud<pcl::PointXYZI> pcl_ground_aux;
		pcl::PointXYZI wp;
		//add the data
		const double width_factor = (XDIFF/(double)WIDTH);
		const double height_factor = (YDIFF/(double)HEIGHT);
		double offset_factor = (max_height_offset_-min_height_offset_);
		//double offset_factor_pos = (max_height_offset_+min_height_offset_);
		//double offset_factor_neg = (max_height_offset_-min_height_offset_);
		//double glortho_transpose_z = -offset_factor_pos/offset_factor_neg;
		//double glortho_inv_factor_z = -offset_factor_neg/2.0;
		tf::Transform transform;
		transform.setIdentity();
		transform.setOrigin(tf::Vector3(xo_, yo_, 0.0));
		//transform.setOrigin(tf::Vector3(depthmap_[current_dm_].center_x, depthmap_[current_dm_].center_y, 0.0));
//			tf::Matrix3x3 mat = transform.getBasis();
//			double _roll = 0.0;
//			double _pitch = 0.0;
//			double _yaw = 0.0;
//			mat.getRPY(_roll, _pitch, _yaw);
//			mat.setRPY(_roll, pitch_, _yaw); //note we use other pitch value
//			tf::Quaternion q;
//			mat.getRotation(q);
//			transform.setRotation(q);

		const double w_factor = (WINDOW_SIZE>>1)*width_factor;
		const double h_factor = (WINDOW_SIZE>>1)*height_factor;
		double gw_factor = 0.0;
		double gh_factor = 0.0;
		if (generate_grid_data_) {
			gw_factor = grid_.width()/(2.0*grid_.cellSize());
			gh_factor = grid_.height()/(2.0*grid_.cellSize());
		}

#define PARALLEL 0
#if (!PARALLEL)
		for(int j=0;j<WINDOW_SIZE;j++){
			double v = (double)j*height_factor - h_factor;
			for(int i=0;i<WINDOW_SIZE;i++){
				double depth = depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i];
				if (depth > -1.0 && depth < 1.0) {
					double u = (double)i*width_factor - w_factor;
					double height= depth*offset_factor + min_height_offset_ + -depthmap_[current_dm_].center_z;
					wp.x = u;
					wp.y = v;
					wp.z = height;
					double distance=wp.x*wp.x+wp.y*wp.y+wp.z*wp.z;
					if(distance<3*3)continue;
					//adds a ground point
					pcl_ground_aux.push_back(wp);

					if (generate_grid_data_) {
						//and adds point to the grid
						grid_.addPoint((int)(u/grid_.cellSize() + gw_factor),
									   (int)(v/grid_.cellSize() + gh_factor),
									   wp);
					}
			  }
		  }
		}
		if (generate_grid_data_) {
			for(int j=0;j<WINDOW_SIZE;j++){
				double v = (double)j*height_factor - h_factor;
				for(int i=0;i<WINDOW_SIZE;i++){
					double depth = depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i];
					if (depth > -1.0 && depth < 1.0) {
						double u = (double)i*width_factor - w_factor;
						double height= depth*offset_factor + min_height_offset_ + -depthmap_[current_dm_].center_z;
						wp.x = u;
						wp.y = v;
						wp.z = height;
						double distance=wp.x*wp.x+wp.y*wp.y+wp.z*wp.z;
						if(distance<3*3)continue;
						//adds point to the grid
						grid_.addPoint((int)(u/grid_.cellSize() + gw_factor),
									   (int)(v/grid_.cellSize() + gh_factor),
									   wp);
				  }
			  }
			}
		}
#else //note: parallel version, but slower than serial
		if (!point_array_) {
			return;
		}
		pcl::PointXYZI *p = point_array_;
//		for (int k=0; k<WINDOW_SIZE*WINDOW_SIZE; k++) {
//			p->x = 0.;
//			p->y = 0.;
//			p->z = 0.;
//			p++;
//		}
//		printf("Parallel fill of points_array\n");
//		p = point_array_;
		#pragma omp parallel for
		for (int k=0; k<WINDOW_SIZE*WINDOW_SIZE; k++) {
			int i = k % WINDOW_SIZE;
			int j = k / WINDOW_SIZE;
			double depth = depthmap_[current_dm_].buffer2[j*WINDOW_SIZE+i];
			if (depth > -1.0 && depth < 1.0) {
				double u = (double)i*width_factor - w_factor;
				double v = (double)j*height_factor - h_factor;
				double height= depth*offset_factor + min_height_offset_ + -depthmap_[current_dm_].center_z;
				p = &(point_array_[k]);
				p->x = u;
				p->y = v;
				p->z = height;
			}
		}
		p = point_array_;

		#pragma omp single
		for (int k=0; k<WINDOW_SIZE*WINDOW_SIZE; k++) {
			double distance=p->x*p->x+p->y*p->y+p->z*p->z;
			if(distance>3*3) {
				//adds a ground point
				pcl_ground_aux.push_back(*p);

				if (generate_grid_data_) {
					//and adds point to the grid
					int i = k % WINDOW_SIZE;
					int j = k / WINDOW_SIZE;
					double u = (double)i*width_factor - w_factor;
					double v = (double)j*height_factor - h_factor;
					grid_.addPoint((int)(u/grid_.cellSize() + gw_factor),
								   (int)(v/grid_.cellSize() + gh_factor),
								   *p);
				}
			}
			p++;
		}
#endif
		//pcl_ground_aux_ = pcl_ground_aux;
		pcl_ros::transformPointCloud(pcl_ground_aux, pcl_depth_ground_, transform);
	    std::cout << "Output pointcloud with size " << pcl_depth_ground_.size() << std::endl;

	    if (generate_grid_data_) {
	    	//apply the transform to each grid cell
			grid_.transformPointCloud(transform);
			//update the grid
			grid_.update();
			pcl_grid_.clear();
			for (auto cell : grid_.grid()) {
//				#pragma omp parallel for schedule(auto)
//				for (std::vector< Cell >::iterator it = grid_.grid().begin();
//					it < grid_.grid().end(); ++it) {
				//Cell cell = *it;
				if (cell.isGround(filter_grid_points_)) {
					auto p = cell.centroid();
					wp.x = p.x;
					wp.y = p.y;
					wp.z = p.z;
					//wp.intensity = 0;
					pcl_grid_.push_back(wp);
				}
			}
			pcl_grid_.header=pcl_out_.header;
			pcl_grid_.header.frame_id=source_tf_;
			if (grid_pointcloud_pub_.getNumSubscribers() > 0) {// anyone listening?
				grid_pointcloud_pub_.publish(pcl_grid_);
			}
			std::cout << "Grid size " << grid_.size() << std::endl;
	    }

		auto elapsed_obs = std::chrono::high_resolution_clock::now() - start_obs;
		long long microseconds_obs = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_obs).count();
		std::cout << "(genGroundPtCloud) computed in " <<(double)microseconds_obs/1000000.0 << " seconds" <<  std::endl;
	}

//	void Mapping::obsSegmentation()
//	{
//#if (USE_GRID_SEARCH)
//		  	if (generate_grid_data_) {
//#endif
//		    if (generate_obstacle_data_ || generate_ground_data_) {
//		    	pcl_obstacles_.clear();
//		    	pcl_ground_.clear();
//		    	auto start_obs = std::chrono::high_resolution_clock::now();
//
//#if (USE_DEPTHBUFF_SEARCH)
//		    	for (auto p : pcl_out_.points) {
//		    		int u = (p.x-xo_)*(WIDTH/XDIFF)+WINDOW_SIZE/2;
//					int v = (p.y-yo_)*(HEIGHT/YDIFF)+WINDOW_SIZE/2;
//				    if(u<0||u>=WINDOW_SIZE)continue;
//				    if(v<0||v>=WINDOW_SIZE)continue;
//					double depth = depthmap_[current_dm_].buffer2[v*WINDOW_SIZE+u];
//					if (depth > -1.0 && depth < 1.0) {
//						double height= depth*offset_factor + min_height_offset_ + -depthmap_[current_dm_].center_z + obstacle_height_threshold_;
//						//printf("point(%f,%f,%f)->(%d,%d), depth:%f, height:%f\n", p.x, p.y, p.z, u, v, depth, height);
//						if (p.z > height) {
//							pcl_obstacles_.points.push_back(p);
//						} else {
//							pcl_ground_.points.push_back(p);
//						}
//					}
//		    	}
//#else
//#if (USE_KDTREE_SEARCH)
//		    	pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_ground_ptr(&pcl_depth_ground_, &DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
//#elif (USE_OCTREE_SEARCH)
//		    	pcl::PointCloud<pcl::PointXYZ> pcl_depth_ground_aux;
//		    	pcl::copyPointCloud(pcl_depth_ground_, pcl_depth_ground_aux);
//		    	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_ground_ptr(&pcl_depth_ground_aux, &DoNotFree< pcl::PointCloud<pcl::PointXYZ> >);
//#endif
//		    	//if (switch_dm_) {
//#if (USE_KDTREE_SEARCH)
//		    		kdtree_.setInputCloud(pcl_ground_ptr);
//#elif (USE_OCTREE_SEARCH)
//		    		octree_.setInputCloud(pcl_ground_ptr);
//		    		octree_.addPointsFromInputCloud();
//#endif
//		    	//}
//#if (USE_KDTREE_SEARCH || USE_OCTREE_SEARCH)
//		    	float radius = 0.3;
//		    	int min_neighbour_count = 5;
//		    	std::vector<int> pointIdxRadiusSearch;
//		    	std::vector<float> pointsSquaredDistRadius;
//		    	// radius search
//		    	int count = 0;
//#endif
//
//		    	for (auto p : pcl_out_.points) {
//#if (USE_KDTREE_SEARCH)
//		    		count = kdtree_.radiusSearch(p, radius, pointIdxRadiusSearch, pointsSquaredDistRadius);
//#elif (USE_OCTREE_SEARCH)
//		    		pcl::PointXYZ q(p.x, p.y, p.z);
//		    		count = octree_.radiusSearch(q, radius, pointIdxRadiusSearch, pointsSquaredDistRadius);
//#endif
//#if (USE_KDTREE_SEARCH || USE_OCTREE_SEARCH)
//		    		if (count < min_neighbour_count) {
//		    			pcl_obstacles_.points.push_back(p);
//		    		} else {
//		    			pcl_ground_.points.push_back(p);
//		    		}
//#endif
//#if (USE_GRID_SEARCH)
//		    		Eigen::Vector3f q;
//					q << (p.x-xo_), (p.y-yo_), p.z;
//		    		int u = (int)((q[0]+grid_.width()/2.0)/grid_.cellSize());
//					int v = (int)((q[1]+grid_.height()/2.0)/grid_.cellSize());
//					Eigen::Vector3f centroid;
//					//printf("point(%f,%f,%f)->(%d,%d)\n", q.x, q.y, q.z, u, v);
//					grid_.at(u,v).centroid(centroid);
//					Eigen::Vector3f normal;
//					grid_.at(u,v).normalVector(normal);
//					if (centroid.norm() == 0.0 || normal.norm() == 0) {
//						continue;
//					}
//					//we know centroid's x,y are on the plane, z may not be, solve z
//					float A, B, C, D;
//					grid_.at(u,v).planeParams(A,B,C,D);
//					centroid[2] = -(A*centroid[0] + B*centroid[1] + D)/C;
//					q = q - centroid;
//					float distance = normal.dot(q)/normal.norm();
////					printf("point(%f,%f,%f), centroid(%f,%f,%f), normal(%f,%f,%f), ABCD(%f,%f,%f,%f), dist %f\n",
////							q[0], q[1], q[2],
////							centroid[0], centroid[1], centroid[2],
////							normal[0], normal[1], normal[2],
////							A,B,C,D, distance);
//		    		if (distance <= 0.3) {
//		    			pcl_obstacles_.points.push_back(p);
//		    		} else {
//		    			pcl_ground_.points.push_back(p);
//		    		}
//#endif
//		    	}
//#endif
//		    }
//#if (USE_GRID_SEARCH)
//		  	}
//#endif
//	}

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

	int Mapping::showdepth(void)
	{
		if (depthmap_[current_dm_].buffer2 && !stop_program_) {
			auto start = std::chrono::high_resolution_clock::now();
			GLint view[4];

			pbo_index_ = (pbo_index_ + 1) % PBO_COUNT;
			unsigned int next_pbo_index = (pbo_index_ + 1) % PBO_COUNT;

		    // Spin up a thread for writing to the ground pointcloud into the height map
			// before creating the new ground pointcloud
			if (save_height_map_ && (switch_dm_ || stop_program_ || save_every_scan_) && !saving_file_) {
//					std::cout << "going to save image" << std::endl;
//					//read the full depth buffer data
//					glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[prev_dm_].fb);
//					glReadPixels(0,0,WIDTH,HEIGHT,GL_DEPTH_COMPONENT, GL_FLOAT, buffer_);
//					glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);
					record_thread_ = boost::thread(&Mapping::saveImage, this);
				}


			//
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);

			if (!hide_opengl_window_) {
				/* 現在のビューポートのサイズを得る */
				glGetIntegerv(GL_VIEWPORT, view);

				/* 画面表示の完了を待つ */
				glFinish();
			}

			/* デプスバッファの読み込み */
			//for display
			////glReadPixels(depthmap_[current_dm_].x1,depthmap_[current_dm_].y1,WINDOW_SIZE,WINDOW_SIZE, GL_DEPTH_COMPONENT, GL_FLOAT, depthmap_[current_dm_].buffer2);
	        // copy pixels from framebuffer to PBO
	        // Use offset instead of ponter.
	        // OpenGL should perform asynch DMA transfer, so glReadPixels() will return immediately.
	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, readPBOIDs_[pbo_index_]);
	        //glReadBuffer(GL_DEPTH_ATTACHMENT_EXT);
	        glReadPixels(depthmap_[current_dm_].x1,depthmap_[current_dm_].y1,WINDOW_SIZE, WINDOW_SIZE, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	        DEBUG_MSG( {
			auto end_time0 = std::chrono::high_resolution_clock::now();
			auto elapsed0 = end_time0 - start;
			long long microseconds0 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed0).count();
			std::cout << "(showdepth0) ellapsed time " <<(double)microseconds0/1000000.0 << " seconds" <<  std::endl;
			} )

	        // map the PBO that contain framebuffer pixels before processing it
	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, readPBOIDs_[next_pbo_index]);
	        GLubyte* src = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
	        if(src)
	        {
	            //copy mapped data from PBO
	            memcpy(depthmap_[current_dm_].buffer2, src, WINDOW_SIZE*WINDOW_SIZE*sizeof(GLfloat));
	            glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);     // release pointer to the mapped buffer
	        }

	        DEBUG_MSG( {
			auto end_time1 = std::chrono::high_resolution_clock::now();
			auto elapsed1 = end_time1 - start;
			long long microseconds1 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed1).count();
			std::cout << "(showdepth1) ellapsed time " <<(double)microseconds1/1000000.0 << " seconds" <<  std::endl;
	        })

			//perform obstacle and ground segmentation
			if (generate_obstacle_data_ || generate_ground_data_) {
				obsSegmentation();

				// Spin up a thread to publish the obstacles pointcloud
				if ((obstacle_pointcloud_pub_.getNumSubscribers() > 0 ||
					ground_pointcloud_pub_.getNumSubscribers() > 0) && !publishing_obst_cloud_) {
					publish_obst_thread_ = boost::thread(&Mapping::publishObstaclePointCloud, this);
				}
			}
	        DEBUG_MSG( {
			auto end_time2 = std::chrono::high_resolution_clock::now();
			auto elapsed2 = end_time2 - start;
			long long microseconds2 = std::chrono::duration_cast<std::chrono::microseconds>(elapsed2).count();
			std::cout << "(showdepth2) ellapsed time " <<(double)microseconds2/1000000.0 << " seconds" <<  std::endl;
	        })

			//Show the opengl window data if so requested
			if (!hide_opengl_window_) {
				displayDepthData();
			}

			//generate the ground pointcloud and grid
			if (depth_ground_pointcloud_pub_.getNumSubscribers() > 0 && !publishing_ground_cloud_) {
				// Spin up a thread to publish the ground pointcloud
				publish_ground_thread_ = boost::thread(&Mapping::publishGroundPointCloud, this);
			}

/*
//			// Spin up a thread for writing to the ground pointcloud into the height map
//			bool saving_file = false;
//			if (save_height_map_ && (switch_dm_ || stop_program_ || save_every_scan_)) {
//				record_thread_ = boost::thread(&Mapping::saveImage, this);
//			}
*/

			auto end_time = std::chrono::high_resolution_clock::now();
			auto elapsed = end_time - start;
			long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			std::cout << "(showdepth) computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;

			elapsed = end_time - global_start_time_;
			microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			double elapsed_seconds = (double)microseconds/1000000.0;
			loops_count_++;
			total_running_time_ += elapsed_seconds;
			std::cout << "--------- (program loop) computed in " << elapsed_seconds << " sec." << ", avg. " << (total_running_time_/loops_count_) << " sec. (" << loops_count_ << " loops)" << std::endl;

	        // it is good idea to release PBOs with ID 0 after use.
	        // Once bound with 0, all pixel operations behave normal ways.
			glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);

			return 1;
		}
		return 0;
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

			if (map_loader_mode_) {
				loadImage(xo_, yo_, zo_, false);
			} else {
				depthmap_[current_dm_].center_x=xo_;
				depthmap_[current_dm_].center_y=yo_;
				depthmap_[current_dm_].center_z=-zo_;
			}

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

	void Mapping::prepareViewport(bool dm_changed, int xc, int yc)
	{
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		glViewport(0, 0, WIDTH, HEIGHT);
		glLoadIdentity();

		//TODO:
		////consider rotate by Pitch
		////before glOrtho
		//glGetIntegerv(GL_MATRIX_MODE,&oldMatrixMode);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(XMIN,XMAX,YMIN,YMAX, min_height_offset_, max_height_offset_); //world coords

//		glMatrixMode(GL_MODELVIEW);
//		glPushMatrix();
//		glRotatef(pitch_*180.0/M_PI, 0.0, 1.0, 0.0);

		//glMatrixMode(GL_PROJECTION);
		//glLoadIdentity();
		//glOrtho(XMIN,XMAX,YMIN,YMAX, min_height_offset_, max_height_offset_); //world coords
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);

		//clear the current frame buffer
		if (dm_changed /*|| (!accumulate_height_)*/) {
			glClear(GL_DEPTH_BUFFER_BIT);
		}

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);

		if(dm_changed && merge_with_past_frame_) {
			int sx1 = depthmap_[prev_dm_].x1;
			int sy1 = depthmap_[prev_dm_].y1;
			int sx2 = depthmap_[prev_dm_].x2;
			int sy2 = depthmap_[prev_dm_].y2;
			int w = std::abs(sx2 - sx1);
			int h = std::abs(sy2 - sy1);
			int sxc = sx1 + w/2;
			int syc = sy1 + h/2;
			int dxc = xc - sxc;
			int dyc = yc - syc;
			int dx1 = xc + dxc - WINDOW_SIZE/2;
			int dy1 = xc + dyc - WINDOW_SIZE/2;
//			printf("copy overlap => \n"
//				   "%*c source-fb(%d) (%d,%d,%d,%d), target-fb(%d) (%d,%d,%d,%d)\n"
//				   "%*c source: (%d,%d,%d,%d), dest: (%d,%d,%d,%d)\n",
//				   8, ' ', prev_dm_, depthmap_[prev_dm_].x1,depthmap_[prev_dm_].y1,depthmap_[prev_dm_].x2,depthmap_[prev_dm_].y2,
//				   current_dm_, depthmap_[current_dm_].x1,depthmap_[current_dm_].y1,depthmap_[current_dm_].x2,depthmap_[current_dm_].y2,
//				   8, ' ', sx1,sy1,sx1+w,sy1+h, dx1,dy1,dx1+w,dy1+h);
			if (w > 0 && h > 0) { //any overlap? copy overlapping area
				glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT,depthmap_[prev_dm_].fb);
				glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT,depthmap_[current_dm_].fb);
				glBlitFramebufferEXT(sx1,sy1,sx1+w,sy1+h,
									 dx1,dy1,dx1+w,dy1+h,
									 GL_DEPTH_BUFFER_BIT,
									 GL_NEAREST);
			}
		}
	}

	void Mapping::raycastPointCloud()
	{
//		static double min_height_val = 1.0e8;
//		double scan_min_height = 1.0e8;

		//render to framebuffer
		glDrawBuffer(GL_BACK);

		glColor3d(1,0,0);

#if (USE_GL_VBO)
		//Using Vertex Buffer Objects
		std::vector<pcl::PointXYZ> vertices;
		std::vector<unsigned int> indices;
		vertices.push_back(pcl::PointXYZ(xo_-depthmap_[current_dm_].center_x,
										 yo_-depthmap_[current_dm_].center_y,
										 -zo_-depthmap_[current_dm_].center_z));
		unsigned int i = 1;
		{
			std::lock_guard<std::mutex> guard(ptcloud_access_);
			for (auto p : pcl_out_.points) {
				double hh = -p.z-depthmap_[current_dm_].center_z;
				vertices.push_back(pcl::PointXYZ(p.x-depthmap_[current_dm_].center_x,
												 p.y-depthmap_[current_dm_].center_y,
												 hh));
				indices.push_back(i);
				indices.push_back(0);
				i++;
	//			if (hh < min_height_val) {
	//				min_height_val = hh;
	//			}
	//			if (hh < scan_min_height) {
	//				scan_min_height = hh;
	//			}
			}
		}
		GLuint IBO;
		GLuint VBO;
		GLuint NO_VBO_ID = 0;
		glGenBuffersARB(1, &IBO);
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, IBO);
		glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB, indices.size()*sizeof (indices[0]), &(indices[0]), GL_STATIC_DRAW_ARB);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB, NO_VBO_ID);
		glGenBuffersARB(1, &VBO);
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, VBO);
		glBufferDataARB(GL_ARRAY_BUFFER_ARB, vertices.size()*sizeof (vertices[0]), &(vertices[0]), GL_STATIC_DRAW_ARB);
		glBindBuffer(GL_ARRAY_BUFFER, NO_VBO_ID);

		glEnableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
		glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZ), 0);

		glDrawElements(GL_LINES, static_cast<GLuint>(indices.size()), GL_UNSIGNED_INT, 0);

		glDisableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		glBindBufferARB(GL_ARRAY_BUFFER, NO_VBO_ID);
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, NO_VBO_ID);
		glDeleteBuffersARB(1, &IBO);
		glDeleteBuffersARB(1, &VBO);
		glFlush();
#else
		//Using slower line drawing
		glBegin(GL_LINES);
		for (auto p : pcl_out_.points) {
			glVertex3f(p.x-depthmap_[current_dm_].center_x,
					   p.y-depthmap_[current_dm_].center_y,
					  -p.z-depthmap_[current_dm_].center_z);
			glVertex3f(xo_-depthmap_[current_dm_].center_x,
					   yo_-depthmap_[current_dm_].center_y,
					   -zo_-depthmap_[current_dm_].center_z);
		}
		glEnd();
		glFlush();
#endif

//		//glPopMatrix();
		//glMatrixMode(oldMatrixMode);

		glViewport(0,0,WINDOW_SIZE,WINDOW_SIZE);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
	}

	void Mapping::display()
	{
		auto start = std::chrono::high_resolution_clock::now();
		int dm_changed=0;

		if (!redisplay_) {
			return;
		}

//		//no point cloud, do nothing
//		if (!pcl_out_.points.size()) {
//			return;
//		}

		if (first_display_) {
			first_display_ = 0;
			if (map_loader_mode_) {
				loadImage(xo_, yo_, zo_, true);
			} else {
				depthmap_[current_dm_].center_x=xo_;
				depthmap_[current_dm_].center_y=yo_;
				depthmap_[current_dm_].center_z=-zo_;
			}
		}

		int xc = (xo_-depthmap_[current_dm_].center_x)*(WIDTH/XDIFF)+WIDTH/2;
		int yc = (yo_-depthmap_[current_dm_].center_y)*(HEIGHT/YDIFF)+HEIGHT/2;
		int x1=xc-WINDOW_SIZE/2;
		int y1=yc-WINDOW_SIZE/2;

		//Checks whether switch of display manager is necessary
		dm_changed = checkSwitchDM(xc, yc, x1, y1);
		redisplay_ = 0;

//		int xc = (xo_-depthmap_[current_dm_].center_x)*(WIDTH/XDIFF)+WIDTH/2;
//		int yc = (yo_-depthmap_[current_dm_].center_y)*(HEIGHT/YDIFF)+HEIGHT/2;
//		int x1=xc-WINDOW_SIZE/2;
//		int y1=yc-WINDOW_SIZE/2;
//		if(x1<0){x1=0;}
//		if(y1<0){y1=0;}
//		if(x1>=WIDTH-WINDOW_SIZE){x1=WIDTH-WINDOW_SIZE-1;}
//		if(y1>=HEIGHT-WINDOW_SIZE){y1=HEIGHT-WINDOW_SIZE-1;}

		if(x1<0){x1=0;}
		if(y1<0){y1=0;}
		if(x1>=WIDTH-WINDOW_SIZE){x1=WIDTH-WINDOW_SIZE-1;}
		if(y1>=HEIGHT-WINDOW_SIZE){y1=HEIGHT-WINDOW_SIZE-1;}

		depthmap_[current_dm_].x1 = x1;
		depthmap_[current_dm_].y1 = y1;
		depthmap_[current_dm_].x2 = x1 + WINDOW_SIZE;
		depthmap_[current_dm_].y2 = y1 + WINDOW_SIZE;

		DEBUG_MSG(std::cout \
				<< "adjust window limits => " << std::endl \
				<< "        previous(" << prev_dm_ << ") (" << depthmap_[prev_dm_].x1 << "," << depthmap_[prev_dm_].y1 << "," << depthmap_[prev_dm_].x2 << "," << depthmap_[prev_dm_].y2 << ")" \
				<< ", current(" << current_dm_ << ") (" << depthmap_[current_dm_].x1 << "," << depthmap_[current_dm_].y1 << "," << depthmap_[current_dm_].x2 << "," << depthmap_[current_dm_].y2 << ")" << std::endl );

		if (!map_loader_mode_) {
			//prepare view for rendering
			int oldMatrixMode;
			glGetIntegerv(GL_MATRIX_MODE,&oldMatrixMode);
			prepareViewport(dm_changed, xc, yc);
			//plot in world coords
			raycastPointCloud();
			glMatrixMode(oldMatrixMode);
		}

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout << "(display) computed in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;
		DEBUG_MSG( std::cout << "[roll, pitch, yaw, xo, yo, zo]: " << roll_ << "," << pitch_ << "," << yaw_ << "," << xo_ << "," << yo_ << "," << zo_ << std::endl );
		showdepth();
		switch_dm_ = false;
		//redisplay_ = 0;
	}

	/*mainで使うglutまとめ*///////////
	void Mapping::glutFunctions(int argc,char* argv[])
	{
		if (hide_opengl_window_) {
			// create a hidden window
			if (!glutGet(GLUT_INIT_STATE))
			{
				// create the context (a little trick)
				int argc = 1;
				char* argv[1] = {nullptr};
				glutInit(&argc, argv);
			}
			glutInitDisplayMode(/*GLUT_RGBA*/GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
			hidden_win_id_ = glutCreateWindow("CFBORender");
			glutHideWindow();
			glutInitWindowPosition(0, 0);
			glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
		} else {
			glutInitWindowPosition(0, 0);
			glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
			glutInit(&argc, argv);
			glutInitDisplayMode(/*GLUT_RGBA*/GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
			glutCreateWindow(argv[0]);
		}

		glutDisplayFunc(displayFuncWrapper);
		glutIdleFunc(idleFuncWrapper);
		glutKeyboardFunc(keyboardFuncWrapper);
		glutSpecialFunc(specialKeyFuncWrapper);
		glutMouseFunc(mouseFuncWrapper);
		glutMouseWheelFunc(mouseWheelFuncWrapper);
		glutReshapeFunc(resizeFuncWrapper);
		glClearColor(1.0, 1.0, 1.0, 1.0);                                          //ウィンドウを塗りつぶす色の指定

		GLenum err = glewInit();
		if (err != GLEW_OK) {
			ROS_ERROR_STREAM("GL Error: " << glewGetErrorString(err) );
			exit(1);
		}
#if (DEBUGGING_MSG > 1)
		glEnable              ( GL_DEBUG_OUTPUT );
		glDebugMessageCallback( MessageCallback, 0 );
#endif


		initDepthBuffers();
	}

	void Mapping::idle()
	{
		//glutPostRedisplay();
		//display();
		if (!ros::ok()) {
			stop_program_ = true;
		}
		if (stop_program_) {
			//stop GLUT loop
			glutLeaveMainLoop();
			throw NULL;
		}
		//if (redisplay_) {
		//	display();
		//}
		display();
		ros::spinOnce();
		//loop_rate_.sleep();
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>( 1.0/LOOP_RATE * 1000.0 )));
	}

	void Mapping::rosLoop()
	{
		while (ros::ok() && !stop_program_) {
			if (!ros::ok()) {
				stop_program_ = true;
			}
			if (stop_program_) {
				throw NULL;
				exit(0);
			}
			if (redisplay_) {
				display();
			}
			//display();
			ros::spinOnce();
			//loop_rate_.sleep();
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>( 1.0/LOOP_RATE * 1000.0 )));
		}
		//terminate program
		deInit();
	}

	void Mapping::loop()
	{
		//run the GLUT main loop on the main thread
		try {
			glutMainLoop();
		} catch (...) {}
	}

	void Mapping::resizeFuncWrapper(int w, int h)
	{
		instance->resize(w, h);
	}
	void Mapping::keyboardFuncWrapper(unsigned char key, int x, int y)
	{
		instance->keyboard(key, x, y);
	}
	void Mapping::specialKeyFuncWrapper(int key, int x, int y)
	{
		instance->special_key(key, x, y);
	}

	void Mapping::mouseWheelFuncWrapper(int wheel, int direction, int x, int y)
	{
		instance->mouse_wheel(wheel, direction, x, y);
	}
	void Mapping::mouseFuncWrapper(int button, int state, int x, int y)
	{
		instance->mouse(button, state, x, y);
	}

	void Mapping::displayFuncWrapper()
	{
		instance->display();
	}

	void Mapping::idleFuncWrapper()
	{
		instance->idle();
	}

//	void Mapping::filterPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &inCloud,
//            					   pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& outCloud)
//	{
//		enum {
//			M_TO_MM = 1000,
//		};
//		std::vector<unsigned int> range_array;
//		unsigned int start_idx = 0;
//		unsigned int end_idx = 0;
//
//		//copy input cloud
//		outCloud = *inCloud;
//		//sort data by ring number
//		std::sort(outCloud.points.begin(), outCloud.points.end(), RingComparator());
//
//		//apply median filter on range data
//		auto curr_ring = outCloud.points[0].ring;
//		range_array.clear();
//		for (auto p : outCloud.points) {
//			if (p.ring == curr_ring) { //for all points in current ring
//				//get range data
//				unsigned int range_mm = (unsigned int)(sqrt(p.x*p.x + p.y*p.y + p.z*p.z) * (double)M_TO_MM); //to mm
//				range_array.push_back(range_mm);
//				end_idx++;
//			} else { //end of current ring
//				//printf("Ring: %d, points: %ld, start: %d, end: %d\n", curr_ring, range_array.size(), start_idx, end_idx);
//				//perform median filter on current ring data
//				medianfilter_5_fast(&(range_array[0]), 0, range_array.size()-1);
//				for (unsigned int i=start_idx; i<end_idx; i++) {
//					auto q = outCloud.points[i];
//					//spherical coords
//					auto r = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
//					auto theta = atan2(q.y, q.x);
//					auto phi = acos(q.z/r);
//					//compute new filtered range
//					double range = (double)(range_array[i-start_idx]) / (double)M_TO_MM; //back to meters
//					//using new range and get back to Cartesian coords
//					q.x = range*sin(phi)*cos(theta);
//					q.y = range*sin(phi)*sin(theta);
//					q.z = range*cos(phi);
//					outCloud.points[i] = q;
//				}
//				//move to next ring and continue
//				curr_ring = p.ring;
//				range_array.clear();
//				start_idx = end_idx;
//				//get range data
//				unsigned int range_mm = (unsigned int)(sqrt(p.x*p.x + p.y*p.y + p.z*p.z) * (double)M_TO_MM); //to mm
//				range_array.push_back(range_mm);
//				end_idx++;
//			}
//		}
//		//process the last ring data
//		if (start_idx < outCloud.points.size()) {
//			//printf("Ring: %d, points: %ld, start: %d, end: %d\n", curr_ring, range_array.size(), start_idx, end_idx);
//			//perform median filter on current ring data
//			medianfilter_5_fast(&(range_array[0]), 0, range_array.size()-1);
//			for (unsigned int i=start_idx; i<end_idx; i++) {
//				auto q = outCloud.points[i];
//				//spherical coords
//				auto r = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
//				auto theta = atan2(q.y, q.x);
//				auto phi = acos(q.z/r);
//				//compute new filtered range
//				double range = (double)(range_array[i-start_idx]) / (double)M_TO_MM; //back to meters
//				//using new range and get back to Cartesian coords
//				q.x = range*sin(phi)*cos(theta);
//				q.y = range*sin(phi)*sin(theta);
//				q.z = range*cos(phi);
//				outCloud.points[i] = q;
//			}
//			start_idx = end_idx;
//		}
//		//printf("Point cloud size: %ld\n", outCloud.points.size());
//
////		std::ofstream logfile;
////		logfile.open ("/tmp/nomedian.log");
////		for (auto p : range_array) {
////			logfile << p << std::endl;
////		}
////		logfile.close();
////		medianfilter_5_fast(&(range_array[0]), 0, range_array.size()-1);
////		logfile.open ("/tmp/w_median.log");
////		for (auto p : range_array) {
////			logfile << p << std::endl;
////		}
////		logfile.close();
//		std::ofstream logfile;
//		logfile.open ("/tmp/veldata_orig.log");
//		for (auto p : inCloud->points) {
//			logfile << p.x << "," << p.y << "," << p.z << "," << p.intensity << "," << p.ring << std::endl;
//		}
//		logfile.close();
//		logfile.open ("/tmp/veldata_new.log");
//		for (auto p : outCloud.points) {
//			logfile << p.x << "," << p.y << "," << p.z << "," << p.intensity << "," << p.ring << std::endl;
//		}
//		logfile.close();
//	}

	void Mapping::filterPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &inCloud,
            					   pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& outCloud,
								   bool filter_range, int filter_rings, bool filter_height)
	{
		enum {
			M_TO_MM = 1000,
		};
		std::vector< std::vector< std::pair<double, unsigned int> > > range_matrix;
		//double range = 0.0;
		void (*median3)(unsigned int *, unsigned int, unsigned int) = &medianfilter_3_fast;//to avoid unused function warning
		void (*median5)(unsigned int *, unsigned int, unsigned int) = &medianfilter_5_fast;//to avoid unused function warning
		(void)median3;
		(void)median5;

		//copy input cloud
		outCloud = *inCloud;
		//if filtering requested
		if (filter_range) {
			//sort data by ring number
			std::sort(outCloud.points.begin(), outCloud.points.end(), RingComparator());
		}
		if (filter_rings > 0) {
			for (auto &p : outCloud.points) {
				if (p.ring <= (filter_rings-1)) {
					p.x = 0;
					p.y = 0;
					p.z = 0;
				}
			}
		}
		if (filter_height) {
			for (auto &p : outCloud.points) {
				if (p.z < filter_height_limit_) {
					p.x = 0;
					p.y = 0;
					p.z = 0;
				}
			}
		}

		if (filter_range) {
			const double HEIGHT_ERROR = 1.0;
			for (unsigned int i = 1; i < outCloud.points.size()-1; i++) {
				auto p = outCloud.points[i];
				auto q = outCloud.points[i-1];
				auto r = outCloud.points[i+1];
				if (p.ring == q.ring && p.ring == r.ring) {
					if (p.z <= -1.0*HEIGHT_ERROR && (fabs(p.z-q.z) > HEIGHT_ERROR && fabs(p.z-r.z) > HEIGHT_ERROR)) {
						p.z=(q.z+r.z)/2.0;
//						p.x=0.0;
//						p.y=0.0;
//						p.z=0.0;
					}
				}
			}
		}

//		if (filter_range) {
//			//resize the range matrix
//			range_matrix.resize(outCloud.points[outCloud.points.size()-1].ring + 1); //as many rings as the pointcloud has
//
//			//fill the matrix
//			unsigned int idx = 0;
//			for (auto p : outCloud.points) {
//				range = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
//				range_matrix[p.ring].push_back(std::pair<double, unsigned int>(range, idx));
//				idx++;
//			}
//
//			//apply average filter
//			const double filter_factor = 1.5; //2.5;
//			for (unsigned int i = 0; i < range_matrix.size(); i++) {
//				//ignore the start/end points
//				for (unsigned int j = 1; j < range_matrix[i].size()-1; j++) {
//					Eigen::Matrix<double,9,1> data;
//					if (i == 0) {
//						data << range_matrix[i+0][j-1].first,
//								range_matrix[i+0][j+0].first, //this point
//								range_matrix[i+0][j+1].first,
//								range_matrix[i+1][j-1].first,
//								range_matrix[i+1][j+0].first,
//								range_matrix[i+1][j+1].first,
//								range_matrix[i+2][j-1].first,
//								range_matrix[i+2][j+0].first,
//								range_matrix[i+2][j+1].first;
//					} else if (i == range_matrix.size()-1) {
//						data << range_matrix[i-2][j-1].first,
//								range_matrix[i-2][j+0].first,
//								range_matrix[i-2][j+1].first,
//								range_matrix[i-1][j-1].first,
//								range_matrix[i-1][j+0].first,
//								range_matrix[i-1][j+1].first,
//								range_matrix[i+0][j-1].first,
//								range_matrix[i+0][j+0].first, //this point
//								range_matrix[i+0][j+1].first;
//					} else {
//						data << range_matrix[i-1][j-1].first,
//								range_matrix[i-1][j+0].first,
//								range_matrix[i-1][j+1].first,
//								range_matrix[i+0][j-1].first,
//								range_matrix[i+0][j+0].first, //this point
//								range_matrix[i+0][j+1].first,
//								range_matrix[i+1][j-1].first,
//								range_matrix[i+1][j+0].first,
//								range_matrix[i+1][j+1].first;
//					}
//					unsigned int n = data.rows();
//					double avg = data.mean();
//					double stdev = sqrt((data.array() - avg).square().sum() / (n-1));
//	//				Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]");
//	//				std::cout << "(" << i << "," << j << "):" << data.format(OctaveFmt) << avg << "," << stdev << std::endl;
//	//				std::cout << "(" << i << "," << j << "):" << avg << "," << stdev << std::endl;
//					//if this point is over 3xsigma remove it
//					if (fabs(range_matrix[i][j].first - avg) > filter_factor*stdev) {
//						//printf("remove point %d -> range %f, avg: %f, stdev: %f\n", range_matrix[i][j].second, range_matrix[i][j].first, avg, stdev);
//						outCloud.points[range_matrix[i][j].second].x = 0.0;
//						outCloud.points[range_matrix[i][j].second].y = 0.0;
//						outCloud.points[range_matrix[i][j].second].z = 0.0;
//					}
//				}
//			}
//
//			//apply average filter for 0th row of data
//			for (unsigned int i = 0; i < range_matrix.size(); i++) {
//				//add start point: since it is 360, 0th and last points are neighbours
//				unsigned int j = 0;
//				Eigen::Matrix<double,9,1> data;
//				if (i == 0) {
//					data << range_matrix[i+0][range_matrix[i+0].size()-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][j+1].first,
//							range_matrix[i+1][range_matrix[i+1].size()-1].first,
//							range_matrix[i+1][j+0].first,
//							range_matrix[i+1][j+1].first,
//							range_matrix[i+2][range_matrix[i+2].size()-1].first,
//							range_matrix[i+2][j+0].first,
//							range_matrix[i+2][j+1].first;
//				} else if (i == range_matrix.size()-1) {
//					data << range_matrix[i-2][range_matrix[i-2].size()-1].first,
//							range_matrix[i-2][j+0].first,
//							range_matrix[i-2][j+1].first,
//							range_matrix[i-1][range_matrix[i-1].size()-1].first,
//							range_matrix[i-1][j+0].first,
//							range_matrix[i-1][j+1].first,
//							range_matrix[i+0][range_matrix[i+0].size()-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][j+1].first;
//				} else {
//					data << range_matrix[i-1][range_matrix[i-1].size()-1].first,
//							range_matrix[i-1][j+0].first,
//							range_matrix[i-1][j+1].first,
//							range_matrix[i+0][range_matrix[i+0].size()-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][j+1].first,
//							range_matrix[i+1][range_matrix[i+1].size()-1].first,
//							range_matrix[i+1][j+0].first,
//							range_matrix[i+1][j+1].first;
//				}
//				unsigned int n = data.rows();
//				double avg = data.mean();
//				double stdev = sqrt((data.array() - avg).square().sum() / (n-1));
//	//				Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]");
//	//				std::cout << "(" << i << "," << j << "):" << data.format(OctaveFmt) << avg << "," << stdev << std::endl;
//	//				std::cout << "(" << i << "," << j << "):" << avg << "," << stdev << std::endl;
//				//if this point is over 3xsigma remove it
//				if (fabs(range_matrix[i][j].first - avg) > filter_factor*stdev) {
//					//printf("remove point %d -> range %f, avg: %f, stdev: %f\n", range_matrix[i][j].second, range_matrix[i][j].first, avg, stdev);
//					outCloud.points[range_matrix[i][j].second].x = 0.0;
//					outCloud.points[range_matrix[i][j].second].y = 0.0;
//					outCloud.points[range_matrix[i][j].second].z = 0.0;
//				}
//			}
//
//			//apply average filter for last row of data
//			for (unsigned int i = 0; i < range_matrix.size(); i++) {
//				//add last point: since it is 360, 0th and last points are neighbours
//				unsigned int j = range_matrix[i].size()-1;
//				Eigen::Matrix<double,9,1> data;
//				if (i == 0) {
//					data << range_matrix[i+0][j-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][0].first,
//							range_matrix[i+1][j-1].first,
//							range_matrix[i+1][j+0].first,
//							range_matrix[i+1][0].first,
//							range_matrix[i+2][j-1].first,
//							range_matrix[i+2][j+0].first,
//							range_matrix[i+2][0].first;
//				} else if (i == range_matrix.size()-1) {
//					data << range_matrix[i-2][j-1].first,
//							range_matrix[i-2][j+0].first,
//							range_matrix[i-2][0].first,
//							range_matrix[i-1][j-1].first,
//							range_matrix[i-1][j+0].first,
//							range_matrix[i-1][0].first,
//							range_matrix[i+0][j-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][0].first;
//				} else {
//					data << range_matrix[i-1][j-1].first,
//							range_matrix[i-1][j+0].first,
//							range_matrix[i-1][0].first,
//							range_matrix[i+0][j-1].first,
//							range_matrix[i+0][j+0].first, //this point
//							range_matrix[i+0][0].first,
//							range_matrix[i+1][j-1].first,
//							range_matrix[i+1][j+0].first,
//							range_matrix[i+1][0].first;
//				}
//				unsigned int n = data.rows();
//				double avg = data.mean();
//				double stdev = sqrt((data.array() - avg).square().sum() / (n-1));
//	//				Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ",", ",", "", "", "[", "]");
//	//				std::cout << "(" << i << "," << j << "):" << data.format(OctaveFmt) << avg << "," << stdev << std::endl;
//	//				std::cout << "(" << i << "," << j << "):" << avg << "," << stdev << std::endl;
//				//if this point is over 3xsigma remove it
//				if (fabs(range_matrix[i][j].first - avg) > filter_factor*stdev) {
//					//printf("remove point %d -> range %f, avg: %f, stdev: %f\n", range_matrix[i][j].second, range_matrix[i][j].first, avg, stdev);
//					outCloud.points[range_matrix[i][j].second].x = 0.0;
//					outCloud.points[range_matrix[i][j].second].y = 0.0;
//					outCloud.points[range_matrix[i][j].second].z = 0.0;
//				}
//			}
//		}
	}

//	void Mapping::velodyneCallbackFuncWrapper(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
//	{
//		instance->velodyneCallback(msg);
//	}

	//void Mapping::velodyneCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
	//void Mapping::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
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

			DEBUG_MSG(std::cout \
					<< "(velodyneCallback)" << std::endl \
					<< "        tf:(x,y,z,r,p,y)-> (" << std::setprecision(4) << xo_ << "," << yo_ << "," << zo_ << "," << roll_ << "," << pitch_ << "," << yaw_ << ")" << std::endl \
					<< "        lidar-> (" << std::setprecision(4) << lidar_transform.getOrigin().x() << ")" << lidar_transform.getOrigin().y() << ")" << lidar_transform.getOrigin().z() << "), center-> (" << depthmap_[current_dm_].center_x << "," << depthmap_[current_dm_].center_y << "," << depthmap_[current_dm_].center_z << ")" << std::endl \
					<< "        dist -> " << std::setprecision(4) << dist << ", deltad -> " << delta_d << ", speed -> " << speed_ << ", limits-> " << min_height_ << "," << max_height_ << "," << min_height_offset_ << "," << max_height_offset_ << std::endl \
					<< "        deltaZ -> " << std::setprecision(4) << avg_delta_z_ << std::endl \
					<< std::defaultfloat << std::setprecision(6) );

//			//for (auto p : prev_points_.points) {
//			#pragma omp parallel for schedule(auto)
//			for (pcl::PointCloud<pcl::PointXYZI>::iterator it = prev_points_.begin();
//				it < prev_points_.end(); ++it) {
//				pcl::PointXYZI p = *it;
//				tf::Point pt(p.x,p.y,p.z);
//				tf::Point pt_world = transform * pt;
//				double distance=pt.x()*pt.x()+pt.y()*pt.y()+pt.z()*pt.z();
//				if(distance<3*3)continue;
//				pcl::PointXYZI wp;
//				wp.x=pt_world.x();
//				wp.y=pt_world.y();
//				wp.z=pt_world.z();
//				wp.intensity=p.intensity;
//
//				#pragma omp critical
//				pcl_out_.push_back(wp);
//			}

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

	void Mapping::savePointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ptcloud)
	{
		if (ptcloud->points.size()) {
			auto start = std::chrono::high_resolution_clock::now();
			//take a copy of the input pointcloud
			pcl::PointCloud<pcl::PointXYZI> ptcloud_aux = *ptcloud;
			//generate the filename with date/time
			std::string target_filename = height_map_filename_prefix_;
			target_filename += std::string("_") + timeToStr(ros::WallTime::now());
			target_filename += std::string(".pcd");
			//save the point cloud
			pcl::io::savePCDFile(target_filename, ptcloud_aux, true);
			//update the map file
			std::ofstream mapfile;
			mapfile.open (std::string(height_map_filename_prefix_ + ".map").c_str(), std::ios::app);
			mapfile << xo_ << "," << yo_ << "," << zo_ << "," << yaw_ << "," << pitch_ << "," << roll_ << "," << target_filename << std::endl;
			mapfile.close();
			auto elapsed = std::chrono::high_resolution_clock::now() - start;
			long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			std::cout << "Saved in " <<(double)microseconds/1000000.0 << " seconds" <<  std::endl;
		}
	}

	void Mapping::publishGroundPointCloud()
	{
		publishing_ground_cloud_ = true;
		//generate the ground pointcloud and grid
		genGroundPointCloud();
		//Publish it
		if (depth_ground_pointcloud_pub_.getNumSubscribers() > 0) {// anyone listening?
			pcl_depth_ground_.header=pcl_out_.header;
			pcl_depth_ground_.header.frame_id=source_tf_;
			depth_ground_pointcloud_pub_.publish(pcl_depth_ground_);
		}
		publishing_ground_cloud_ = false;
	}

	void Mapping::publishObstaclePointCloud()
	{
		publishing_obst_cloud_ = true;
		if (generate_obstacle_data_) {
			pcl_obstacles_.header=pcl_out_.header;
			pcl_obstacles_.header.frame_id=source_tf_;
			if (obstacle_pointcloud_pub_.getNumSubscribers() > 0) {// anyone listening?
				obstacle_pointcloud_pub_.publish(pcl_obstacles_);
			}
		}
		if (generate_ground_data_) {
			pcl_ground_.header=pcl_out_.header;
			pcl_ground_.header.frame_id=source_tf_;
			if (ground_pointcloud_pub_.getNumSubscribers() > 0) {// anyone listening?
				ground_pointcloud_pub_.publish(pcl_ground_);
			}
		}
		publishing_obst_cloud_ = false;
	}

	template <typename T>
	T Mapping::getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
	{
	  T value;
	  if (nh.hasParam(param_name))
	  {
		nh.getParam(param_name, value);
	  }
	  else
	  {
		ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
		value = default_value;
	  }
	  return value;
	}

	//based on glInfo.cpp by Song Ho Ahn (song.ahn@gmail.com)
	bool Mapping::isGLExtensionSupported(const std::string& ext)
	{
	    std::string str;
	    std::vector <std::string> extensions;

	    // get all extensions as a string
	    str = (const char*)glGetString(GL_EXTENSIONS);

	    // split extensions
	    if(str.size() > 0)
	    {
	        char* str2 = new char[str.size() + 1];
	        strcpy(str2, str.c_str());
	        char* tok = strtok(str2, " ");
	        while(tok)
	        {
	            extensions.push_back(tok);    // put a extension into vector
	            tok = strtok(0, " ");         // next token
	        }
	        delete [] str2;
	    }

	    // search corresponding extension
	    std::vector<std::string>::const_iterator iter = extensions.begin();
	    std::vector<std::string>::const_iterator endIter = extensions.end();

	    while(iter != endIter)
	    {
	        if(ext == *iter)
	            return true;
	        else
	            ++iter;
	    }
	    return false;
	}

	//based on fboDepth by Song Ho Ahn (song.ahn@gmail.com)
	bool Mapping::checkFramebufferStatus(GLuint fbo)
	{
	    // check FBO status
	    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo); // bind
	    GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER);
	    switch(status)
	    {
	    case GL_FRAMEBUFFER_COMPLETE:
	        std::cout << "Framebuffer complete." << std::endl;
	        return true;

	    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
	        std::cout << "[ERROR] Framebuffer incomplete: Attachment is NOT complete." << std::endl;
	        return false;

	    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
	        std::cout << "[ERROR] Framebuffer incomplete: No image is attached to FBO." << std::endl;
	        return false;
	/*
	    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
	        std::cout << "[ERROR] Framebuffer incomplete: Attached images have different dimensions." << std::endl;
	        return false;

	    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
	        std::cout << "[ERROR] Framebuffer incomplete: Color attached images have different internal formats." << std::endl;
	        return false;
	*/
	    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
	        std::cout << "[ERROR] Framebuffer incomplete: Draw buffer." << std::endl;
	        return false;

	    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
	        std::cout << "[ERROR] Framebuffer incomplete: Read buffer." << std::endl;
	        return false;

	    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
	        std::cout << "[ERROR] Framebuffer incomplete: Multisample." << std::endl;
	        return false;

	    case GL_FRAMEBUFFER_UNSUPPORTED:
	        std::cout << "[ERROR] Framebuffer incomplete: Unsupported by FBO implementation." << std::endl;
	        return false;

	    default:
	        std::cout << "[ERROR] Framebuffer incomplete: Unknown error." << std::endl;
	        return false;
	    }
	    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);   // unbind
	}
} //namespace

/*
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_mapping");
	ground_mapping::Mapping mapping;

	//install the signal handler to be able to stop with CTRL-C
	signal(SIGINT, &signalHandler);

	mapping.init(argc,argv);
	mapping.loop();

    return 0;
}
*/
// %EndTag(FULLTEXT)%
