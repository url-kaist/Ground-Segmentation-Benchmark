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
 * mapping.h
 *
 *  Created on: 2018/03/14
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#ifndef MAPPING_H_
#define MAPPING_H_

#define USE_OCTREE_SEARCH 0
#define USE_KDTREE_SEARCH 0
#define USE_GRID_SEARCH 0
#define USE_DEPTHBUFF_SEARCH 1

//Std
#include <iostream>
#include <chrono>
#include <mutex>

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

//PCL
#include <pcl/point_types.h>
#if (USE_OCTREE_SEARCH)
#include <pcl/octree/octree.h> //Too slow!!!
#endif

//BOOST
#include <boost/thread.hpp>

//OpenGL
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include "grid.h"

//namespace ground_mapping {
//	//Taken from <velodyne_pointcloud/point_types.h>
//	/** Euclidean Velodyne coordinate, including intensity and ring number. */
//	struct PointXYZIR
//	{
//	  PCL_ADD_POINT4D;                    // quad-word XYZ
//	  float    intensity;                 ///< laser intensity reading
//	  uint16_t ring;                      ///< laser ring number
//	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
//	} EIGEN_ALIGN16;
//}; //namespace
//
//POINT_CLOUD_REGISTER_POINT_STRUCT(ground_mapping::PointXYZIR,
//                                (float, x, x)
//                                (float, y, y)
//                                (float, z, z)
//                                (float, intensity, intensity)
//                                (uint16_t, ring, ring))

#define USE_GL_VBO 1
#define PBO_COUNT 2
namespace ground_mapping {
	class Mapping {
		public:
			Mapping();
			~Mapping();
			void init(int argc, char* argv[]);
			void deInit();
			void loop();
			void rosLoop();

		public:
			static Mapping *instance;

		public:
			//wrappers for OpenGL
			static void resizeFuncWrapper(int w, int h);
			static void keyboardFuncWrapper(unsigned char key, int x, int y);
			static void specialKeyFuncWrapper(int key, int x, int y);
			static void mouseWheelFuncWrapper(int wheel, int direction, int x, int y);
			static void mouseFuncWrapper(int button, int state, int x, int y);
			static void displayFuncWrapper();
			static void idleFuncWrapper();
			//static void velodyneCallbackFuncWrapper(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);
			void loadDepthImage(const std::string filename);
			void saveDepthImage(GLfloat *buffer_src, const std::string filename, bool asPFM=true);


		protected:
			void setInstance();
			void loadAllParameters();
			void initDepthBuffers();
			void resize(int w, int h);
			void keyboard(unsigned char key, int x, int y);
			void special_key(int key, int x, int y);
			void mouse_wheel(int wheel, int direction, int x, int y);
			void mouse(int button, int state, int x, int y);
			void display();
			void idle();
			void glutFunctions(int argc,char* argv[]);
			void init_fb(unsigned int id);
			int showdepth(void);
			void raycastPointCloud();
			void prepareViewport(bool dm_changed, int xc, int yc);
			bool checkSwitchDM(int &xc, int &yc, int &x1, int &y1);

			void saveImage();
			void loadImage(double curr_x, double curr_y, double curr_z, bool reload=false);
			void displayDepthData();
			void genGroundPointCloud();
			void obsSegmentation();
			void velodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &msg);
			void savePointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ptcloud);
			void publishGroundPointCloud();
			void publishObstaclePointCloud();
			void filterPointCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &inCloud,
								  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& outCloud,
								  bool filter_range = false,
								  int filter_rings = -1,
								  bool filter_height = false);

			//!
			//! \brief getParam Get parameter from node handle
			//! \param nh The nodehandle
			//! \param param_name Key string
			//! \param default_value Default value if not found
			//! \return The parameter value
			//!
			template <typename T>
			T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value);

		public:
			const static int WINDOW_SIZE;
			const static int WIDTH;
			const static int HEIGHT;
			const static double XMAX;
			const static double XMIN;
			const static double YMAX;
			const static double YMIN;
			const static double XDIFF;
			const static double YDIFF;
			const static double HMAX;
			const static double HMIN;
			const static double CELL_SIZE;
			const static double GRID_LENGTH_AUTO;
			const static double MAX_DISTANCE;
			const static double DELTA_Z_THRESHOLD;
			const static unsigned int DELTA_Z_SAMPLES;
			const static unsigned int MAX_DEPTH_BUFFERS;
			const static double LOOP_RATE;

		public:
			int stop_program_;

		private:
			template<typename T>
			friend std::ostream& operator<<(std::ostream& s, const std::vector<T>& v) {
			    char comma[3] = {'\0', ' ', '\0'};
			    for (const auto& e : v) {
			        s << comma << e;
			        comma[0] = ',';
			    }
			    return s;
			}

			template<class T>
			static std::string timeToStr(T ros_t);

			template <class T>
			static void DoNotFree(T*) {}

			bool isGLExtensionSupported(const std::string& ext);
			bool checkFramebufferStatus(GLuint fbo);

		private:
			//Parameters
			double min_height_;
			double max_height_;
			double min_height_offset_;
			double max_height_offset_;
			double grid_cell_size_;
			double grid_width_;
			double grid_height_;
			bool generate_grid_data_;
			bool generate_obstacle_data_;
			bool generate_ground_data_;
			bool filter_grid_points_;
			double obstacle_height_threshold_;
			std::string input_pointcloud_topic_;
			std::string output_pointcloud_topic_;
			std::string grid_pointcloud_topic_;
			std::string obstacle_pointcloud_topic_;
			std::string ground_pointcloud_topic_;
			std::string source_tf_;
			std::string target_tf_;
			bool filter_input_points_;
			int filter_input_rings_;
			bool filter_by_height_;
			double filter_height_limit_;
			bool hide_opengl_window_;
			double max_update_distance_;
			bool merge_with_past_frame_;
			bool accumulate_height_;
			bool map_loader_mode_;
			std::vector<double> lidar_pose_;
			bool save_height_map_;
			std::string height_map_filename_prefix_;
			bool save_every_scan_;

			ros::NodeHandle nodehandle_;
			ros::Subscriber vel_subscriber_;

			//pcl::PointCloud<velodyne_pointcloud::PointXYZIR> map_;
			tf::TransformListener *tf_listener_;
			ros::Publisher depth_ground_pointcloud_pub_;
			ros::Publisher grid_pointcloud_pub_;
			ros::Publisher obstacle_pointcloud_pub_;
			ros::Publisher ground_pointcloud_pub_;

			pcl::PointCloud<pcl::PointXYZI> prev_points_;
			pcl::PointCloud<pcl::PointXYZI> pcl_out_;
			pcl::PointCloud<pcl::PointXYZI> pcl_depth_ground_;
			pcl::PointCloud<pcl::PointXYZI> pcl_grid_;
			pcl::PointCloud<pcl::PointXYZI> pcl_obstacles_;
			pcl::PointCloud<pcl::PointXYZI> pcl_ground_;
			pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_ground_ptr_;
			pcl::PointCloud<pcl::PointXYZI> pcl_ground_aux_;
			pcl::PointXYZI *point_array_;

#if (USE_KDTREE_SEARCH)
			pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
#endif
#if (USE_OCTREE_SEARCH)
			float octree_resolution_;
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
#endif

			double xo_;
			double yo_;
			double zo_;
			double roll_;
			double pitch_;
			double yaw_;

			typedef struct DepthMap{
				DepthMap() {
					fb = 0;
					cb = 0;
					rb = 0;
					center_x = 0.0;
					center_y = 0.0;
					center_z = 0.0;
					length = 0.0;
					time = 0.0;
//					cx = 0;
//					cy = 0;
//					cz = 0;
					x1 = 0;
					y1 = 0;
					x2 = 0;
					y2 = 0;
					buffer2 = NULL;
				}
				GLuint fb;  //frame buffer
				GLuint cb;  //color buffer
				GLuint rb;  //render buffer
				double center_x;   //center position
				double center_y;
				double center_z;
				double length;// created run length
				double time;  // created time
//				int cx;  //offset from global
//				int cy;  //offset from global
//				int cz;  //offset from global
				int x1;
				int y1;
				int x2;
				int y2;
				GLfloat *buffer2;
			} DepthMap;

			DepthMap depthmap_[10];
			int current_dm_;
			int prev_dm_;
			int dm_num_;

			typedef struct MapInfo {
				MapInfo() {
					x = 0.0;
					y = 0.0;
					z = 0.0;
					roll = 0.0;
					pitch = 0.0;
					yaw = 0.0;
					image_file = std::string("");
				}
				double x;
				double y;
				double z;
				double yaw;
				double pitch;
				double roll;
				std::string image_file;
			} MapInfo;

			std::vector<MapInfo> hd_map_info_;

			float zoom_;
			unsigned int redisplay_;
			unsigned int first_display_;
			double update_distance_;
			double speed_;
			double prev_xo_;
			double prev_yo_;
			double prev_zo_;
			double delta_z_;
			double avg_delta_z_;
			unsigned int delta_z_count_;

			int hidden_win_id_;
			bool waiting_for_tf_;
			bool switch_dm_;

			std::chrono::time_point<std::chrono::high_resolution_clock> global_start_time_;
			double total_running_time_;
			unsigned int loops_count_;

			GLfloat *buffer_;
			int max_threads_;
			pcl::PointCloud<pcl::PointXYZI> *pcl_ground_aux_array_;

			Grid grid_;

			GLuint readPBOIDs_[PBO_COUNT];
			unsigned int pbo_index_;
			GLuint writePBOIDs_;
			GLuint displayPBOIDs_;

			bool saving_file_ = false;
			bool publishing_obst_cloud_ = false;
			bool publishing_ground_cloud_ = false;
			boost::thread record_thread_;
			boost::thread publish_obst_thread_;
			boost::thread publish_ground_thread_;
			boost::thread ros_thread_;

			std::mutex ptcloud_access_;

			ros::Rate loop_rate_;
	};

} //namespace


#endif /*MAPPING_H_*/
