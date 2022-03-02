#ifndef R_GPF_H
#define R_GPF_H

/*
 * This code is from below github page:
 * https://github.com/LimHyungTae/ERASOR
 * In fact, R-GPF is one of module of ERASOR!
 */

#include "../common.hpp"

#define NORMAL_ENOUGH 0.55
#define TOO_TILTED 1.0
#define LITTLE_TILTED 0.0
#define NUM_EXPERINEMTAL_MAX_PATCH 10000

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

typedef std::vector<pcl::PointCloud<PointXYZILID> > Ring;
typedef std::vector<Ring> Zone;

class RegionwiseGPF{
public:
    RegionwiseGPF(){};
    RegionwiseGPF(ros::NodeHandle* nh):node_handle_(*nh)
    {
      // Init ROS related
      ROS_INFO("Inititalizing Ground Plane Fitter...");

      node_handle_.param("sensor_height", sensor_height_, 1.723);
      ROS_INFO("Sensor Height: %f", sensor_height_);
      node_handle_.param<string>("/r_gpf/mode", mode_, "polar");

      node_handle_.param("/r_gpf/num_iter", num_iter_, 3);
      ROS_INFO("Num of Iteration: %d", num_iter_);

      node_handle_.param("/r_gpf/num_lpr", num_lpr_, 20);
      ROS_INFO("Num of LPR: %d", num_lpr_);

      node_handle_.param("/r_gpf/num_min_pts", num_min_pts_, 9);
      ROS_INFO("Num of min. points: %d", num_min_pts_);

      node_handle_.param("/r_gpf/th_seeds", th_seeds_, 0.4);
      ROS_INFO("Seeds Threshold: %f", th_seeds_);

      node_handle_.param("/r_gpf/th_dist", th_dist_, 0.3);
      ROS_INFO("Distance Threshold: %f", th_dist_);

      node_handle_.param("/r_gpf/max_r", max_range_, 80.0);
      ROS_INFO("Max. range:: %f", max_range_);

      node_handle_.param("/r_gpf/num_rings", num_rings_, 20);
      ROS_INFO("Num. rings: %d", num_rings_);

      node_handle_.param("/r_gpf/num_sectors", num_sectors_, 108);
      ROS_INFO("Num. sectors: %d", num_sectors_);

      node_handle_.param("/r_gpf/visualize", visualize_, true);
      poly_list_.header.frame_id = "/map";
      poly_list_.polygons.reserve(130000);

      ring_size = max_range_ / num_rings_;
      sector_size = 2 * M_PI / num_sectors_;

      ground_pc_.reserve(NUM_EXPERINEMTAL_MAX_PATCH);
      non_ground_pc_.reserve(NUM_EXPERINEMTAL_MAX_PATCH);
      regionwise_ground_.reserve(NUM_EXPERINEMTAL_MAX_PATCH);
      regionwise_nonground_.reserve(NUM_EXPERINEMTAL_MAX_PATCH);

      PlaneViz = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/gpf/plane",100);

      init_regionwise_patches();
    }
    void estimate_ground(const pcl::PointCloud<PointXYZILID>& cloudIn,
                         pcl::PointCloud<PointXYZILID>& cloudOut,
                         pcl::PointCloud<PointXYZILID>& cloudNonground,
                         double& time_taken);

    geometry_msgs::PolygonStamped set_plane_polygon(const MatrixXf& normal_v, const float& d);
private:
    ros::NodeHandle node_handle_;
    // ROS parameters
    double sensor_height_;
    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    double th_seeds_;
    double th_dist_;
    double max_range_;
    int num_rings_;
    int num_sectors_;
    std::string mode_;


    float d_;
    MatrixXf normal_;
    float th_dist_d_;

    double ring_size;
    double sector_size;
    // For visualization
    bool visualize_;
    jsk_recognition_msgs::PolygonArray poly_list_;


    Zone patches;

    ros::Publisher PlaneViz;

    pcl::PointCloud<PointXYZILID> ground_pc_;
    pcl::PointCloud<PointXYZILID> non_ground_pc_;

    pcl::PointCloud<PointXYZILID> regionwise_ground_;
    pcl::PointCloud<PointXYZILID> regionwise_nonground_;

    void init_regionwise_patches();
    void clear_patches();
    double xy2theta(const double& x, const double& y);
    double xy2radius(const double& x, const double& y);
    void pc2patches(const pcl::PointCloud<PointXYZILID>& src, Zone& patches);
    void extract_piecewiseground(const pcl::PointCloud<PointXYZILID>& src,
                                 pcl::PointCloud<PointXYZILID>& dst,
                                 pcl::PointCloud<PointXYZILID>& non_ground_dst);
    void estimate_plane_(const pcl::PointCloud<PointXYZILID>& ground);
    void extract_initial_seeds_(const pcl::PointCloud<PointXYZILID>& p_sorted,
                                      pcl::PointCloud<PointXYZILID>& init_seeds);
    geometry_msgs::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split);
    float calc_pt_z(float pt_x, float pt_y);
};    

void RegionwiseGPF::init_regionwise_patches(){
  pcl::PointCloud<PointXYZILID> cloud;
  cloud.reserve(400);
  Ring ring;
  for (int i=0; i< num_sectors_; i++){
    ring.emplace_back(cloud);
  }
  for (int j=0; j< num_rings_; j++){
    patches.emplace_back(ring);
  }
}

void RegionwiseGPF::clear_patches(){
  for (int i=0; i< num_sectors_; i++){
    for (int j=0; j< num_rings_; j++){
      if (!patches[j][i].points.empty()) patches[j][i].points.clear();
    }
  }
}


void RegionwiseGPF::estimate_plane_(const pcl::PointCloud<PointXYZILID>& ground){
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose()*seeds_mean)(0,0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = th_dist_ - d_;
}

void RegionwiseGPF::extract_initial_seeds_(const pcl::PointCloud<PointXYZILID>& p_sorted,
                                  pcl::PointCloud<PointXYZILID>& init_seeds){
  init_seeds.points.clear();

  // LPR is the mean of low point representative
//  std::cout<<"hey!! p sorted"<<p_sorted.points.size()<<std::endl;
  double sum = 0;
  int cnt = 0;
  // Calculate the mean height value.

  for(int i=0;i<p_sorted.points.size() && cnt< num_lpr_ ;i++){
      sum += p_sorted.points[i].z;
      cnt++;
  }
  double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for(int i=0;i<p_sorted.points.size();i++){
      if(p_sorted.points[i].z < lpr_height + th_seeds_){
          init_seeds.points.push_back(p_sorted.points[i]);
      }
  }
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void RegionwiseGPF::estimate_ground(const pcl::PointCloud<PointXYZILID>& cloudIn,
                                    pcl::PointCloud<PointXYZILID>& cloudOut,
                                    pcl::PointCloud<PointXYZILID>& cloudNonground,
                                    double& time_taken){
    poly_list_.header.stamp = ros::Time::now();
    if (!poly_list_.polygons.empty()) poly_list_.polygons.clear();
    if (!poly_list_.likelihood.empty()) poly_list_.likelihood.clear();

    auto start = chrono::high_resolution_clock::now();
    // 1.Msg to pointcloud
    pcl::PointCloud<PointXYZILID> laserCloudIn;
    laserCloudIn = cloudIn;
//    pcl::PointCloud<PointXYZILID> laserCloudIn_org = cloudIn;
    // For mark ground points and hold all points

    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(),laserCloudIn.end(), point_cmp);

    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<PointXYZILID>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -1.5*sensor_height_){
            it++;
        }else{
            break;
        }
    }

    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);
    // 4. pointcloud -> regionwise setting
    clear_patches();
    pc2patches(laserCloudIn, patches);

    static int num_max_p = 0;
    static int num_max_rg = 0;
    static int num_max_rng = 0;

    cloudOut.clear();
    cloudNonground.clear();
    for (uint16_t sector_idx=0; sector_idx < num_sectors_; ++sector_idx){
      for (uint16_t ring_idx=0; ring_idx < num_rings_; ++ring_idx){
        if (patches[ring_idx][sector_idx].points.size() > num_min_pts_){
          extract_piecewiseground(patches[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);
          if (visualize_){
            auto polygons = set_polygons(ring_idx, sector_idx, 3);
            polygons.header = poly_list_.header;
            poly_list_.polygons.push_back(polygons);
            if (normal_(2,0) > 0.8){
              poly_list_.likelihood.push_back(NORMAL_ENOUGH);
            }else if (normal_(2,0) > 0.4){
              poly_list_.likelihood.push_back(LITTLE_TILTED);
            }else{
              poly_list_.likelihood.push_back(TOO_TILTED);
            }
          }

          cloudOut += regionwise_ground_;
          cloudNonground += regionwise_nonground_;

          if (patches[ring_idx][sector_idx].size() > num_max_p) num_max_p = patches[ring_idx][sector_idx].size();
          if (regionwise_ground_.size() > num_max_rg) num_max_rg = regionwise_ground_.size();
          if (regionwise_nonground_.size() > num_max_rng) num_max_rng = regionwise_nonground_.size();

        }
      }
    }
    std::cout<<num_max_p<< " | " << num_max_rg<<" | "<<num_max_rng<<std::endl;

    auto end = chrono::high_resolution_clock::now();
    time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;

    PlaneViz.publish(poly_list_);
}

double RegionwiseGPF::xy2theta(const double& x, const double& y){ // 0 ~ 2 * PI
  if (y >= 0)
  {
    return atan2(y,x); // 1, 2 quadrant
  }else{
     return 2 * M_PI + atan2(y,x);// 3, 4 quadrant
  }
}

double RegionwiseGPF::xy2radius(const double& x, const double& y){
  return sqrt(pow(x, 2) + pow(y, 2));
}

void RegionwiseGPF::pc2patches(const pcl::PointCloud<PointXYZILID>& src, Zone& patches){
  for (auto const &pt : src.points){
      double r = xy2radius(pt.x, pt.y);
      if (r <= max_range_){
        double theta = xy2theta(pt.x, pt.y);

        int sector_idx = min(static_cast<int>((theta / sector_size)), num_sectors_ - 1);
        int ring_idx = min(static_cast<int>((r / ring_size)), num_rings_ - 1);
        patches[ring_idx][sector_idx].points.push_back(pt);
      }

  }
}

void RegionwiseGPF::extract_piecewiseground(const pcl::PointCloud<PointXYZILID>& src,
                                                  pcl::PointCloud<PointXYZILID>& dst,
                                                  pcl::PointCloud<PointXYZILID>& non_ground_dst){
  // 0. Initialization
  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!dst.empty()) dst.clear();
  if (!non_ground_dst.empty()) non_ground_dst.clear();
  // 1. set seeds!

  extract_initial_seeds_(src, ground_pc_);
  // 2. Extract ground
  for(int i=0;i<num_iter_;i++){
      estimate_plane_(ground_pc_);
      ground_pc_.clear();

      //pointcloud to matrix
      Eigen::MatrixXf points(src.points.size(),3);
      int j = 0;
      for(auto& p:src.points){
          points.row(j++)<<p.x,p.y,p.z;
      }
      // ground plane model
      Eigen::VectorXf result = points*normal_;
      // threshold filter
      for(int r=0;r<result.rows();r++){
        if (i < num_iter_ - 1){
          if(result[r]<th_dist_d_){
              ground_pc_.points.push_back(src[r]);
          }
        }else{ // Final stage
          if(result[r]<th_dist_d_){
              dst.points.push_back(src[r]);
          }else{
            if (i==num_iter_-1){
              non_ground_dst.push_back(src[r]);
            }
          }
       }
     }
  }
}

geometry_msgs::PolygonStamped RegionwiseGPF::set_polygons(int r_idx, int theta_idx, int num_split){
  geometry_msgs::PolygonStamped polygons;
  // Set point of polygon. Start from RL and ccw
  geometry_msgs::Point32 point;


  // RL
  double r_len = r_idx * ring_size;
  double angle = theta_idx * sector_size;

  point.x = r_len * cos(angle);   point.y = r_len * sin(angle);
  point.z = calc_pt_z(point.x, point.y);
  polygons.polygon.points.push_back(point);
  // RU
  r_len = r_len + ring_size;
  point.x = r_len * cos(angle);   point.y = r_len * sin(angle);
  point.z = calc_pt_z(point.x, point.y);
  polygons.polygon.points.push_back(point);

  // RU -> LU
  for (int idx = 1; idx <= num_split;++idx){
    angle = angle + sector_size / num_split;
    point.x = r_len * cos(angle);   point.y = r_len * sin(angle);
    point.z = calc_pt_z(point.x, point.y);
    polygons.polygon.points.push_back(point);
  }

  r_len = r_len - ring_size;
  point.x = r_len * cos(angle);   point.y = r_len * sin(angle);
  point.z = calc_pt_z(point.x, point.y);
  polygons.polygon.points.push_back(point);

  for (int idx = 1; idx < num_split; ++idx){
    angle = angle - sector_size / num_split;
    point.x = r_len * cos(angle);   point.y = r_len * sin(angle);
    point.z = calc_pt_z(point.x, point.y);
    polygons.polygon.points.push_back(point);
  }

  return polygons;
}

float RegionwiseGPF::calc_pt_z(float pt_x, float pt_y){
  float& a_ = normal_(0,0);
  float& b_ = normal_(1,0);
  float& c_ = normal_(2,0);
  return 1.5;
//  return (d_ - (a_ * pt_x + b_ * pt_y) ) / c_;
}

#endif
