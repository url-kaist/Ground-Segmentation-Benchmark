#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>
#include "common.hpp"

using namespace unavlib;

void parse_pcd(const pcl::PointCloud<pcl::PointXYZI>& src,
               pcl::PointCloud<pcl::PointXYZI>& TP, pcl::PointCloud<pcl::PointXYZI>& FP,
               pcl::PointCloud<pcl::PointXYZI>& FN, pcl::PointCloud<pcl::PointXYZI>& TN){

  if (!TP.points.empty()) TP.points.clear();
  if (!FP.points.empty()) FP.points.clear();
  if (!FN.points.empty()) FN.points.clear();
  if (!TN.points.empty()) TN.points.clear();

  for (const auto &pt: src.points){
    int state = pt.intensity;
    if (state == TRUEPOSITIVE){
      TP.points.push_back(pt);
    }else if (state == TRUENEGATIVE){
      TN.points.push_back(pt);
    }else if (state == FALSENEGATIVE){
      FN.points.push_back(pt);
    }else if (state == FALSEPOSITIVE){
      FP.points.push_back(pt);
    }
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;

    std::string abs_dir, target_seq;
    int target_frame;

    nodeHandler.param<std::string>("/abs_dir", abs_dir, "/media/shapelim/UX960NVMe1/gpf_output/pcds");
    nodeHandler.param<std::string>("/seq", target_seq, "10");
    nodeHandler.param<int>("/frame", target_frame, 1);
    // 0. Declare publishers
    ros::Publisher casTP_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cascade/TP", 100);
    ros::Publisher casFP_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cascade/FP", 100);
    ros::Publisher casFN_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cascade/FN", 100);
    ros::Publisher casTN_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/cascade/TN", 100);


    // 1. Set each filename
    std::string count_str = std::to_string(target_frame);
    std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
    std::string cascade_name = abs_dir + "/cascaded_gseg/" + target_seq + "/" + count_str_padded + ".pcd";


    ////////////////////////////////////////////////////////////////////
    std::cout<<"\033[1;32m Loading map..."<<std::endl;

    // load Cascaded_gseg
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cas(new pcl::PointCloud<pcl::PointXYZI>);
    datahandle3d::load_pcd(cascade_name, ptr_cas);

//    // load original src
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
//    datahandle3d::load_pcd(srcName, ptr_src);

//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_removert(new pcl::PointCloud<pcl::PointXYZI>);
//    datahandle3d::load_pcd(removertName, ptr_removert);
    ////////////////////////////////////////////////////////////////////
//    pcl::PointCloud<pcl::PointXYZI> oursStatic, oursDynamic;
//    pcl::PointCloud<pcl::PointXYZI> removertStatic, removertDynamic;
//    pcl::PointCloud<pcl::PointXYZI> mapStatic, mapDynamic;

    pcl::PointCloud<pcl::PointXYZI> casTP, casFP, casFN, casTN;

    parse_pcd(*ptr_cas, casTP, casFP, casFN, casTN);

    auto cas_tp_msg = cvt::cloud2msg(casTP);
    auto cas_fp_msg = cvt::cloud2msg(casFP);
    auto cas_fn_msg = cvt::cloud2msg(casFN);
    auto cas_tn_msg = cvt::cloud2msg(casTN);

    ros::Rate loop_rate(2);
    static int count_ = 0;
    while (ros::ok())
    {
      casTP_pub.publish(cas_tp_msg);
      casFP_pub.publish(cas_fp_msg);
      casFN_pub.publish(cas_fn_msg);
      casTN_pub.publish(cas_tn_msg);
      if (++count_ %2 == 0) std::cout<<"On "<<count_<<"th publish!"<<std::endl;

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
