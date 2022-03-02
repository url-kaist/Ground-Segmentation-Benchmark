#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>
#include "common.hpp"
#include <gseg_benchmark/node.h>
#include <iostream>
ros::Publisher  UnlabeledPub;
ros::Publisher  OutlierPub;
ros::Publisher  RoadPub;
ros::Publisher  ParkingPub;
ros::Publisher  SideWalkPub;
ros::Publisher  OGPub;
ros::Publisher  LanePub;
ros::Publisher  TerrainPub;
ros::Publisher  BuildingPub;
ros::Publisher  FensePub;
ros::Publisher  VegetationPub;
ros::Publisher  TrafficSignPub;

using namespace unavlib;


void callbackNode(const gseg_benchmark::node::ConstPtr& msg)
{
  static int count = 0;
  pcl::PointCloud<PointXYZILID> pc_curr = cvt::cloudmsg2cloud<PointXYZILID>(msg->lidar);
  pcl::PointCloud<PointXYZILID> pc_road, pc_parking, pc_sidewalk;
  pcl::PointCloud<PointXYZILID> pc_og, pc_lane, pc_terrain;
  pcl::PointCloud<PointXYZILID> pc_unlabeled, pc_outlier;
  pcl::PointCloud<PointXYZILID> pc_building, pc_fense, pc_vegetation, pc_traffic;

  for (const auto &pt: pc_curr.points){
//    if ( ( pt.x > -5.) && (pt.x < 5.) && (pt.y < -5) ) std::cout<<pt.label<<", ";
    if (pt.label == ROAD){
      pc_road.points.push_back(pt);
    }else if (pt.label == PARKING){
      pc_parking.points.push_back(pt);
    }else if (pt.label == SIDEWALKR){
      pc_sidewalk.points.push_back(pt);
    }else if (pt.label == OTHER_GROUND){
      pc_og.points.push_back(pt);
    }else if (pt.label == LANE_MARKING){
      pc_lane.points.push_back(pt);
    }else if (pt.label == TERRAIN){
      pc_terrain.points.push_back(pt);
    }else if (pt.label == UNLABELED){
      pc_unlabeled.points.push_back(pt);
    }else if (pt.label == OUTLIER){
      pc_outlier.points.push_back(pt);
    }else if (pt.label == BUILDING){
      pc_building.points.push_back(pt);
    }else if (pt.label == FENSE){
      pc_fense.points.push_back(pt);
    }else if (pt.label == VEGETATION){
      pc_vegetation.points.push_back(pt);
    }else if (pt.label == 81){
      pc_traffic.points.push_back(pt);
    }
  }
  std::cout<<msg->header.seq<<std::endl;

  // Publish msg
  UnlabeledPub.publish(cvt::cloud2msg(pc_unlabeled));
  OutlierPub.publish(cvt::cloud2msg(pc_outlier));
  RoadPub.publish(cvt::cloud2msg(pc_road));
  ParkingPub.publish(cvt::cloud2msg(pc_parking));
  SideWalkPub.publish(cvt::cloud2msg(pc_sidewalk));
  OGPub.publish(cvt::cloud2msg(pc_og));
  LanePub.publish(cvt::cloud2msg(pc_lane));
  TerrainPub.publish(cvt::cloud2msg(pc_terrain));

  BuildingPub.publish(cvt::cloud2msg(pc_building));
  FensePub.publish(cvt::cloud2msg(pc_fense));
  VegetationPub.publish(cvt::cloud2msg(pc_vegetation));
  TrafficSignPub.publish(cvt::cloud2msg(pc_traffic));
  ++count;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");
    std::cout<<"MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;
    UnlabeledPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/unlabeled", 10);
    OutlierPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/outlier", 10);
    RoadPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/road", 10);
    ParkingPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/parking", 10);
    SideWalkPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/sidewalk", 10);
    OGPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/other_ground", 10);
    LanePub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/lane", 10);
    TerrainPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/terrain", 10);

    BuildingPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/building", 10);
    FensePub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/fense", 10);
    VegetationPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/vegetation", 10);

    TrafficSignPub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/trafficSign", 10);

    ros::Subscriber NodeSubscriber = nodeHandler.subscribe<gseg_benchmark::node>("/node",1000, callbackNode);

    ros::spin();

    return 0;
}
