#pragma once

#include "../common.hpp"

/*Basic includes.*/
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>
#include <memory>

/*Includes for ROS.*/
#include <ros/ros.h>

/*Includes for Markers.*/
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*Includes for GUI.*/
#include <dynamic_reconfigure/server.h>
#include <gseg_benchmark/LidarFiltersConfig.h>

/*Includes for PCL.*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

/*ramer-douglas-peucker*/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

using namespace boost::assign;

typedef boost::geometry::model::d2::point_xy<float> xy;

struct Point2D{
    pcl::PointXYZI p;
    float d;
    float alpha;
    short isCurbPoint;
};

struct Point3D:public Point2D{
    float newY;
};

struct Polar    //polar-coordinate struct for the points
{
    int id;     //original ID of point (from input cloud)
    float r;    //radial coordinate
    float fi;   //angular coordinate (ccw angle from x-axis)
};

struct box      //struct for detection beams
{
    std::vector<Polar> p; //points within the beam's area
    box *l, *r;           //pointer to adjacent beams (currently not used)
    bool yx;              //whether it is aligned more with the y-axis (than the x-axis)
    float o, d;           //internal parameters (trigonometry)
};

int         channels = 64;                  //The number of channels of the LIDAR .
int         ghostcount = 0;                 //counter variable helping to remove obsolete markers (ghosts)

namespace params{
    std::string fixedFrame;                               /* Fixed Frame.*/
    std::string topicName;                                /* subscribed topic.*/
    bool x_zero_method, z_zero_method, star_shaped_method ; /*Methods of roadside detection*/
    bool blind_spots;                                     /*Vakfolt javító algoritmus.*/
    int xDirection;                                       /*A vakfolt levágás milyen irányú.*/
    float interval;                                       /*A LIDAR vertikális szögfelbontásának, elfogadott intervalluma.*/
    float curbHeight;                                     /*Becsült minimum szegély magasság.*/
    int curbPoints;                                       /*A pontok becsült száma, a szegélyen.*/
    float beamZone;                                       /*A vizsgált sugárzóna mérete.*/
    float angleFilter1;                                   /*X = 0 érték mellett, három pont által bezárt szög.*/
    float angleFilter2;                                   /*Z = 0 érték mellett, két vektor által bezárt szög.*/
    float angleFilter3;                                   /*Csaplár László kódjához szükséges. Sugár irányú határérték (fokban).*/
    float min_X, max_X, min_Y, max_Y, min_Z, max_Z;       /*A vizsgált terület méretei.*/
    int dmin_param;                 //(see below)
    float kdev_param;               //(see below)
    float kdist_param;              //(see below)
    bool polysimp_allow;                           /*polygon-eygszerűsítés engedélyezése*/
    bool zavg_allow;                               /*egyszerűsített polygon z-koordinátái átlagból (engedély)*/
    float polysimp;                                 /*polygon-egyszerűsítési tényező (Ramer-Douglas-Peucker)*/
    float polyz;                                   /*manuálisan megadott z-koordináta (polygon)*/
};
/*For pointcloud filtering*/
template <typename PointT>
class FilteringCondition : public pcl::ConditionBase<PointT>
{
public:
  typedef std::shared_ptr<FilteringCondition<PointT>> Ptr;
  typedef std::shared_ptr<const FilteringCondition<PointT>> ConstPtr;
  typedef std::function<bool(const PointT&)> FunctorT;

  FilteringCondition(FunctorT evaluator): 
    pcl::ConditionBase<PointT>(),_evaluator( evaluator ) 
  {}

  virtual bool evaluate (const PointT &point) const {
    // just delegate ALL the work to the injected std::function
    return _evaluator(point);
  }
private:
  FunctorT _evaluator;
};

void paramsCallback(urban_road_filter::LidarFiltersConfig &config, uint32_t level){
    params::fixedFrame = config.fixed_frame;
    params::topicName = config.topic_name;
    params::x_zero_method = config.x_zero_method;
    params::z_zero_method = config.z_zero_method;
    params::star_shaped_method  = config.star_shaped_method ;
    params::blind_spots = config.blind_spots;
    params::xDirection = config.xDirection;
    params::interval = config.interval;
    params::curbHeight = config.curb_height;
    params::curbPoints = config.curb_points;
    params::beamZone = config.beamZone;
    params::angleFilter1 = config.cylinder_deg_x;
    params::angleFilter2 = config.cylinder_deg_z;
    params::angleFilter3 = config.sector_deg;
    params::min_X = config.min_x;
    params::max_X = config.max_x;
    params::min_Y = config.min_y;
    params::max_Y = config.max_y;
    params::min_Z = config.min_z;
    params::max_Z = config.max_z;
    params::dmin_param = config.dmin_param;
    params::kdev_param = config.kdev_param;
    params::kdist_param = config.kdist_param;
    params::polysimp_allow = config.simple_poly_allow;
    params::polysimp = config.poly_s_param;
    params::zavg_allow = config.poly_z_avg_allow;
    params::polyz = config.poly_z_manual;
    ROS_INFO("Updated params %s", ros::this_node::getName().c_str());
}

class Detector{
    public:
    Detector() {};
    Detector(ros::NodeHandle* nh);

    int partition(std::vector<std::vector<Point3D>>& array3D, int arc,int low, int high);

    void quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);

    void estimate_ground(pcl::PointCloud<PointXYZILID> &cloudIn,
                         pcl::PointCloud<PointXYZILID> &cloudOut,
                         pcl::PointCloud<PointXYZILID> &cloudNonground,
                         double &time_taken);

    //void filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void filtered( pcl::PointCloud<PointXYZILID> &cloudIn,
                   pcl::PointCloud<PointXYZILID> &cloudOut,
                   pcl::PointCloud<PointXYZILID> &cloudNonground);

    void starShapedSearch(std::vector<Point2D>& array2D);

    void beam_init();

    void xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);

    void zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);

    void blindSpots(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray,float* maxDistance);

    geometry_msgs::PolygonStamped set_plane_polygon(const MatrixXf &normal_v, const float &d);

    private:
    ros::Publisher pub_road;        
    ros::Publisher pub_high;        
    ros::Publisher pub_box;         
//    ros::Publisher pub_pobroad;
    ros::Publisher pub_marker;      

    ros::Subscriber sub;

    boost::geometry::model::linestring<xy> line;
    boost::geometry::model::linestring<xy> simplified;
};
void marker_init(visualization_msgs::Marker& m)
{
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;

    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;
}

inline std_msgs::ColorRGBA setcolor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

Detector::Detector(ros::NodeHandle* nh){
    /*subscribing to the given topic*/
//    sub = nh->subscribe(params::topicName, 1, &Detector::filtered,this);
    /*publishing filtered points*/
//    pub_road = nh->advertise<pcl::PCLPointCloud2>("road", 1);
//    pub_high = nh->advertise<pcl::PCLPointCloud2>("curb", 1);
//    pub_box = nh->advertise<pcl::PCLPointCloud2>("roi", 1); // ROI - region of interest
//    pub_pobroad = nh->advertise<pcl::PCLPointCloud2>("road_probably", 1);
    pub_marker = nh->advertise<visualization_msgs::MarkerArray>("road_marker", 1);

    Detector::beam_init();

    ROS_INFO("Ready");

}

/*recursive, quick sorting function (1/2)*/
int Detector::partition(std::vector<std::vector<Point3D>>& array3D, int arc,int low, int high)
{
    float pivot = array3D[arc][high].alpha;
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++){
        if (array3D[arc][j].alpha < pivot){
            i++;
            std::swap(array3D[arc][i],array3D[arc][j]);
        }
    }
    std::swap(array3D[arc][i+1],array3D[arc][high]);
    return (i + 1);
}
/*recursive, quick sorting function (2/2)*/
void Detector::quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high)
{
    if (low < high)
    {
        int pi = partition(array3D, arc, low, high);
        quickSort(array3D, arc, low, pi - 1);
        quickSort(array3D, arc, pi + 1, high);
    }
}

void Detector::estimate_ground(pcl::PointCloud<PointXYZILID> &cloudIn,
                     pcl::PointCloud<PointXYZILID> &cloudOut,
                     pcl::PointCloud<PointXYZILID> &cloudNonground,
                     double &time_taken) {
    pcl::PointCloud<PointXYZILID> nonground_points;
    pcl::PointCloud<PointXYZILID> ground_points;
    nonground_points.header = cloudIn.header;
    ground_points.header    = cloudIn.header;
    nonground_points.reserve(200000);
    ground_points.reserve(200000);

    auto start = chrono::high_resolution_clock::now();


    dynamic_reconfigure::Server<urban_road_filter::LidarFiltersConfig> server;
    dynamic_reconfigure::Server<urban_road_filter::LidarFiltersConfig>::CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh;
    Detector detector(&nh);
    detector.filtered(cloudIn, nonground_points, ground_points);

    cloudOut       = ground_points;
    cloudNonground = nonground_points;

   // cout<<"out size: "<<cloudOut.size()<<", non size: "<<cloudNonground.size()<<endl;

    auto end = chrono::high_resolution_clock::now();
    time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;
}

void Detector::filtered( pcl::PointCloud<PointXYZILID> &cloudIn_xyzilid,
                         pcl::PointCloud<PointXYZILID> &cloudOut,
                         pcl::PointCloud<PointXYZILID> &cloudNonground) {
    /*variables for the "for" loops*/
    int i, j, k, l;
    pcl::PointCloud<pcl::PointXYZI> cloudIn;
    PointXYZILID2XYZI(cloudIn_xyzilid, cloudIn);

    PointXYZILID pt;                                                                      //temporary variable for storing a point
    auto cloud_filtered_Box = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloudIn);   //all points in the detection area
    pcl::PointCloud<PointXYZILID> cloud_filtered_Road;                                    //filtered points (driveable road)
//    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_ProbablyRoad;                            //filtered points (non-driveable road)
    pcl::PointCloud<PointXYZILID> cloud_filtered_High;                                    //filtered points (non-road)

    auto filterCondition = boost::make_shared<FilteringCondition<pcl::PointXYZI>>(
            [=](const pcl::PointXYZI& point){
                return point.x >= params::min_X && point.x <= params::max_X &&
                       point.y >= params::min_Y && point.y <= params::max_Y &&
                       point.z >= params::min_Z && point.z <= params::max_Z &&
                       point.x + point.y + point.z != 0;
            }
    );
    pcl::ConditionalRemoval<pcl::PointXYZI> condition_removal;
    condition_removal.setCondition(filterCondition);
    condition_removal.setInputCloud(cloud_filtered_Box);
    condition_removal.filter(*cloud_filtered_Box);

    /*number of points in the detection area*/
    size_t piece = cloud_filtered_Box->points.size();

    /*A minimum of 30 points are requested in the detection area to avoid errors.
    Also, there is not much point in evaluating less data than that.*/
    if (piece < 30){
        return;
    }

    std::vector<Point2D> array2D(piece);

    /*variable for storing the input for trigonometric functions*/
    float bracket;

    /*A 1D array containing the various angular resolutions.
    This equals to the number of LiDAR channels.
    It is important to fill it with 0 values.*/
    float angle[channels] = {0};

    /*This helps to fill the 1D array containing the angular resolutions.*/
    int index = 0;

    /*whether the given angle corresponds to a new arc*/
    int newCircle;

    /*filling the 2D array*/
    for (i = 0; i < piece; i++){
        /*--- filling the first 4 columns ---*/
        array2D[i].p = cloud_filtered_Box->points[i];
        array2D[i].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2) + pow(array2D[i].p.z, 2));

        /*--- filling the 5. column ---*/
        bracket = abs(array2D[i].p.z) / array2D[i].d;

        /*required because of rounding errors*/
        if (bracket < -1)
            bracket = -1;
        else if (bracket > 1)
            bracket = 1;

        /*calculation and conversion to degrees*/
        if (array2D[i].p.z < 0)
        {
            array2D[i].alpha = acos(bracket) * 180 / M_PI;
        }
        else{
            array2D[i].alpha = (asin(bracket) * 180 / M_PI) + 90;
        }

        /*setting the index*/
        /*Our basic assumption is that the angle corresponds to a new circle/arc.*/
        newCircle = 1;

        /*If this value has already occured (within the specified interval), then this is not a new arc.
        Which means that "newCircle = 0", we can exit the loop, no further processing required.*/
        for (j = 0; j < channels; j++)
        {
            if (angle[j] == 0)
                break;

            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
            {
                newCircle = 0;
                break;
            }
        }

        /*If no such value is registered in the array, then it's a new circle/arc.*/
        if (newCircle == 1)
        {
            /*We cannot allow the program to halt with a segmentation fault error.
            If for any reason there would come to be more than 64 arcs/circles, an error would occur.*/
            if (index < channels)
            {
                angle[index] = array2D[i].alpha;
                index++;
            }
        }
    }
    /*calling starShapedSearch algorithm*/
    if (params::star_shaped_method )
        Detector::starShapedSearch(array2D);


    /*Sorting the angular resolutions by ascending order...
    The smallest will be the first arc, etc..*/
    std::sort(angle, angle + index);

    std::vector<std::vector<Point3D>> array3D(channels,std::vector<Point3D>(piece));

    /*This is required to set up the row indices of
    the groups ("channels") containing the arcs.
    It is important to fill it with 0 values.*/
    int indexArray[channels] = {0};

    /*A 1D array. The values of points that have the greatest distance from the origo.*/
    float maxDistance[channels] = {0};

    /*variable helping to handle errors caused by wrong number of channels.*/
    int results;

    /*filling the 3D array*/
    for (i = 0; i < piece; i++)
    {
        results = 0;

        /*selecting the desired arc*/
        for (j = 0; j < index; j++)
        {
            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
            {
                results = 1;
                break;
            }
        }

        if (results == 1)
        {
            /*assigning values from the 2D array*/
            array3D[j][indexArray[j]].p = array2D[i].p;

            /*the known "high" points*/
            if (params::star_shaped_method )
                array3D[j][indexArray[j]].isCurbPoint = array2D[i].isCurbPoint;

            /*The only difference here is that the distance is calculated in 2D - with no regard to the 'z' value.*/
            array3D[j][indexArray[j]].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2));

            /*filling the 5. column with the angular position of points, in degrees.*/
            bracket = (abs(array3D[j][indexArray[j]].p.x)) / (array3D[j][indexArray[j]].d);
            if (bracket < -1)
                bracket = -1;
            else if (bracket > 1)
                bracket = 1;

            if (array3D[j][indexArray[j]].p.x >= 0 && array3D[j][indexArray[j]].p.y <= 0)
            {
                array3D[j][indexArray[j]].alpha = asin(bracket) * 180 / M_PI;
            }
            else if (array3D[j][indexArray[j]].p.x >= 0 && array3D[j][indexArray[j]].p.y > 0)
            {
                array3D[j][indexArray[j]].alpha = 180 - (asin(bracket) * 180 / M_PI);
            }
            else if (array3D[j][indexArray[j]].p.x < 0 && array3D[j][indexArray[j]].p.y >= 0)
            {
                array3D[j][indexArray[j]].alpha = 180 + (asin(bracket) * 180 / M_PI);
            }
            else
            {
                array3D[j][indexArray[j]].alpha = 360 - (asin(bracket) * 180 / M_PI);
            }

            if (array3D[j][indexArray[j]].d > maxDistance[j])
            {
                maxDistance[j] = array3D[j][indexArray[j]].d;
            }

            indexArray[j]++;
        }
    }

    if(params::x_zero_method)
        Detector::xZeroMethod(array3D,index,indexArray);
    if(params::z_zero_method)
        Detector::zZeroMethod(array3D,index,indexArray);

    float d;

    /*-- step 2.: filtering road points --*/
    /*ordering the elements of the array by angle on every arc*/
    for (i = 0; i < index; i++){
        quickSort(array3D, i, 0, indexArray[i] - 1);
    }
    /*blindspot detection*/
    Detector::blindSpots(array3D,index,indexArray,maxDistance);

    /*-- step 3: searching for marker points - the farthest green point within the given angle --*/
    /*It contains the points of the marker. The first three columns contain the X - Y - Z coordinates
    and the fourth column contains value 0 or 1 depending on whether there is a point within the given angle that is not marked as road.*/
    float markerPointsArray[piece][4];
    float maxDistanceRoad;              //the distance of the farthest green point within the given angle
    int cM = 0;                         //variable helping to fill the marker with points (c - counter, M - Marker)
    int ID1, ID2;                       //which arc does the point fall onto (ID1) and (ordinally) which point is it (ID2)
    int redPoints;                      //whether there is a high point in the examined segment or a point that has not been marked as either road or high point

    /*checking the points by 1 degree at a time*/
    for (i = 0; i <= 360; i++)
    {
        ID1 = -1;
        ID2 = -1;
        maxDistanceRoad = 0;
        redPoints = 0;

        /*iterating through all the points of all the arcs*/
        for (j = 0; j < index; j++)
        {
            for (k = 0; k < indexArray[j]; k++)
            {
                /*If a non-road point is found, then we break the loop, because there will not be a road point found later on and value 1 will be assigned to the variable "redPoints".*/
                if (array3D[j][k].isCurbPoint != 1 && array3D[j][k].alpha >= i && array3D[j][k].alpha < i + 1)
                {
                    redPoints = 1;
                    break;
                }

                /*checking the distance for the detected green point*/
                if (array3D[j][k].isCurbPoint == 1 && array3D[j][k].alpha >= i && array3D[j][k].alpha < i + 1)
                {
                    d = sqrt(pow(0 - array3D[j][k].p.x, 2) + pow(0 - array3D[j][k].p.y, 2));

                    if (d > maxDistanceRoad)
                    {
                        maxDistanceRoad = d;
                        ID1 = j;
                        ID2 = k;
                    }
                }
            }
            /*The previous "break" was used to exit the current circle, this one will exit all of them and proceed to the next angle.*/
            if (redPoints == 1)
                break;
        }

        /*adding the marker points to the array*/
        if (ID1 != -1 && ID2 != -1)
        {
            markerPointsArray[cM][0] = array3D[ID1][ID2].p.x;
            markerPointsArray[cM][1] = array3D[ID1][ID2].p.y;
            markerPointsArray[cM][2] = array3D[ID1][ID2].p.z;
            markerPointsArray[cM][3] = redPoints;
            cM++;
        }
    }

    /*-- step 4.: filling the groups --*/
    for (i = 0; i < index; i++)
    {
        for (j = 0; j < indexArray[i]; j++)
        {
            pcl::PointXYZI pt_tmp = array3D[i][j].p;
            pt.x = pt_tmp.x;
            pt.y = pt_tmp.y;
            pt.z = pt_tmp.z;
            pt.intensity = pt_tmp.intensity;

            /*road points*/
            if (array3D[i][j].isCurbPoint == 1) {
                pt.label = 1;
                cloud_filtered_Road.push_back(pt);
            }

                /*high points*/
            else if (array3D[i][j].isCurbPoint == 2) {
                pt.label = 0;
                cloud_filtered_High.push_back(pt);
            }
        }
    }

    /*-- step 5.: setting up the marker --*/
    /*There need to be at least 3 points to connect, otherwise errors might occur.*/
    if (cM > 2)
    {
        /*There might be a case where points are in red-green-red (or the other way around) order next to each other.
        This is bad is because the green / red marker (line strip) in this case will only consist of 1 point.
        This is not recommended, every point needs to have a pair of the same color.
        If the 3. column of "markerPointsArray" has the value 1 then it belongs to the red line strip,
        otherwise it belongs to the green one.*/

        /*If the first point is green but the second one is red,
        then the first one will be added to the red line strip too.*/
        if (markerPointsArray[0][3] == 0 && markerPointsArray[1][3] == 1)
            markerPointsArray[0][3] = 1;

        /*If the last point is green but the second to last is red,
        then the last one will be added to the red line strip too.*/
        if (markerPointsArray[cM - 1][3] == 0 && markerPointsArray[cM - 2][3] == 1)
            markerPointsArray[cM - 1][3] = 1;

        /*If the first point is red but the second one is green,
        then the first one will be added to the green line strip too.*/
        if (markerPointsArray[0][3] == 1 && markerPointsArray[1][3] == 0)
            markerPointsArray[0][3] = 0;

        /*If the last point is red but the second to last is green,
        then the last one will be added to the green line strip too.*/
        if (markerPointsArray[cM - 1][3] == 1 && markerPointsArray[cM - 2][3] == 0)
            markerPointsArray[cM - 1][3] = 0;

        /*Here we iterate through all the points.
        If a green point gets between two red ones, then it will be added to the red line strip too.
        The first two and last two points are not checked - they were already set before.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 0 && markerPointsArray[i - 1][3] == 1 && markerPointsArray[i + 1][3] == 1)
                markerPointsArray[i][3] = 1;
        }

        /*Here we iterate through all the points.
        If a red point gets between two green ones, then it will be added to the green line strip too.
        The first two and last two points are not checked - they were already set before.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 1 && markerPointsArray[i - 1][3] == 0 && markerPointsArray[i + 1][3] == 0)
                markerPointsArray[i][3] = 0;
        }

        visualization_msgs::MarkerArray ma;     //a marker array containing the green / red line strips
        visualization_msgs::Marker line_strip;  //the current green or red section / line strip
        geometry_msgs::Point point;             //point to fill the line strip with
        float zavg = 0.0;                       //average z value (for the simplified polygon)

        int lineStripID = 0;                    //ID of the given line strip

        line_strip.header.frame_id = params::fixedFrame;
        line_strip.header.stamp = ros::Time();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;

        /*We iterate through the points which will make up the marker.*/
        for (i = 0; i < cM; i++)
        {
            /*adding the given point to a "geometry_msgs::Point" type variable*/
            point.x = markerPointsArray[i][0];
            point.y = markerPointsArray[i][1];
            point.z = markerPointsArray[i][2];
            zavg *= i;
            zavg += point.z;
            zavg /= i+1;

            /*Adding the first point to the current line strip.
            No conditions need to be met for the first point.*/
            if (i == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }

                /*If the next point is from the same group (red or green) as the previous one
                then it will be added to the line strip aswell.*/
            else if (markerPointsArray[i][3] == markerPointsArray[i - 1][3])
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*In this "else if" section we will reach the last point and the last line strip will be created.*/
                if (i == cM - 1)
                {
                    line_strip.id = lineStripID;
                    marker_init(line_strip);

                    /*setting the color of the line strip*/
                    if (markerPointsArray[i][3] == 0)
                    {
                        line_strip.color = setcolor(0.0, 1.0, 0.0, 1.0); //green
                    }
                    else
                    {
                        line_strip.color = setcolor(1.0, 0.0, 0.0, 1.0); //red
                    }

                    if (params::polysimp_allow)
                    {
                        line_strip.points.clear();
                        boost::geometry::clear(simplified);
                        boost::geometry::simplify(line, simplified, params::polysimp);
                        for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                        {
                            geometry_msgs::Point p;
                            p.x = boost::geometry::get<0>(*it);
                            p.y = boost::geometry::get<1>(*it);
                            p.z = params::polyz;

                            line_strip.points.push_back(p);
                        }
                    }

                    ma.markers.push_back(line_strip); //adding the line strip to the marker array
                    line_strip.points.clear();        //We clear the points from the last line strip as there's no need for them anymore.
                    boost::geometry::clear(line);
                }
            }

                /*change of category: red -> green
                The line joining the two points is still red, so we add the point to the given line strip.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*The following points belong to a new line strip - a red one is being made here.*/
                line_strip.id = lineStripID;
                lineStripID++;

                marker_init(line_strip);

                line_strip.color = setcolor(1.0, 0.0, 0.0, 1.0); //red

                if (params::polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, params::polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = params::polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip);   //adding the line strip to the marker array
                line_strip.points.clear();          //the points are not needed anymore
                boost::geometry::clear(line);
                line_strip.points.push_back(point); //This point is needed for the next line strip aswell, so we add it.
                line += xy(point.x,point.y);
            }

                /*change of category: green -> red
                First we set up the green line strip, then we add the last point to the red one aswell,
                since there is always a red line strip between a green and a red point.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 1)
            {
                /*the green marker*/
                line_strip.id = lineStripID;
                lineStripID++;

                marker_init(line_strip);

                line_strip.color = setcolor(0.0, 1.0, 0.0, 1.0); //green

                if (params::polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, params::polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = params::polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip);   //adding the line strip to the marker array
                line_strip.points.clear();          //These points are not needed anymore.
                boost::geometry::clear(line);

                /*The previous point is required for the next line strip aswell.*/
                point.x = markerPointsArray[i - 1][0];
                point.y = markerPointsArray[i - 1][1];
                point.z = markerPointsArray[i - 1][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*The current point is required for the next line strip aswell.*/
                point.x = markerPointsArray[i][0];
                point.y = markerPointsArray[i][1];
                point.z = markerPointsArray[i][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }
            line_strip.lifetime = ros::Duration(0);
        }
        if (params::zavg_allow)
        {
            for (int seg=0; seg < ma.markers.size(); seg++)
            {
                for (int mz = 0; mz < ma.markers[seg].points.size(); mz++)  //setting the height of the polygon from the average height of points
                {
                    ma.markers[seg].points[mz].z = zavg;
                }
            }
        }

        /*removal of obsolete markers*/
        line_strip.action = visualization_msgs::Marker::DELETE;
        for (int del = lineStripID; del<ghostcount; del++)
        {
            line_strip.id++;
            ma.markers.push_back(line_strip);
        }
        ghostcount = lineStripID;

        /*publishing the marker array*/
        pub_marker.publish(ma);
    }


    for (j = 0; j < indexArray[10]; j++){
        pcl::PointXYZI pt_tmp = array3D[10][j].p;
        pt.x = pt_tmp.x;
        pt.y = pt_tmp.y;
        pt.z = pt_tmp.z;
        pt.intensity = pt_tmp.intensity;

        pt.label = 1;
        cloud_filtered_Road.push_back(pt);
        //cloud_filtered_ProbablyRoad.push_back(pt);
    }


   // cout<<"road size: "<<cloud_filtered_Road.size()<<"| size high: "<<cloud_filtered_High.size()<<endl;
    /*Road and High topic header*/
    cloud_filtered_Road.header = cloudIn.header;
//    cloud_filtered_ProbablyRoad.header = cloudIn.header;
    cloud_filtered_High.header = cloudIn.header;
    cloud_filtered_Box->header = cloudIn.header;

    cloudOut = cloud_filtered_Road;// + cloud_filtered_ProbablyRoad;
    cloudNonground = cloud_filtered_High;

    /*publishing*/
//    pub_road.publish(cloud_filtered_Road);  //filtered points (driveable road)
//    pub_high.publish(cloud_filtered_High);  //filtered points (non-driveable road)
//    pub_box.publish(cloud_filtered_Box);    //filtered points (non-road)
//    pub_pobroad.publish(cloud_filtered_ProbablyRoad);
}

/*  starShapedSearch algorithm
    (by László Csaplár)

    description: a complementary algorithm for roadside detection, part of the "urban_road_filter" package
*/

int rep = 360;                  //number of detection beams (how many parts/sectors will the pointcloud be divided along the angular direction -> one beam per sector)
float width = 0.2;              //width of beams
float Kfi;                      //internal parameter, for assigning the points to their corresponding sectors ( = 1 / [2pi/rep] = 1 / [angle between neighboring beams] )
float slope_param;              //"slope" parameter for edge detection (given by 2 points, in radial direction/plane)

std::vector<box> beams(rep);        //beams
std::vector<box *> beamp(rep + 1);  //pointers to the beams (+1 -> 0 AND 360)

bool ptcmpr(Polar a, Polar b)   //comparison by r-coordinates
{
    return (a.r < b.r);
}

float slope(float x0, float y0, float x1, float y1) //get slope between 2 points given by x and y coordinates
{
    return (y1 - y0) / (x1 - x0);
}

void Detector::beam_init()    //beam initialization
{
    {
        float fi, off = 0.5 * width;    //temporary variables
        for (int i = 0; i < rep; i++)   //for every beam...
        {
            fi = i * 2 * M_PI / rep;    //angle of beam
            if (abs(tan(fi)) > 1)       //"slope" of beam ( x <---> y )
            {
                beams[i].yx = true;                 //aligning more with y-direction
                beams[i].d = tan(0.5 * M_PI - fi);  // = 1/tan(fi) [coefficient]
                beams[i].o = off / sin(fi);         //projection of half beam-width in the x-direction (how far from its centerline is the edge of the beam in the x-dir)
            }
            else
            {
                beams[i].yx = false;        //aligning more with x-direction
                beams[i].d = tan(fi);       //coefficient
                beams[i].o = off / cos(fi); //y-axis projection of half beam-width
            }
            beamp[i] = &beams[i];   //initializing the pointers
        }
    }

    for (int i = 0, j = 1; j < rep; i++, j++)   //initializing the pointers to adjacent beams
    {
        beams[i].l = &beams[j];
        beams[j].r = &beams[i];
    }
    beams[0].r = &beams[rep];
    beams[rep].l = &beams[0];

    Kfi = rep / (2 * M_PI); //should be 2pi/rep, but then we would have to divide by it every time - using division only once and then multiplying later on should be somewhat faster (?)
}

void beamfunc(const int tid, std::vector<Point2D> &array2D) //beam algorithm (filtering, sorting, edge-/roadside detection) - input: beam ID (ordinal position/"which one" by angle), pointcloud (as std::vector<Point2D>, see: 'array2D' of 'lidarSegmentation')
{
    int i = 0, s = beams[tid].p.size(); //loop variables
    float c;                            //temporary variable to simplify things

    if (beams[tid].yx)  //filtering the points lying outside the area of the beam... (case 1/2, y-direction)
    {
        while (i < s)   //iterating through points in the current sector (instead of a for loop - since 's' is not constant and 'i' needs to be incremented conditionally)
        {
            c = abs(beams[tid].d * array2D[beams[tid].p[i].id].p.y);                        //x-coordinate of the beam's centerline at the point (at the "height" of its y-coordinate)
            if ((c - beams[tid].o) < array2D[beams[tid].p[i].id].p.x < (c + beams[tid].o))  //whether it is inside the beam (by checking only x values on the line/"height" of the point's y-coordinate: whether the [x-coordinate of the] point falls between the [x-coordinates of the] two sides/borders of the beam
            {
                i++;    //okay, next one
            }
            else    //if outside the area
            {
                beams[tid].p.erase(beams[tid].p.begin() + i);   //remove point
                s--;                                            //the size has shrunk because of the deletion of a point (and its place is taken by the next point, so 'i' does not need to be changed)
            }
        }
    }
    else    //same but with x and y swapped (case 2/2, x-direction)
    {
        while (i < s)
        {
            c = abs(beams[tid].d * array2D[beams[tid].p[i].id].p.x);
            if ((c - beams[tid].o) < array2D[beams[tid].p[i].id].p.y < (c + beams[tid].o))
            {
                i++;
            }
            else
            {
                beams[tid].p.erase(beams[tid].p.begin() + i);
                s--;
            }
        }
    }

    std::sort(beams[tid].p.begin(), beams[tid].p.end(), ptcmpr);    //sorting points by r-coordinate (radius)

    {               //edge detection (edge of the roadside)
        if (s > 1)  //for less than 2 points it would be pointless
        {
            int dmin = params::dmin_param;      //minimal number of points required to begin adaptive evaluation
            float kdev = params::kdev_param;    //coefficient: weighting the impact of deviation (difference) from average ( = overall sensitivity to changes)
            float kdist = params::kdist_param;  //coefficient: weighting the impact of the distance between points ( = sensitivity for height error at close points)

            float avg = 0, dev = 0, nan = 0;            //average value and absolute average deviation of the slope for adaptive detection + handling Not-a-Number values
            float ax, ay, bx, by, slp;                  //temporary variables (points 'a' and 'b' + slope)
            bx = beams[tid].p[0].r;                     //x = r-coordinate of the first point (radial position)
            by = array2D[beams[tid].p[0].id].p.z;   //y = z-coordinate of the first point (height)

            for (int i = 1; i < s; i++) //edge detection based on the slope between point a and b
            {                           //updating points (a=b, b=next)
                ax = bx;
                bx = beams[tid].p[i].r;
                ay = by;
                by = array2D[beams[tid].p[i].id].p.z;
                slp = slope(ax, ay, bx, by);

                if (isnan(slp))
                    nan++;  //Not-a-Number correction
                else        //calculating (updating) average and deviation
                {
                    avg *= i - nan - 1;     //"unpacking" average value (average -> sum, + NaN correction)
                    avg += slp;             //insertion of new element
                    avg *= 1 / (i - nan);   //sum -> average
                    dev *= i - nan - 1;     //Absolute Average Deviation -> sum ("unpacking")
                    dev += abs(slp - avg);  //insertion of new element (absolute difference from average)
                    dev *= 1 / (i - nan);   //sum -> AAD
                }
                if  ( slp > slope_param ||                                                          //evaluation of the slope -> using a constant + adaptive:
                      (i > dmin && (slp * slp - avg * avg) * kdev * ((bx - ax) * kdist) > dev)    //if sufficient number of points: does the (weighted) "squared difference" from average - corrected with...
                        )                                                                               //... the (weighted) distance of adjacent points - exceed the average absolute deviation?
                {
                    array2D[beams[tid].p[i].id].isCurbPoint = 2;    //the point in the 2D array gets marked as curbpoint
                    break;                                          //(the roadside is found, time to break the loop)
                }
            }
        }
    }
    beams[tid].p.clear();   //evaluation done, the points are no longer needed
}

void Detector::starShapedSearch(std::vector<Point2D> &array2D)  //entry point to the code, everything gets called here (except for initialization - that needs to be called separately, at the start of the program - "beam_init()")
{
    beamp.push_back(&beams[0]);     //initializing "360 deg = 0 deg" pointer
    int f, s = array2D.size();   //temporary variables
    float r, fi;                    //polar coordinates
    slope_param = params::angleFilter3 * (M_PI / 180);

    for (int i = 0; i < s; i++) //points to polar coordinate-system + sorting into sectors
    {
        r = sqrt(array2D[i].p.x * array2D[i].p.x + array2D[i].p.y * array2D[i].p.y);    //r = sqRoot(x^2+y^2) = distance of point from sensor

        fi = atan2(array2D[i].p.y, array2D[i].p.x);   //angular position of point

        if (fi < 0)
            fi += 2 * M_PI;     //handling negative values (-180...+180 -> 0...360)

        f = (int)(fi * Kfi);    //which one of the beams (sectors, containing the beam) does it fall into

        beamp[f]->p.push_back(Polar{i, r, fi}); //adding the point to the 'f'-th beam (still unfiltered)
    }
    beamp.pop_back();   //removing pointer (to prevent "double free" error)

    for (int i = 0; i < rep; i++)   //for every beam...
    {
        beamfunc(i, array2D);  //the heart of the starshaped method (beam algorithm)
    }
}

void Detector::xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray) {
    /*-- step 1.: filtering the NON-road points --*/
    int p2, p3;     //2nd and 3rd of the points that are being examined

    /*
    --> alpha - angle between the three points and the two vectors
    --> x1, x2, x3 - length of the sides of the triangle given by the three points
    --> curbPoints - estimated number of points on the roadside
    --> va1, va2, vb1, vb2 - the two vectors
    --> max1, max2 - not only angle, but height needs to be checked as well
    --> d - distance between the two outermost points - the rotation of the LiDAR and arc gaps make it necessary
        d - this variable also gets used later on in the code
    */
    float alpha, x1, x2, x3, va1, va2, vb1, vb2, max1, max2, d, bracket;
    for (int i = 0; i < index; i++) {
        /*assigning new Y values while keeping X = 0 */
        for (int j = 1; j < indexArray[i]; j++) {
            array3D[i][j].newY = array3D[i][j - 1].newY + 0.0100;
        }

        /*evaluation of the points in an arc - x-zero method*/
        for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++) {
            p2 = j + params::curbPoints / 2;
            p3 = j + params::curbPoints;

            d = sqrt(
                    pow(array3D[i][p3].p.x - array3D[i][j].p.x, 2) +
                    pow(array3D[i][p3].p.y - array3D[i][j].p.y, 2));

            /*set the distance to be less than 5 meters*/
            if (d < 5.0000) {
                x1 = sqrt(
                        pow(array3D[i][p2].newY - array3D[i][j].newY, 2) +
                        pow(array3D[i][p2].p.z - array3D[i][j].p.z, 2));
                x2 = sqrt(
                        pow(array3D[i][p3].newY - array3D[i][p2].newY, 2) +
                        pow(array3D[i][p3].p.z - array3D[i][p2].p.z, 2));
                x3 = sqrt(
                        pow(array3D[i][p3].newY - array3D[i][j].newY, 2) +
                        pow(array3D[i][p3].p.z - array3D[i][j].p.z, 2));

                bracket = (pow(x3, 2) - pow(x1, 2) - pow(x2, 2)) / (-2 * x1 * x2);
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * 180 / M_PI;

                /*condition and assignment to group*/
                if (alpha <= params::angleFilter1 &&
                    (abs(array3D[i][j].p.z - array3D[i][p2].p.z) >= params::curbHeight ||
                     abs(array3D[i][p3].p.z - array3D[i][p2].p.z) >= params::curbHeight) &&
                    abs(array3D[i][j].p.z - array3D[i][p3].p.z) >= 0.05) {
                    array3D[i][p2].isCurbPoint = 2;
                }
            }
        }
    }
}
void Detector::zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray){
    /*-- step 1.: filtering the NON-road points --*/
    int p2, p3;     //2nd and 3rd of the points that are being examined

    /*
    --> alpha - angle between the three points and the two vectors
    --> x1, x2, x3 - length of the sides of the triangle given by the three points
    --> curbPoints - estimated number of points on the roadside
    --> va1, va2, vb1, vb2 - the two vectors
    --> max1, max2 - not only angle, but height needs to be checked as well
    --> d - distance between the two outermost points - the rotation of the LiDAR and arc gaps make it necessary
        d - this variable also gets used later on in the code
    */
    float alpha, x1, x2, x3, va1, va2, vb1, vb2, max1, max2, d, bracket;
    for (int i = 0; i < index; i++){
        /*evaluation of points in an arc - z-zero method*/
        for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
        {
            d = sqrt(
                    pow(array3D[i][j+params::curbPoints].p.x - array3D[i][j-params::curbPoints].p.x, 2) +
                    pow(array3D[i][j+params::curbPoints].p.y - array3D[i][j-params::curbPoints].p.y, 2));

            /*set the distance to be less than 5 meters*/
            if (d < 5.0000)
            {
                /*initialization*/
                max1 = max2 = abs(array3D[i][j].p.z);
                va1 = va2 = vb1 = vb2 = 0;

                /*initializing vector 'a' and maximal height*/
                for (int k = j - 1; k >= j - params::curbPoints; k--)
                {
                    va1 = va1 + (array3D[i][k].p.x - array3D[i][j].p.x);
                    va2 = va2 + (array3D[i][k].p.y - array3D[i][j].p.y);
                    if (abs(array3D[i][k].p.z) > max1)
                        max1 = abs(array3D[i][k].p.z);
                }

                /*initializing vector 'b' and maximal height*/
                for (int k = j + 1; k <= j + params::curbPoints; k++)
                {
                    vb1 = vb1 + (array3D[i][k].p.x - array3D[i][j].p.x );
                    vb2 = vb2 + (array3D[i][k].p.y  - array3D[i][j].p.y );
                    if (abs(array3D[i][k].p.z ) > max2)
                        max2 = abs(array3D[i][k].p.z);
                }

                va1 = (1 / (float)params::curbPoints) * va1;
                va2 = (1 / (float)params::curbPoints) * va2;
                vb1 = (1 / (float)params::curbPoints) * vb1;
                vb2 = (1 / (float)params::curbPoints) * vb2;

                bracket = (va1 * vb1 + va2 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * 180 / M_PI;

                /*condition and assignment to group*/
                if (alpha <= params::angleFilter2 &&
                    (max1 - abs(array3D[i][j].p.z ) >= params::curbHeight ||
                     max2 - abs(array3D[i][j].p.z) >= params::curbHeight) &&
                    abs(max1 - max2) >= 0.05)
                {
                    array3D[i][j].isCurbPoint = 2;
                }
            }
        }
    }
}

void Detector::blindSpots(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray,float* maxDistance){
    /*Blind spot detection:
    We examine the second arc. (First one gives inaccurate results.)
    The intervals (segments) [90°-180° --- 180°-270°] and [0°-90° --- 270°-360°] are of greatest significance (at the two sides of the car).
    We search for 2 pieces of high points in both intervals.
    If there are two high points on the first arc in the interval, then the area between has a high chance of being a blind spot.*/
    float q1 = 0, q2 = 180, q3 = 180, q4 = 360; //the four segments (quarters) of the arc
    int c1 = -1, c2 = -1, c3 = -1, c4 = -1;     //ID of the points found on the first arc
    int i,j,k,l;                                //"temporary" variables

    if (params::blind_spots)
    {
        for (i = 0; i < indexArray[1]; i++)
        {
            if(array3D[1][i].isCurbPoint==2)
            {
                if (array3D[1][i].alpha >= 0 && array3D[1][i].alpha < 90)
                {
                    if (array3D[1][i].alpha > q1)
                    {
                        q1 = array3D[1][i].alpha;
                        c1 = i;
                    }
                }
                else if (array3D[1][i].alpha >= 90 && array3D[1][i].alpha < 180)
                {
                    if (array3D[1][i].alpha < q2)
                    {
                        q2 = array3D[1][i].alpha;
                        c2 = i;
                    }
                }
                else if (array3D[1][i].alpha >= 180 && array3D[1][i].alpha < 270)
                {
                    if (array3D[1][i].alpha > q3)
                    {
                        q3 = array3D[1][i].alpha;
                        c3 = i;
                    }
                }
                else
                {
                    if (array3D[1][i].alpha < q4)
                    {
                        q4 = array3D[1][i].alpha;
                        c4 = i;
                    }
                }
            }
        }
    }

    float arcDistance;      //arc length at the given angle - It is important to use the same arc length to examine every arc.
    int notRoad;            //If there is a high point in the given segment on the given arc, then value 1 will be assigned to it, 0 otherwise.
    int blindSpot;          //blind spots by the car
    float currentDegree;    //the angle on the current arc

    /*determining arc length*/
    arcDistance = ((maxDistance[0] * M_PI) / 180) * params::beamZone;

    /*from 0° to [360° - beamZone]*/
    for (i = 0; i <= 360 - params::beamZone; i++)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*If these conditions are met, then we have reached a blind spot and we stop checking.*/
            if (params::xDirection == 0)
            {
                /*evaluating the points in both directions (+-X)*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*evaluating the points in +X direction.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*evaluating the points in -X direction.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*By default settings there's no high point in the given segment.*/
            notRoad = 0;

            /*evaluation of the given segment of the first arc*/
            for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
            {
                if (array3D[0][j].alpha >= i)
                {
                    /*The segment needs no further checking if a high point is found.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*If no high point is found in the given segment of the first arc, we can proceed to the next arc.*/
            if (notRoad == 0)
            {
                /*We accept the segment of the first arc.*/
                for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
                {
                    if (array3D[0][j].alpha >= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*checking the rest of the arcs*/
                for (k = 1; k < index; k++)
                {
                    /*A new angle needs to be defined to get the same arc length at every radius.*/
                    if (i == 360 - params::beamZone)
                    {
                        currentDegree = 360;
                    }
                    else
                    {
                        currentDegree = i + arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*checking the points of the new arc*/
                    for (l = 0; array3D[k][l].alpha <= currentDegree && l < indexArray[k]; l++)
                    {
                        if (array3D[k][l].alpha >= i)
                        {
                            /*No further processing is needed if a high point is found within the segment.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*The rest of the arcs do not need to be checked if the beam stops at a high point.*/
                    if (notRoad == 1)
                        break;

                    /*else: the given segment of the given arc is accepted*/
                    for (l = 0; array3D[k][l].alpha <= currentDegree && l < indexArray[k]; l++)
                    {
                        if (array3D[k][l].alpha >= i)
                        {
                            array3D[k][l].isCurbPoint = 1;
                        }
                    }
                }
            }
        }
    }

    /*same as before but we check from 360° to [0° + beamZone] this time*/
    for (i = 360; i >= 0 + params::beamZone; --i)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*If these conditions are met, then we have reached a blind spot and we stop checking.*/
            if (params::xDirection == 0)
            {
                /*evaluating the points in both directions (+-X)*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*evaluating the points in +X direction.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*evaluating the points in -X direction.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*By default settings there's no high point in the given segment.*/
            notRoad = 0;

            /*evaluation of the given segment of the first arc*/
            for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
            {
                if (array3D[0][j].alpha <= i)
                {
                    /*The segment needs no further checking if a high point is found.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*If no high point is found in the given segment of the first arc, we can proceed to the next arc.*/
            if (notRoad == 0)
            {
                /*We accept the segment of the first arc.*/
                for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
                {
                    if (array3D[0][j].alpha <= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*checking the rest of the arcs*/
                for (k = 1; k < index; k++)
                {
                    /*A new angle needs to be defined to get the same arc length at every radius.*/
                    if (i == 0 + params::beamZone)
                    {
                        currentDegree = 0;
                    }
                    else
                    {
                        currentDegree = i - arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*checking the points of the new arc*/
                    for (l = indexArray[k] - 1; array3D[k][l].alpha >= currentDegree && l >= 0; --l)
                    {
                        if (array3D[k][l].alpha <= i)
                        {
                            /*The segment needs no further processing if a high point is found.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*The rest of the arcs do not need to be checked if the beam stops at a high point.*/
                    if (notRoad == 1)
                        break;

                    /*else: the given segment of the given arc is accepted*/
                    for (l = indexArray[k] - 1; array3D[k][l].alpha >= currentDegree && l >= 0; --l)
                    {
                        if (array3D[k][l].alpha <= i)
                        {
                            array3D[k][l].isCurbPoint = 1;
                        }
                    }
                }
            }
        }
    }
}

