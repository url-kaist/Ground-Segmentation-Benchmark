#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <gseg_benchmark/node.h>
#include <unavlib/convt.h>
#include "cascadedseg/cascaded_groundseg.hpp"
#include "gpf/groundplanefit.hpp"
#include "r_gpf/r_gpf.hpp"
#include "ransac/ransac_gpf.hpp"
#include "patchwork/patchwork.hpp"

using namespace std;
using namespace unavlib;

ros::Publisher CloudPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher OutputPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;


boost::shared_ptr<GroundPlaneFit>    gpf;
boost::shared_ptr<RegionwiseGPF>     r_gpf;
boost::shared_ptr<RansacGPF>         ransac_gpf;
boost::shared_ptr<PatchWork>         sp;
boost::shared_ptr<CascadedGroundSeg> cascaded_gseg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
string      output_csvpath;
string      w_veg_path, wo_veg_path;
bool        save_flag;
bool        use_z_thr;
bool        save_csv_file;

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::Marker marker;
    marker.header.frame_id                  = "map";
    marker.header.stamp                     = ros::Time();
    marker.ns                               = "my_namespace";
    marker.id                               = 0;
    marker.type                             = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action                           = visualization_msgs::Marker::ADD;
    if (mode == "p") marker.pose.position.x = 28.5;
    if (mode == "r") marker.pose.position.x = 25;
    marker.pose.position.y                  = 30;

    marker.pose.position.z    = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = SCALE;
    marker.scale.y            = SCALE;
    marker.scale.z            = SCALE;
    marker.color.a            = 1.0; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.text               = mode + ": " + std::to_string(measure);
    if (mode == "p") PrecisionPublisher.publish(marker);
    if (mode == "r") RecallPublisher.publish(marker);

}


void callbackNode(const gseg_benchmark::node::ConstPtr &msg) {
    int n = msg->header.seq;
    cout << n << "th node come" << endl;
    pcl::PointCloud<PointXYZILID> pc_curr = cvt::cloudmsg2cloud<PointXYZILID>(msg->lidar);
    pcl::PointCloud<PointXYZILID> pc_ground;
    pcl::PointCloud<PointXYZILID> pc_non_ground;

    static double time_taken;
    if (algorithm == "gpf") {
        cout << "Operating gpf..." << endl;
        gpf->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    } else if (algorithm == "r_gpf") {
        cout << "Operating r-gpf..." << endl;
        r_gpf->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    } else if (algorithm == "ransac") {
        cout << "Operating ransac..." << endl;
        ransac_gpf->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    } else if (algorithm == "patchwork") {
        cout << "Operating patchwork..." << endl;
        sp->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    } else if (algorithm == "cascaded_gseg") {
        cout << "Operating cascaded_gseg..." << endl;
        int num1 = (int) pc_curr.size();
        cascaded_gseg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
        pc_curr.points.clear();
        pc_curr = pc_ground + pc_non_ground;
        int num2 = (int) pc_curr.size();
        cout << "\033[1;33m" << "point num diff: " << num1 - num2 << "\033[0m" << endl;

    }
    // Estimation
    static double      precision, recall, precision_wo_veg, recall_wo_veg;
    static vector<int> TPFNs; // TP, FP, FN, TF order
    static vector<int> TPFNs_wo_veg; // TP, FP, FN, TF order

    calculate_precision_recall(pc_curr, pc_ground, precision, recall, TPFNs);
    calculate_precision_recall_without_vegetation(pc_curr, pc_ground, precision_wo_veg, recall_wo_veg, TPFNs_wo_veg);

    cout << "\033[1;32m" << n << "th, " << " takes : " << time_taken << " | " << pc_curr.size() << " -> "
         << pc_ground.size()
         << "\033[0m" << endl;

    cout << "\033[1;32m [W/ Vegi.] P: " << precision << " | R: " << recall << "\033[0m" << endl;
    cout << "\033[1;32m [WO Vegi.] P: " << precision_wo_veg << " | R: " << recall_wo_veg << "\033[0m" << endl;

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save precision/recall in a text file, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    static string w_veg_path  = output_csvpath + "w_veg_" + seq + ".csv";
    static string wo_veg_path = output_csvpath + "wo_veg_" + seq + ".csv";
    if (save_csv_file) {
        // Save w/ veg
        ofstream output(w_veg_path, ios::app);
        output << n << "," << time_taken << "," << precision << "," << recall << "," << TPFNs[0] << "," << TPFNs[1] << "," << TPFNs[2]
               << "," << TPFNs[3];
        output << std::endl;
        output.close();
        // Save w/o veg
        output.open(wo_veg_path, ios::app);
        output << n << "," << time_taken << "," << precision_wo_veg << "," << recall_wo_veg << "," << TPFNs_wo_veg[0] << ","
               << TPFNs_wo_veg[1] << "," << TPFNs_wo_veg[2] << "," << TPFNs_wo_veg[3];
        output << std::endl;
        output.close();
    }


    // Publish msg
    pcl::PointCloud<PointXYZILID> TP;
    pcl::PointCloud<PointXYZILID> FP;
    pcl::PointCloud<PointXYZILID> FN;
    pcl::PointCloud<PointXYZILID> TN;
    discern_ground(pc_ground, TP, FP);
    discern_ground(pc_non_ground, FN, TN);

//    if (save_flag) {
//        std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
//        double             accuracy;
//        save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);
//
//        std::string count_str        = std::to_string(msg->header.seq);
//        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
//        std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";
//    pc2pcdfile(TP, FP, FN, TN, pcd_filename);
//    }
    // Write data
    CloudPublisher.publish(cvt::cloud2msg(pc_curr));
    TPPublisher.publish(cvt::cloud2msg(TP));
    FPPublisher.publish(cvt::cloud2msg(FP));
    FNPublisher.publish(cvt::cloud2msg(FN));
    pub_score("p", precision);
    pub_score("r", recall);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param("/save_flag", save_flag, false);
    nh.param<string>("/seq", seq, "00");
    nh.param<bool>("/save_csv_file", save_csv_file, false);
    nh.param<string>("/output_csvpath", output_csvpath, "/data/");


    std::cout << "\033[1;32mHey??" << std::endl;
    std::cout << "Hey??" << algorithm << "\033[0m" << std::endl;

    gpf.reset(new GroundPlaneFit(&nh));
    r_gpf.reset(new RegionwiseGPF(&nh));
    ransac_gpf.reset(new RansacGPF(&nh));
    sp.reset(new PatchWork(&nh));
    cascaded_gseg.reset(new CascadedGroundSeg(&nh));

    string HOME = std::getenv("HOME");
    output_csvpath = HOME + output_csvpath + algorithm + "_";

    // For initialization
//    ofstream sc_output(output_filename);
//    sc_output.close();
//    if (save_flag) {
//        acc_filename = ABS_DIR + "/" + algorithm + "_" + std::to_string(use_z_thr) + "_acc_" + seq + ".csv";
//        pcd_savepath = ABS_DIR + "/pcds/" + algorithm + "/" + seq;
//        ofstream sc_output2(acc_filename);
//        sc_output2.close();
//    }

    OutputPublisher = nh.advertise<gseg_benchmark::node>("/node_out", 100);
    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100);
    TPPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100);
    FPPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100);
    FNPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100);

    PrecisionPublisher = nh.advertise<visualization_msgs::Marker>("/precision", 1);
    RecallPublisher    = nh.advertise<visualization_msgs::Marker>("/recall", 1);

    ros::Subscriber NodeSubscriber = nh.subscribe<gseg_benchmark::node>("/node", 5000, callbackNode);

    ros::spin();

    return 0;
}
