#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <gseg_benchmark/node.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include "lib/cvt.h"
#include "gpf/groundplanefit.hpp"
#include "r_gpf/r_gpf.hpp"
#include "ransac/ransac_gpf.hpp"
#include "patchwork/patchwork.hpp"
#include "cascadedseg/cascaded_groundseg.hpp"
#include "linefit/ground_segmentation.h"
#include "gaussian/GaussianFloorSegmentation.h"

//#include "urban_road_filter/data_structures.hpp"
//#include <gseg_benchmark/LidarFiltersConfig.h>

#include <csignal>

using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;


boost::shared_ptr<GroundPlaneFit>    gpf;
boost::shared_ptr<RegionwiseGPF>     r_gpf;
boost::shared_ptr<RansacGPF>         ransac_gpf;
boost::shared_ptr<PatchWork>         patchwork;
boost::shared_ptr<CascadedGroundSeg> cascaded_gseg;
//boost::shared_ptr<Detector>          urban_road_filt;
boost::shared_ptr<pcl::GaussianFloorSegmentation<PointXYZILID>> gaussian;

std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
string      output_csvpath;
string      data_path;
bool        save_flag;
bool        use_z_thr;
bool        save_csv_file;
bool        stop_for_each_frame;
bool        show_rviz;
int         init_idx;

// Utils for linefit algorithm
void set_linefit_params(ros::NodeHandle *nh, GroundSegmentationParams &params) {
    nh->param("/linefit/visualize", params.visualize, params.visualize);
    nh->param("/linefit/n_bins", params.n_bins, params.n_bins);
    nh->param("/linefit/n_segments", params.n_segments, params.n_segments);
    nh->param("/linefit/max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
    nh->param("/linefit/max_slope", params.max_slope, params.max_slope);
    nh->param("/linefit/long_threshold", params.long_threshold, params.long_threshold);
    nh->param("/linefit/max_long_height", params.max_long_height, params.max_long_height);
    nh->param("/linefit/max_start_height", params.max_start_height, params.max_start_height);
    nh->param("/linefit/sensor_height", params.sensor_height, params.sensor_height);
    nh->param("/linefit/line_search_angle", params.line_search_angle, params.line_search_angle);
    nh->param("/linefit/n_threads", params.n_threads, params.n_threads);

    double r_min, r_max, max_fit_error;
    if (nh->getParam("/linefit/r_min", r_min)) {
        params.r_min_square = r_min * r_min;
    }
    if (nh->getParam("/linefit/r_max", r_max)) {
        params.r_max_square = r_max * r_max;
    }
    if (nh->getParam("/linefit/max_fit_error", max_fit_error)) {
        params.max_error_square = max_fit_error * max_fit_error;
    }
}

void xyzilid2xyz(const pcl::PointCloud<PointType> &src, pcl::PointCloud<pcl::PointXYZ> &dst) {
    dst.clear();
    pcl::PointXYZ pt_dst;
    for (const auto &pt_src: src.points) {
        pt_dst.x = pt_src.x;
        pt_dst.y = pt_src.y;
        pt_dst.z = pt_src.z;
        dst.points.emplace_back(pt_dst);
    }
}
void xyzilid2xyzi(const pcl::PointCloud<PointType> &src, pcl::PointCloud<pcl::PointXYZI> &dst) {
    dst.points.clear();
    for (const auto &pt: src.points){
        pcl::PointXYZI pt_xyzi;
        pt_xyzi.x = pt.x;
        pt_xyzi.y = pt.y;
        pt_xyzi.z = pt.z;
        pt_xyzi.intensity = pt.intensity;
        dst.points.push_back(pt_xyzi);
    }
}

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

class KittiLoader {
public:
    KittiLoader(const std::string &abs_path) {
        pc_path_    = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "Error: no files in " << pc_path_ << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "Error: The # of point clouds and # of labels are not same" << std::endl;
        }
    }

    ~KittiLoader() {}

    size_t size() const { return num_frames_; }

    template<typename T>
    int get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const {
        std::string filename          = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
        FILE        *file             = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return -1;
        }

        std::vector<float> buffer(1000000);
        size_t             num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        cloud.resize(num_points);
        if (std::is_same<T, pcl::PointXYZ>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x         = buffer[i * 4];
                pt.y         = buffer[i * 4 + 1];
                pt.z         = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
            }

        } else if (std::is_same<T, PointXYZILID>::value) {
            std::string   label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
            std::ifstream label_input(label_name, std::ios::binary);
            if (!label_input.is_open()) {
                std::cerr << "Could not open the label!" << std::endl;
                return -1;
            }
            label_input.seekg(0, std::ios::beg);

            std::vector<uint32_t> labels(num_points);
            label_input.read((char *) &labels[0], num_points * sizeof(uint32_t));

            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x         = buffer[i * 4];
                pt.y         = buffer[i * 4 + 1];
                pt.z         = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
                pt.label     = labels[i] & 0xFFFF;
                pt.id        = labels[i] >> 16;
            }

        }
    }

private:
    int         num_frames_;
    std::string label_path_;
    std::string pc_path_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param("/save_flag", save_flag, false);
    nh.param<string>("/sequence", seq, "00");
    nh.param<bool>("/patchwork/use_z_thr", use_z_thr, false);
    nh.param<bool>("/save_csv_file", save_csv_file, false);
    nh.param<bool>("/stop_for_each_frame", stop_for_each_frame, false);
    nh.param<bool>("/show_rviz", show_rviz, true);
    nh.param<int>("/init_idx", init_idx, 0);
    nh.param<string>("/output_csvpath", output_csvpath, "/data/");
    nh.param<string>("/data_path", data_path, "/");

    CloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    TPPublisher    = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100, true);
    FPPublisher    = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100, true);
    FNPublisher    = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100, true);

    std::cout << "\033[1;32mTarget Seq.: " << seq << "\033[0m" << std::endl;
    std::cout << "\033[1;32mTarget Alg.: " << algorithm << "\033[0m" << std::endl;

    // Special: initialization of linefit is different!
    // Should declare linefit inside ground segmentation every time
    GroundSegmentationParams linefit_params;
    set_linefit_params(&nh, linefit_params);

    if (algorithm == "gpf") {           // "multiple" mode is the original gpf algorithm
        gpf.reset(new GroundPlaneFit(&nh));
    } else if (algorithm == "r_gpf") {
        r_gpf.reset(new RegionwiseGPF(&nh));
    } else if (algorithm == "ransac") {
        ransac_gpf.reset(new RansacGPF(&nh));
    } else if (algorithm == "patchwork") {
        patchwork.reset(new PatchWork(&nh));
    } else if (algorithm == "cascaded_gseg") {
        cascaded_gseg.reset(new CascadedGroundSeg(&nh));
        cout << "CascadedSeg init. complete" << endl;
    } else if (algorithm == "gaussian") {
        gaussian.reset(new pcl::GaussianFloorSegmentation<PointType>(&nh));
        gaussian->print_rosparam(&nh);
        cout << "Guassian Floor Segmentation init. complete" << endl;
    }
//    else if (algorithm == "urban_road_filter") {
//        urban_road_filt.reset(new Detector(&nh));
//        cout << "UrbanRoadFilter init. complete" << endl;
//    }

    string HOME = std::getenv("HOME");

    output_csvpath = HOME + output_csvpath + algorithm + "_";
    data_path      = data_path + "/" + seq;

    KittiLoader loader(data_path);

    int      N = loader.size();
    for (int n = init_idx; n < N; ++n) {
        signal(SIGINT, signal_callback_handler);

//        cout << n << "th frame comes" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;
        pc_ground.reserve(150000);
        pc_non_ground.reserve(150000);

//        std::cout << "cloud before filtering: "<<pc_curr.size()<<endl;

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
            patchwork->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
        } else if (algorithm == "gaussian") {
            cout << "Operating gaussian..." << endl;
            gaussian->estimate_ground(pc_curr,pc_ground, pc_non_ground, time_taken);
        } else if (algorithm == "cascaded_gseg") {
            cout << "Operating cascaded_gseg..." << endl;
            int num1 = (int) pc_curr.size();
            cascaded_gseg->estimate_ground(pc_curr, pc_ground, pc_non_ground,time_taken);
            pc_curr.points.clear();
            pc_curr = pc_ground + pc_non_ground;
            int num2 = (int) pc_curr.size();
            cout << "\033[1;33m" << "point num diff: " << num1 - num2 << "\033[0m" << endl;
        } else if (algorithm == "linefit") {
            pcl::PointCloud<pcl::PointXYZ> pc_curr_tmp;
            // To run linefit, dummy pcl::PointXYZ is set
            xyzilid2xyz(pc_curr, pc_curr_tmp);
            pc_ground.clear();
            pc_non_ground.clear();

            GroundSegmentation linefit(linefit_params);
            std::vector<int>   labels;
            auto               start = chrono::high_resolution_clock::now();
            linefit.segment(pc_curr_tmp, &labels);
            for (size_t i = 0; i < pc_curr_tmp.size(); ++i) {
                const auto &pt = pc_curr.points[i];
                if (labels[i] == 1) pc_ground.points.emplace_back(pt);
                else pc_non_ground.points.emplace_back(pt);
            }
            auto end = chrono::high_resolution_clock::now();
            time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;
        }
//        else if (algorithm == "urban_road_filter") {
//            pc_ground.clear();
//            pc_non_ground.clear();
//            cout << "Operating urban_road_filter..." << endl;
//            urban_road_filt->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
//            for (size_t i = 0; i < pc_curr.size(); i++) {
//                const auto &pt = pc_curr.points[i];
//                for (size_t j=0; j< pc_ground.size(); j++) {
//                    const auto &pt_g = pc_ground.points[j];
//                    if (pt.x == pt_g.x && pt.y == pt_g.y && pt.z == pt_g.z ) {
//                        pc_ground.points[j]=pt; //pc_ground.points.emplace_back(pt);
//                        break;
//                    }
//                    //else pc_non_ground.points.emplace_back(pt);
//                }
//                for (size_t j=0; j< pc_non_ground.size(); j++) {
//                    const auto &pt_g = pc_non_ground.points[j];
//                    if (pt.x == pt_g.x && pt.y == pt_g.y && pt.z == pt_g.z ) {
//                        pc_non_ground.points[j] = pt; //pc_ground.points.emplace_back(pt);
//                        break;
//                    }
//                }
//            }
//        }
        else {
            throw invalid_argument("The type of algorithm is invalid");
        }

        //cout<<"ground: "<< pc_ground.size()<<" | nonground: "<<pc_non_ground.size()<<endl;

        // Estimation
        static double      precision, recall, precision_wo_veg, recall_wo_veg;
        static vector<int> TPFNs; // TP, FP, FN, TF order
        static vector<int> TPFNs_wo_veg; // TP, FP, FN, TF order

        calculate_precision_recall(pc_curr, pc_ground, precision, recall, TPFNs);
        calculate_precision_recall_without_vegetation(pc_curr, pc_ground, precision_wo_veg, recall_wo_veg, TPFNs_wo_veg);
        //calculate_precision_recall(pc_curr, pc_ground, precision_naive, recall_naive, false);
        //calculate_precision_recall_without_vegetation(pc_curr, pc_ground, precision_naive, recall_naive, false);

        //Print
        cout << "\033[1;32m" << n << "th:" << " takes " << setprecision(4) <<  time_taken << " sec.\033[0m" << endl;
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

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        // Publish msg
        pcl::PointCloud<PointType> TP;
        pcl::PointCloud<PointType> FP;
        pcl::PointCloud<PointType> FN;
        pcl::PointCloud<PointType> TN;
        // discern_ground(pc_ground, TP, FP);
        discern_ground_without_vegetation(pc_ground, TP, FP);
        // discern_ground(pc_non_ground, FN, TN);
        discern_ground_without_vegetation(pc_non_ground, FN, TN);

        //Print
        cout << "TP: " << TP.points.size();
        cout << " | FP: " << FP.points.size();
        cout << " | TN: " << TN.points.size() << endl;
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save the output of pcd, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        if (save_flag) {
//            std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
//            double             accuracy;
//            save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);
//
//            std::string count_str        = std::to_string(n);
//            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
//            std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";
//            pc2pcdfile(TP, FP, FN, TN, pcd_filename);
//        }
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        CloudPublisher.publish(cvt::cloud2msg(pc_curr));
        TPPublisher.publish(cvt::cloud2msg(TP));
        FPPublisher.publish(cvt::cloud2msg(FP));
        FNPublisher.publish(cvt::cloud2msg(FN));
        ros::spinOnce();

        if (stop_for_each_frame) {
            cout << "[Break]: Stop at " << n << " th frame: press any button to continue" << endl;
            cin.ignore();
        }
    }

    return 0;
}
