/**
 * Author: BIgRunner
 * Date: 20200908
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"
#include "ConfigParser.hpp"
#include "Timer.hpp"


int main()
{
    // Read parameters
    ConfigParser config_parser = ConfigParser("./config_nyu.txt");

    cv::Mat depth = cv::imread(config_parser.getData("depth"), -1);
    cv::Mat rgb = cv::imread(config_parser.getData("color"));

    std::string save_path = config_parser.getData("save_path");

    CAMERA_INFO camera;
    camera.r0 = atof(config_parser.getData("camera.r0").c_str());
    camera.c0 = atof(config_parser.getData("camera.c0").c_str());
    camera.fr = atof(config_parser.getData("camera.fr").c_str());
    camera.fc = atof(config_parser.getData("camera.fc").c_str());
    camera.scale = atof(config_parser.getData("camera.scale").c_str());

    int seeding_method = atoi(config_parser.getData("seeding_method").c_str());
    int seeds_count = atoi(config_parser.getData("seeds_count").c_str());
    double distance_thresh = atof(config_parser.getData("distance_thresh").c_str());
    double angle_diff = atof(config_parser.getData("angle_diff").c_str());
    double angle_thresh = cos(angle_diff*M_PI);
    int size_thresh = atoi(config_parser.getData("size_thresh").c_str());
    bool boundary = atoi(config_parser.getData("boundary").c_str());
    bool norm_cont = atoi(config_parser.getData("norm_cont").c_str());

    bool merge = atoi(config_parser.getData("merge").c_str());
    double plane_distance_thresh = atof(config_parser.getData("plane_distance_thresh").c_str());
    double plane_angle_diff = atof(config_parser.getData("plane_angle_diff").c_str());
    double plane_angle_thresh = cos(plane_angle_diff*M_PI);
    
    SEEDING_TYPE seeding_type = (SEEDING_TYPE)seeding_method;

    TickMeter total_time;
    total_time.start();
    RGBD_FRAME frame(rgb, depth, camera);
    std::vector<Plane> planes = frame.extract_planes(seeding_type, seeds_count, distance_thresh, angle_thresh, size_thresh, boundary, norm_cont, merge, plane_distance_thresh, plane_angle_thresh);
    total_time.stop();
    std::cout << "total time: " << total_time.getTimeSec() << std::endl;
    std::cout << "seeding time: " << frame.seeding_meter.getTimeSec() <<  "s, for " << frame.seeding_meter.counter << " seeds." << std::endl;
    std::cout << "plane time: " << frame.plane_meter.getTimeSec() <<  "s, for " << frame.plane_meter.counter << " rounds." << std::endl;
    std::cout << "grow time: " << frame.grow_meter.getTimeSec() <<  "s, for " << frame.grow_meter.counter << " rounds." << std::endl;
    std::cout << "refine time: " << frame.refine_meter.getTimeSec() <<  "s, for " << frame.refine_meter.counter << " rounds." << std::endl;
    std::cout << "inPlane time: " << frame.inPlane_meter.getTimeSec() <<  "s, for " << frame.inPlane_meter.counter << " rounds." << std::endl;
    std::cout << "norm time: " << frame.norm_meter.getTimeSec() <<  "s, for " << frame.norm_meter.counter << " rounds." << std::endl;

    cv::imshow("planes", frame.planes_mask * 10);
    cv::waitKey();

}