/**
 * Author: BIgRunner
 * Date: 20200908
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"
#include "ConfigParser.hpp"


int main()
{
    // Read parameters
    ConfigParser config_parser = ConfigParser("cfg/config_abw.txt");

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

    RGBD_FRAME frame(rgb, depth, camera);
    std::vector<Plane> planes = frame.extract_planes(seeding_type, seeds_count, distance_thresh, angle_thresh, size_thresh, boundary, norm_cont, merge, plane_distance_thresh, plane_angle_thresh);
    std::cout<< planes.size() << " planes has been found." << std::endl;
    cv::imwrite(save_path, frame.planes_mask);
    // cv::imshow("result", frame.planes_mask*10);
    // cv::waitKey();
}