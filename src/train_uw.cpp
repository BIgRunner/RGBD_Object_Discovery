/**
 * Author: BIgRunner
 * Date: 20200908
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"
#include "ConfigParser.hpp"
#include "Timer.hpp"

cv::Scalar plane_color[13] = {cv::Scalar(80, 127, 255), cv::Scalar(35, 142, 107), cv::Scalar(158, 168, 3), cv::Scalar(31, 102, 156),
                             cv::Scalar(214, 112, 218), cv::Scalar(255,0,0), cv::Scalar(202, 230, 252), cv::Scalar(20, 97, 199),
                             cv::Scalar(140, 199, 0), cv::Scalar(71, 99, 255), cv::Scalar(255, 255, 0), cv::Scalar(203, 192, 255),
                             cv::Scalar(250, 51, 153)};
cv::Scalar horizontal_color[8] = {cv::Scalar(18, 153, 255), cv::Scalar(87, 207, 227), cv::Scalar(0, 215, 255),
                                   cv::Scalar(64, 125, 255), cv::Scalar(132, 227, 255), cv::Scalar(0, 128, 255),
                                   cv::Scalar(33, 145, 237), cv::Scalar(0, 102, 85)};


int main()
{
    // Read parameters
    ConfigParser config_parser = ConfigParser("./cfg/config_uw.txt");

    cv::Mat depth = cv::imread(config_parser.getData("depth"), -1);
    cv::Mat rgb = cv::imread(config_parser.getData("color"));
    cv::Mat saliency = cv::imread(config_parser.getData("saliency"), -1);

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

    std::vector<Plane> support_planes = frame.find_support_plane(planes, M_PI/4);
    frame.saliency = saliency;

    cv::Mat left=frame.remove_planes(support_planes);
    cv::imshow("planes", frame.planes_mask*10);
    cv::imshow("left", left);
    cv::imwrite("visual/left.png", left);
    // cv::waitKey();

    std::vector<OBJECT> objects = frame.find_supported_objects(left, support_planes, 0.04, 0.1);

    for(int i=0;i<objects.size(); i++)
    {
        cv::Mat object_mask = cv::Mat(depth.size(), CV_8UC3, cv::Scalar(255, 255, 255));
        rgb.copyTo(object_mask, objects[i].mask);
        cv::imshow("object", object_mask);
        // std::cout << save_path + "/object_"+std::to_string(i)+".png" << std::endl;
        cv::imwrite(save_path + "/object_"+std::to_string(i)+".png", object_mask);

        cv::waitKey(1000);
    }
    cv::imwrite(save_path+"/planes.png", frame.planes_mask);
    cv::imwrite(save_path+"/supports.png", frame.horizontal_planes_mask);

}