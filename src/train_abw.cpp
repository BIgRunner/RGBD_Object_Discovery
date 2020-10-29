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
    ConfigParser config_parser = ConfigParser("./config_abw.txt");

    cv::Mat depth = cv::imread(config_parser.getData("depth"), -1);
    cv::Mat rgb = cv::imread(config_parser.getData("color"));

    std::string save_path = config_parser.getData("save_path");

    CAMERA_INFO camera;
    camera.r0 = atof(config_parser.getData("camera.r0").c_str());
    camera.c0 = atof(config_parser.getData("camera.c0").c_str());
    camera.fr = atof(config_parser.getData("camera.fr").c_str());
    camera.fc = atof(config_parser.getData("camera.fc").c_str());
    camera.scale = atof(config_parser.getData("camera.scale").c_str());

    int seeds_per_cell = atoi(config_parser.getData("seeds_per_cell").c_str());
    int cell_size = atoi(config_parser.getData("cell_size").c_str());
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


    // int column_cells = depth.cols/cell_size;
    // int row_cells = depth.rows/cell_size;

    // std::cout << "column cells: " << column_cells<< std::endl;
    // std::cout << "row cells: " << row_cells << std::endl;

    RGBD_FRAME frame(rgb, depth, camera);
    std::vector<Plane> planes = frame.extract_planes_by_grid(seeds_per_cell, cell_size, distance_thresh, angle_thresh, size_thresh, boundary, norm_cont);
    if (merge)
    {
        merge_planes(planes, plane_distance_thresh, plane_angle_thresh);
    }
    cv::Mat plane = cv::Mat::zeros(depth.size(), CV_8UC1);
    for(int i = 0; i<planes.size(); i++)
    {
        cv::add(plane, i+1, plane, planes[i].mask);
    }
    std::cout<< planes.size() << " planes has been found." << std::endl;
    cv::imwrite(save_path, plane);
    // cv::imshow("result", plane);
    // cv::waitKey();
}