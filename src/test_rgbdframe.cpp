/**
 * Author: BIgRunner
 * Date: 20200701
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"


int main()
{
    cv::Mat rgb = cv::imread("data/rgb.png");
    cv::Mat depth = cv::imread("data/depth.png", -1);

    CAMERA_INFO camera;
    camera.r0 = 253.5;
    camera.c0 = 325.5;
    camera.fr = 519.0;
    camera.fc = 518.0;
    camera.scale = 1000.0;
    // std::cout << depth << std::endl;
    // cv::waitKey();
    RGBD_FRAME frame(rgb, depth, camera);
    // frame.bifilter(2, 10, 3);
    // cv::imwrite("./data/depth_filter.png", frame.depth);
    // std::vector<Plane> planes = frame.extract_planes_by_ransac(1000);
    std::vector<Plane> planes = frame.extract_planes_by_grid(10, 10, 10);
    cv::Mat plane;
    for(int i = 0; i<planes.size(); i++)
    {
        plane = cv::Mat::zeros(depth.size(), CV_8UC1);
        depth.copyTo(plane, planes[i].mask);
        cv::imshow("plane", plane);
        cv::waitKey();
    }
    rgb.copyTo(plane, frame.planes_mask);
    cv::imshow("mask", plane);
    cv::waitKey();

    cv::imshow("normals", frame.normals/2.0+0.5);
    cv::waitKey();
    cv::imwrite("./data/normals.png", (frame.normals/2.0+0.5)*255);
 
}
