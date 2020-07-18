/**
 * Author: BIgRunner
 * Date: 20200701
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"


int main()
{
    cv::Mat rgb = cv::imread("data/00000-color.png");
    cv::Mat depth = cv::imread("data/00000-depth.png", -1);

    CAMERA_INFO camera;
    camera.r0 = 240;
    camera.c0 = 320;
    camera.fr = 519.0;
    camera.fc = 518.0;
    camera.scale = 5000.0;

    RGBD_FRAME frame(rgb, depth, camera);

    std::vector<Plane> planes = frame.extract_planes_by_grid(12, 28, 28);
    cv::Mat plane;
    for(int i = 0; i<planes.size(); i++)
    {
        // plane = cv::Mat::zeros(depth.size(), CV_8UC1);
        // depth.copyTo(plane, planes[i].mask);
        cv::imshow("plane", planes[i].mask);
        cv::waitKey();
        if(planes[i].is_horizontal())
            std::cout << "plane " << i << " is horizontal." << std::endl;
        else if(planes[i].is_vertical())
            std::cout << "plane " << i << " is vertical." << std::endl;
    }

    cv::Mat left;
    cv::Mat mask;
    cv::bitwise_not(frame.planes_mask, mask);
    depth.copyTo(left, mask);
    cv::imshow("left", left);
    cv::waitKey();

    cv::imshow("normals", frame.normals/2.0+0.5);
    cv::waitKey();
    cv::imwrite("./result/normals.png", (frame.normals/2.0+0.5)*255);
 
}
