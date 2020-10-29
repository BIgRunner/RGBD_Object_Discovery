/**
 * Author: BIgRunner
 * Date: 20200701
 * Description: Test plane extractor
 */

#include "RGBDFrame.hpp"

int main()
{
    cv::Mat depth = cv::imread("/home/runner/Dataset/ABW-Dataset/depth/abw_test_0.png", -1);
    cv::Mat rgb = cv::Mat::zeros(depth.size(), CV_8UC3);

    CAMERA_INFO camera;
    camera.r0 = 255;
    camera.c0 = 255;
    camera.fr = 2337.212;
    camera.fc = 1610.982;
    camera.scale = 773.545;

    RGBD_FRAME frame(rgb, depth, camera);

    std::vector<Plane> planes = frame.extract_planes_by_grid(10, 64, 32, 0.05, 200, 1);
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

    std::cout << planes.size() << " planes has been found." << std::endl;

    // cv::Mat left;
    // cv::Mat mask;
    // cv::bitwise_not(frame.planes_mask, mask);
    // depth.copyTo(left, mask);

    cv::Mat left = frame.remove_planes(planes);
    cv::imshow("left", left);
    cv::waitKey();

    // cv::GaussianBlur(left, left, cv::Size(3,3), 0);
    // cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, edge;

    // cv::Sobel(left, grad_x, CV_64F, 0, 1);
    // abs_grad_x = cv::abs(grad_x);
    // abs_grad_x.convertTo(abs_grad_x, CV_16U);
    // cv::imshow("grad_x", abs_grad_x);
    // cv::imwrite("data/sobel_grad_x.png", abs_grad_x);
    // cv::Sobel(left, grad_y, CV_64F, 1, 0);
    // abs_grad_y = cv::abs(grad_y);
    // abs_grad_y.convertTo(abs_grad_y, CV_16U);
    // cv::imshow("grad_y", abs_grad_y);
    // cv::imwrite("data/sobel_grad_y.png", abs_grad_y);
    // cv::addWeighted(abs_grad_y, 0.5, abs_grad_y, 0.5, 0, edge);

    // imshow("edge", edge);
    // cv::waitKey();

    std::vector<Plane> horizontal_planes = frame.find_horizontal_plane(planes);

    std::vector<OBJECT> objects = frame.find_supported_objects(left, horizontal_planes);

    std::cout << "to show" << std::endl;

    for(int i=0; i<objects.size(); i++)
    {
        cv::imshow("object", objects[i].mask);
        cv::waitKey();
    }

    cv::imshow("normals", frame.normals/2.0+0.5);
    cv::waitKey();
    cv::imwrite("./result/normals.png", (frame.normals/2.0+0.5)*255);
 
}
