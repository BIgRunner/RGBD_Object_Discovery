#ifndef OBJECT_DISCOVERY_RGBD_FRAME_HPP
#define OBJECT_DISCOVERY_RGBD_FRAME_HPP


/*
 * Author: BIgRunner
 * Date: 20200701
 * Description: This is a class for a single rgbd frame, which consists
 *              of a rgb image and a depth image, enable plane extraction 
 *              from depht images based on RACSAC.
 */


#include <opencv2/opencv.hpp>
#include "Plane.hpp"


class RGBD_FRAME
{

private:
    CAMERA_INFO camera;

    inline void mark_global(DEPTH_PIXEL &pixel)
    {
        planes_mask.ptr<uchar>(pixel.r)[pixel.c] = 255;
    }

    inline void mark_local(cv::Mat &local, DEPTH_PIXEL &pixel)
    {
        local.ptr<uchar>(pixel.r)[pixel.c] = 255;
    }

    inline bool in_image(DEPTH_PIXEL &pixel)
    {
        return 0<=pixel.r && 0<=pixel.c 
            && pixel.r<depth.rows && pixel.c<depth.cols;
    }

    inline double normal_distance(DEPTH_PIXEL &p1, DEPTH_PIXEL &p2)
    {
        cv::Vec3d normal1 = normals.at<cv::Vec3d>(p1.r, p1.c);
        cv::Vec3d normal2 = normals.at<cv::Vec3d>(p2.r, p2.c);
        return normal1.dot(normal2);
    }

    // preproceses of depth image
    void bifilter(int kernal_size, double sigma_c, double sigma_d);
    cv::Mat estimate_normals();

    // the flow of plane extraction
    bool initialize_seeds_in_image(std::vector<DEPTH_PIXEL> &seeds);
    bool initialize_seeds_in_grid(std::vector<DEPTH_PIXEL> &seeds, int r, int c, int kernal);
    bool plane_by_three_points(std::vector<DEPTH_PIXEL> &seeds, Plane &plane);
    bool plane_grow(DEPTH_PIXEL &seed, Plane &plane, double distance_thresh, int min_pixels);
    bool refine_plane(std::vector<DEPTH_PIXEL> &pixels, Plane &plane, double distance_thresh);

public:
    cv::Mat rgb, depth;
    cv::Mat planes_mask;
    cv::Mat normals;
    cv::Mat depth_inv;

    RGBD_FRAME(cv::Mat &rgb, cv::Mat &depth, CAMERA_INFO &camera);
    std::vector<Plane> extract_planes_by_ransac(uint iteration);
    std::vector<Plane> extract_planes_by_grid(uint cell_interation, uint width_size, uint height_size);

};

#endif