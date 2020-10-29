#ifndef OBJECT_DISCOVERY_RGBD_FRAME_HPP
#define OBJECT_DISCOVERY_RGBD_FRAME_HPP


/*
 * Author: BIgRunner
 * Date: 20200701
 * Description: This is a class for a single rgbd frame, which consists
 *              of a rgb image and a depth image, enable plane extraction 
 *              from depht images based on RACSAC.
 * 
 * Modified: 20201006
 */


#include <opencv2/opencv.hpp>
#include "Plane.hpp"
#include "Timer.hpp"

enum SEEDING_TYPE{R, L, G, GL};

class RGBD_FRAME
{
private:
  cv::Mat min_distance;
  CAMERA_INFO camera;

private:
  inline void mark_global(cv::Point3d &p, uchar plane_idx)
  {
    mark_mat(planes_mask, p, plane_idx);
  }

  inline void mark_mat(cv::Mat &mat, cv::Point3d &p, uchar value)
  {
    if (mat.type() != CV_8UC1)
      std::cerr << "BAD IMAGE TYPE FOR MARK! only CV_8UC1 allowed." << std::endl;
    mat.ptr<uchar>((int)p.y)[(int)p.x] = value;
  }

  inline bool in_image(std::vector<cv::Point3d> &seeds)
  {
    for(int i=0; i<seeds.size(); i++)
    {
      if(!in_image(seeds[i]))
        return false;
    }
    return true;
  }

  inline bool in_image(cv::Point3d p)
  {
    return 0<=p.y && 0<=p.x 
        && p.y<depth.rows && p.x<depth.cols;
  }

  inline bool have_valid_depth(std::vector<cv::Point3d> &seeds)
  {
    for (int i=0; i<seeds.size(); i++)
    {
      if(!have_valid_depth(seeds[i]))
        return false;
    }
    return true;
  }

  inline bool have_valid_depth(cv::Point3d &p)
  {
    return depth.ptr<ushort>((int)p.y)[(int)p.x];
  }

  cv::Point3d estimate_normal(cv::Point3d &p);

  bool normal_consensus_check(cv::Point3d &p1, cv::Point3d &p2, double angle_thresh);  
  bool normal_consensus_check(Plane &plane, cv::Point3d &point, double angle_thresh);  
  double normal_distance(Plane &plane, cv::Point3d &point);

  // preproceses of depth image
  void bifilter(int kernal_size, double sigma_c, double sigma_d);
  cv::Mat estimate_normals();

  // the flow of plane extraction
  void seeding(std::vector<cv::Point3d> &seeds, SEEDING_TYPE seeding_type, int seeds_count);
  bool plane_grow(Plane &plane, std::set<int> &neighbor_planes, double distance_thresh, double angle_thresh, int size_thresh, bool boundary, cv::Mat &edge, bool norm_cont, cv::Mat &normals);
  void merge_planes(Plane &plane1, Plane &plane2, double merge_dist_thresh, double merge_angle_thresh);


public:
  cv::Mat rgb, depth, saliency;
  cv::Mat depth_inv;
  cv::Mat planes_mask;
  cv::Mat objects_mask;
  cv::Mat horizontal_planes_mask;

  TickMeter seeding_meter, plane_meter, grow_meter, refine_meter;
  TickMeter inPlane_meter;
  TickMeter norm_meter;

  RGBD_FRAME(cv::Mat &rgb, cv::Mat &depth, CAMERA_INFO &camera);
  std::vector<Plane> extract_planes(SEEDING_TYPE seeding_type, int seeds_count, double distance_thresh, double angle_thresh, int size_thresh, bool boundary, bool norm_cont, bool merge, double merge_dist_thresh, double merge_angle_thresh);
  std::vector<Plane> find_support_plane(std::vector<Plane> &planes, double angle)
  {
      std::vector<Plane> horizontal_planes;
      for(int i=0; i<planes.size(); i++)
          if(planes[i].is_horizontal(angle))
              horizontal_planes.push_back(planes[i]);
      return horizontal_planes;
  }

  cv::Mat remove_planes(std::vector<Plane> &planes);

  std::vector<OBJECT> find_supported_objects(cv::Mat &left, std::vector<Plane> &horizontal_planes, double min_h, double max_h);
};
    

#endif