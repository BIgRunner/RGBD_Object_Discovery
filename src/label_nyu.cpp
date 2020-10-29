#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_type.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::ModelCoefficients PlaneCoeffs;
typedef pcl::PointIndices Indiecs;


struct CAMERA_INTRINSIC_PARAMETERS
{
  double r0, c0, fr, fc, scale;
};

PointCloud::Ptr generate_point_cloud(cv::Mat &color, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
  PointCloud::Ptr cloud(new PointCloud);

  for(int r=0; r<depth.rows; r++)
    for(int c=0; c<depth.cols; c++)
    {
      ushort d = depth.ptr<ushort>(r)[c];
      
      if(d==0)
	continue;

      PointT p;
      
      p.z = double(d)/camera.scale;
      p.x = (c-camera.c0)*p.z/camera.fc;
      p.y = (r-camera.r0)*p.z/camera.fr;

      p.b = rgb.ptr<uchar>(r)[c*3];
      p.g = rgb.ptr<uchar>(r)[c*3+1];
      p.r = rgb.ptr<uchar>(r)[c*3+2];      

      cloud->points.push_back(p);
    }

  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;

  return cloud;
}

void get_planes(PointCloud::Ptr &cloud, PlaneCoeffs::Ptr &coeffs, Indices::Ptr &inliers, string method)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::
  if (method == "RACSAC")
    seg.setMethodType(pcl::SAC_RANSAC);
  seg.MaxIterations(1000);
  seg.setDistanceThreshold(0.015);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coeffs);

