#ifndef OBJECT_DISCOVERY_PLANE_HPP
#define OBJECT_DISCOVERY_PLANE_HPP


/**********************
 *    X      Y      Z
 *    c      r      d
 *********************/

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


struct CAMERA_INFO{
  double r0, c0;
  double fr, fc;
  double scale;
};


class Plane
{
public:
  double n_a, n_b, n_c, d;
  double alpha, beta, gamma, theta;
  cv::Mat mask;
  bool valid;
  cv::Point3d seed;
  double init_normal_cont;

  int min_r, min_c, max_r, max_c;

  Plane(CAMERA_INFO &camera);

  void insert_point(cv::Point3d &p);

  void delete_point(cv::Point3d &p);

  bool in_plane(cv::Point3d &p, double distance)
  {
      return calculate_distance(p)<distance;
  }

  bool above_plane(cv::Point3d &p, double min_h, double max_h)
  {
    if(calculate_relative_distance(p) < min_h && calculate_relative_distance(p) > max_h)
      return false;
    // Project to plane

    double delta=alpha*p.x+beta*p.y+gamma+theta*p.z;
    double factor=sqrt(alpha*alpha+beta*beta+theta*theta);

    int c = p.x - alpha*delta/factor;
    int r = p.y - beta*delta/factor;
    return c>=min_c && c<=max_c && r>=min_r && r <= max_r;
  }

  double calculate_distance(cv::Point3d &p)
  {
      return abs((alpha*p.x+beta*p.y+gamma+theta*p.z)/p.z)/sqrt(1+theta*theta);
  }
  
  double calculate_relative_distance(cv::Point3d &p)
  {
    return ((alpha*p.x+beta*p.y+gamma+theta*p.z)/p.z)/sqrt(1+theta*theta);
  }

  bool is_normalized()
  {
      return abs(n_a*n_a+n_b*n_b+n_c*n_c-1)<0.0001;
  }

  bool is_horizontal(double angle)
  {
      return abs(n_b)>cos(angle);
  }

  bool is_vertical(double angle)
  {
      return abs(n_b)<1-cos(angle);
  }

  bool initialize(std::vector<cv::Point3d> &seeds)
  {
    if(seeds.size()!=3)
    {
        std::cerr << "Error: THREE POINTS TO DETERMINE A PLANE!" << std::endl;
        exit(-1);
    }

    cv::Point3d coeffs = (seeds[1]-seeds[0]).cross(seeds[2]-seeds[0]);
    seed = seeds[0];

    alpha = coeffs.x;
    beta = coeffs.y;
    theta = coeffs.z;
    gamma = -alpha*seeds[0].x-beta*seeds[0].y-theta*seeds[0].z;

    min_c = seeds[0].x-1;
    max_c = seeds[0].x+1;
    min_r = seeds[0].y-1;
    max_r = seeds[0].y+1;

    valid = false;

    return normalize();
  }

  bool is_coplanar(Plane &p, double merge_angle_thresh, double merge_dist_thresh)
  {
    return abs(cv::Vec3d(p.n_a, p.n_b, p.n_c).dot(cv::Vec3d(n_a, n_b, n_c)))>merge_angle_thresh && abs(abs(d)-abs(p.d))<merge_dist_thresh;
  }


  void merge(Plane &p);

  bool refit();

  int size()
  {
    return point_count;
  }

  bool operator < (const Plane& p) const
  {
    return 1-init_normal_cont < 1-p.init_normal_cont;
  }

private:
  CAMERA_INFO camera;
  int point_count;
  double c_acc, r_acc, invd_acc,
         cc_acc, rr_acc, invd2_acc,
         cr_acc, cinvd_acc, rinvd_acc;
  

  bool normalize()
  {
    n_a = alpha*camera.fc*camera.scale;
    n_b = beta*camera.fr*camera.scale;
    n_c = (gamma+beta*camera.r0+alpha*camera.c0)*camera.scale;
    d = theta;

    if(n_b>0.0)
    {
        n_a=-n_a;
        n_b=-n_b;
        n_c=-n_c;
        d=-d;
    }

    double s = sqrt(n_a*n_a+n_b*n_b+n_c*n_c);
    n_a = n_a/s;
    n_b = n_b/s;
    n_c = n_c/s;
    d = d/s; 

    alpha = n_a/(camera.fc*camera.scale);
    beta = n_b/(camera.fr*camera.scale);
    gamma = n_c/camera.scale-beta*camera.r0-alpha*camera.c0;
    theta = d;

    return is_normalized();
  }
};


struct OBJECT
{
  double objectness;
  int left, right, top, bottom;
  cv::Mat mask;
};

#endif