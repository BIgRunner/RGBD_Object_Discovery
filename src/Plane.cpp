/******************************************************** 
 * Author: BIgRunner
 * Email: xnhan
 * Licnese: 
 * Created: 20201028
 * Modified:
 * 
 ********************************************************/

#include "Plane.hpp"

Plane::Plane(CAMERA_INFO &camera)
{
  this->camera = camera;

  n_a = 0.0;
  n_b = 0.0;
  n_c = 0.0;
  d = 0.0;

  alpha = 0.0;
  beta = 0.0;
  gamma = 0.0;
  theta = 0.0;

  valid = false;

  point_count = 0;

  c_acc = 0.0; r_acc = 0.0; invd_acc = 0.0;
  cc_acc = 0.0; rr_acc = 0.0; invd2_acc = 0.0;
  cr_acc = 0.0; cinvd_acc = 0.0; rinvd_acc = 0.0;
}

void Plane::insert_point(cv::Point3d &p)
{
  point_count += 1;

  c_acc     += p.x;
  r_acc     += p.y;
  invd_acc  += p.z;
  cc_acc    += p.x*p.x;
  rr_acc    += p.y*p.y;
  invd2_acc += p.z*p.z;
  cr_acc    += p.x*p.y;
  cinvd_acc += p.x*p.z;
  rinvd_acc += p.y*p.z;
}

void Plane::delete_point(cv::Point3d &p)
{
  point_count -= 1;

  c_acc     -= p.x;
  r_acc     -= p.y;
  invd_acc  -= p.z;
  cc_acc    -= p.x*p.x;
  rr_acc    -= p.y*p.y;
  invd2_acc -= p.z*p.z;
  cr_acc    -= p.x*p.y;
  cinvd_acc -= p.x*p.z;
  rinvd_acc -= p.y*p.z;
}

void Plane::merge(Plane &plane)
{
  point_count += plane.point_count;

  c_acc     += plane.c_acc;
  r_acc     += plane.r_acc;
  invd_acc  += plane.invd_acc;
  cc_acc    += plane.cc_acc;
  rr_acc    += plane.rr_acc;
  invd2_acc += plane.invd2_acc;
  cr_acc    += plane.cr_acc;
  cinvd_acc += plane.cinvd_acc;
  rinvd_acc += plane.rinvd_acc;

  refit();
  plane.valid = false;
}

bool Plane::refit()
{
  double mean[3] = {c_acc/point_count, r_acc/point_count, invd_acc/point_count};
  double cov[3][3] = {{cc_acc-c_acc*c_acc/point_count, cr_acc-c_acc*r_acc/point_count, cinvd_acc-c_acc*invd_acc/point_count},
                      {0, rr_acc-r_acc*r_acc/point_count, rinvd_acc-r_acc*invd_acc/point_count},
                      {0, 0, invd2_acc-invd_acc*invd_acc/point_count}};
  cov[1][0] = cov[0][1]; 
  cov[2][0] = cov[0][2];
  cov[2][1] = cov[2][1];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Eigen::Map<Eigen::Matrix3d>(cov[0], 3, 3) );
	Eigen::VectorXd v = es.eigenvectors().col(0);

  gamma = -(v[0]*mean[0]+v[1]*mean[1]+v[2]*mean[2]);
  alpha = v[0];
  beta = v[1];
  theta = v[2];

  if(not normalize())
    return false;

  return true;
}