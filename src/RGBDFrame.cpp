#include "RGBDFrame.hpp"

const int SEED_R1=13;
const int SEED_R2=18;


const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar TEAL = cv::Scalar(128, 128, 0);



// Initialize a rgbd frame with a rgb and a depht image
RGBD_FRAME::RGBD_FRAME(cv::Mat &rgb, cv::Mat &depth, CAMERA_INFO &camera)
{
    this->rgb = rgb.clone();
    this->depth = depth.clone();
    this->camera=camera;
    cv::divide(1, depth, depth_inv, CV_64FC1);
}


// Extract plane based depth image, return a mask
std::vector<Plane> RGBD_FRAME::extract_planes(SEEDING_TYPE seeding_type, int seeds_count, double distance_thresh, double angle_thresh, int size_thresh, bool boundary, bool norm_cont, bool merge, double merge_dist_thresh=0.04, double merge_angle_thresh=0.05*M_PI)
{
  min_distance = cv::Mat(depth.size(), CV_64FC1, distance_thresh);
  planes_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
  horizontal_planes_mask = cv::Mat::zeros(depth.size(), CV_8UC1);

  cv::Mat normals = cv::Mat::zeros(depth.size(), CV_64FC3);
  cv::Mat edge = cv::Mat::zeros(depth.size(), CV_8UC1);
  if(boundary)
  { 
    cv::Mat edge_lap;
    cv::Laplacian(depth, edge_lap, CV_32F, 5);
    cv::threshold(edge_lap, edge, 200, 255, CV_8U);
  }

  if(norm_cont)
  {
    TickMeter normal_meter;
    normal_meter.start();
    normals = estimate_normals();
    cv::imwrite("visual/normals.png", normals);
    normal_meter.stop();
    std::cout << "normal time:" << normal_meter.getTimeSec() << std::endl;
  }

  // Seeding ...
  // std::cout << "seeding ... " << std::endl;
  std::vector<cv::Point3d> seeds;
  seeding(seeds, seeding_type, seeds_count);

  seeds_count = seeds.size();
  if(seeds_count%3 != 0)
  {
    std::cerr << "Something wrong with seeding." << std::endl;
    exit(-1);
  }

  cv::Mat all_seeds_rgb = rgb.clone();
  cv::Mat select_seeds_rgb = rgb.clone();

  cv::Mat all_seeds_depth = depth.clone();
  cv::Mat select_seeds_depth = depth.clone();

  std::vector<Plane> init_planes;
  for(int i=0; i<seeds_count; i+=3) 
  {
    // std::cout << "Select seeds and check ... " << std::endl;
    std::vector<cv::Point3d> current_seeds(seeds.begin()+i,seeds.begin()+i+3);

    cv::Point s0 = cv::Point(current_seeds[0].x, current_seeds[0].y);
    cv::Point s1 = cv::Point(current_seeds[1].x, current_seeds[1].y);
    cv::Point s2 = cv::Point(current_seeds[2].x, current_seeds[2].y);

    cv::circle(all_seeds_rgb, s0, 4, BLUE, -1);
    cv::circle(all_seeds_rgb, s1, 3, TEAL, -1);
    cv::circle(all_seeds_rgb, s2, 3, TEAL, -1);
    cv::line(all_seeds_rgb, s0, s1, TEAL, 2);
    cv::line(all_seeds_rgb, s0, s2, TEAL, 2);

    cv::circle(all_seeds_depth, s0, 4, BLUE, -1);
    cv::circle(all_seeds_depth, s1, 3, TEAL, -1);
    cv::circle(all_seeds_depth, s2, 3, TEAL, -1);
    cv::line(all_seeds_depth, s0, s1, TEAL, 2);
    cv::line(all_seeds_depth, s0, s2, TEAL, 2);

    if(current_seeds.size() != 3)
      continue;
    if(!in_image(current_seeds))
      continue;
    if(!have_valid_depth(current_seeds))
      continue;
    // std::cout << "before" << std::endl;
    if(!normal_consensus_check(current_seeds[0], current_seeds[1], angle_thresh)
      || !normal_consensus_check(current_seeds[0], current_seeds[2], angle_thresh))
       continue;
    // std::cout << "after" << std::endl;

    Plane plane(camera);
    if(!plane.initialize(current_seeds))
      continue;

    // std::cout << current_seeds[0] << std::endl;
    // std::cout << current_seeds[1] << std::endl;
    // std::cout << current_seeds[2] << std::endl;
    // std::cout << estimate_normal(current_seeds[0]) << std::endl;
    // std::cout << estimate_normal(current_seeds[1]) << std::endl;
    // std::cout << estimate_normal(current_seeds[2]) << std::endl;
    // std::cout << cv::Point3d(plane.n_a, plane.n_b, plane.n_c) << std::endl;
    // std::cout << "distance: " << std::endl;
    // std::cout << plane.calculate_distance(current_seeds[0])<< std::endl;
    // std::cout << plane.calculate_distance(current_seeds[1])<< std::endl;
    // std::cout << plane.calculate_distance(current_seeds[2])<< std::endl;
    // std::cout<< cv::Point3d(plane.n_a, plane.n_b, plane.n_c).dot(estimate_normal(current_seeds[0])) << std::endl;
    // std::cout<< cv::Point3d(plane.n_a, plane.n_b, plane.n_c).dot(estimate_normal(current_seeds[1])) << std::endl;
    // std::cout<< cv::Point3d(plane.n_a, plane.n_b, plane.n_c).dot(estimate_normal(current_seeds[2])) << std::endl;
    
    // if(!plane.in_plane(current_seeds[0], distance_thresh) || !normal_consensus_check(plane, current_seeds[0], angle_thresh))
    //   continue;
    plane.init_normal_cont = (normal_distance(plane, current_seeds[0]) + normal_distance(plane, current_seeds[1]) + normal_distance(plane, current_seeds[2]))/3;
    // std::cout << "init error: " << plane.init_normal_cont << std::endl;
    // std::cout << "angle thersh: " << angle_thresh << std::endl;
    if(plane.init_normal_cont < angle_thresh)
      continue;
    
    cv::circle(select_seeds_rgb, s0, 4, BLUE, -1);
    cv::circle(select_seeds_rgb, s1, 3, TEAL, -1);
    cv::circle(select_seeds_rgb, s2, 3, TEAL, -1);
    cv::line(select_seeds_rgb, s0, s1, TEAL, 2);
    cv::line(select_seeds_rgb, s0, s2, TEAL, 2);
    cv::circle(select_seeds_depth, s0, 4, BLUE, -1);
    cv::circle(select_seeds_depth, s1, 3, TEAL, -1);
    cv::circle(select_seeds_depth, s2, 3, TEAL, -1);
    cv::line(select_seeds_depth, s0, s1, TEAL, 2);
    cv::line(select_seeds_depth, s0, s2, TEAL, 2);
    
    init_planes.push_back(plane);
  }

  cv::imwrite("visual/all_seeds_rgb.png", all_seeds_rgb);
  cv::imwrite("visual/all_seeds_depth.png", all_seeds_depth);
  cv::imwrite("visual/select_seeds_rgb.png", select_seeds_rgb);
  cv::imwrite("visual/select_seeds_depth.png", select_seeds_depth);

  sort(init_planes.begin(), init_planes.end());
  // std::cout<< "norm: " << init_planes[50].init_normal_cont << std::endl;
  std::vector<Plane> planes;

  for (int i=0; i<init_planes.size(); i++)
  {
    std::set<int> neighbor_planes;
    Plane plane = init_planes[i];

    if(planes_mask.ptr<uchar>((int)plane.seed.y)[(int)plane.seed.x])
      continue;

    // std::cout << "plane growing ... " << std::endl;
    if (!plane_grow(plane, neighbor_planes, distance_thresh, angle_thresh, size_thresh, boundary, edge, norm_cont, normals))
      continue;

    // std::cout << "merging ... " << std::endl;
    if(merge)
    {
      std::set<int>::iterator it;
      for(it=neighbor_planes.begin(); it!=neighbor_planes.end(); it++)
      {
        if(planes[*it-1].valid && plane.is_coplanar(planes[*it-1], merge_angle_thresh, merge_dist_thresh))
        {
          plane.merge(planes[*it-1]);
          plane.mask |= planes[*it-1].mask;
        }
        // if(planes[*it-1].size() < size_thresh)
        // {
        //   planes[*it-1].valid = false;
        //   planes_mask.setTo(0, planes[*it-1].mask);
        // }
      }
    }
    planes.push_back(plane);
    
    // cv::imshow("plane", plane.mask);
    // cv::waitKey();

    planes_mask.setTo(planes.size(), plane.mask);

    if(plane.is_horizontal(M_PI/4))
    {
      horizontal_planes_mask.setTo(planes.size(), plane.mask);
    }
  }

  // cv::imwrite("visual/planes.png", planes_mask*10);
  // cv::imwrite("visual/horizontal_planes.png", horizontal_planes_mask*10);

  return planes;
}


bool RGBD_FRAME::normal_consensus_check(cv::Point3d &p1, cv::Point3d &p2, double angle_thresh)
{
  return abs(estimate_normal(p1).dot(estimate_normal(p2))) >angle_thresh;
}

bool RGBD_FRAME::normal_consensus_check(Plane &plane, cv::Point3d &point, double angle_thresh)
{
  return abs(cv::Point3d(plane.n_a, plane.n_b, plane.n_c).dot(estimate_normal(point)))>angle_thresh;
}

double RGBD_FRAME::normal_distance(Plane &plane, cv::Point3d &point)
{
  return abs(cv::Point3d(plane.n_a, plane.n_b, plane.n_c).dot(estimate_normal(point)));
}

void RGBD_FRAME::seeding(std::vector<cv::Point3d> &seeds, SEEDING_TYPE seeding_type, int seeds_count)
{
  if(seeds_count < 0)
  {
    std::cerr << "SEEDS COUNT SHOULE > 0!" << std::endl;
    exit(-1);
  }

  double cell_size = sqrt(depth.rows*depth.cols/seeds_count);
  int row_cells = (double)depth.rows/cell_size + 1;
  int col_cells = (double)depth.cols/cell_size + 1;
  seeds_count = row_cells * col_cells;

  srand((int)time(NULL));

  bool local = seeding_type & 0x01;
  bool grid = seeding_type & 0x02;

  // std::cout << seeding_type << std::endl;

  seeds.clear();

  for(int i=0; i<seeds_count; i++)
  {
    int r[3]={0, 0, 0};
    int c[3]={0, 0, 0};
    double inv_d[3]={0.0, 0.0, 0.0};
  
    if(grid)
    {
      double row_cell_size = (double)depth.rows/row_cells;
      double col_cell_size = (double)depth.cols/col_cells;
      // r[0] = (round((i/col_cells+(double)rand()/RAND_MAX)*row_cell_size));
      // c[0] = (round((i%col_cells+(double)rand()/RAND_MAX)*col_cell_size));
      r[0] = (round((i/col_cells+(double)1/2.)*row_cell_size));
      c[0] = (round((i%col_cells+(double)1/2.)*col_cell_size));
    }
    else
    {
      r[0] = rand()%depth.rows;
      c[0] = rand()%depth.cols;
    }
    inv_d[0] = depth_inv.ptr<double>(r[0])[c[0]];
    

    if(local)
    {
      // double angle_1 = (rand()/(double)RAND_MAX)*2*M_PI;
      // double angle_delta = (rand()/(double)RAND_MAX)*M_PI/2.0 + M_PI/4.0;
      // double angle_2;
      // if (rand()/(double)RAND_MAX<0.5)
      //   angle_2 = angle_1 - angle_delta;
      // else
      //   angle_2 = angle_1 + angle_delta;

      //uint radius_1 = rand()%(SEED_R2-SEED_R1) + SEED_R1;
      //uint radius_2 = rand()%(SEED_R2-SEED_R1) + SEED_R1;
      uint radius_1 = SEED_R1;
      uint radius_2 = SEED_R2;
      double angle_1 = M_PI/4;
      double angle_2 = M_PI/2;

      r[1]=r[0]+radius_1*cos(angle_1);
      c[1]=c[0]+radius_1*sin(angle_1);
      r[2]=r[0]+radius_2*cos(angle_2);
      c[2]=c[0]+radius_2*sin(angle_2);
      if (r[1]<depth.rows && r[1]>=0 && c[1]<depth.cols && c[1]>=0)
        inv_d[1] = depth_inv.ptr<double>(r[1])[c[1]];
      if (r[2]<depth.rows && r[2]>=0 && c[2]<depth.cols && c[2]>=0)
        inv_d[2] = depth_inv.ptr<double>(r[2])[c[2]];
    }
    else
    {
      r[1] = rand()%depth.rows;
      c[1] = rand()%depth.cols;
      inv_d[1] = depth_inv.ptr<double>(r[1])[c[1]];
      r[2] = rand()%depth.rows;
      c[2] = rand()%depth.cols;
      inv_d[2] = depth_inv.ptr<double>(r[2])[c[2]];
    }
    seeds.push_back(cv::Point3d(c[0], r[0], inv_d[0]));
    seeds.push_back(cv::Point3d(c[1], r[1], inv_d[1]));
    seeds.push_back(cv::Point3d(c[2], r[2], inv_d[2]));
  }
}


/************************************************************************
 * Description: this function gathers pixels in a plane from a seed, 
 *   within distance threshold.
 * Input:
 *     seed: the beginning point.
 *     
    

*************************************************************************/

bool RGBD_FRAME::plane_grow(Plane &plane, std::set<int> &neighbor_planes, double distance_thresh, double angle_thresh, int size_thresh, bool boundary, cv::Mat &edge, bool norm_cont, cv::Mat &normals)
{
  cv::Point3d seed = plane.seed;

  cv::Mat marked = cv::Mat::zeros(depth.size(), CV_8UC1);
  cv::Mat local_min_distance = min_distance;
  plane.mask = cv::Mat::zeros(depth.size(), CV_8UC1);

  std::vector<cv::Point3d> inliers;
  std::vector<cv::Point3d> neighbors;
  std::vector<cv::Point3d> candidates;
  std::vector<cv::Point3d> normal_vec;
  neighbors.push_back(seed);

  mark_mat(marked, seed, 255);
  const short near_four[4][2] = {-1, 0, 1, 0, 0, -1, 0, 1};

  int refine_iteration = 50;

  while(neighbors.size()>0)
  {
    cv::Mat neighbors_mat = cv::Mat(neighbors).reshape(1);
    cv::Mat plane_mat = (cv::Mat_<double>(3,1) << plane.alpha, plane.beta, plane.theta);
    cv::Mat distance_mat = abs(neighbors_mat*plane_mat + plane.gamma)/neighbors_mat.col(2)/sqrt(1+plane.theta*plane.theta);

    cv::Mat angle_dist_mat;
    if(norm_cont)
    {
      int neighbor_count = neighbors.size();
      for(int i=0; i<neighbor_count; i++)
        normal_vec.push_back(cv::Point3d(normals.at<cv::Vec3d>((int)neighbors[i].y, (int)neighbors[i].x)));
      cv::Mat normal_mat = cv::Mat(normal_vec).reshape(1);
      cv::Mat plane_normal = (cv::Mat_<double>(3,1) << plane.n_a, plane.n_b, plane.n_c);
      angle_dist_mat = abs(normal_mat*plane_normal);
      normal_vec.clear();
    }

    for (int i=0; i<distance_mat.rows; i++)
    {
      int current_r = neighbors[i].y;
      int current_c = neighbors[i].x;
      if (distance_mat.ptr<double>(i)[0]>=min_distance.ptr<double>(current_r)[current_c])
        continue; 
      
      local_min_distance.ptr<double>(current_r)[current_c] = distance_mat.ptr<double>(i)[0];

      uchar merge_plane_idx = planes_mask.ptr<uchar>(current_r)[current_c];
      if(merge_plane_idx)
      {
        neighbor_planes.insert(merge_plane_idx);
        // planes[merge_plane_idx].delete_point(neighbors[i]);
      }
      
      plane.insert_point(neighbors[i]);
      plane.min_c = std::min(plane.min_c, (int)neighbors[i].x-1);
      plane.min_r = std::min(plane.min_r, (int)neighbors[i].y-1);
      plane.max_c = std::max(plane.max_c, (int)neighbors[i].x+1);
      plane.max_r = std::max(plane.max_r, (int)neighbors[i].y+1);
      mark_mat(plane.mask, neighbors[i], 255);

      if (plane.size() > refine_iteration)
      {
        if (refine_iteration<=8000)
          refine_iteration = 2*refine_iteration;
        else
          refine_iteration = 1.2*refine_iteration;
        if(!plane.refit() || !plane.in_plane(seed, distance_thresh))
        {
          return false;
        }
      }
      
      if (norm_cont && angle_dist_mat.ptr<double>(i)[0]<angle_thresh)
          continue;

      if (boundary && edge.ptr<uchar>(current_r)[current_c])
          continue;

      for (int j=0; j<4; j++)
      {
        int next_r = current_r + near_four[j][0];
        int next_c = current_c + near_four[j][1];
        if (next_r<0 || next_r>=depth.rows || next_c<0 || next_c>=depth.cols)
          continue;
        if (marked.ptr<uchar>(next_r)[next_c])
          continue;
        if(!depth.ptr<ushort>(next_r)[next_c])
          continue;
        // ungreedy
        // if(planes_mask.ptr<ushort>(next_r)[next_c])
        //   continue;
        cv::Point3d next = cv::Point3d(next_c, next_r, depth_inv.ptr<double>(next_r)[next_c]);
        candidates.push_back(next);
        mark_mat(marked, next, 255);
      }
    }
    neighbors = candidates;
    candidates.clear();
  }

  if(plane.size()<size_thresh)
    return false;

  // std::cout << "success " << std::endl;
  local_min_distance.copyTo(min_distance, plane.mask);
  plane.valid = true;

  return true;
}

cv::Point3d RGBD_FRAME::estimate_normal(cv::Point3d &p)
{
  int r = p.y;
  int c = p.x;
  ushort top_d = r<1 ? 0 : depth.ptr<ushort>(r-1)[c];
  ushort bottom_d = r>depth.rows-1 ? 0 : depth.ptr<ushort>(r+1)[c];
  ushort left_d = c<1 ? 0 : depth.ptr<ushort>(r)[c-1];
  ushort right_d = c>depth.cols-1 ? 0 : depth.ptr<ushort>(r)[c+1];
  ushort center_d = depth.ptr<ushort>(r)[c];

  double dz_dr, dz_dc;

  if(!center_d || (!left_d && !right_d) || (!top_d && !top_d))
    return cv::Point3d(0, 0, 0);
  
  double valid_left = (double)((bool)left_d);
  double valid_right = (double)((bool)right_d);
  double valid_top = (double)((bool)top_d);
  double valid_bottom = (double)((bool)bottom_d);

  dz_dr = (valid_bottom*(bottom_d-center_d)+valid_top*(center_d-top_d))/(valid_bottom+valid_top);
  dz_dc = (valid_right*(right_d-center_d)+valid_left*(center_d-left_d))/(valid_right+valid_left);

  double dz_dx = camera.fc*dz_dc / (center_d + (c-camera.c0)*dz_dc + (r-camera.r0)*dz_dr);
  double dz_dy = camera.fr*dz_dr / (center_d + (c-camera.c0)*dz_dc + (r-camera.r0)*dz_dr);
  return cv::normalize(cv::Vec3d(dz_dx, dz_dy, -1));
}

cv::Mat RGBD_FRAME::estimate_normals()
{
  cv::Mat normals=cv::Mat::zeros(depth.size(), CV_64FC3);

  for (int r=1; r<depth.rows-1; r++)
    for(int c=1; c<depth.cols-1; c++)
    {
      cv::Point3d one = cv::Point3d(c, r, 0);
      normals.at<cv::Vec3d>(r,c) = estimate_normal(one);
    }

  return normals;
}

void RGBD_FRAME::bifilter(int radius, double sigma_c, double sigma_d)
{
  /***************************************************************
   * This function aims to fill depth image with bilateral filter.
   * radius -- the radius of filter window
   * sigma_c -- the standard variance in color space
   * sigma_d -- the standard variance in spatial space
  ***************************************************************/

  cv::Mat depth_tmp = depth.clone();

  for (int r=0; r < depth.rows; r++)
    for (int c=0; c < depth.cols; c++)
    {
      if (radius<0)
        radius = 1;

      double weight_sum = 0.0;
      double depth_sum = 0.0;
      
      for(int m=-radius; m<=radius; m++)
        for(int n=-radius; n<=radius; n++)
        {
          int r_tmp = r + m;
          int c_tmp = c + n;

          if (n==0 && m==0)
            continue;

          r_tmp = r_tmp < 0 ? 0 : r_tmp;
          r_tmp = r_tmp > depth.rows - 1 ? depth.rows - 1 : r_tmp;
          c_tmp = c_tmp < 0 ? 0 : c_tmp;
          c_tmp = c_tmp > depth.cols - 1 ? depth.cols - 1 : c_tmp;

          if(depth_tmp.ptr<ushort>(r_tmp)[c_tmp] < 50)
            continue;
          
          double color_diff = 0.0;
          for (int channel = 0; channel < 3; channel++)
            color_diff += pow(abs(rgb.ptr<uchar>(r)[3*c+channel]-rgb.ptr<uchar>(r_tmp)[3*c_tmp+channel]), 2);

          if (color_diff > 40)
            continue;

          double weight_tmp = exp(-color_diff/(2*sigma_c*sigma_c))*exp(-(m*m+n*n)/(2*sigma_d*sigma_d));

          weight_sum += weight_tmp;
          depth_sum += weight_tmp*depth_tmp.ptr<ushort>(r_tmp)[c_tmp];
        }

      if(weight_sum > 4)
        depth.ptr<ushort>(r)[c] = depth_sum/weight_sum;
    }
}



cv::Mat RGBD_FRAME::remove_planes(std::vector<Plane> &planes)
{
  cv::Mat depth_clone;
  cv::Mat total_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
  for (int i=0; i<planes.size(); i++)
  {
    total_mask = total_mask | planes[i].mask;
  }
  cv::bitwise_not(total_mask, total_mask);
  depth.copyTo(depth_clone, total_mask);

  return depth_clone;
}


std::vector<OBJECT> RGBD_FRAME::find_supported_objects(cv::Mat &left, std::vector<Plane> &horizeotal_planes, double min_h, double max_h)
{
  objects_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
  std::vector<OBJECT> objects;

  for(int r=0; r<left.rows; ++r)
    for(int c=0; c<left.cols; ++c) 
    {
      if(planes_mask.ptr<uchar>(r)[c])
        continue;

      if(objects_mask.ptr<uchar>(r)[c])
        continue;

      if(saliency.ptr<uchar>(r)[c] < 80)
        continue;

      cv::Point3d p(c, r, depth_inv.ptr<double>(r)[c]);
      
      // only points above certain horizontal planes in a certain distance can serve as a seed
      bool is_seed = false;
      for(int i=0; i<horizeotal_planes.size(); ++i)
      {
        // if(horizeotal_planes[i].calculate_relative_distance(p) > min_h && 
        //     horizeotal_planes[i].calculate_relative_distance(p) < max_h)
        if(horizeotal_planes[i].above_plane(p, min_h, max_h))
        {
          is_seed = true;
          break;
        }
      }


      if(!is_seed)
        continue;

      OBJECT object;

      cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
      cv::Mat marked = cv::Mat::zeros(depth.size(), CV_8UC1);

      std::vector<cv::Point3d> inliers;
      std::vector<cv::Point3d> neighbors;
      std::vector<cv::Point3d> candidates;

      // inliers.push_back(p);
      // mark_local(mask, p);
      // mark_local(marked, p);
      const short near_four[4][2] = {-1, 0, 1, 0, 0, -1, 0, 1};

      neighbors.push_back(p);

      object.top = p.y-1;
      object.bottom = p.y+1;
      object.left = p.x-1;
      object.right = p.x+1;

      mark_mat(marked, p, 255);

      while(neighbors.size()>0)
      {
        for (int i=0; i<neighbors.size(); i++)
        {
          cv::Point3d current = neighbors[i];
          inliers.push_back(current);
          object.top = std::min(object.top, (int)current.y-1);
          object.bottom = std::max(object.bottom, (int)current.y+1);
          object.left = std::min(object.left, (int)current.x-1);
          object.right = std::max(object.right, (int)current.x+1);
          mark_mat(mask, current, 255);
          for (int i=0; i<4; i++)
          {
            cv::Point3d next(current.x+near_four[i][1],current.y+near_four[i][0], 0);
            if (!in_image(next) || objects_mask.ptr<uchar>((int)next.y)[(int)next.x] || marked.ptr<uchar>((int)next.y)[(int)next.x] || !left.ptr<ushort>((int)next.y)[(int)next.x])
              continue;
            next.z = depth_inv.ptr<double>((int)next.y)[(int)next.x];

            if(abs(depth.ptr<ushort>((int)next.y)[(int)next.x]-depth.ptr<ushort>((int)current.y)[(int)current.x])>800)
              continue;

            candidates.push_back(next);
            // mark_local(marked, next);
            mark_mat(marked, next, 255);
          }
        }
        neighbors = candidates;
        candidates.clear();
      }

      // filter out strip object
      // if((object.bottom-object.top)/(object.right-object.left)<0.15)
      //   continue;
      // if((object.bottom-object.top)/(object.right-object.left)>8)
      //   continue;

      // std::cout << "height: "  << object.bottom-object.top << std::endl;
      // std::cout << "width: " << object.right-object.left << std::endl;
      // std::cout << "size: " << inliers.size() << std::endl;
      // std::cout << (double)inliers.size()/(object.bottom-object.top)/(object.right-object.left) << std::endl;
      if((double)inliers.size()/(object.bottom-object.top)/(object.right-object.left)<0.3)
        continue;


      if(inliers.size()>1000)
      {
          object.mask = mask.clone();
          object.objectness = 1.0;
          objects.push_back(object);
          objects_mask.setTo(objects.size(), mask);
      }
    }
  return objects;

}
