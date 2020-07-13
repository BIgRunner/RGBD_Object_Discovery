#include "RGBDFrame.hpp"


const double cos_distance = cos(M_PI/10);


// Initialize a rgbd frame with a rgb and a depht image
RGBD_FRAME::RGBD_FRAME(cv::Mat &rgb, cv::Mat &depth, CAMERA_INFO &camera)
{
    this->rgb = rgb.clone();
    this->depth = depth.clone();
    this->planes_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    this->camera=camera;

    // bifilter(2, 10, 3);
    cv::divide(1.0, depth, depth_inv, CV_64FC1);

    this->normals = estimate_normals();

    cv::waitKey();
}


// Extract plane based depth image, return a mask
std::vector<Plane> RGBD_FRAME::extract_planes_by_ransac(uint iteration)
{
    std::vector<Plane> planes;

    for(uint iter=0; iter<iteration; iter++)
    {
        std::vector<DEPTH_PIXEL> seeds;
        Plane plane;
        plane.mask = cv::Mat::zeros(depth.size(), CV_8UC1);

        if(!initialize_seeds_in_image(seeds))
            continue;

        if(!plane_by_three_points(seeds, plane))
            continue;

        // Region Growing

        if(!plane_grow(seeds[0], plane, 0.01, depth.cols*depth.rows/100))
            continue;

        planes.push_back(plane);
        
        planes_mask = planes_mask | plane.mask;
    }

    return planes;
}

std::vector<Plane> RGBD_FRAME::extract_planes_by_grid(uint cell_iteration, uint width_size, uint height_size)
{
    std::vector<Plane> planes;

    uint cell_size_width = depth.cols/width_size;
    uint cell_size_height = depth.rows/height_size;

    uint kernal_size = std::min(cell_size_height, cell_size_width);

    for(int cell_r=cell_size_height/2; cell_r<depth.rows; cell_r+=cell_size_height)
        for(int cell_c=cell_size_width/2; cell_c<depth.cols; cell_c+=cell_size_width)
            for(uint iter=0; iter<cell_iteration; iter++)
            {
                std::vector<DEPTH_PIXEL> seeds;
                Plane plane;
                plane.mask = cv::Mat::zeros(depth.size(), CV_8UC1);

                if(!initialize_seeds_in_grid(seeds, cell_r, cell_c, kernal_size))
                    continue;

                if(!plane_by_three_points(seeds, plane))
                    continue;

                // Region Growing

                if(!plane_grow(seeds[0], plane, 0.01, depth.cols*depth.rows/100))
                    continue;

                planes.push_back(plane);
        
                planes_mask = planes_mask | plane.mask;
            }
    return planes;
}

cv::Mat RGBD_FRAME::estimate_normals()
{
    cv::Mat normals=cv::Mat::zeros(depth.size(), CV_64FC3);

    for (int r=1; r<=depth.rows; r++)
        for(int c=1; c<=depth.cols; c++)
        {
            ushort top_d = depth.ptr<ushort>(r-1)[c];
            ushort bottom_d = depth.ptr<ushort>(r+1)[c];
            ushort left_d = depth.ptr<ushort>(r)[c-1];
            ushort center_d = depth.ptr<ushort>(r)[c];
            ushort right_d = depth.ptr<ushort>(r)[c+1];

            if(!left_d || !center_d || !right_d || !top_d || !bottom_d)
                continue;

            double dz_dr = (bottom_d-top_d)/2.0;
            double dz_dc = (right_d-left_d)/2.0;

            double dz_dx = camera.fc*dz_dc / (center_d + (c-camera.c0)*dz_dc + (r-camera.r0)*dz_dr);
            double dz_dy = camera.fr*dz_dr / (center_d + (c-camera.c0)*dz_dc + (r-camera.r0)*dz_dr);
            normals.at<cv::Vec3d>(r, c) = cv::normalize(cv::Vec3d(dz_dx, dz_dy, -1.0));
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
    // gettimeofday(&stop, NULL);
    // timeuse = stop.tv_sec - start.tv_sec + (stop.tv_usec - start.tv_usec)/1000000.0;
    // std::cout << timeuse<< std::endl;
}



bool RGBD_FRAME::initialize_seeds_in_image(std::vector<DEPTH_PIXEL> &seeds)
{
    // set seeds
    // std::cout << "set seeds" << std::endl;

    srand((int)(time(NULL)));

    DEPTH_PIXEL seed;
    seed.c = rand() % depth.cols;
    seed.r = rand() % depth.rows;
    // std::cout << "first seed" << std::endl;

    // radius between 10 and 20
    uint radius_1 = rand()%50 + 15;
    uint radius_2 = rand()%50 + 15;

    // angle in 2*pi
    double angle_1 = (rand()/(double)RAND_MAX)*2*M_PI;
    double angle_2 = (rand()/(double)RAND_MAX)*2*M_PI;

    while (abs(angle_2-angle_1)<M_PI/8 || abs(abs(angle_2-angle_1)-M_PI)<M_PI/8)
    {
        angle_2 = (rand()/(double)RAND_MAX)*2*M_PI;
    }

    DEPTH_PIXEL seed1, seed2;

    seed1.r=seed.r+radius_1*cos(angle_1);
    seed1.c=seed.c+radius_1*sin(angle_1);
    seed2.r=seed.r+radius_2*cos(angle_2);
    seed2.c=seed.c+radius_2*sin(angle_2);

    if(seed1.c<0 || seed1.c>=depth.cols || seed1.r<0 || seed1.r>=depth.rows 
        || seed2.c<0 || seed2.c>=depth.cols || seed2.r<0 || seed2.r>=depth.rows)
        return false;
    
    if(!depth.ptr<ushort>(seed.r)[seed.c] || !depth.ptr<ushort>(seed1.r)[seed1.c]
        || !depth.ptr<ushort>(seed2.r)[seed2.c])
        return false;

    if(normal_distance(seed, seed1)<cos_distance || normal_distance(seed, seed2)<cos_distance)
        return false;

    seed.inv_d = depth_inv.ptr<double>(seed.r)[seed.c];
    seed1.inv_d = depth_inv.ptr<double>(seed1.r)[seed1.c];
    seed2.inv_d = depth_inv.ptr<double>(seed2.r)[seed2.c];

    seeds.clear();
    seeds.push_back(seed);
    seeds.push_back(seed1);
    seeds.push_back(seed2);

    return true;
}


bool RGBD_FRAME::initialize_seeds_in_grid(std::vector<DEPTH_PIXEL> &seeds, int r, int c, int kernal)
{
    // set seeds
    // std::cout << "set seeds" << std::endl;
    DEPTH_PIXEL seed;
    seed.c = r-kernal/2+rand()%kernal;
    seed.r = c-kernal/2+rand()%kernal;
    if(!in_image(seed))
        return false;
    // std::cout << "first seed" << std::endl;

    // radius between 10 and 20
    uint radius_1 = rand()%50 + 15;
    uint radius_2 = rand()%50 + 15;

    // angle in 2*pi
    double angle_1 = (rand()/(double)RAND_MAX)*2*M_PI;
    double angle_2 = (rand()/(double)RAND_MAX)*2*M_PI;

    while (abs(angle_2-angle_1)<M_PI/8 || abs(abs(angle_2-angle_1)-M_PI)<M_PI/8)
    {
        angle_2 = (rand()/(double)RAND_MAX)*2*M_PI;
    }

    DEPTH_PIXEL seed1, seed2;

    seed1.r=seed.r+radius_1*cos(angle_1);
    seed1.c=seed.c+radius_1*sin(angle_1);
    seed2.r=seed.r+radius_2*cos(angle_2);
    seed2.c=seed.c+radius_2*sin(angle_2);

    if(seed1.c<0 || seed1.c>=depth.cols || seed1.r<0 || seed1.r>=depth.rows 
        || seed2.c<0 || seed2.c>=depth.cols || seed2.r<0 || seed2.r>=depth.rows)
        return false;
    
    if(!depth.ptr<ushort>(seed.r)[seed.c] || !depth.ptr<ushort>(seed1.r)[seed1.c]
        || !depth.ptr<ushort>(seed2.r)[seed2.c])
        return false;

    if(normal_distance(seed, seed1)<cos_distance || normal_distance(seed, seed2)<cos_distance)
        return false;

    seed.inv_d = depth_inv.ptr<double>(seed.r)[seed.c];
    seed1.inv_d = depth_inv.ptr<double>(seed1.r)[seed1.c];
    seed2.inv_d = depth_inv.ptr<double>(seed2.r)[seed2.c];

    seeds.clear();
    seeds.push_back(seed);
    seeds.push_back(seed1);
    seeds.push_back(seed2);

    return true;
}


bool RGBD_FRAME::plane_by_three_points(std::vector<DEPTH_PIXEL> &seeds, Plane &p)
{
    if(seeds.size() != 3)
    {
        std::cerr << "Warnning:  three points to determine a plane."  << std::endl;
        return false;
    }

    double alpha = (seeds[1].r-seeds[0].r)*(seeds[2].inv_d-seeds[0].inv_d)-(seeds[2].r-seeds[0].r)*(seeds[1].inv_d-seeds[0].inv_d);
    double beta = (seeds[2].c-seeds[0].c)*(seeds[1].inv_d-seeds[0].inv_d)-(seeds[1].c-seeds[0].c)*(seeds[2].inv_d-seeds[0].inv_d);
    double theta = (seeds[1].c-seeds[0].c)*(seeds[2].r-seeds[0].r)-(seeds[2].c-seeds[0].c)*(seeds[1].r-seeds[0].r);
    double gamma = -alpha*seeds[0].c-beta*seeds[0].r-theta*seeds[0].inv_d;

    double a = alpha*camera.fc*camera.scale;
    double b = beta*camera.fr*camera.scale;
    double c = (gamma+beta*camera.r0+alpha*camera.c0)*camera.scale;
    double d = theta;

    double s = sqrt(a*a+b*b+c*c);
    p.a = a/s;
    p.b = b/s;
    p.c = c/s;
    p.d = d/s; 

    p.alpha = p.a/(camera.fc*camera.scale);
    p.beta = p.b/(camera.fr*camera.scale);
    p.gamma = p.c/camera.scale-p.beta*camera.r0-p.alpha*camera.c0;
    p.theta = p.d;

    p.mask = cv::Mat::zeros(depth.size(), CV_8UC1);

    std::cout<<"first seed"<<std::endl;
    std::cout << "p.alpha: " << p.alpha << std::endl;
    std::cout << "p.beta: " << p.beta << std::endl;
    std::cout << "p.gamma: " << p.gamma << std::endl;
    std::cout << "p.theta: " << p.theta << std::endl;
    std::cout << "seeds[0].c: " << seeds[0].c << std::endl;
    std::cout << "seeds[0].r: " << seeds[0].r << std::endl;
    std::cout << "seeds[0].inv_d: " << seeds[0].inv_d << std::endl;
    std::cout<<p.alpha*seeds[0].c + p.beta*seeds[0].r+p.gamma+p.theta*seeds[0].inv_d<<std::endl;

    return true;
}


/************************************************************************
 * Description: this function gathers pixels in a plane from a seed, 
 *   within distance threshold.
 * Input:
 *     seed: the beginning point.
 *     
    

*************************************************************************/

bool RGBD_FRAME::plane_grow(DEPTH_PIXEL &seed, Plane &plane, double distance_thresh,
                        int min_pixels)
{
    // std::cout << seed.r << " " << seed.c << " " << seed.inv_d << std::endl;
    if(!plane.in_plane(seed, distance_thresh))
    {
        std::cerr<<"The seed is NOT IN THE PLANE!!" << std::endl;
        return false;
    }

    cv::Mat marked = cv::Mat::zeros(depth.size(), CV_8UC1);

    std::vector<DEPTH_PIXEL> inliers;
    std::queue<DEPTH_PIXEL> neighbors;
    inliers.push_back(seed);
    mark_local(plane.mask, seed);
    mark_local(marked, seed);
    const short near_four[4][2] = {-1, 0, 1, 0, 0, -1, 0, 1};

    neighbors.push(seed);

    // std::cout << "growing ..." << std::endl;
    while(neighbors.size()>0)
    {
        DEPTH_PIXEL current = neighbors.front();
        neighbors.pop();
        if (plane.in_plane(current, distance_thresh))
        {
            inliers.push_back(current);
            mark_local(plane.mask, current);
            for (int i=0; i<4; i++)
            {
                DEPTH_PIXEL next{r:current.r + near_four[i][0], c:current.c + near_four[i][1]};
                if (!in_image(next) && !depth.ptr<ushort>(next.r)[next.c])
                    continue;
                next.inv_d = depth_inv.ptr<double>(next.r)[next.c];

                if (planes_mask.ptr<uchar>(next.r)[next.c] || marked.ptr<uchar>(next.r)[next.c])
                    continue;

                neighbors.push(next);
                mark_local(marked, next);
            }
            if (inliers.size()%1000 == 0)
            {
                if(!refine_plane(inliers, plane, distance_thresh))
                {
                    return false;
                }
                // std::cout << "fit result: " << plane_coeff.alpha*seed.c+plane_coeff.beta*seed.r+plane_coeff.gamma+plane_coeff.theta*seed.inv_d << std::endl;

            }
        }
    }

    if(inliers.size()<min_pixels)
        return false;

    // cv::imshow("plane",plane.mask);
    // cv::waitKey();

    return true;
}


bool RGBD_FRAME::refine_plane(std::vector<DEPTH_PIXEL> &pixels, Plane &p, double distance_thresh)
{
    
    // std::cout << "refine the plane ..." << std::endl;

    cv::Mat A = cv::Mat::zeros(pixels.size(), 3, CV_64FC1);
    cv::Mat y = cv::Mat::zeros(pixels.size(), 1, CV_64FC1);

    for(int i=0; i<pixels.size(); ++i)
    {
        A.ptr<double>(i)[0]=pixels[i].c;
        A.ptr<double>(i)[1]=pixels[i].r;
        A.ptr<double>(i)[2]=1.0;

        y.ptr<double>(i)[0]=-1.0*pixels[i].inv_d;
    }

    cv::Mat x;

    cv::solve(A, y, x, cv::DECOMP_SVD);
    double alpha = x.ptr<double>(0)[0];
    double beta = x.ptr<double>(1)[0];
    double gamma = x.ptr<double>(2)[0];
    double theta = 1.0;

    double a = alpha*camera.fc*camera.scale;
    double b = beta*camera.fr*camera.scale;
    double c = (gamma+beta*camera.r0+alpha*camera.c0)*camera.scale;
    double d = theta;

    double s = sqrt(a*a+b*b+c*c);
    p.a = a/s;
    p.b = b/s;
    p.c = c/s;
    p.d = d/s; 

    p.alpha = p.a/(camera.fc*camera.scale);
    p.beta = p.b/(camera.fr*camera.scale);
    p.gamma = p.c/camera.scale-p.beta*camera.r0-p.alpha*camera.c0;
    p.theta = p.d;

    if(!p.in_plane(pixels[0], distance_thresh))
        return false;

    return true;
    
    // std::cout << "plane refined " << std::endl;
}